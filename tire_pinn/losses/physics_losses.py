"""
Physics-informed loss functions for PINN tire model.

Implements constraints to address known issues (C1-C6):
- Zero slip constraint (C3: Fx=Fy=0 when SR=SA=0)
- Symmetry constraint (C4: Fy(-SA) = -Fy(SA) when IA=0)
- Friction circle constraint (C1/C2: Kamm's circle)
- Combined slip consistency
- Small-slip stiffness load sensitivity
- High-slip lateral force load sensitivity
- Mz pneumatic trail sanity check
- Camber thrust symmetry/cap
- Pure-slip section smoothness
- Monotonicity constraint (C6: ∂Fy/∂Fz ≥ 0)
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Dict, Optional, Tuple


class ZeroSlipLoss(nn.Module):
    """
    Enforce Fx = Fy = Mz = 0 when SR = SA = IA = 0.

    Addresses C3: Current DNN outputs ~5N at zero slip.
    Target: <2N error.
    """

    def __init__(self, force_threshold: float = 2.0, moment_threshold: float = 5.0):
        """
        Args:
            force_threshold: Maximum acceptable force at zero slip (N)
            moment_threshold: Maximum acceptable moment at zero slip (Nm)
        """
        super().__init__()
        self.force_threshold = force_threshold
        self.moment_threshold = moment_threshold

    def forward(
        self,
        model: nn.Module,
        colloc_batch: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            model: PINN model returning raw physical outputs
            colloc_batch: (N, 5) raw physical inputs with SR=SA=IA=0
                          [0, 0, Fz, 0, TSTC]

        Returns:
            loss: threshold-scaled MSE of forces at zero slip
            metrics: dict with max/mean violations
        """
        # Enforce zero slip constraints only when IA is zero (camber thrust exists for IA != 0)
        ia = colloc_batch[:, 3]
        mask = torch.abs(ia) < 1e-6
        if not mask.any():
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "zero_slip_loss": 0.0,
                "zero_slip_max_fx": 0.0,
                "zero_slip_max_fy": 0.0,
                "zero_slip_max_mz": 0.0,
            }

        colloc_filtered = colloc_batch[mask]
        pred = model(colloc_filtered)  # (N_filtered, 4) raw
        fx, fy, mz = pred[:, 0], pred[:, 1], pred[:, 3]

        force_scale = max(self.force_threshold, 1e-6)
        moment_scale = max(self.moment_threshold, 1e-6)
        loss = (
            (fx / force_scale) ** 2
            + (fy / force_scale) ** 2
            + (mz / moment_scale) ** 2
        ).mean()

        # Metrics
        max_fx = fx.abs().max().item()
        max_fy = fy.abs().max().item()
        max_mz = mz.abs().max().item()

        metrics = {
            "zero_slip_loss": loss.item(),
            "zero_slip_max_fx": max_fx,
            "zero_slip_max_fy": max_fy,
            "zero_slip_max_mz": max_mz,
            "zero_slip_force_threshold": self.force_threshold,
            "zero_slip_moment_threshold": self.moment_threshold,
        }

        return loss, metrics


class SymmetryLoss(nn.Module):
    """
    Enforce Fy(SA) = -Fy(-SA) when IA = 0.

    Addresses C4: TTC/SAE sign convention inconsistency.
    Target: <1% asymmetry.
    """

    def __init__(self):
        super().__init__()

    def forward(
        self,
        model: nn.Module,
        colloc_pairs: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            model: PINN model
            colloc_pairs: (N, 5) raw physical inputs with IA=0, SR=0
                          Generate pairs by flipping SA sign

        Returns:
            loss: MSE of (Fy(SA) + Fy(-SA))
            metrics: dict with symmetry violations
        """
        # Generate SA-flipped pairs
        x_pos = colloc_pairs.clone()
        x_neg = colloc_pairs.clone()
        x_neg[:, 1] = -x_pos[:, 1]  # Flip SA (index 1)

        pred_pos = model(x_pos)
        pred_neg = model(x_neg)

        fy_pos = pred_pos[:, 1]
        fy_neg = pred_neg[:, 1]

        # Fy should be antisymmetric: Fy(SA) ≈ -Fy(-SA)
        denom = torch.maximum(
            torch.maximum(fy_pos.abs(), fy_neg.abs()),
            torch.full_like(fy_pos, 1.0),
        )
        asymmetry = (fy_pos + fy_neg) / denom
        loss = (asymmetry**2).mean()

        # Metrics
        max_asymmetry = asymmetry.abs().max().item()
        mean_asymmetry = asymmetry.abs().mean().item()

        metrics = {
            "symmetry_loss": loss.item(),
            "symmetry_max_error": max_asymmetry,
            "symmetry_mean_error": mean_asymmetry,
        }

        return loss, metrics


class FrictionCircleLoss(nn.Module):
    """
    Enforce Kamm's circle: sqrt(Fx² + Fy²) ≤ μ_cap * Fz.

    Addresses C1/C2: Combined slip extrapolation, friction circle violation.
    Target: <2% violation rate.
    """

    def __init__(
        self,
        mu_cap: float = 2.6,
        fz_mean: float = 0.0,
        fz_std: float = 1.0,
        fx_mean: float = 0.0,
        fx_std: float = 1.0,
        fy_mean: float = 0.0,
        fy_std: float = 1.0,
    ):
        """
        Args:
            mu_cap: Peak friction coefficient + margin (TTC ~2.4, use 2.6)
            fz_mean/std: Normalization params for Fz (to denormalize)
            force_mean/std: Normalization params for Fx/Fy
        """
        super().__init__()
        self.mu_cap = mu_cap
        self.register_buffer("fz_mean", torch.tensor(fz_mean))
        self.register_buffer("fz_std", torch.tensor(fz_std))
        self.register_buffer("fx_mean", torch.tensor(fx_mean))
        self.register_buffer("fx_std", torch.tensor(fx_std))
        self.register_buffer("fy_mean", torch.tensor(fy_mean))
        self.register_buffer("fy_std", torch.tensor(fy_std))

    def set_normalization_params(
        self,
        fz_mean: float,
        fz_std: float,
        fx_mean: float,
        fx_std: float,
        fy_mean: float,
        fy_std: float,
    ):
        """Update normalization parameters from data."""
        self.fz_mean.copy_(torch.tensor(fz_mean, device=self.fz_mean.device))
        self.fz_std.copy_(torch.tensor(fz_std, device=self.fz_std.device))
        self.fx_mean.copy_(torch.tensor(fx_mean, device=self.fx_mean.device))
        self.fx_std.copy_(torch.tensor(fx_std, device=self.fx_std.device))
        self.fy_mean.copy_(torch.tensor(fy_mean, device=self.fy_mean.device))
        self.fy_std.copy_(torch.tensor(fy_std, device=self.fy_std.device))

    def forward(
        self,
        model: nn.Module,
        colloc_batch: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            model: PINN model
            colloc_batch: (N, 5) normalized inputs (dense in combined slip)

        Returns:
            loss: Mean violation (ReLU of excess)
            metrics: dict with violation stats
        """
        pred = model(colloc_batch)  # (N, 4) raw
        fx = pred[:, 0]
        fy = pred[:, 1]
        fz = colloc_batch[:, 2]

        # Friction circle check
        force_mag = torch.sqrt(fx**2 + fy**2)
        limit = self.mu_cap * fz.abs()

        # Penalize relative excess to keep the loss dimensionless and load-scaled.
        violation = torch.relu(force_mag - limit)
        violation_ratio = torch.relu(force_mag / (limit + 1e-6) - 1.0)
        loss = (violation_ratio**2).mean()

        # Metrics
        violation_rate = (violation > 0).float().mean().item()
        max_violation = violation.max().item()

        metrics = {
            "friction_circle_loss": loss.item(),
            "friction_violation_rate": violation_rate,
            "friction_max_violation": max_violation,
        }

        return loss, metrics


class CombinedSlipLoss(nn.Module):
    """
    Penalize combined-slip force growth relative to the pure-slip response.

    A tire under combined longitudinal/lateral slip should generally trade one
    force component for the other. This loss does not prescribe a full tire
    model; it only catches the dangerous case where adding the other slip makes
    Fx or Fy grow beyond its pure-slip counterpart.
    """

    def __init__(self, mu_cap: float = 2.6, margin: float = 0.03, epsilon: float = 1e-6):
        super().__init__()
        self.mu_cap = mu_cap
        self.margin = margin
        self.epsilon = epsilon

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute combined-slip consistency loss."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "combined_slip_loss": 0.0,
                "combined_slip_violation_rate_fx": 0.0,
                "combined_slip_violation_rate_fy": 0.0,
                "combined_slip_max_excess": 0.0,
            }

        x_full = colloc_batch
        x_pure_long = colloc_batch.clone()
        x_pure_lat = colloc_batch.clone()
        x_pure_long[:, 1] = 0.0
        x_pure_lat[:, 0] = 0.0

        pred_full = model(x_full)
        pred_pure_long = model(x_pure_long)
        pred_pure_lat = model(x_pure_lat)

        fx_full = torch.abs(pred_full[:, 0])
        fy_full = torch.abs(pred_full[:, 1])
        fx_pure = torch.abs(pred_pure_long[:, 0])
        fy_pure = torch.abs(pred_pure_lat[:, 1])

        force_scale = self.mu_cap * torch.abs(colloc_batch[:, 2]) + self.epsilon
        fx_excess = F.relu((fx_full - fx_pure) / force_scale - self.margin)
        fy_excess = F.relu((fy_full - fy_pure) / force_scale - self.margin)

        loss = torch.mean(fx_excess**2 + fy_excess**2)

        metrics = {
            "combined_slip_loss": loss.item(),
            "combined_slip_violation_rate_fx": (fx_excess > 0).float().mean().item(),
            "combined_slip_violation_rate_fy": (fy_excess > 0).float().mean().item(),
            "combined_slip_max_excess": torch.max(torch.cat([fx_excess, fy_excess])).item(),
        }

        return loss, metrics


class StiffnessLoadSensitivityLoss(nn.Module):
    """
    Constrain small-slip stiffness trends with vertical load.

    This is intentionally a weak trend prior rather than a full Pacejka/TMeasy
    law. It checks that longitudinal small-slip stiffness keeps the expected
    sign, and that |Ckappa| / |Calpha| do not decrease when Fz is nudged upward.
    """

    def __init__(
        self,
        sr_delta: float = 0.01,
        sa_delta_deg: float = 0.25,
        load_step_ratio: float = 0.20,
        min_load_step: float = 100.0,
        normalized_margin: float = 0.02,
        epsilon: float = 1e-6,
    ):
        super().__init__()
        self.sr_delta = sr_delta
        self.sa_delta_deg = sa_delta_deg
        self.load_step_ratio = load_step_ratio
        self.min_load_step = min_load_step
        self.normalized_margin = normalized_margin
        self.epsilon = epsilon

    def _eval_force(
        self,
        model: nn.Module,
        base: torch.Tensor,
        sr: float,
        sa_deg: float,
        fz: torch.Tensor,
    ) -> torch.Tensor:
        x = base.clone()
        x[:, 0] = sr
        x[:, 1] = sa_deg
        x[:, 2] = fz
        return model(x)

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute load-sensitivity loss using finite-difference stiffness."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "stiffness_load_loss": 0.0,
                "stiffness_negative_kx_rate": 0.0,
                "stiffness_load_violation_rate_kx": 0.0,
                "stiffness_load_violation_rate_ky": 0.0,
            }

        base = colloc_batch.clone()
        fz_low = torch.abs(base[:, 2]).clamp_min(self.min_load_step)
        load_step = torch.clamp(
            self.load_step_ratio * fz_low,
            min=self.min_load_step,
        )
        fz_high = fz_low + load_step

        fx_plus_low = self._eval_force(model, base, self.sr_delta, 0.0, fz_low)[:, 0]
        fx_minus_low = self._eval_force(model, base, -self.sr_delta, 0.0, fz_low)[:, 0]
        fx_plus_high = self._eval_force(model, base, self.sr_delta, 0.0, fz_high)[:, 0]
        fx_minus_high = self._eval_force(model, base, -self.sr_delta, 0.0, fz_high)[:, 0]

        fy_plus_low = self._eval_force(model, base, 0.0, self.sa_delta_deg, fz_low)[:, 1]
        fy_minus_low = self._eval_force(model, base, 0.0, -self.sa_delta_deg, fz_low)[:, 1]
        fy_plus_high = self._eval_force(model, base, 0.0, self.sa_delta_deg, fz_high)[:, 1]
        fy_minus_high = self._eval_force(model, base, 0.0, -self.sa_delta_deg, fz_high)[:, 1]

        kx_low = (fx_plus_low - fx_minus_low) / (2.0 * self.sr_delta)
        kx_high = (fx_plus_high - fx_minus_high) / (2.0 * self.sr_delta)
        ky_low = (fy_plus_low - fy_minus_low) / (2.0 * self.sa_delta_deg)
        ky_high = (fy_plus_high - fy_minus_high) / (2.0 * self.sa_delta_deg)

        kx_low_norm = kx_low / (fz_low + self.epsilon)
        kx_high_norm = kx_high / (fz_high + self.epsilon)
        ky_low_abs_norm = torch.abs(ky_low) / (fz_low + self.epsilon)
        ky_high_abs_norm = torch.abs(ky_high) / (fz_high + self.epsilon)

        kx_sign = F.relu(-kx_low_norm)
        kx_load_drop = F.relu((kx_low - kx_high) / (fz_high + self.epsilon))
        ky_load_drop = F.relu((torch.abs(ky_low) - torch.abs(ky_high)) / (fz_high + self.epsilon))
        kx_eff_growth = F.relu(kx_high_norm - kx_low_norm - self.normalized_margin)
        ky_eff_growth = F.relu(ky_high_abs_norm - ky_low_abs_norm - self.normalized_margin)

        loss = torch.mean(
            kx_sign**2 + kx_load_drop**2 + ky_load_drop**2 + kx_eff_growth**2 + ky_eff_growth**2
        )

        metrics = {
            "stiffness_load_loss": loss.item(),
            "stiffness_negative_kx_rate": (kx_sign > 0).float().mean().item(),
            "stiffness_load_violation_rate_kx": (kx_load_drop > 0).float().mean().item(),
            "stiffness_load_violation_rate_ky": (ky_load_drop > 0).float().mean().item(),
        }

        return loss, metrics


class HighSaLateralLoadSensitivityLoss(nn.Module):
    """
    Constrain saturated lateral-force magnitude to remain non-decreasing with Fz.

    This targets the high-|SA| regime where a model can look good in aggregate
    R² while letting a higher-load Fy(SA) curve cross below a lower-load curve.
    Load sensitivity of mu may decrease, but absolute |Fy| should not drop.
    """

    def __init__(
        self,
        min_abs_sa_deg: float = 6.0,
        load_step_ratio: float = 0.20,
        min_load_step: float = 100.0,
        drop_margin: float = 0.01,
        mu_cap: float = 2.6,
        epsilon: float = 1e-6,
    ):
        super().__init__()
        self.min_abs_sa_deg = min_abs_sa_deg
        self.load_step_ratio = load_step_ratio
        self.min_load_step = min_load_step
        self.drop_margin = drop_margin
        self.mu_cap = mu_cap
        self.epsilon = epsilon

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute high-slip |Fy| load monotonicity loss."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "high_sa_lateral_load_loss": 0.0,
                "high_sa_lateral_load_violation_rate": 0.0,
                "high_sa_lateral_load_max_drop_ratio": 0.0,
            }

        base = colloc_batch.clone()
        sa_sign = torch.where(base[:, 1] >= 0.0, 1.0, -1.0)
        base[:, 0] = 0.0
        base[:, 1] = sa_sign * torch.clamp(torch.abs(base[:, 1]), min=self.min_abs_sa_deg)
        base[:, 3] = 0.0

        fz_low = torch.abs(base[:, 2]).clamp_min(self.min_load_step)
        load_step = torch.clamp(self.load_step_ratio * fz_low, min=self.min_load_step)
        fz_high = fz_low + load_step

        x_low = base.clone()
        x_high = base.clone()
        x_low[:, 2] = fz_low
        x_high[:, 2] = fz_high

        fy_low = torch.abs(model(x_low)[:, 1])
        fy_high = torch.abs(model(x_high)[:, 1])

        force_scale = self.mu_cap * fz_high + self.epsilon
        drop_ratio = (fy_low - fy_high) / force_scale
        violation = F.relu(drop_ratio - self.drop_margin) / max(self.drop_margin, self.epsilon)
        loss = torch.mean(violation**2)

        metrics = {
            "high_sa_lateral_load_loss": loss.item(),
            "high_sa_lateral_load_violation_rate": (violation > 0).float().mean().item(),
            "high_sa_lateral_load_max_drop_ratio": drop_ratio.max().item(),
        }
        return loss, metrics


class MzPneumaticTrailLoss(nn.Module):
    """
    Constrain aligning torque through a pneumatic-trail sanity check.

    Uses the convention trail = -Mz / Fy. The loss is intentionally small in
    training because Mz sign conventions are easy to break at integration
    boundaries; the test catches obviously inconsistent exported behavior.
    """

    def __init__(
        self,
        small_sa_deg: float = 1.5,
        large_sa_deg: float = 6.0,
        min_abs_sa_deg: float = 1.0,
        max_trail_m: float = 0.18,
        min_trail_m: float = 0.0,
        trail_margin_m: float = 0.01,
        min_lateral_force_n: float = 75.0,
        sign_loss_weight: float = 1.0,
        odd_loss_weight: float = 0.25,
        epsilon: float = 1e-3,
    ):
        super().__init__()
        self.small_sa_deg = small_sa_deg
        self.large_sa_deg = large_sa_deg
        self.min_abs_sa_deg = min_abs_sa_deg
        self.max_trail_m = max_trail_m
        self.min_trail_m = min_trail_m
        self.trail_margin_m = trail_margin_m
        self.min_lateral_force_n = min_lateral_force_n
        self.sign_loss_weight = sign_loss_weight
        self.odd_loss_weight = odd_loss_weight
        self.epsilon = epsilon

    def _safe_trail(self, fy: torch.Tensor, mz: torch.Tensor) -> torch.Tensor:
        """Estimate pneumatic trail without exploding when the model predicts tiny Fy."""
        fy_abs = torch.abs(fy)
        fy_floor = torch.clamp(fy_abs, min=self.min_lateral_force_n)
        return -mz * torch.sign(fy) / (fy_floor + self.epsilon)

    @staticmethod
    def _runtime_projected_lateral_outputs(
        model: nn.Module,
        x: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Match the runtime pure-lateral odd projection for Fy and Mz."""
        mirror = x.clone()
        mirror[:, 1] = -mirror[:, 1]

        pred = model(x)
        pred_mirror = model(mirror)
        fy_projected = 0.5 * (pred[:, 1] - pred_mirror[:, 1])
        mz_projected = 0.5 * (pred[:, 3] - pred_mirror[:, 3])
        return fy_projected, mz_projected, pred[:, 3], pred_mirror[:, 3]

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute pneumatic-trail loss."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "mz_trail_loss": 0.0,
                "mz_negative_trail_rate": 0.0,
                "mz_trail_growth_rate": 0.0,
                "mz_trail_cap_rate": 0.0,
            }

        sign = torch.where(colloc_batch[:, 1] >= 0.0, 1.0, -1.0)
        x_small = colloc_batch.clone()
        x_large = colloc_batch.clone()
        x_small[:, 0] = 0.0
        x_large[:, 0] = 0.0
        x_small[:, 1] = sign * self.small_sa_deg
        x_large[:, 1] = sign * self.large_sa_deg

        fy_small, mz_small, _, _ = self._runtime_projected_lateral_outputs(model, x_small)
        fy_large, mz_large, _, _ = self._runtime_projected_lateral_outputs(model, x_large)

        trail_small = self._safe_trail(fy_small, mz_small)
        trail_large = self._safe_trail(fy_large, mz_large)

        negative_trail = F.relu(-trail_small) + F.relu(-trail_large)
        min_trail_violation = F.relu(self.min_trail_m - trail_small) + F.relu(
            self.min_trail_m - trail_large
        )
        trail_cap = F.relu(trail_small - self.max_trail_m) + F.relu(trail_large - self.max_trail_m)
        trail_growth = F.relu(trail_large - trail_small + self.trail_margin_m)

        x_shape = colloc_batch.clone()
        x_shape[:, 0] = 0.0
        x_shape[:, 1] = sign * torch.clamp(torch.abs(x_shape[:, 1]), min=self.min_abs_sa_deg)
        x_shape[:, 3] = 0.0

        fy_shape, mz_shape, mz_raw, mz_mirror_raw = self._runtime_projected_lateral_outputs(
            model, x_shape
        )

        trail_shape = self._safe_trail(fy_shape, mz_shape)
        sign_violation = F.relu(self.min_trail_m - trail_shape)

        mz_scale = torch.maximum(
            torch.maximum(torch.abs(mz_raw), torch.abs(mz_mirror_raw)),
            torch.full_like(mz_shape, 1.0),
        )
        mz_oddness = (mz_raw + mz_mirror_raw) / mz_scale

        trail_scale = max(self.max_trail_m, self.trail_margin_m, self.epsilon)
        loss = torch.mean(
            (min_trail_violation / trail_scale) ** 2
            + (trail_cap / trail_scale) ** 2
            + (trail_growth / trail_scale) ** 2
            + self.sign_loss_weight * (sign_violation / trail_scale) ** 2
            + self.odd_loss_weight * mz_oddness**2
        )

        metrics = {
            "mz_trail_loss": loss.item(),
            "mz_negative_trail_rate": (negative_trail > 0).float().mean().item(),
            "mz_min_trail_violation_rate": (min_trail_violation > 0).float().mean().item(),
            "mz_trail_growth_rate": (trail_growth > 0).float().mean().item(),
            "mz_trail_cap_rate": (trail_cap > 0).float().mean().item(),
            "mz_shape_negative_trail_rate": (sign_violation > 0).float().mean().item(),
            "mz_oddness_mean": mz_oddness.abs().mean().item(),
        }

        return loss, metrics


class CamberThrustLoss(nn.Module):
    """
    Constrain camber thrust at zero slip.

    This avoids hard-coding the sign convention of IA. It only enforces odd
    symmetry around IA=0 and caps camber-generated lateral force.
    """

    def __init__(self, ia_deg: float = 3.0, camber_mu_cap: float = 0.25, epsilon: float = 1e-6):
        super().__init__()
        self.ia_deg = ia_deg
        self.camber_mu_cap = camber_mu_cap
        self.epsilon = epsilon

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute camber thrust odd-symmetry and cap loss."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "camber_thrust_loss": 0.0,
                "camber_asymmetry_mean": 0.0,
                "camber_cap_violation_rate": 0.0,
            }

        x_pos = colloc_batch.clone()
        x_neg = colloc_batch.clone()
        x_pos[:, 0] = 0.0
        x_neg[:, 0] = 0.0
        x_pos[:, 1] = 0.0
        x_neg[:, 1] = 0.0
        x_pos[:, 3] = self.ia_deg
        x_neg[:, 3] = -self.ia_deg

        fy_pos = model(x_pos)[:, 1]
        fy_neg = model(x_neg)[:, 1]

        fz = torch.abs(colloc_batch[:, 2])
        force_scale = self.camber_mu_cap * fz + self.epsilon
        asymmetry = (fy_pos + fy_neg) / force_scale
        cap_excess = F.relu(torch.maximum(torch.abs(fy_pos), torch.abs(fy_neg)) / force_scale - 1.0)

        loss = torch.mean(asymmetry**2 + cap_excess**2)

        metrics = {
            "camber_thrust_loss": loss.item(),
            "camber_asymmetry_mean": torch.mean(torch.abs(asymmetry)).item(),
            "camber_cap_violation_rate": (cap_excess > 0).float().mean().item(),
        }

        return loss, metrics


class SectionSmoothnessLoss(nn.Module):
    """
    Prefer smooth pure-slip sections without prescribing an exact tire formula.

    Fx(SR), Fy(SA), and Mz(SA) tire sections should look like smooth saturation
    curves, not piecewise patches stitched together by local gates. This loss
    penalizes only large second finite differences, with a deadband so normal
    tanh/Magic-Formula-like curvature is not flattened away.
    """

    def __init__(
        self,
        sr_delta: float = 0.015,
        sa_delta_deg: float = 0.75,
        mu_cap: float = 2.6,
        mz_scale_m: float = 0.06,
        force_deadband: float = 0.015,
        mz_deadband: float = 0.030,
        mz_weight: float = 0.5,
        slope_sign_weight: float = 0.0,
        epsilon: float = 1e-6,
    ):
        super().__init__()
        self.sr_delta = sr_delta
        self.sa_delta_deg = sa_delta_deg
        self.mu_cap = mu_cap
        self.mz_scale_m = mz_scale_m
        self.force_deadband = force_deadband
        self.mz_deadband = mz_deadband
        self.mz_weight = mz_weight
        self.slope_sign_weight = slope_sign_weight
        self.epsilon = epsilon

    @staticmethod
    def _second_difference(
        model: nn.Module,
        base: torch.Tensor,
        input_index: int,
        delta: float,
        output_index: int,
    ) -> torch.Tensor:
        x_minus = base.clone()
        x_plus = base.clone()
        x_minus[:, input_index] -= delta
        x_plus[:, input_index] += delta

        y_minus = model(x_minus)[:, output_index]
        y_mid = model(base)[:, output_index]
        y_plus = model(x_plus)[:, output_index]
        return y_plus - 2.0 * y_mid + y_minus

    def forward(
        self, model: nn.Module, colloc_batch: torch.Tensor
    ) -> Tuple[torch.Tensor, Dict[str, float]]:
        """Compute deadbanded pure-section curvature loss."""
        if colloc_batch.numel() == 0:
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "section_smoothness_loss": 0.0,
                "smoothness_fx_violation_rate": 0.0,
                "smoothness_fy_violation_rate": 0.0,
                "smoothness_mz_violation_rate": 0.0,
                "smoothness_fx_slope_sign_rate": 0.0,
                "smoothness_fy_slope_sign_rate": 0.0,
            }

        base_long = colloc_batch.clone()
        base_long[:, 1] = 0.0
        base_long[:, 3] = 0.0

        base_lat = colloc_batch.clone()
        base_lat[:, 0] = 0.0
        base_lat[:, 3] = 0.0

        x_long_minus = base_long.clone()
        x_long_plus = base_long.clone()
        x_long_minus[:, 0] -= self.sr_delta
        x_long_plus[:, 0] += self.sr_delta
        y_long_minus = model(x_long_minus)[:, 0]
        y_long_mid = model(base_long)[:, 0]
        y_long_plus = model(x_long_plus)[:, 0]
        fx_second = y_long_plus - 2.0 * y_long_mid + y_long_minus
        fx_slope_delta = y_long_plus - y_long_minus

        x_lat_minus = base_lat.clone()
        x_lat_plus = base_lat.clone()
        x_lat_minus[:, 1] -= self.sa_delta_deg
        x_lat_plus[:, 1] += self.sa_delta_deg
        y_lat_minus = model(x_lat_minus)
        y_lat_mid = model(base_lat)
        y_lat_plus = model(x_lat_plus)
        fy_second = y_lat_plus[:, 1] - 2.0 * y_lat_mid[:, 1] + y_lat_minus[:, 1]
        mz_second = y_lat_plus[:, 3] - 2.0 * y_lat_mid[:, 3] + y_lat_minus[:, 3]
        fy_slope_delta = y_lat_plus[:, 1] - y_lat_minus[:, 1]

        fz = torch.abs(colloc_batch[:, 2])
        force_scale = self.mu_cap * fz + self.epsilon
        mz_scale = torch.clamp(self.mz_scale_m * fz, min=1.0)

        fx_curv = torch.abs(fx_second) / force_scale
        fy_curv = torch.abs(fy_second) / force_scale
        mz_curv = torch.abs(mz_second) / mz_scale

        fx_excess = F.relu(fx_curv - self.force_deadband)
        fy_excess = F.relu(fy_curv - self.force_deadband)
        mz_excess = F.relu(mz_curv - self.mz_deadband)
        fx_slope_sign = F.relu(-fx_slope_delta / force_scale)
        fy_slope_sign = F.relu(fy_slope_delta / force_scale)

        loss = torch.mean(
            fx_excess**2
            + fy_excess**2
            + self.mz_weight * mz_excess**2
            + self.slope_sign_weight * (fx_slope_sign**2 + fy_slope_sign**2)
        )

        metrics = {
            "section_smoothness_loss": loss.item(),
            "smoothness_fx_curvature_mean": fx_curv.mean().item(),
            "smoothness_fy_curvature_mean": fy_curv.mean().item(),
            "smoothness_mz_curvature_mean": mz_curv.mean().item(),
            "smoothness_fx_violation_rate": (fx_excess > 0).float().mean().item(),
            "smoothness_fy_violation_rate": (fy_excess > 0).float().mean().item(),
            "smoothness_mz_violation_rate": (mz_excess > 0).float().mean().item(),
            "smoothness_fx_slope_sign_rate": (fx_slope_sign > 0).float().mean().item(),
            "smoothness_fy_slope_sign_rate": (fy_slope_sign > 0).float().mean().item(),
        }
        return loss, metrics


class MonotonicityLoss(nn.Module):
    """
    Enforce increasing force magnitude with vertical load.

    Addresses C6: Non-physical load sensitivity.
    Warning: Computationally expensive (autograd), use sparingly.
    """

    def __init__(self, epsilon: float = 1e-6):
        """
        Args:
            epsilon: Smoothing term for numerical stability
        """
        super().__init__()
        self.epsilon = epsilon

    def forward(
        self,
        model: nn.Module,
        colloc_batch: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            model: PINN model returning raw physical outputs
            colloc_batch: (N, 5) raw physical inputs with requires_grad=True

        Returns:
            loss: Mean negative gradient penalty
            metrics: dict with monotonicity violations
        """
        # Ensure gradient tracking
        x = colloc_batch.requires_grad_(True)
        pred = model(x)

        fx_mag = torch.sqrt(pred[:, 0] ** 2 + self.epsilon)
        fy_mag = torch.sqrt(pred[:, 1] ** 2 + self.epsilon)

        # Compute ∂|Fx|/∂Fz and ∂|Fy|/∂Fz. Magnitude monotonicity is the physically
        # correct criterion because braking/cornering forces can be negative.
        grad_fx = torch.autograd.grad(
            outputs=fx_mag.sum(),
            inputs=x,
            create_graph=True,
            retain_graph=True,
        )[0][:, 2]

        grad_fy = torch.autograd.grad(
            outputs=fy_mag.sum(),
            inputs=x,
            create_graph=True,
            retain_graph=True,
        )[0][:, 2]

        # Penalize negative gradients
        loss_fx = torch.relu(-grad_fx).mean()
        loss_fy = torch.relu(-grad_fy).mean()
        loss = loss_fx + loss_fy

        # Metrics
        violation_fx = (grad_fx < 0).float().mean().item()
        violation_fy = (grad_fy < 0).float().mean().item()

        metrics = {
            "monotonicity_loss": loss.item(),
            "monotonicity_violation_fx": violation_fx,
            "monotonicity_violation_fy": violation_fy,
        }

        return loss, metrics


class FySaZeroLoss(nn.Module):
    """
    Enforce Fy ≈ 0 when SA = IA = 0 for any SR value.

    Addresses Fy leakage at SA=0 during DriveBrake conditions: a symmetric tire
    at zero slip angle and zero camber must not generate lateral force regardless
    of longitudinal slip. Camber thrust is handled by CamberThrustLoss.
    Collocation points have SA=IA=0 with random SR / Fz / TSTC.
    """

    def __init__(self, force_threshold: float = 20.0):
        """
        Args:
            force_threshold: Acceptable |Fy| at SA=0 in Newtons.
        """
        super().__init__()
        self.threshold = force_threshold

    def forward(
        self,
        model: nn.Module,
        colloc_batch: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """
        Args:
            model: PINN model
            colloc_batch: (N, 5) raw physical inputs with SA=IA=0
                          [SR, 0, Fz, 0, TSTC]
        """
        # Camber thrust generates Fy even if SA=0. Restrict to IA=0.
        ia = colloc_batch[:, 3]
        mask = torch.abs(ia) < 1e-6
        if not mask.any():
            zero = colloc_batch.sum() * 0.0
            return zero, {
                "fy_sa_zero_loss": 0.0,
                "fy_sa_zero_max_fy": 0.0,
                "fy_sa_zero_mean_fy": 0.0,
            }

        colloc_filtered = colloc_batch[mask]
        pred = model(colloc_filtered)  # (N_filtered, 4) raw
        fy = pred[:, 1]

        scale = max(self.threshold, 1e-6)
        loss = ((fy / scale) ** 2).mean()

        metrics = {
            "fy_sa_zero_loss": loss.item(),
            "fy_sa_zero_max_fy": fy.abs().max().item(),
            "fy_sa_zero_mean_fy": fy.abs().mean().item(),
        }
        return loss, metrics


class PhysicsLossManager:
    """
    Unified interface for all physics losses.
    Handles weighting and curriculum scheduling.
    """

    def __init__(
        self,
        zero_slip_loss: ZeroSlipLoss,
        symmetry_loss: SymmetryLoss,
        friction_loss: FrictionCircleLoss,
        monotonicity_loss: Optional[MonotonicityLoss] = None,
        fy_sa_zero_loss: Optional[FySaZeroLoss] = None,
        combined_slip_loss: Optional[CombinedSlipLoss] = None,
        stiffness_load_loss: Optional[StiffnessLoadSensitivityLoss] = None,
        high_sa_lateral_load_loss: Optional[HighSaLateralLoadSensitivityLoss] = None,
        mz_trail_loss: Optional[MzPneumaticTrailLoss] = None,
        camber_thrust_loss: Optional[CamberThrustLoss] = None,
        section_smoothness_loss: Optional[SectionSmoothnessLoss] = None,
    ):
        self.zero_slip = zero_slip_loss
        self.symmetry = symmetry_loss
        self.friction = friction_loss
        self.monotonicity = monotonicity_loss
        self.fy_sa_zero = fy_sa_zero_loss
        self.combined_slip = combined_slip_loss
        self.stiffness_load = stiffness_load_loss
        self.high_sa_lateral_load = high_sa_lateral_load_loss
        self.mz_trail = mz_trail_loss
        self.camber_thrust = camber_thrust_loss
        self.section_smoothness = section_smoothness_loss

    def compute_all(
        self,
        model: nn.Module,
        colloc_zero: torch.Tensor,
        colloc_sym: torch.Tensor,
        colloc_fric: torch.Tensor,
        colloc_mono: Optional[torch.Tensor] = None,
        colloc_fy_sa_zero: Optional[torch.Tensor] = None,
        colloc_combined: Optional[torch.Tensor] = None,
        colloc_stiffness: Optional[torch.Tensor] = None,
        colloc_high_sa_lateral_load: Optional[torch.Tensor] = None,
        colloc_mz_trail: Optional[torch.Tensor] = None,
        colloc_camber: Optional[torch.Tensor] = None,
        colloc_smoothness: Optional[torch.Tensor] = None,
        weights: dict = None,
    ) -> tuple[torch.Tensor, dict]:
        """
        Compute all physics losses with given weights.

        Args:
            model: PINN model
            colloc_*: Collocation point batches
            weights: Physics loss weights keyed by curriculum names.

        Returns:
            total_loss: Weighted sum of physics losses
            metrics: dict with all individual losses
        """
        if weights is None:
            weights = {
                "zero": 1.0,
                "sym": 1.0,
                "fric": 1.0,
                "mono": 1.0,
                "fy_sa_zero": 1.0,
                "combined": 1.0,
                "stiffness": 1.0,
                "high_sa_lateral": 1.0,
                "mz_trail": 1.0,
                "camber": 1.0,
                "smoothness": 1.0,
            }

        metrics = {}

        # Zero slip
        loss_zero, m_zero = self.zero_slip(model, colloc_zero)
        metrics.update(m_zero)

        # Symmetry
        loss_sym, m_sym = self.symmetry(model, colloc_sym)
        metrics.update(m_sym)

        # Friction circle
        loss_fric, m_fric = self.friction(model, colloc_fric)
        metrics.update(m_fric)

        # Total physics loss
        total_loss = (
            weights.get("zero", 0.0) * loss_zero
            + weights.get("sym", 0.0) * loss_sym
            + weights.get("fric", 0.0) * loss_fric
        )

        # Monotonicity (optional, expensive)
        if self.monotonicity is not None and colloc_mono is not None:
            loss_mono, m_mono = self.monotonicity(model, colloc_mono)
            metrics.update(m_mono)
            total_loss += weights.get("mono", 0.0) * loss_mono

        # Fy at SA=0 leakage constraint
        if self.fy_sa_zero is not None and colloc_fy_sa_zero is not None:
            w = weights.get("fy_sa_zero", 0.0)
            if w > 0.0:
                loss_fy_sa_zero, m_fy_sa_zero = self.fy_sa_zero(model, colloc_fy_sa_zero)
                metrics.update(m_fy_sa_zero)
                total_loss += w * loss_fy_sa_zero
            else:
                metrics.update(
                    {"fy_sa_zero_loss": 0.0, "fy_sa_zero_max_fy": 0.0, "fy_sa_zero_mean_fy": 0.0}
                )

        if self.combined_slip is not None and colloc_combined is not None:
            w = weights.get("combined", 0.0)
            if w > 0.0:
                loss_combined, m_combined = self.combined_slip(model, colloc_combined)
                metrics.update(m_combined)
                total_loss += w * loss_combined

        if self.stiffness_load is not None and colloc_stiffness is not None:
            w = weights.get("stiffness", 0.0)
            if w > 0.0:
                loss_stiffness, m_stiffness = self.stiffness_load(model, colloc_stiffness)
                metrics.update(m_stiffness)
                total_loss += w * loss_stiffness

        if self.high_sa_lateral_load is not None and colloc_high_sa_lateral_load is not None:
            w = weights.get("high_sa_lateral", 0.0)
            if w > 0.0:
                loss_high_sa_lateral, m_high_sa_lateral = self.high_sa_lateral_load(
                    model, colloc_high_sa_lateral_load
                )
                metrics.update(m_high_sa_lateral)
                total_loss += w * loss_high_sa_lateral

        if self.mz_trail is not None and colloc_mz_trail is not None:
            w = weights.get("mz_trail", 0.0)
            if w > 0.0:
                loss_mz_trail, m_mz_trail = self.mz_trail(model, colloc_mz_trail)
                metrics.update(m_mz_trail)
                total_loss += w * loss_mz_trail

        if self.camber_thrust is not None and colloc_camber is not None:
            w = weights.get("camber", 0.0)
            if w > 0.0:
                loss_camber, m_camber = self.camber_thrust(model, colloc_camber)
                metrics.update(m_camber)
                total_loss += w * loss_camber

        if self.section_smoothness is not None and colloc_smoothness is not None:
            w = weights.get("smoothness", 0.0)
            if w > 0.0:
                loss_smoothness, m_smoothness = self.section_smoothness(
                    model, colloc_smoothness
                )
                metrics.update(m_smoothness)
                total_loss += w * loss_smoothness

        metrics["physics_loss_total"] = total_loss.item()

        return total_loss, metrics
