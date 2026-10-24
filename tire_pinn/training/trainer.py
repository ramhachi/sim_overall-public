"""
PINN training loop with curriculum learning.
"""

import torch
import torch.optim as optim
from torch.utils.data import DataLoader
from pathlib import Path
from typing import Dict, Optional
import json
from tqdm import tqdm

from ..models.pinn_model import NormalizedTirePINN
from ..losses.data_loss import DataLoss, R2Score
from ..losses.physics_losses import (
    ZeroSlipLoss,
    SymmetryLoss,
    FrictionCircleLoss,
    CombinedSlipLoss,
    StiffnessLoadSensitivityLoss,
    HighSaLateralLoadSensitivityLoss,
    MzPneumaticTrailLoss,
    CamberThrustLoss,
    SectionSmoothnessLoss,
    MonotonicityLoss,
    FySaZeroLoss,
    PhysicsLossManager,
)
from .collocation_sampler import CollocationSampler
from .curriculum_scheduler import CurriculumScheduler


class PINNTrainer:
    """
    Main training loop for PINN tire model.

    Implements:
    - Curriculum learning (4 phases)
    - Physics constraint integration
    - Early stopping
    - Checkpointing
    """

    def __init__(
        self,
        model: NormalizedTirePINN,
        train_loader: DataLoader,
        val_loader: DataLoader,
        collocation_sampler: CollocationSampler,
        device: str = "cuda" if torch.cuda.is_available() else "cpu",
        learning_rate: float = 0.002,
        weight_decay: float = 1e-5,
        output_dir: Path = Path("output/pinn"),
        curriculum_total_epochs: int = 260,
        curriculum_final_weights: Optional[Dict[str, float]] = None,
        data_loss_config: Optional[Dict[str, object]] = None,
        physics_config: Optional[Dict[str, object]] = None,
        gradient_clip_norm: float = 1.0,
    ):
        self.model = model.to(device)
        self.device = device
        self.train_loader = train_loader
        self.val_loader = val_loader
        self.collocation_sampler = collocation_sampler
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.gradient_clip_norm = gradient_clip_norm
        self.data_loss_config = data_loss_config or {}
        self.physics_config = physics_config or {}

        # Optimizer
        self.optimizer = optim.Adam(
            self.model.parameters(),
            lr=learning_rate,
            weight_decay=weight_decay,
        )

        # Learning rate scheduler (cosine annealing)
        self.lr_scheduler = optim.lr_scheduler.CosineAnnealingLR(
            self.optimizer,
            T_max=max(curriculum_total_epochs, 1),
            eta_min=1e-6,
        )

        # Loss functions
        component_cfg = self.data_loss_config.get("component_weights", {})
        component_weights = [
            float(component_cfg.get("fx", 1.0)),
            float(component_cfg.get("fy", 1.0)),
            float(component_cfg.get("mx", 0.5)),
            float(component_cfg.get("mz", 0.5)),
        ]
        self.data_loss = DataLoss(component_weights=component_weights).to(device)
        self.local_stiffness_targets = self._build_local_stiffness_targets()
        self.mz_shape_targets = self._build_mz_shape_targets()

        # Physics losses
        zero_slip_cfg = self.physics_config.get("zero_slip", {})
        friction_cfg = self.physics_config.get("friction_circle", {})
        monotonicity_cfg = self.physics_config.get("monotonicity", {})
        combined_cfg = self.physics_config.get("combined_slip", {})
        stiffness_cfg = self.physics_config.get("stiffness_load", {})
        fy_sa_zero_cfg = self.physics_config.get("fy_sa_zero", {})
        high_sa_cfg = self.physics_config.get("high_sa_lateral_load", {})
        friction_loss = FrictionCircleLoss(
            mu_cap=float(friction_cfg.get("mu_cap", 2.6)),
        ).to(device)
        mz_trail_cfg = self.physics_config.get("mz_trail", {})
        smoothness_cfg = self.physics_config.get("section_smoothness", {})
        self.physics_manager = PhysicsLossManager(
            zero_slip_loss=ZeroSlipLoss(
                force_threshold=float(
                    zero_slip_cfg.get("force_threshold_n", zero_slip_cfg.get("force_threshold", 2.0))
                ),
                moment_threshold=float(
                    zero_slip_cfg.get("moment_threshold_nm", zero_slip_cfg.get("moment_threshold", 5.0))
                ),
            ).to(device),
            symmetry_loss=SymmetryLoss().to(device),
            friction_loss=friction_loss,
            monotonicity_loss=MonotonicityLoss(
                epsilon=float(monotonicity_cfg.get("epsilon", 1e-6)),
            ).to(device),
            fy_sa_zero_loss=FySaZeroLoss(
                force_threshold=float(fy_sa_zero_cfg.get("force_threshold", 20.0)),
            ).to(device),
            combined_slip_loss=CombinedSlipLoss(
                mu_cap=float(combined_cfg.get("mu_cap", 2.6)),
                margin=float(combined_cfg.get("margin", 0.03)),
            ).to(device),
            stiffness_load_loss=StiffnessLoadSensitivityLoss(
                sr_delta=float(stiffness_cfg.get("sr_delta", 0.01)),
                sa_delta_deg=float(stiffness_cfg.get("sa_delta_deg", 0.25)),
            ).to(device),
            high_sa_lateral_load_loss=HighSaLateralLoadSensitivityLoss(
                min_abs_sa_deg=float(high_sa_cfg.get("min_abs_sa_deg", 6.0)),
                drop_margin=float(high_sa_cfg.get("drop_margin", 0.01)),
                mu_cap=float(high_sa_cfg.get("mu_cap", 2.6)),
            ).to(device),
            mz_trail_loss=MzPneumaticTrailLoss(
                small_sa_deg=float(mz_trail_cfg.get("small_sa_deg", 1.5)),
                large_sa_deg=float(mz_trail_cfg.get("large_sa_deg", 6.0)),
                min_abs_sa_deg=float(mz_trail_cfg.get("min_abs_sa_deg", 1.0)),
                max_trail_m=float(mz_trail_cfg.get("max_trail_m", 0.18)),
                min_trail_m=float(mz_trail_cfg.get("min_trail_m", 0.0)),
                trail_margin_m=float(mz_trail_cfg.get("trail_margin_m", 0.01)),
                min_lateral_force_n=float(mz_trail_cfg.get("min_lateral_force_n", 75.0)),
                sign_loss_weight=float(mz_trail_cfg.get("sign_loss_weight", 1.0)),
                odd_loss_weight=float(mz_trail_cfg.get("odd_loss_weight", 0.25)),
            ).to(device),
            camber_thrust_loss=CamberThrustLoss().to(device),
            section_smoothness_loss=SectionSmoothnessLoss(
                sr_delta=float(smoothness_cfg.get("sr_delta", 0.015)),
                sa_delta_deg=float(smoothness_cfg.get("sa_delta_deg", 0.75)),
                mu_cap=float(smoothness_cfg.get("mu_cap", 2.6)),
                mz_scale_m=float(smoothness_cfg.get("mz_scale_m", 0.06)),
                force_deadband=float(smoothness_cfg.get("force_deadband", 0.015)),
                mz_deadband=float(smoothness_cfg.get("mz_deadband", 0.030)),
                mz_weight=float(smoothness_cfg.get("mz_weight", 0.5)),
                slope_sign_weight=float(smoothness_cfg.get("slope_sign_weight", 0.0)),
            ).to(device),
        )

        self.collocation_counts = {
            "zero": int(zero_slip_cfg.get("n_colloc_points", 128)),
            "sym": int(self.physics_config.get("symmetry", {}).get("n_colloc_points", 256)),
            "fric": int(friction_cfg.get("n_colloc_points", 128)),
            "mono": int(monotonicity_cfg.get("n_colloc_points", 64)),
            "fy_sa_zero": int(fy_sa_zero_cfg.get("n_colloc_points", 128)),
            "combined": int(combined_cfg.get("n_colloc_points", 128)),
            "stiffness": int(stiffness_cfg.get("n_colloc_points", 64)),
            "high_sa_lateral": int(high_sa_cfg.get("n_colloc_points", 64)),
            "mz_trail": int(mz_trail_cfg.get("n_colloc_points", 128)),
            "camber": int(self.physics_config.get("camber_thrust", {}).get("n_colloc_points", 128)),
            "smoothness": int(smoothness_cfg.get("n_colloc_points", 128)),
        }
        print("Physics collocation counts:", self.collocation_counts)
        print(
            "Physics thresholds:",
            {
                "zero_force_n": float(
                    zero_slip_cfg.get("force_threshold_n", zero_slip_cfg.get("force_threshold", 2.0))
                ),
                "zero_moment_nm": float(
                    zero_slip_cfg.get("moment_threshold_nm", zero_slip_cfg.get("moment_threshold", 5.0))
                ),
                "friction_mu_cap": float(friction_cfg.get("mu_cap", 2.6)),
            },
        )

        # Curriculum scheduler
        self.curriculum = CurriculumScheduler(
            total_epochs=curriculum_total_epochs,
            final_weights=curriculum_final_weights,
        )

    def set_normalization_params_for_physics(self, data_loader):
        """Set normalization parameters for FrictionCircleLoss after data loading."""
        norm_params = data_loader.get_normalization_params()
        # Fz is index 2 in input, forces are indices 0,1 in output
        fz_mean = norm_params["input_mean"][2].item()
        fz_std = norm_params["input_std"][2].item()
        fx_mean = norm_params["output_mean"][0].item()
        fx_std = norm_params["output_std"][0].item()
        fy_mean = norm_params["output_mean"][1].item()
        fy_std = norm_params["output_std"][1].item()

        self.physics_manager.friction.set_normalization_params(
            fz_mean=fz_mean,
            fz_std=fz_std,
            fx_mean=fx_mean,
            fx_std=fx_std,
            fy_mean=fy_mean,
            fy_std=fy_std,
        )

        # Training state
        self.epoch = 0
        self.best_val_loss = float("inf")
        self.best_r2_mean = -float("inf")
        self.patience_counter = 0
        self.history = {
            "train_loss": [],
            "val_loss": [],
            "val_r2_fx": [],
            "val_r2_fy": [],
            "physics_metrics": [],
        }
        self.best_monitor = "val_r2_mean"
        self.best_monitor_value = -float("inf")

    @staticmethod
    def _unpack_batch(batch):
        if len(batch) == 3:
            return batch[0], batch[1], batch[2]
        if len(batch) == 2:
            return batch[0], batch[1], None
        raise ValueError(f"Unexpected batch structure with {len(batch)} tensors")

    @staticmethod
    def _fit_slope(x: torch.Tensor, y: torch.Tensor) -> tuple[float, int]:
        if x.numel() < 20:
            return 0.0, int(x.numel())
        x64 = x.detach().cpu().double()
        y64 = y.detach().cpu().double()
        x_centered = x64 - x64.mean()
        denom = torch.sum(x_centered**2)
        if denom <= 0.0:
            return 0.0, int(x.numel())
        slope = torch.sum(x_centered * (y64 - y64.mean())) / denom
        return float(slope), int(x.numel())

    def _build_local_stiffness_targets(self) -> Dict[str, list[dict]]:
        cfg = self.data_loss_config.get("local_stiffness_targets", {})
        if not cfg or not bool(cfg.get("enabled", False)):
            return {"fx": [], "fy": []}
        if not hasattr(self.train_loader.dataset, "tensors"):
            return {"fx": [], "fy": []}

        x_train, y_train, component_mask = self.train_loader.dataset.tensors
        fz_targets = [float(v) for v in cfg.get("fz_targets", [200, 650, 950, 1150])]
        fz_tolerance = float(cfg.get("fz_tolerance", 90.0))
        min_samples = int(cfg.get("min_samples", 30))
        targets: Dict[str, list[dict]] = {"fx": [], "fy": []}

        fx_cfg = cfg.get("fx", {})
        if bool(fx_cfg.get("enabled", True)):
            sr_abs = float(fx_cfg.get("sr_abs", 0.02))
            sa_abs = float(fx_cfg.get("sa_abs", 0.75))
            for fz in fz_targets:
                mask = (
                    (component_mask[:, 0] > 0.0)
                    & (torch.abs(x_train[:, 2] - fz) <= fz_tolerance)
                    & (torch.abs(x_train[:, 0]) <= sr_abs)
                    & (torch.abs(x_train[:, 1]) <= sa_abs)
                )
                slope, n_samples = self._fit_slope(x_train[mask, 0], y_train[mask, 0])
                if n_samples >= min_samples and abs(slope) > 1.0:
                    temp = float(x_train[mask, 4].median().item())
                    targets["fx"].append(
                        {"fz": fz, "temp": temp, "target_slope": slope, "n_samples": n_samples}
                    )

        fy_cfg = cfg.get("fy", {})
        if bool(fy_cfg.get("enabled", True)):
            sa_abs = float(fy_cfg.get("sa_abs", 1.0))
            sr_abs = float(fy_cfg.get("sr_abs", 0.03))
            for fz in fz_targets:
                mask = (
                    (component_mask[:, 1] > 0.0)
                    & (torch.abs(x_train[:, 2] - fz) <= fz_tolerance)
                    & (torch.abs(x_train[:, 1]) <= sa_abs)
                    & (torch.abs(x_train[:, 0]) <= sr_abs)
                )
                slope, n_samples = self._fit_slope(x_train[mask, 1], y_train[mask, 1])
                if n_samples >= min_samples and abs(slope) > 1.0:
                    temp = float(x_train[mask, 4].median().item())
                    targets["fy"].append(
                        {"fz": fz, "temp": temp, "target_slope": slope, "n_samples": n_samples}
                    )

        if targets["fx"] or targets["fy"]:
            print("\nLocal stiffness targets from training data:")
            for component, rows in targets.items():
                for row in rows:
                    unit = "N/SR" if component == "fx" else "N/deg"
                    print(
                        f"  {component.upper()} Fz={row['fz']:.0f} N: "
                        f"{row['target_slope']:.1f} {unit} (n={row['n_samples']})"
                    )
        return targets

    def _build_mz_shape_targets(self) -> list[dict]:
        cfg = self.data_loss_config.get("mz_shape_targets", {})
        if not cfg or not bool(cfg.get("enabled", False)):
            return []
        if not hasattr(self.train_loader.dataset, "tensors"):
            return []

        x_train, y_train, component_mask = self.train_loader.dataset.tensors
        fz_targets = [float(v) for v in cfg.get("fz_targets", [200, 650, 950, 1150])]
        fz_tolerance = float(cfg.get("fz_tolerance", 90.0))
        sr_abs = float(cfg.get("sr_abs", 0.03))
        ia_abs = float(cfg.get("ia_abs", 1.0))
        min_samples = int(cfg.get("min_samples", 25))
        low_fz_max = float(cfg.get("low_fz_max", 300.0))
        low_fz_weight = float(cfg.get("low_fz_weight", 1.0))
        raw_temp_bins = cfg.get("temperature_bins", [[None, None]])
        temp_bins = [
            (
                None if temp_lo is None else float(temp_lo),
                None if temp_hi is None else float(temp_hi),
            )
            for temp_lo, temp_hi in raw_temp_bins
        ]
        default_bins = [
            [-12.0, -9.0],
            [-9.0, -6.0],
            [-6.0, -3.0],
            [-3.0, -1.0],
            [1.0, 3.0],
            [3.0, 6.0],
            [6.0, 9.0],
            [9.0, 12.0],
        ]
        sa_bins = [(float(lo), float(hi)) for lo, hi in cfg.get("sa_bins", default_bins)]
        targets: list[dict] = []

        for fz in fz_targets:
            for sa_lo, sa_hi in sa_bins:
                base_mask = (
                    (component_mask[:, 3] > 0.0)
                    & (torch.abs(x_train[:, 2] - fz) <= fz_tolerance)
                    & (torch.abs(x_train[:, 0]) <= sr_abs)
                    & (torch.abs(x_train[:, 3]) <= ia_abs)
                    & (x_train[:, 1] >= sa_lo)
                    & (x_train[:, 1] < sa_hi)
                )
                for temp_lo, temp_hi in temp_bins:
                    temp_mask = torch.ones_like(base_mask, dtype=torch.bool)
                    if temp_lo is not None:
                        temp_mask = temp_mask & (x_train[:, 4] >= temp_lo)
                    if temp_hi is not None:
                        temp_mask = temp_mask & (x_train[:, 4] < temp_hi)
                    mask = base_mask & temp_mask
                    n_samples = int(mask.sum().item())
                    if n_samples < min_samples:
                        continue

                    weight = float(cfg.get("target_weight", 1.0))
                    if temp_lo is not None or temp_hi is not None:
                        weight *= float(cfg.get("temperature_bin_weight", 1.0))
                    if fz <= low_fz_max:
                        weight *= low_fz_weight
                    targets.append(
                        {
                            "fz": fz,
                            "sa": float(x_train[mask, 1].median().item()),
                            "temp": float(x_train[mask, 4].median().item()),
                            "target_mz": float(y_train[mask, 3].median().item()),
                            "weight": weight,
                            "n_samples": n_samples,
                        }
                    )

        if targets:
            print("\nMz shape targets from training data:")
            for row in targets:
                print(
                    f"  Mz Fz={row['fz']:.0f} N, SA={row['sa']:+.1f} deg: "
                    f"{row['target_mz']:+.2f} Nm (n={row['n_samples']}, w={row['weight']:.2f})"
                )
        return targets

    def _build_sample_component_weight(
        self,
        x_batch: torch.Tensor,
        component_mask: Optional[torch.Tensor],
    ) -> Optional[torch.Tensor]:
        cfg = self.data_loss_config.get("near_zero_weights", {})
        if not cfg:
            return None

        weights = torch.ones((x_batch.shape[0], 4), device=x_batch.device, dtype=x_batch.dtype)
        fx_cfg = cfg.get("fx", {})
        if bool(fx_cfg.get("enabled", False)):
            mask = (torch.abs(x_batch[:, 0]) <= float(fx_cfg.get("sr_abs", 0.03))) & (
                torch.abs(x_batch[:, 1]) <= float(fx_cfg.get("sa_abs", 0.75))
            )
            weights[:, 0] = torch.where(
                mask,
                torch.full_like(weights[:, 0], float(fx_cfg.get("weight", 1.0))),
                weights[:, 0],
            )

        fy_cfg = cfg.get("fy", {})
        if bool(fy_cfg.get("enabled", False)):
            mask = (torch.abs(x_batch[:, 1]) <= float(fy_cfg.get("sa_abs", 1.0))) & (
                torch.abs(x_batch[:, 0]) <= float(fy_cfg.get("sr_abs", 0.03))
            )
            weights[:, 1] = torch.where(
                mask,
                torch.full_like(weights[:, 1], float(fy_cfg.get("weight", 1.0))),
                weights[:, 1],
            )

        mz_cfg = cfg.get("mz", {})
        if bool(mz_cfg.get("enabled", False)):
            sa_abs = torch.abs(x_batch[:, 1])
            mask = (
                (torch.abs(x_batch[:, 0]) <= float(mz_cfg.get("sr_abs", 0.03)))
                & (sa_abs >= float(mz_cfg.get("sa_min_abs", 1.0)))
                & (sa_abs <= float(mz_cfg.get("sa_max_abs", 12.5)))
            )
            mz_weight = torch.full_like(weights[:, 3], float(mz_cfg.get("weight", 1.0)))
            low_fz_max = mz_cfg.get("low_fz_max")
            if low_fz_max is not None:
                low_fz_mask = x_batch[:, 2] <= float(low_fz_max)
                mz_weight = torch.where(
                    low_fz_mask,
                    torch.full_like(
                        mz_weight,
                        float(mz_cfg.get("low_fz_weight", mz_cfg.get("weight", 1.0))),
                    ),
                    mz_weight,
                )
            weights[:, 3] = torch.where(mask, mz_weight, weights[:, 3])

        if component_mask is not None:
            active = component_mask.to(device=x_batch.device, dtype=x_batch.dtype) > 0
            weights = torch.where(active, weights, torch.ones_like(weights))
        return weights

    def _compute_local_stiffness_loss(self) -> tuple[torch.Tensor, Dict[str, float]]:
        cfg = self.data_loss_config.get("local_stiffness_targets", {})
        weight = float(cfg.get("loss_weight", 0.0)) if cfg else 0.0
        if weight <= 0.0 or not (self.local_stiffness_targets["fx"] or self.local_stiffness_targets["fy"]):
            zero = next(self.model.parameters()).sum() * 0.0
            return zero, {"local_stiffness_loss": 0.0}

        sr_delta = float(cfg.get("sr_delta", 0.005))
        sa_delta_deg = float(cfg.get("sa_delta_deg", 0.1))
        losses = []
        metrics: Dict[str, float] = {}

        for row in self.local_stiffness_targets["fx"]:
            base = torch.tensor(
                [[0.0, 0.0, row["fz"], 0.0, row["temp"]]],
                dtype=torch.float32,
                device=self.device,
            )
            x_plus = base.clone()
            x_minus = base.clone()
            x_plus[:, 0] = sr_delta
            x_minus[:, 0] = -sr_delta
            slope = (self.model(x_plus)[:, 0] - self.model(x_minus)[:, 0]) / (2.0 * sr_delta)
            target = torch.tensor(row["target_slope"], dtype=torch.float32, device=self.device)
            losses.append(((slope - target) / target.abs().clamp_min(1.0)) ** 2)

        for row in self.local_stiffness_targets["fy"]:
            base = torch.tensor(
                [[0.0, 0.0, row["fz"], 0.0, row["temp"]]],
                dtype=torch.float32,
                device=self.device,
            )
            x_plus = base.clone()
            x_minus = base.clone()
            x_plus[:, 1] = sa_delta_deg
            x_minus[:, 1] = -sa_delta_deg
            slope = (self.model(x_plus)[:, 1] - self.model(x_minus)[:, 1]) / (2.0 * sa_delta_deg)
            target = torch.tensor(row["target_slope"], dtype=torch.float32, device=self.device)
            losses.append(((slope - target) / target.abs().clamp_min(1.0)) ** 2)

        if not losses:
            zero = next(self.model.parameters()).sum() * 0.0
            return zero, {"local_stiffness_loss": 0.0}
        raw_loss = torch.stack([loss.reshape(()) for loss in losses]).mean()
        metrics["local_stiffness_loss"] = raw_loss.item()
        return weight * raw_loss, metrics

    def _compute_mz_shape_target_loss(self) -> tuple[torch.Tensor, Dict[str, float]]:
        cfg = self.data_loss_config.get("mz_shape_targets", {})
        weight = float(cfg.get("loss_weight", 0.0)) if cfg else 0.0
        if weight <= 0.0 or not self.mz_shape_targets:
            zero = next(self.model.parameters()).sum() * 0.0
            return zero, {"mz_shape_target_loss": 0.0}

        x_rows = [
            [0.0, row["sa"], row["fz"], 0.0, row["temp"]]
            for row in self.mz_shape_targets
        ]
        x_targets = torch.tensor(x_rows, dtype=torch.float32, device=self.device)
        y_targets = torch.tensor(
            [row["target_mz"] for row in self.mz_shape_targets],
            dtype=torch.float32,
            device=self.device,
        )
        target_weights = torch.tensor(
            [row["weight"] for row in self.mz_shape_targets],
            dtype=torch.float32,
            device=self.device,
        )

        pred_mz = self.model(x_targets)[:, 3]
        scale = float(cfg.get("target_scale_nm", 25.0))
        normalized_error = (pred_mz - y_targets) / max(scale, 1.0)
        raw_loss = torch.mean(target_weights * normalized_error**2)
        metrics = {"mz_shape_target_loss": raw_loss.item()}
        return weight * raw_loss, metrics

    def train_epoch(self) -> Dict[str, float]:
        """Train for one epoch."""
        self.model.train()
        self.epoch += 1

        # Get curriculum weights
        weights = self.curriculum.get_weights(self.epoch)
        phase = self.curriculum.get_phase(self.epoch)

        # Metrics accumulator
        metrics = {
            "train_data_loss": 0.0,
            "train_physics_loss": 0.0,
            "train_total_loss": 0.0,
        }
        n_batches = 0

        # Progress bar
        pbar = tqdm(self.train_loader, desc=f"Epoch {self.epoch} [Phase {phase}]")

        for batch_idx, batch in enumerate(pbar):
            x_batch, y_batch, component_mask = self._unpack_batch(batch)
            x_batch = x_batch.to(self.device)
            y_batch = y_batch.to(self.device)
            if component_mask is not None:
                component_mask = component_mask.to(self.device)

            self.optimizer.zero_grad()

            # Forward pass in normalized space (raw input → normalize → PINN → normalized output)
            x_norm = self.model.normalize_input(x_batch)
            y_pred_norm = self.model.forward_normalized(x_norm)

            # Normalize targets to match (raw targets → normalized)
            y_target_norm = (y_batch - self.model.output_mean) / self.model.output_std
            sample_component_weight = self._build_sample_component_weight(x_batch, component_mask)
            loss_data, data_metrics = self.data_loss(
                y_pred_norm,
                y_target_norm,
                component_mask=component_mask,
                sample_component_weight=sample_component_weight,
            )
            total_loss = weights["data"] * loss_data

            loss_local_stiffness, local_stiffness_metrics = self._compute_local_stiffness_loss()
            total_loss += loss_local_stiffness
            loss_mz_shape, mz_shape_metrics = self._compute_mz_shape_target_loss()
            total_loss += loss_mz_shape

            # Physics losses (only in Phase B+)
            if self.epoch > self.curriculum.PHASE_A_END:
                # Sample collocation points
                counts = self.collocation_counts
                colloc = self.collocation_sampler.sample_batch(
                    n_zero=counts["zero"],
                    n_sym=counts["sym"],
                    n_fric=counts["fric"],
                    n_mono=counts["mono"] if weights["mono"] > 0 else 0,
                    n_fy_sa_zero=counts["fy_sa_zero"],
                    n_combined=counts["combined"] if weights.get("combined", 0.0) > 0 else 0,
                    n_stiffness=counts["stiffness"] if weights.get("stiffness", 0.0) > 0 else 0,
                    n_high_sa_lateral=counts["high_sa_lateral"] if weights.get("high_sa_lateral", 0.0) > 0 else 0,
                    n_mz_trail=counts["mz_trail"] if weights.get("mz_trail", 0.0) > 0 else 0,
                    n_camber=counts["camber"] if weights.get("camber", 0.0) > 0 else 0,
                    n_smoothness=counts["smoothness"] if weights.get("smoothness", 0.0) > 0 else 0,
                )

                # Compute physics losses
                physics_weights = {
                    "zero": weights["zero"],
                    "sym": weights["sym"],
                    "fric": weights["fric"],
                    "mono": weights["mono"],
                    "fy_sa_zero": weights.get("fy_sa_zero", 0.0),
                    "combined": weights.get("combined", 0.0),
                    "stiffness": weights.get("stiffness", 0.0),
                    "high_sa_lateral": weights.get("high_sa_lateral", 0.0),
                    "mz_trail": weights.get("mz_trail", 0.0),
                    "camber": weights.get("camber", 0.0),
                    "smoothness": weights.get("smoothness", 0.0),
                }

                loss_physics, physics_metrics = self.physics_manager.compute_all(
                    model=self.model,
                    colloc_zero=colloc["zero"],
                    colloc_sym=colloc["symmetry"],
                    colloc_fric=colloc["friction"],
                    colloc_mono=colloc["monotonicity"] if weights["mono"] > 0 else None,
                    colloc_fy_sa_zero=colloc["sa_zero_fy"],
                    colloc_combined=colloc["combined_slip"],
                    colloc_stiffness=colloc["stiffness"],
                    colloc_high_sa_lateral_load=colloc["high_sa_lateral"],
                    colloc_mz_trail=colloc["mz_trail"],
                    colloc_camber=colloc["camber"],
                    colloc_smoothness=colloc["smoothness"],
                    weights=physics_weights,
                )

                total_loss += loss_physics
                metrics["train_physics_loss"] += loss_physics.item()

            # Backward pass
            total_loss.backward()

            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(
                self.model.parameters(), max_norm=self.gradient_clip_norm
            )

            self.optimizer.step()

            # Update metrics
            metrics["train_data_loss"] += loss_data.item()
            metrics["train_local_stiffness_loss"] = metrics.get("train_local_stiffness_loss", 0.0) + (
                local_stiffness_metrics["local_stiffness_loss"]
            )
            metrics["train_mz_shape_target_loss"] = metrics.get(
                "train_mz_shape_target_loss", 0.0
            ) + mz_shape_metrics["mz_shape_target_loss"]
            metrics["train_total_loss"] += total_loss.item()
            n_batches += 1

            # Update progress bar
            pbar.set_postfix(
                {
                    "loss": f"{total_loss.item():.4f}",
                    "data": f"{loss_data.item():.4f}",
                    "lr": f"{self.optimizer.param_groups[0]['lr']:.2e}",
                }
            )

        # Average metrics
        for key in metrics:
            metrics[key] /= n_batches

        # LR scheduler step
        self.lr_scheduler.step()

        return metrics

    def validate(self) -> Dict[str, float]:
        """Validate on validation set."""
        self.model.eval()

        all_preds = []
        all_targets = []

        all_component_masks = []

        with torch.no_grad():
            for batch in self.val_loader:
                x_batch, y_batch, component_mask = self._unpack_batch(batch)
                x_batch = x_batch.to(self.device)
                y_batch = y_batch.to(self.device)
                if component_mask is not None:
                    component_mask = component_mask.to(self.device)

                # Forward pass: raw input → raw output
                y_pred = self.model(x_batch)

                # y_batch is already raw (DataLoader returns raw data)
                all_preds.append(y_pred)
                all_targets.append(y_batch)
                if component_mask is not None:
                    all_component_masks.append(component_mask)

        # Concatenate
        all_preds = torch.cat(all_preds, dim=0)
        all_targets = torch.cat(all_targets, dim=0)
        all_component_mask = torch.cat(all_component_masks, dim=0) if all_component_masks else None

        # Data loss (both in raw units now)
        loss_data, _ = self.data_loss(all_preds, all_targets, component_mask=all_component_mask)

        # R² scores
        r2_dict = R2Score.compute_dict(all_preds, all_targets, component_mask=all_component_mask)
        r2_mean = (r2_dict["r2_fx"] + r2_dict["r2_fy"]) / 2

        # Physics validation (on separate collocation points), including monotonicity.
        colloc = self.collocation_sampler.sample_batch(
            n_zero=500,
            n_sym=1000,
            n_fric=500,
            n_mono=128,
            n_fy_sa_zero=500,
            n_combined=500,
            n_stiffness=256,
            n_high_sa_lateral=256,
            n_mz_trail=500,
            n_camber=500,
            n_smoothness=500,
        )

        physics_weights = {
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
        _, physics_metrics = self.physics_manager.compute_all(
            model=self.model,
            colloc_zero=colloc["zero"],
            colloc_sym=colloc["symmetry"],
            colloc_fric=colloc["friction"],
            colloc_mono=colloc["monotonicity"],
            colloc_fy_sa_zero=colloc["sa_zero_fy"],
            colloc_combined=colloc["combined_slip"],
            colloc_stiffness=colloc["stiffness"],
            colloc_high_sa_lateral_load=colloc["high_sa_lateral"],
            colloc_mz_trail=colloc["mz_trail"],
            colloc_camber=colloc["camber"],
            colloc_smoothness=colloc["smoothness"],
            weights=physics_weights,
        )

        # Combine metrics
        metrics = {
            "val_data_loss": loss_data.item(),
            "val_r2_mean": r2_mean,
            **{f"val_{k}": v for k, v in r2_dict.items()},
            **{f"val_{k}": v for k, v in physics_metrics.items()},
        }

        return metrics

    def save_checkpoint(self, filename: str = "checkpoint.pt"):
        """Save training checkpoint."""
        checkpoint = {
            "epoch": self.epoch,
            "model_state_dict": self.model.state_dict(),
            "optimizer_state_dict": self.optimizer.state_dict(),
            "scheduler_state_dict": self.lr_scheduler.state_dict(),
            "best_val_loss": self.best_val_loss,
            "best_r2_mean": self.best_r2_mean,
            "best_monitor": self.best_monitor,
            "best_monitor_value": self.best_monitor_value,
            "history": self.history,
        }
        torch.save(checkpoint, self.output_dir / filename)

    def load_checkpoint(self, filename: str = "checkpoint.pt"):
        """Load training checkpoint."""
        checkpoint_path = Path(filename)
        if not checkpoint_path.is_absolute() and not checkpoint_path.exists():
            checkpoint_path = self.output_dir / checkpoint_path
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        self.model.load_state_dict(checkpoint["model_state_dict"])
        self.optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        self.lr_scheduler.load_state_dict(checkpoint["scheduler_state_dict"])
        self.epoch = checkpoint["epoch"]
        self.best_val_loss = checkpoint["best_val_loss"]
        self.best_r2_mean = checkpoint["best_r2_mean"]
        self.best_monitor = checkpoint.get("best_monitor", "val_r2_mean")
        self.best_monitor_value = checkpoint.get(
            "best_monitor_value",
            self.best_r2_mean if self.best_monitor == "val_r2_mean" else float("inf"),
        )
        self.history = checkpoint["history"]

    def train(
        self,
        max_epochs: int = 260,
        patience: int = 20,
        val_interval: int = 5,
    ):
        """
        Main training loop.

        Args:
            max_epochs: Maximum number of epochs
            patience: Early stopping patience
            val_interval: Validation frequency
        """
        print("=" * 80)
        print("Starting PINN Training")
        print("=" * 80)
        self.curriculum.print_schedule()

        while self.epoch < max_epochs:
            # Train
            train_metrics = self.train_epoch()
            current_epoch = self.epoch

            # Validate
            if self.curriculum.should_validate(current_epoch, val_interval):
                val_metrics = self.validate()
                monitor = self.curriculum.get_early_stopping_metric(current_epoch)
                if monitor != self.best_monitor:
                    self.best_monitor = monitor
                    self.best_monitor_value = (
                        -float("inf") if monitor == "val_r2_mean" else float("inf")
                    )
                    self.patience_counter = 0

                # Log
                print(f"\nEpoch {current_epoch}/{max_epochs}")
                print(
                    f"  Train Loss: {train_metrics['train_total_loss']:.4f} "
                    f"(Data: {train_metrics['train_data_loss']:.4f}, "
                    f"Physics: {train_metrics['train_physics_loss']:.4f})"
                )
                print(
                    f"  Val Loss: {val_metrics['val_data_loss']:.4f}, "
                    f"R² mean: {val_metrics['val_r2_mean']:.4f}"
                )
                print(
                    f"  Val R²: Fx={val_metrics['val_r2_fx']:.4f}, "
                    f"Fy={val_metrics['val_r2_fy']:.4f}"
                )

                # Update history
                self.history["train_loss"].append(train_metrics["train_total_loss"])
                self.history["val_loss"].append(val_metrics["val_data_loss"])
                self.history["val_r2_fx"].append(val_metrics["val_r2_fx"])
                self.history["val_r2_fy"].append(val_metrics["val_r2_fy"])
                self.history["physics_metrics"].append(val_metrics)

                if val_metrics["val_r2_mean"] > self.best_r2_mean:
                    self.best_r2_mean = val_metrics["val_r2_mean"]
                    self.save_checkpoint("best_r2_model.pt")
                    print(f"  ✓ New best R² mean: {self.best_r2_mean:.4f}")

                current_monitor_value = val_metrics[monitor]
                better = (
                    current_monitor_value > self.best_monitor_value
                    if monitor == "val_r2_mean"
                    else current_monitor_value < self.best_monitor_value
                )

                # Early stopping / checkpoint monitor
                if better:
                    self.best_monitor_value = current_monitor_value
                    self.patience_counter = 0
                    self.save_checkpoint("best_model.pt")
                    comparator = "max" if monitor == "val_r2_mean" else "min"
                    print(f"  ✓ New best {monitor} ({comparator}): {current_monitor_value:.4f}")
                else:
                    self.patience_counter += 1

                # Check early stopping
                if self.patience_counter >= patience:
                    print(f"\n Early stopping at epoch {current_epoch} (patience={patience})")
                    break

            # Phase boundary checkpoints
            if current_epoch in [
                self.curriculum.PHASE_A_END,
                self.curriculum.PHASE_B_END,
                self.curriculum.PHASE_C_END,
                self.curriculum.PHASE_D_END,
            ]:
                self.save_checkpoint(f"phase_{self.curriculum.get_phase(current_epoch)}_end.pt")

        # Final save
        self.save_checkpoint("final_model.pt")
        self._save_history()

        print("\n" + "=" * 80)
        print("Training completed!")
        print(f"Best R² mean: {self.best_r2_mean:.4f}")
        print("=" * 80)

    def _save_history(self):
        """Save training history to JSON."""
        with open(self.output_dir / "training_history.json", "w") as f:
            json.dump(self.history, f, indent=2)
