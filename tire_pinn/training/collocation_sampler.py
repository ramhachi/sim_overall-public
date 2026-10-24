"""
Collocation point sampler for physics constraints.

Generates synthetic data points where physics constraints are enforced,
separate from actual TTC tire test data.
"""

import torch


class CollocationSampler:
    """
    Generate collocation points for physics-informed constraints.

    Input ranges (physical TTC units):
    - SR: slip ratio [-]
    - SA: slip angle [deg]
    - Fz: vertical load [N]
    - IA: inclination angle [deg]
    - TSTC: tire surface temperature [degC]
    """

    def __init__(
        self,
        input_mean: torch.Tensor,
        input_std: torch.Tensor,
        sr_range: tuple = (-0.3, 0.3),
        sa_range: tuple = (-13.0, 13.0),
        fz_range: tuple = (500, 3000),
        ia_range: tuple = (-5.0, 5.0),
        tstc_range: tuple = (25.0, 90.0),
        device: str = "cpu",
    ):
        """
        Args:
            input_mean/std: Z-score normalization params (5,)
            *_range: Physical ranges for each input
            device: 'cpu' or 'cuda'
        """
        self.input_mean = input_mean.to(device)
        self.input_std = input_std.to(device)
        self.device = device

        # Physical ranges
        self.ranges = {
            "sr": sr_range,
            "sa": sa_range,
            "fz": fz_range,
            "ia": ia_range,
            "tstc": tstc_range,
        }

    def _uniform(self, name: str, n_samples: int) -> torch.Tensor:
        """Sample one input dimension uniformly in physical units."""
        low, high = self.ranges[name]
        return torch.rand(n_samples, device=self.device) * (high - low) + low

    def sample_zero_slip(self, n_samples: int = 500) -> torch.Tensor:
        """
        Generate points with SR=SA=IA=0, random Fz/TSTC.

        Used for: Zero slip constraint (Fx=Fy=Mz=0 at zero camber)

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_samples, device=self.device)
        sa = torch.zeros(n_samples, device=self.device)
        fz = self._uniform("fz", n_samples)
        ia = torch.zeros(n_samples, device=self.device)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_symmetry(self, n_pairs: int = 1000) -> torch.Tensor:
        """
        Generate pure-lateral points with IA=0 and SR=0.

        Used for: Symmetry constraint (Fy(-SA) = -Fy(SA))
        Note: Caller should flip SA sign to create pairs.

        Returns:
            (n_pairs, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_pairs, device=self.device)
        sa = self._uniform("sa", n_pairs)
        fz = self._uniform("fz", n_pairs)
        ia = torch.zeros(n_pairs, device=self.device)  # IA=0 for symmetry
        tstc = self._uniform("tstc", n_pairs)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_friction_circle(
        self, n_samples: int = 500, bias_combined: float = 0.7
    ) -> torch.Tensor:
        """
        Generate points dense in combined slip region (SR≠0, SA≠0).

        Used for: Friction circle constraint (√(Fx²+Fy²) ≤ μ*Fz)
        Bias sampling toward high slip magnitudes where violations likely.

        Args:
            n_samples: Number of points
            bias_combined: Fraction of samples with both SR≠0 and SA≠0

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        n_combined = int(n_samples * bias_combined)
        n_single = n_samples - n_combined

        sr_span = self.ranges["sr"][1] - self.ranges["sr"][0]
        sa_span = self.ranges["sa"][1] - self.ranges["sa"][0]

        # Combined slip region (high magnitude)
        sr_combined = torch.rand(n_combined, device=self.device) * sr_span + self.ranges["sr"][0]
        sa_combined = torch.rand(n_combined, device=self.device) * sa_span + self.ranges["sa"][0]

        # Single slip region
        sr_single = torch.cat(
            [
                torch.rand(n_single // 2, device=self.device) * sr_span + self.ranges["sr"][0],
                torch.zeros(n_single - n_single // 2, device=self.device),  # Pure lateral
            ]
        )
        sa_single = torch.cat(
            [
                torch.zeros(n_single // 2, device=self.device),
                torch.rand(n_single - n_single // 2, device=self.device) * sa_span
                + self.ranges["sa"][0],
            ]
        )

        sr = torch.cat([sr_combined, sr_single])
        sa = torch.cat([sa_combined, sa_single])

        # Shuffle
        perm = torch.randperm(n_samples, device=self.device)
        sr, sa = sr[perm], sa[perm]

        # Other variables (random)
        fz = self._uniform("fz", n_samples)
        ia = self._uniform("ia", n_samples)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_monotonicity(self, n_samples: int = 300) -> torch.Tensor:
        """
        Generate points for load monotonicity check (∂F/∂Fz ≥ 0).

        Used for: Monotonicity constraint
        Sample across full input space.

        Returns:
            (n_samples, 5) raw physical-unit tensor with requires_grad=True
        """
        sr = self._uniform("sr", n_samples)
        sa = self._uniform("sa", n_samples)
        fz = self._uniform("fz", n_samples)
        ia = self._uniform("ia", n_samples)
        tstc = self._uniform("tstc", n_samples)

        x = torch.stack([sr, sa, fz, ia, tstc], dim=1)
        x.requires_grad_(True)
        return x

    def sample_sa_zero_fy(self, n_samples: int = 500) -> torch.Tensor:
        """
        Generate points with SA=IA=0, random SR / Fz / TSTC.

        Used for: Fy-at-SA=0 leakage constraint at zero camber.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = self._uniform("sr", n_samples)
        sa = torch.zeros(n_samples, device=self.device)  # SA=0
        fz = self._uniform("fz", n_samples)
        ia = torch.zeros(n_samples, device=self.device)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_combined_slip(self, n_samples: int = 500) -> torch.Tensor:
        """
        Generate points where SR and SA are both non-zero.

        Used for: Combined slip consistency against pure-slip slices.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr_abs_max = max(abs(self.ranges["sr"][0]), abs(self.ranges["sr"][1]))
        sa_abs_max = max(abs(self.ranges["sa"][0]), abs(self.ranges["sa"][1]))
        sr_abs = torch.rand(n_samples, device=self.device) * (sr_abs_max - 0.02) + 0.02
        sa_abs = torch.rand(n_samples, device=self.device) * (sa_abs_max - 1.0) + 1.0
        sr_sign = torch.where(torch.rand(n_samples, device=self.device) >= 0.5, 1.0, -1.0)
        sa_sign = torch.where(torch.rand(n_samples, device=self.device) >= 0.5, 1.0, -1.0)

        sr = sr_abs * sr_sign
        sa = sa_abs * sa_sign
        fz = self._uniform("fz", n_samples)
        ia = self._uniform("ia", n_samples)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_stiffness(self, n_samples: int = 300) -> torch.Tensor:
        """
        Generate low-slip operating points for stiffness load-sensitivity checks.

        The loss itself constructs +/- small SR/SA perturbations and a higher-Fz
        counterpart from these base points.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_samples, device=self.device)
        sa = torch.zeros(n_samples, device=self.device)
        fz_low, fz_high = self.ranges["fz"]
        fz_limit = max(fz_low, fz_high * 0.80)
        fz = torch.rand(n_samples, device=self.device) * (fz_limit - fz_low) + fz_low
        ia = self._uniform("ia", n_samples)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_high_sa_lateral_load(
        self, n_samples: int = 300, min_abs_sa_deg: float = 6.0
    ) -> torch.Tensor:
        """
        Generate pure-lateral high-|SA| points for saturated Fy load checks.

        The loss itself constructs a higher-Fz counterpart from these base points.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_samples, device=self.device)
        sa_abs_max = max(abs(self.ranges["sa"][0]), abs(self.ranges["sa"][1]))
        sa_abs = torch.rand(n_samples, device=self.device) * (
            sa_abs_max - min_abs_sa_deg
        ) + min_abs_sa_deg
        sa_sign = torch.where(torch.rand(n_samples, device=self.device) >= 0.5, 1.0, -1.0)
        sa = sa_abs * sa_sign
        fz_low, fz_high = self.ranges["fz"]
        fz_limit = max(fz_low, fz_high * 0.80)
        fz = torch.rand(n_samples, device=self.device) * (fz_limit - fz_low) + fz_low
        ia = torch.zeros(n_samples, device=self.device)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_mz_trail(self, n_samples: int = 500) -> torch.Tensor:
        """
        Generate pure-lateral points for Mz/pneumatic-trail checks.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_samples, device=self.device)
        sa = self._uniform("sa", n_samples)
        fz = self._uniform("fz", n_samples)
        ia = torch.zeros(n_samples, device=self.device)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_camber_thrust(self, n_samples: int = 500) -> torch.Tensor:
        """
        Generate zero-slip points for camber thrust checks.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr = torch.zeros(n_samples, device=self.device)
        sa = torch.zeros(n_samples, device=self.device)
        fz = self._uniform("fz", n_samples)
        ia = self._uniform("ia", n_samples)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_smoothness(
        self,
        n_samples: int = 500,
        sr_delta: float = 0.015,
        sa_delta_deg: float = 0.75,
    ) -> torch.Tensor:
        """
        Generate pure-section center points for finite-difference smoothness.

        The loss evaluates +/- perturbations around these centers, so the
        centers are sampled away from the configured SR/SA boundaries.

        Returns:
            (n_samples, 5) raw physical-unit tensor
        """
        sr_low, sr_high = self.ranges["sr"]
        sa_low, sa_high = self.ranges["sa"]
        sr_center_low = min(sr_low + sr_delta, sr_high)
        sr_center_high = max(sr_high - sr_delta, sr_center_low)
        sa_center_low = min(sa_low + sa_delta_deg, sa_high)
        sa_center_high = max(sa_high - sa_delta_deg, sa_center_low)

        sr = torch.rand(n_samples, device=self.device) * (
            sr_center_high - sr_center_low
        ) + sr_center_low
        sa = torch.rand(n_samples, device=self.device) * (
            sa_center_high - sa_center_low
        ) + sa_center_low
        fz = self._uniform("fz", n_samples)
        ia = torch.zeros(n_samples, device=self.device)
        tstc = self._uniform("tstc", n_samples)

        return torch.stack([sr, sa, fz, ia, tstc], dim=1)

    def sample_batch(
        self,
        n_zero: int = 500,
        n_sym: int = 1000,
        n_fric: int = 500,
        n_mono: int = 300,
        n_fy_sa_zero: int = 500,
        n_combined: int = 500,
        n_stiffness: int = 300,
        n_high_sa_lateral: int = 300,
        n_mz_trail: int = 500,
        n_camber: int = 500,
        n_smoothness: int = 500,
    ) -> dict:
        """
        Generate a full batch of collocation points.

        Returns:
            dict keyed by physics constraint name.
        """
        return {
            "zero": self.sample_zero_slip(n_zero),
            "symmetry": self.sample_symmetry(n_sym),
            "friction": self.sample_friction_circle(n_fric),
            "monotonicity": self.sample_monotonicity(n_mono),
            "sa_zero_fy": self.sample_sa_zero_fy(n_fy_sa_zero),
            "combined_slip": self.sample_combined_slip(n_combined),
            "stiffness": self.sample_stiffness(n_stiffness),
            "high_sa_lateral": self.sample_high_sa_lateral_load(n_high_sa_lateral),
            "mz_trail": self.sample_mz_trail(n_mz_trail),
            "camber": self.sample_camber_thrust(n_camber),
            "smoothness": self.sample_smoothness(n_smoothness),
        }
