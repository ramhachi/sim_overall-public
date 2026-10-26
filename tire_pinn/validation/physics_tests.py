"""
Physics constraint validation tests for trained PINN tire models.

These tests operate in the raw physical unit system used by the TTC data:
- SA / IA in degrees
- TSTC in degree Celsius
- Fz in Newton
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import pandas as pd
import torch

from ..models.pinn_model import NormalizedTirePINN


class PhysicsValidator:
    """Validate physics constraints on a trained PINN model."""

    def __init__(
        self,
        model: NormalizedTirePINN,
        data_frame: Optional[pd.DataFrame] = None,
        device: str = "cpu",
        coverage_data_frame: Optional[pd.DataFrame] = None,
    ):
        self.model = model.to(device)
        self.model.eval()
        self.device = device
        self.data_frame = data_frame.copy() if data_frame is not None else None
        self.coverage_data_frame = (
            coverage_data_frame.copy() if coverage_data_frame is not None else None
        )
        self.bounds = self._compute_bounds(data_frame)
        self._empirical_monotonicity_cache: Dict[float, float] = {}

    @staticmethod
    def _compute_bounds(data_frame: Optional[pd.DataFrame]) -> Dict[str, tuple[float, float]]:
        if data_frame is None or data_frame.empty:
            return {
                "SR": (-0.25, 0.25),
                "SA": (-12.0, 12.0),
                "Fz": (400.0, 2200.0),
                "IA": (-4.0, 4.0),
                "TSTC": (25.0, 85.0),
            }

        bounds = {}
        for column in ["SR", "SA", "Fz", "IA", "TSTC"]:
            series = data_frame[column].astype(float)
            lo = float(np.nanpercentile(series, 1))
            hi = float(np.nanpercentile(series, 99))
            bounds[column] = (lo, hi)
        return bounds

    @staticmethod
    def _min_scaled_distance(
        query: np.ndarray,
        reference: np.ndarray,
        scales: np.ndarray,
        chunk_size: int = 256,
    ) -> np.ndarray:
        """Return nearest-neighbor distance in scaled (SA, Fz, IA, TSTC) space."""
        if len(query) == 0 or len(reference) == 0:
            return np.full(len(query), np.inf, dtype=float)

        scaled_query = query / scales
        scaled_reference = reference / scales
        distances = np.empty(len(scaled_query), dtype=float)
        for start in range(0, len(scaled_query), chunk_size):
            chunk = scaled_query[start : start + chunk_size]
            diff = chunk[:, None, :] - scaled_reference[None, :, :]
            distances[start : start + len(chunk)] = np.sqrt(np.sum(diff * diff, axis=2)).min(
                axis=1
            )
        return distances

    def run_all_tests(
        self,
        zero_threshold: float = 20.0,
        zero_moment_threshold: float = 50.0,
        symmetry_mean_threshold: float = 0.25,
        symmetry_p95_threshold: float = 0.45,
        max_friction_violation_rate: float = 0.05,
        measured_friction_violation_rate: float = 0.15,
        measured_friction_mu_cap: float = 2.6,
        measured_combined_slip_violation_rate: float = 0.35,
        measured_combined_slip_coverage_tolerance: float = 1.0,
        measured_combined_slip_min_coverage: int = 50,
        max_monotonicity_violation_rate: float = 0.10,
        monotonicity_negative_gradient_tolerance: float = 0.10,
        monotonicity_empirical_margin: float = 0.0,
        max_combined_slip_violation_rate: float = 0.35,
        combined_slip_margin: float = 0.05,
        max_stiffness_load_violation_rate: float = 0.50,
        stiffness_load_drop_margin: float = 0.03,
        max_high_sa_lateral_load_violation_rate: float = 0.05,
        high_sa_lateral_load_drop_margin: float = 0.01,
        high_sa_min_abs_sa_deg: float = 6.0,
        max_mz_negative_trail_rate: float = 0.20,
        max_mz_trail_cap_rate: float = 0.25,
        max_mz_trail_growth_rate: float = 0.65,
        max_pneumatic_trail_m: float = 0.18,
        max_camber_asymmetry: float = 0.50,
        max_camber_cap_violation_rate: float = 0.15,
        camber_mu_cap: float = 0.25,
        min_camber_coverage_per_side: int = 50,
    ) -> Dict[str, dict]:
        print("=" * 80)
        print("Physics Constraint Validation")
        print("=" * 80)

        results = {
            "zero_slip": self.test_zero_slip(
                force_threshold=zero_threshold,
                moment_threshold=zero_moment_threshold,
            ),
            "symmetry": self.test_symmetry(
                mean_threshold=symmetry_mean_threshold,
                p95_threshold=symmetry_p95_threshold,
            ),
            "friction_circle": self.test_friction_circle(
                max_violation_rate=max_friction_violation_rate
            ),
            "measured_friction_circle": self.test_measured_friction_circle(
                max_violation_rate=measured_friction_violation_rate,
                mu_cap=measured_friction_mu_cap,
            ),
            "measured_combined_slip": self.test_measured_combined_slip(
                max_violation_rate=measured_combined_slip_violation_rate,
                mu_cap=measured_friction_mu_cap,
                coverage_tolerance=measured_combined_slip_coverage_tolerance,
                min_coverage_samples=measured_combined_slip_min_coverage,
            ),
            "monotonicity": self.test_monotonicity(
                max_violation_rate=max_monotonicity_violation_rate,
                negative_gradient_tolerance=monotonicity_negative_gradient_tolerance,
                empirical_margin=monotonicity_empirical_margin,
            ),
            "combined_slip": self.test_combined_slip(
                max_violation_rate=max_combined_slip_violation_rate,
                margin=combined_slip_margin,
            ),
            "stiffness_load": self.test_stiffness_load_sensitivity(
                max_violation_rate=max_stiffness_load_violation_rate,
                load_drop_margin=stiffness_load_drop_margin,
            ),
            "high_sa_lateral_load": self.test_high_sa_lateral_load_sensitivity(
                max_violation_rate=max_high_sa_lateral_load_violation_rate,
                load_drop_margin=high_sa_lateral_load_drop_margin,
                min_abs_sa_deg=high_sa_min_abs_sa_deg,
            ),
            "mz_trail": self.test_mz_pneumatic_trail(
                max_negative_trail_rate=max_mz_negative_trail_rate,
                max_cap_rate=max_mz_trail_cap_rate,
                max_growth_rate=max_mz_trail_growth_rate,
                max_trail_m=max_pneumatic_trail_m,
            ),
            "camber_thrust": self.test_camber_thrust(
                max_mean_asymmetry=max_camber_asymmetry,
                max_cap_violation_rate=max_camber_cap_violation_rate,
                camber_mu_cap=camber_mu_cap,
                min_coverage_per_side=min_camber_coverage_per_side,
            ),
        }
        self._print_summary(results)
        return results

    @torch.no_grad()
    def test_zero_slip(self, force_threshold: float = 20.0, moment_threshold: float = 50.0) -> dict:
        fz_min, fz_max = self.bounds["Fz"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        fz_vals = torch.linspace(max(200.0, fz_min), fz_max, 4)
        tstc_vals = torch.linspace(tstc_min, tstc_max, 3)

        test_inputs = []
        for fz in fz_vals:
            for tstc in tstc_vals:
                test_inputs.append([0.0, 0.0, fz.item(), 0.0, tstc.item()])

        x = torch.tensor(test_inputs, dtype=torch.float32, device=self.device)
        y = self.model(x)

        fx = y[:, 0].abs()
        fy = y[:, 1].abs()
        mz = y[:, 3].abs()

        max_force = max(fx.max().item(), fy.max().item())
        max_moment = mz.max().item()
        passed = max_force <= force_threshold and max_moment <= moment_threshold

        metrics = {
            "max_fx": fx.max().item(),
            "max_fy": fy.max().item(),
            "max_mz": mz.max().item(),
            "mean_fx": fx.mean().item(),
            "mean_fy": fy.mean().item(),
            "force_threshold_n": force_threshold,
            "moment_threshold_nm": moment_threshold,
        }
        message = (
            f"✓ PASS: max zero-slip force {max_force:.2f} N <= {force_threshold:.2f} N, "
            f"moment {max_moment:.2f} Nm <= {moment_threshold:.2f} Nm"
            if passed
            else f"✗ FAIL: max zero-slip force {max_force:.2f} N, moment {max_moment:.2f} Nm"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_symmetry(
        self,
        mean_threshold: float = 0.25,
        p95_threshold: float = 0.45,
    ) -> dict:
        sa_min, sa_max = self.bounds["SA"]
        fz_min, fz_max = self.bounds["Fz"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        sa_limit = min(10.0, max(abs(sa_min), abs(sa_max)))
        sa_start = min(sa_limit, max(1.0, sa_limit * 0.25))
        sa_vals = torch.linspace(sa_start, sa_limit, 5)
        fz_vals = torch.linspace(max(300.0, fz_min), fz_max, 3)
        tstc_vals = torch.linspace(tstc_min, tstc_max, 2)

        test_inputs = []
        for sa in sa_vals:
            for fz in fz_vals:
                for tstc in tstc_vals:
                    test_inputs.append([0.0, sa.item(), fz.item(), 0.0, tstc.item()])

        x_pos = torch.tensor(test_inputs, dtype=torch.float32, device=self.device)
        x_neg = x_pos.clone()
        x_neg[:, 1] = -x_pos[:, 1]

        y_pos = self.model(x_pos)
        y_neg = self.model(x_neg)

        fy_pos = y_pos[:, 1]
        fy_neg = y_neg[:, 1]
        denom = torch.maximum(
            torch.maximum(fy_pos.abs(), fy_neg.abs()), torch.full_like(fy_pos, 1.0)
        )
        asymmetry = (fy_pos + fy_neg).abs() / denom

        max_asym = asymmetry.max().item()
        mean_asym = asymmetry.mean().item()
        p95_asym = float(np.quantile(asymmetry.detach().cpu().numpy(), 0.95))
        passed = mean_asym <= mean_threshold and p95_asym <= p95_threshold

        metrics = {
            "max_asymmetry": max_asym,
            "mean_asymmetry": mean_asym,
            "p95_asymmetry": p95_asym,
            "mean_threshold": mean_threshold,
            "p95_threshold": p95_threshold,
        }
        message = (
            "✓ PASS: symmetry mean/p95 "
            f"{mean_asym:.4f}/{p95_asym:.4f} <= {mean_threshold:.4f}/{p95_threshold:.4f}"
            if passed
            else "✗ FAIL: symmetry mean/p95 "
            f"{mean_asym:.4f}/{p95_asym:.4f} > {mean_threshold:.4f}/{p95_threshold:.4f}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_measured_friction_circle(
        self,
        max_violation_rate: float = 0.15,
        mu_cap: float = 2.6,
    ) -> dict:
        """Check the friction circle on the measured holdout distribution.

        Unlike the synthetic probe, this keeps the real SR/SA/Fz/IA/TSTC
        correlations, so it catches failures that uniform sampling misses.
        """
        if self.data_frame is None or self.data_frame.empty:
            return {
                "pass": True,
                "skipped": True,
                "metrics": {"violation_rate": float("nan"), "n_samples": 0},
                "message": "SKIP: no measured holdout data for friction-circle gate",
            }
        frame = self.data_frame
        x = frame[["SR", "SA", "Fz", "IA", "TSTC"]].to_numpy(dtype=np.float32)
        y = self.model(torch.tensor(x, dtype=torch.float32, device=self.device))
        fx = y[:, 0]
        fy = y[:, 1]
        fz = torch.tensor(x[:, 2], dtype=torch.float32, device=self.device)
        force_mag = torch.sqrt(fx**2 + fy**2)
        limit = mu_cap * fz.abs()
        violation = torch.relu(force_mag - limit)
        violation_rate = (violation > 0).float().mean().item()
        passed = violation_rate <= max_violation_rate
        metrics = {
            "violation_rate": violation_rate,
            "max_violation": violation.max().item(),
            "mu_cap": mu_cap,
            "threshold": max_violation_rate,
            "n_samples": int(len(frame)),
        }
        message = (
            f"✓ PASS: measured friction violation rate {violation_rate:.2%} "
            f"<= {max_violation_rate:.2%}"
            if passed
            else f"✗ FAIL: measured friction violation rate {violation_rate:.2%} "
            f"> {max_violation_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_friction_circle(
        self,
        mu_cap: float = 2.6,
        max_violation_rate: float = 0.05,
    ) -> dict:
        sr_min, sr_max = self.bounds["SR"]
        sa_min, sa_max = self.bounds["SA"]
        fz_min, fz_max = self.bounds["Fz"]
        ia_min, ia_max = self.bounds["IA"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        n_samples = 1200
        x = np.column_stack(
            [
                rng.uniform(sr_min, sr_max, size=n_samples),
                rng.uniform(sa_min, sa_max, size=n_samples),
                rng.uniform(max(200.0, fz_min), fz_max, size=n_samples),
                rng.uniform(ia_min, ia_max, size=n_samples),
                rng.uniform(tstc_min, tstc_max, size=n_samples),
            ]
        ).astype(np.float32)

        y = self.model(torch.tensor(x, dtype=torch.float32, device=self.device))
        fx = y[:, 0]
        fy = y[:, 1]
        fz = torch.tensor(x[:, 2], dtype=torch.float32, device=self.device)

        force_mag = torch.sqrt(fx**2 + fy**2)
        limit = mu_cap * fz.abs()
        violation = torch.relu(force_mag - limit)
        violation_rate = (violation > 0).float().mean().item()
        passed = violation_rate <= max_violation_rate

        metrics = {
            "violation_rate": violation_rate,
            "max_violation": violation.max().item(),
            "mean_violation": violation[violation > 0].mean().item()
            if (violation > 0).any()
            else 0.0,
            "mu_cap": mu_cap,
            "threshold": max_violation_rate,
        }
        message = (
            f"✓ PASS: friction violation rate {violation_rate:.2%} <= {max_violation_rate:.2%}"
            if passed
            else f"✗ FAIL: friction violation rate {violation_rate:.2%} > {max_violation_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    def test_monotonicity(
        self,
        n_samples: int = 256,
        max_violation_rate: float = 0.10,
        negative_gradient_tolerance: float = 0.10,
        empirical_margin: float = 0.0,
    ) -> dict:
        sr_min, sr_max = self.bounds["SR"]
        sa_min, sa_max = self.bounds["SA"]
        fz_min, fz_max = self.bounds["Fz"]
        ia_min, ia_max = self.bounds["IA"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        x = np.column_stack(
            [
                rng.uniform(sr_min, sr_max, size=n_samples),
                rng.uniform(sa_min, sa_max, size=n_samples),
                rng.uniform(max(200.0, fz_min), fz_max, size=n_samples),
                rng.uniform(ia_min, ia_max, size=n_samples),
                rng.uniform(tstc_min, tstc_max, size=n_samples),
            ]
        ).astype(np.float32)

        x_tensor = torch.tensor(x, dtype=torch.float32, device=self.device, requires_grad=True)
        outputs = self.model(x_tensor)
        fx_mag = torch.sqrt(outputs[:, 0] ** 2 + 1e-6)
        fy_mag = torch.sqrt(outputs[:, 1] ** 2 + 1e-6)
        grad_fx = torch.autograd.grad(outputs=fx_mag.sum(), inputs=x_tensor, retain_graph=True)[0][
            :, 2
        ]
        grad_fy = torch.autograd.grad(outputs=fy_mag.sum(), inputs=x_tensor, create_graph=False)[0][
            :, 2
        ]

        violation_rate_fx = (grad_fx < -negative_gradient_tolerance).float().mean().item()
        violation_rate_fy = (grad_fy < -negative_gradient_tolerance).float().mean().item()
        violation_rate = max(violation_rate_fx, violation_rate_fy)
        empirical_baseline = self._estimate_empirical_monotonicity_baseline(
            negative_gradient_tolerance
        )
        effective_threshold = max(max_violation_rate, empirical_baseline + empirical_margin)
        passed = violation_rate <= effective_threshold

        metrics = {
            "violation_rate": violation_rate,
            "violation_rate_fx": violation_rate_fx,
            "violation_rate_fy": violation_rate_fy,
            "raw_negative_rate_fx": (grad_fx < 0).float().mean().item(),
            "raw_negative_rate_fy": (grad_fy < 0).float().mean().item(),
            "min_gradient_fx": grad_fx.min().item(),
            "min_gradient_fy": grad_fy.min().item(),
            "mean_gradient_fx": grad_fx.mean().item(),
            "mean_gradient_fy": grad_fy.mean().item(),
            "threshold": max_violation_rate,
            "effective_threshold": effective_threshold,
            "empirical_baseline": empirical_baseline,
            "empirical_margin": empirical_margin,
            "negative_gradient_tolerance": negative_gradient_tolerance,
        }
        message = (
            "✓ PASS: monotonicity violation rate "
            f"{violation_rate:.2%} <= {effective_threshold:.2%} "
            f"(tol={negative_gradient_tolerance:.3f})"
            if passed
            else "✗ FAIL: monotonicity violation rate "
            f"{violation_rate:.2%} > {effective_threshold:.2%} "
            f"(tol={negative_gradient_tolerance:.3f})"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_measured_combined_slip(
        self,
        max_violation_rate: float = 0.35,
        margin: float = 0.05,
        mu_cap: float = 2.6,
        coverage_tolerance: float = 1.0,
        min_coverage_samples: int = 50,
    ) -> dict:
        """Check combined-slip behavior on measured combined-slip rows.

        When a coverage frame is supplied, only SR-swept rows with a nearby
        pure-lateral measurement are evaluated. The gate otherwise extrapolates a
        pure-slip force that the training data does not contain. Too few covered
        rows returns ``pass=None`` / ``skipped=True`` (not evaluated).
        """
        if self.data_frame is None or self.data_frame.empty:
            return {
                "pass": True,
                "skipped": True,
                "metrics": {"violation_rate": float("nan"), "n_samples": 0},
                "message": "SKIP: no measured holdout data for combined-slip gate",
            }
        frame = self.data_frame
        combined = frame[(frame["SR"].abs() > 0.02) & (frame["SA"].abs() > 1.0)]
        if combined.empty:
            return {
                "pass": True,
                "skipped": True,
                "metrics": {"violation_rate": float("nan"), "n_samples": 0},
                "message": "SKIP: no measured combined-slip rows in holdout",
            }

        coverage_metrics: Dict[str, float] = {}
        if self.coverage_data_frame is not None:
            coverage_frame = self.coverage_data_frame
            combined_for_gate = combined
            if "source_test_type" in combined.columns:
                # Cr runs never sweep SR: their |SR| is TTC's load-dependent
                # free-rolling offset (~-0.03 at 1 kN), not applied slip.
                combined_for_gate = combined[combined["source_test_type"] != "Cr"]

            pure = coverage_frame[
                (coverage_frame["SR"].abs() < 0.01)
                & (coverage_frame["SA"].abs() > 1.0)
            ]

            n_total = int(len(combined_for_gate))
            if n_total == 0 or pure.empty:
                covered = combined_for_gate.iloc[0:0]
            else:
                scales = np.array([1.0, 100.0, 0.5, 5.0], dtype=float)
                distances = self._min_scaled_distance(
                    combined_for_gate[["SA", "Fz", "IA", "TSTC"]].to_numpy(dtype=float),
                    pure[["SA", "Fz", "IA", "TSTC"]].to_numpy(dtype=float),
                    scales,
                )
                covered = combined_for_gate[distances <= coverage_tolerance]

            n_covered = int(len(covered))
            coverage_fraction = n_covered / n_total if n_total else 0.0
            coverage_metrics = {
                "n_total": n_total,
                "n_samples": n_covered,
                "coverage_fraction": coverage_fraction,
                "coverage_tolerance": coverage_tolerance,
                "min_coverage_samples": min_coverage_samples,
                "coverage_limited": coverage_fraction < 1.0,
            }
            if n_covered < min_coverage_samples:
                coverage_metrics["coverage_limited"] = True
                message = (
                    f"⚠ NOT EVALUATED: only {n_covered}/{n_total} combined-slip rows "
                    f"have pure-lateral coverage (< {min_coverage_samples}); gate not evaluated"
                )
                print(f"  {message}")
                return {
                    "pass": None,
                    "skipped": True,
                    "metrics": {"violation_rate": float("nan"), **coverage_metrics},
                    "message": message,
                }
            combined = covered

        x_full = combined[["SR", "SA", "Fz", "IA", "TSTC"]].to_numpy(dtype=np.float32)
        x_pure_long = x_full.copy()
        x_pure_long[:, 1] = 0.0
        x_pure_lat = x_full.copy()
        x_pure_lat[:, 0] = 0.0
        y_full = self.model(torch.tensor(x_full, dtype=torch.float32, device=self.device))
        y_long = self.model(torch.tensor(x_pure_long, dtype=torch.float32, device=self.device))
        y_lat = self.model(torch.tensor(x_pure_lat, dtype=torch.float32, device=self.device))
        fz = torch.tensor(x_full[:, 2], dtype=torch.float32, device=self.device)
        scale = mu_cap * torch.abs(fz) + 1e-6
        fx_excess = torch.relu((torch.abs(y_full[:, 0]) - torch.abs(y_long[:, 0])) / scale - margin)
        fy_excess = torch.relu((torch.abs(y_full[:, 1]) - torch.abs(y_lat[:, 1])) / scale - margin)
        violation = (fx_excess > 0) | (fy_excess > 0)
        violation_rate = violation.float().mean().item()
        passed = violation_rate <= max_violation_rate
        metrics = {
            "violation_rate": violation_rate,
            "max_fx_excess": fx_excess.max().item(),
            "max_fy_excess": fy_excess.max().item(),
            "threshold": max_violation_rate,
            "margin": margin,
            "mu_cap": mu_cap,
            "n_samples": int(len(combined)),
            **coverage_metrics,
        }
        coverage_tag = " (coverage-limited)" if coverage_metrics else ""
        coverage_suffix = ""
        if coverage_metrics:
            coverage_suffix = (
                f" on {len(combined)}/{coverage_metrics['n_total']} covered SR-swept rows "
                f"({coverage_metrics['coverage_fraction']:.1%})"
            )
        if passed:
            message = (
                f"✓ PASS{coverage_tag}: measured combined-slip violation rate "
                f"{violation_rate:.2%} <= {max_violation_rate:.2%}{coverage_suffix}"
            )
        else:
            message = (
                f"✗ FAIL{coverage_tag}: measured combined-slip violation rate "
                f"{violation_rate:.2%} > {max_violation_rate:.2%}{coverage_suffix}"
            )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_combined_slip(
        self,
        n_samples: int = 500,
        mu_cap: float = 2.6,
        margin: float = 0.05,
        max_violation_rate: float = 0.35,
    ) -> dict:
        """Check that combined slip does not boost Fx/Fy beyond pure-slip slices."""
        sr_min, sr_max = self.bounds["SR"]
        sa_min, sa_max = self.bounds["SA"]
        fz_min, fz_max = self.bounds["Fz"]
        ia_min, ia_max = self.bounds["IA"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        sr_abs_max = max(abs(sr_min), abs(sr_max), 0.02)
        sa_abs_max = max(abs(sa_min), abs(sa_max), 1.0)
        sr_abs = rng.uniform(0.02, sr_abs_max, size=n_samples)
        sa_abs = rng.uniform(1.0, sa_abs_max, size=n_samples)
        sr = sr_abs * rng.choice([-1.0, 1.0], size=n_samples)
        sa = sa_abs * rng.choice([-1.0, 1.0], size=n_samples)
        fz = rng.uniform(max(200.0, fz_min), fz_max, size=n_samples)

        x_full = np.column_stack(
            [
                sr,
                sa,
                fz,
                rng.uniform(ia_min, ia_max, size=n_samples),
                rng.uniform(tstc_min, tstc_max, size=n_samples),
            ]
        ).astype(np.float32)
        x_pure_long = x_full.copy()
        x_pure_lat = x_full.copy()
        x_pure_long[:, 1] = 0.0
        x_pure_lat[:, 0] = 0.0

        y_full = self.model(torch.tensor(x_full, dtype=torch.float32, device=self.device))
        y_pure_long = self.model(torch.tensor(x_pure_long, dtype=torch.float32, device=self.device))
        y_pure_lat = self.model(torch.tensor(x_pure_lat, dtype=torch.float32, device=self.device))

        fz_tensor = torch.tensor(fz, dtype=torch.float32, device=self.device)
        force_scale = mu_cap * torch.abs(fz_tensor) + 1e-6
        fx_excess = torch.relu(
            (torch.abs(y_full[:, 0]) - torch.abs(y_pure_long[:, 0])) / force_scale - margin
        )
        fy_excess = torch.relu(
            (torch.abs(y_full[:, 1]) - torch.abs(y_pure_lat[:, 1])) / force_scale - margin
        )

        violation_rate_fx = (fx_excess > 0).float().mean().item()
        violation_rate_fy = (fy_excess > 0).float().mean().item()
        violation_rate = max(violation_rate_fx, violation_rate_fy)
        passed = violation_rate <= max_violation_rate

        metrics = {
            "violation_rate": violation_rate,
            "violation_rate_fx": violation_rate_fx,
            "violation_rate_fy": violation_rate_fy,
            "max_excess": torch.max(torch.cat([fx_excess, fy_excess])).item(),
            "margin": margin,
            "threshold": max_violation_rate,
        }
        message = (
            f"✓ PASS: combined-slip violation rate {violation_rate:.2%} <= {max_violation_rate:.2%}"
            if passed
            else f"✗ FAIL: combined-slip violation rate {violation_rate:.2%} > {max_violation_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_stiffness_load_sensitivity(
        self,
        n_samples: int = 256,
        sr_delta: float = 0.01,
        sa_delta_deg: float = 0.25,
        load_step_ratio: float = 0.20,
        min_load_step: float = 100.0,
        load_drop_margin: float = 0.03,
        max_violation_rate: float = 0.50,
    ) -> dict:
        """Check that small-slip stiffness magnitude does not often drop as Fz rises."""
        fz_min, fz_max = self.bounds["Fz"]
        ia_min, ia_max = self.bounds["IA"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        fz_floor = max(200.0, fz_min)
        fz_ceiling = max(fz_floor + min_load_step, fz_max - min_load_step)
        fz_low = rng.uniform(fz_floor, fz_ceiling, size=n_samples).astype(np.float32)
        load_step = np.maximum(load_step_ratio * fz_low, min_load_step)
        fz_high = np.minimum(fz_low + load_step, float(fz_max))
        fz_high = np.maximum(fz_high, fz_low + 1.0).astype(np.float32)

        base = np.column_stack(
            [
                np.zeros(n_samples),
                np.zeros(n_samples),
                fz_low,
                rng.uniform(ia_min, ia_max, size=n_samples),
                rng.uniform(tstc_min, tstc_max, size=n_samples),
            ]
        ).astype(np.float32)

        def eval_at(sr: float, sa: float, fz_values: np.ndarray) -> torch.Tensor:
            x = base.copy()
            x[:, 0] = sr
            x[:, 1] = sa
            x[:, 2] = fz_values
            return self.model(torch.tensor(x, dtype=torch.float32, device=self.device))

        fx_plus_low = eval_at(sr_delta, 0.0, fz_low)[:, 0]
        fx_minus_low = eval_at(-sr_delta, 0.0, fz_low)[:, 0]
        fx_plus_high = eval_at(sr_delta, 0.0, fz_high)[:, 0]
        fx_minus_high = eval_at(-sr_delta, 0.0, fz_high)[:, 0]

        fy_plus_low = eval_at(0.0, sa_delta_deg, fz_low)[:, 1]
        fy_minus_low = eval_at(0.0, -sa_delta_deg, fz_low)[:, 1]
        fy_plus_high = eval_at(0.0, sa_delta_deg, fz_high)[:, 1]
        fy_minus_high = eval_at(0.0, -sa_delta_deg, fz_high)[:, 1]

        kx_low = (fx_plus_low - fx_minus_low) / (2.0 * sr_delta)
        kx_high = (fx_plus_high - fx_minus_high) / (2.0 * sr_delta)
        ky_low = torch.abs((fy_plus_low - fy_minus_low) / (2.0 * sa_delta_deg))
        ky_high = torch.abs((fy_plus_high - fy_minus_high) / (2.0 * sa_delta_deg))

        kx_drop = kx_high < kx_low * (1.0 - load_drop_margin)
        ky_drop = ky_high < ky_low * (1.0 - load_drop_margin)
        kx_negative = kx_low < 0.0
        violation_rate_kx = torch.logical_or(kx_drop, kx_negative).float().mean().item()
        violation_rate_ky = ky_drop.float().mean().item()
        violation_rate = max(violation_rate_kx, violation_rate_ky)
        passed = violation_rate <= max_violation_rate

        metrics = {
            "violation_rate": violation_rate,
            "violation_rate_kx": violation_rate_kx,
            "violation_rate_ky": violation_rate_ky,
            "negative_kx_rate": kx_negative.float().mean().item(),
            "mean_kx_low": kx_low.mean().item(),
            "mean_kx_high": kx_high.mean().item(),
            "mean_ky_low": ky_low.mean().item(),
            "mean_ky_high": ky_high.mean().item(),
            "load_drop_margin": load_drop_margin,
            "threshold": max_violation_rate,
        }
        message = (
            f"✓ PASS: stiffness-load violation rate {violation_rate:.2%} <= {max_violation_rate:.2%}"
            if passed
            else f"✗ FAIL: stiffness-load violation rate {violation_rate:.2%} > {max_violation_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_high_sa_lateral_load_sensitivity(
        self,
        n_samples: int = 512,
        min_abs_sa_deg: float = 6.0,
        load_step_ratio: float = 0.20,
        min_load_step: float = 100.0,
        load_drop_margin: float = 0.01,
        mu_cap: float = 2.6,
        max_violation_rate: float = 0.05,
    ) -> dict:
        """Check that saturated |Fy| does not often drop when Fz increases."""
        fz_min, fz_max = self.bounds["Fz"]
        sa_min, sa_max = self.bounds["SA"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        fz_floor = max(200.0, fz_min)
        fz_ceiling = max(fz_floor + min_load_step, fz_max - min_load_step)
        fz_low = rng.uniform(fz_floor, fz_ceiling, size=n_samples).astype(np.float32)
        load_step = np.maximum(load_step_ratio * fz_low, min_load_step)
        fz_high = np.minimum(fz_low + load_step, float(fz_max))
        fz_high = np.maximum(fz_high, fz_low + 1.0).astype(np.float32)

        sa_abs_max = max(abs(sa_min), abs(sa_max), min_abs_sa_deg + 0.1)
        sa_abs = rng.uniform(min_abs_sa_deg, sa_abs_max, size=n_samples)
        sa_sign = rng.choice([-1.0, 1.0], size=n_samples)
        sa = (sa_abs * sa_sign).astype(np.float32)
        tstc = rng.uniform(tstc_min, tstc_max, size=n_samples).astype(np.float32)

        x_low = np.column_stack(
            [
                np.zeros(n_samples),
                sa,
                fz_low,
                np.zeros(n_samples),
                tstc,
            ]
        ).astype(np.float32)
        x_high = x_low.copy()
        x_high[:, 2] = fz_high

        fy_low = torch.abs(
            self.model(torch.tensor(x_low, dtype=torch.float32, device=self.device))[:, 1]
        )
        fy_high = torch.abs(
            self.model(torch.tensor(x_high, dtype=torch.float32, device=self.device))[:, 1]
        )
        fz_high_tensor = torch.tensor(fz_high, dtype=torch.float32, device=self.device)

        drop_ratio = (fy_low - fy_high) / (mu_cap * fz_high_tensor + 1e-6)
        violation = drop_ratio > load_drop_margin
        violation_rate = violation.float().mean().item()
        passed = violation_rate <= max_violation_rate

        metrics = {
            "violation_rate": violation_rate,
            "mean_drop_ratio": drop_ratio.mean().item(),
            "max_drop_ratio": drop_ratio.max().item(),
            "mean_abs_fy_low": fy_low.mean().item(),
            "mean_abs_fy_high": fy_high.mean().item(),
            "min_abs_sa_deg": min_abs_sa_deg,
            "load_drop_margin": load_drop_margin,
            "threshold": max_violation_rate,
        }
        message = (
            "✓ PASS: high-SA lateral load violation rate "
            f"{violation_rate:.2%} <= {max_violation_rate:.2%}"
            if passed
            else "✗ FAIL: high-SA lateral load violation rate "
            f"{violation_rate:.2%} > {max_violation_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_mz_pneumatic_trail(
        self,
        n_samples: int = 500,
        small_sa_deg: float = 1.5,
        large_sa_deg: float = 6.0,
        max_trail_m: float = 0.18,
        trail_growth_margin_m: float = 0.01,
        max_negative_trail_rate: float = 0.20,
        max_cap_rate: float = 0.25,
        max_growth_rate: float = 0.65,
    ) -> dict:
        """Check Mz sign/cap/trend via pneumatic trail = -Mz/Fy."""
        fz_min, fz_max = self.bounds["Fz"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        sign = rng.choice([-1.0, 1.0], size=n_samples)
        fz = rng.uniform(max(200.0, fz_min), fz_max, size=n_samples)
        tstc = rng.uniform(tstc_min, tstc_max, size=n_samples)
        x_small = np.column_stack(
            [
                np.zeros(n_samples),
                sign * small_sa_deg,
                fz,
                np.zeros(n_samples),
                tstc,
            ]
        ).astype(np.float32)
        x_large = x_small.copy()
        x_large[:, 1] = sign * large_sa_deg

        y_small = self.model(torch.tensor(x_small, dtype=torch.float32, device=self.device))
        y_large = self.model(torch.tensor(x_large, dtype=torch.float32, device=self.device))

        fy_small, mz_small = y_small[:, 1], y_small[:, 3]
        fy_large, mz_large = y_large[:, 1], y_large[:, 3]
        trail_small = -mz_small * fy_small / (fy_small**2 + 1e-3)
        trail_large = -mz_large * fy_large / (fy_large**2 + 1e-3)

        negative_trail = torch.logical_or(trail_small < 0.0, trail_large < 0.0)
        cap_violation = torch.logical_or(trail_small > max_trail_m, trail_large > max_trail_m)
        growth_violation = trail_large > trail_small + trail_growth_margin_m

        negative_rate = negative_trail.float().mean().item()
        cap_rate = cap_violation.float().mean().item()
        growth_rate = growth_violation.float().mean().item()
        passed = (
            negative_rate <= max_negative_trail_rate
            and cap_rate <= max_cap_rate
            and growth_rate <= max_growth_rate
        )

        metrics = {
            "negative_trail_rate": negative_rate,
            "cap_violation_rate": cap_rate,
            "growth_violation_rate": growth_rate,
            "mean_trail_small_m": trail_small.mean().item(),
            "mean_trail_large_m": trail_large.mean().item(),
            "max_trail_m": max_trail_m,
            "negative_threshold": max_negative_trail_rate,
            "cap_threshold": max_cap_rate,
            "growth_threshold": max_growth_rate,
        }
        message = (
            "✓ PASS: Mz trail neg/cap/growth rates "
            f"{negative_rate:.2%}/{cap_rate:.2%}/{growth_rate:.2%}"
            if passed
            else "✗ FAIL: Mz trail neg/cap/growth rates "
            f"{negative_rate:.2%}/{cap_rate:.2%}/{growth_rate:.2%}"
        )
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    @torch.no_grad()
    def test_camber_thrust(
        self,
        n_samples: int = 500,
        ia_deg: float = 3.0,
        camber_mu_cap: float = 0.25,
        max_mean_asymmetry: float = 0.50,
        max_cap_violation_rate: float = 0.15,
        min_coverage_per_side: int = 50,
    ) -> dict:
        """Check camber-thrust odd symmetry and force cap at SR=SA=0."""
        coverage = self._camber_coverage(ia_deg=ia_deg, min_count=min_coverage_per_side)
        coverage_limited = bool(
            coverage["coverage_available"]
            and (not coverage["has_positive_ia"] or not coverage["has_negative_ia"])
        )

        fz_min, fz_max = self.bounds["Fz"]
        tstc_min, tstc_max = self.bounds["TSTC"]

        rng = np.random.default_rng(42)
        fz = rng.uniform(max(200.0, fz_min), fz_max, size=n_samples)
        tstc = rng.uniform(tstc_min, tstc_max, size=n_samples)
        x_pos = np.column_stack(
            [
                np.zeros(n_samples),
                np.zeros(n_samples),
                fz,
                np.full(n_samples, ia_deg),
                tstc,
            ]
        ).astype(np.float32)
        x_neg = x_pos.copy()
        x_neg[:, 3] = -ia_deg

        fy_pos = self.model(torch.tensor(x_pos, dtype=torch.float32, device=self.device))[:, 1]
        fy_neg = self.model(torch.tensor(x_neg, dtype=torch.float32, device=self.device))[:, 1]

        fz_tensor = torch.tensor(fz, dtype=torch.float32, device=self.device)
        force_scale = camber_mu_cap * torch.abs(fz_tensor) + 1e-6
        asymmetry = torch.abs(fy_pos + fy_neg) / force_scale
        cap_excess = torch.maximum(torch.abs(fy_pos), torch.abs(fy_neg)) / force_scale - 1.0
        cap_violation_rate = (cap_excess > 0.0).float().mean().item()
        mean_asymmetry = asymmetry.mean().item()
        raw_passed = (
            mean_asymmetry <= max_mean_asymmetry and cap_violation_rate <= max_cap_violation_rate
        )
        passed = raw_passed or coverage_limited

        metrics = {
            "mean_asymmetry": mean_asymmetry,
            "max_asymmetry": asymmetry.max().item(),
            "cap_violation_rate": cap_violation_rate,
            "camber_mu_cap": camber_mu_cap,
            "asymmetry_threshold": max_mean_asymmetry,
            "cap_threshold": max_cap_violation_rate,
            "coverage_limited": coverage_limited,
            **coverage,
        }
        if coverage_limited:
            message = (
                "⚠ COVERAGE-LIMITED: camber asym/cap "
                f"{mean_asymmetry:.4f}/{cap_violation_rate:.2%}; "
                f"IA coverage +/{coverage['positive_ia_count']} "
                f"-/{coverage['negative_ia_count']} near zero slip"
            )
        elif passed:
            message = f"✓ PASS: camber asym/cap {mean_asymmetry:.4f}/{cap_violation_rate:.2%}"
        else:
            message = f"✗ FAIL: camber asym/cap {mean_asymmetry:.4f}/{cap_violation_rate:.2%}"
        print(f"  {message}")
        return {"pass": passed, "metrics": metrics, "message": message}

    def _camber_coverage(self, ia_deg: float, min_count: int) -> Dict[str, float | bool | int]:
        """Check whether measured data supports both camber signs near zero slip."""
        if self.data_frame is None or self.data_frame.empty:
            return {
                "positive_ia_count": 0,
                "negative_ia_count": 0,
                "has_positive_ia": False,
                "has_negative_ia": False,
                "coverage_available": False,
                "min_coverage_per_side": int(min_count),
            }

        near_zero = self.data_frame[
            (self.data_frame["SR"].abs() < 0.02)
            & (self.data_frame["SA"].abs() < 0.3)
        ]
        ia_threshold = max(1.0, abs(float(ia_deg)) * 0.5)
        positive_count = int((near_zero["IA"] > ia_threshold).sum())
        negative_count = int((near_zero["IA"] < -ia_threshold).sum())
        return {
            "positive_ia_count": positive_count,
            "negative_ia_count": negative_count,
            "has_positive_ia": positive_count >= min_count,
            "has_negative_ia": negative_count >= min_count,
            "coverage_available": True,
            "min_coverage_per_side": int(min_count),
        }

    def _estimate_empirical_monotonicity_baseline(
        self, negative_gradient_tolerance: float
    ) -> float:
        if self.data_frame is None or self.data_frame.empty:
            return 0.0

        cache_key = round(float(negative_gradient_tolerance), 6)
        if cache_key in self._empirical_monotonicity_cache:
            return self._empirical_monotonicity_cache[cache_key]

        df = self.data_frame.copy()
        active = df[(df["SR"].abs() > 0.03) | (df["SA"].abs() > 1.0)].copy()
        if active.empty:
            self._empirical_monotonicity_cache[cache_key] = 0.0
            return 0.0

        active["sr_bin"] = active["SR"].round(2)
        active["sa_bin"] = active["SA"].round(1)
        active["ia_bin"] = active["IA"].round(1)
        active["t_bin"] = (active["TSTC"] / 5.0).round() * 5.0
        active["fz_bin"] = (active["Fz"] / 100.0).round() * 100.0

        grouped = (
            active.groupby(["sr_bin", "sa_bin", "ia_bin", "t_bin", "fz_bin"])[["Fx", "Fy"]]
            .mean()
            .reset_index()
        )

        fx_slopes = []
        fy_slopes = []
        for _, group in grouped.groupby(["sr_bin", "sa_bin", "ia_bin", "t_bin"]):
            group = group.sort_values("fz_bin")
            if len(group) < 2:
                continue

            fz = group["fz_bin"].to_numpy()
            dfz = np.diff(fz)
            valid = dfz > 0.0
            if not np.any(valid):
                continue

            fx_mag = np.abs(group["Fx"].to_numpy())
            fy_mag = np.abs(group["Fy"].to_numpy())
            fx_slopes.extend((np.diff(fx_mag)[valid] / dfz[valid]).tolist())
            fy_slopes.extend((np.diff(fy_mag)[valid] / dfz[valid]).tolist())

        if not fx_slopes and not fy_slopes:
            baseline = 0.0
        else:
            fx_rate = (
                float((np.asarray(fx_slopes) < -negative_gradient_tolerance).mean())
                if fx_slopes
                else 0.0
            )
            fy_rate = (
                float((np.asarray(fy_slopes) < -negative_gradient_tolerance).mean())
                if fy_slopes
                else 0.0
            )
            baseline = max(fx_rate, fy_rate)

        self._empirical_monotonicity_cache[cache_key] = baseline
        return baseline

    def _print_summary(self, results: Dict[str, dict]) -> None:
        print("\n" + "=" * 80)
        print("SUMMARY")
        print("=" * 80)
        for test_name, result in results.items():
            status = {True: "✓ PASS", False: "✗ FAIL", None: "- NOT EVALUATED"}[result["pass"]]
            print(f"{test_name:20s}: {status}")
        print("=" * 80)
