import pandas as pd
import torch

from tire_pinn.main_pinn_train import summarize_validation_results
from tire_pinn.validation.physics_tests import PhysicsValidator


class ZeroForceModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return torch.zeros((x.shape[0], 4), dtype=x.dtype, device=x.device)


class OverLimitLateralModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros(x.shape[0], dtype=x.dtype, device=x.device)
        fy = 3.0 * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


def _measured_frame() -> pd.DataFrame:
    return pd.DataFrame(
        {
            "SR": [0.0, 0.05, -0.05, 0.1],
            "SA": [0.0, 2.0, -2.0, 4.0],
            "Fz": [1000.0, 1200.0, 900.0, 1100.0],
            "IA": [0.0, 0.0, 0.0, 0.0],
            "TSTC": [25.0, 30.0, 35.0, 40.0],
        }
    )


def test_measured_friction_circle_uses_holdout_distribution():
    frame = _measured_frame()
    good = PhysicsValidator(ZeroForceModel(), data_frame=frame)
    result = good.test_measured_friction_circle(max_violation_rate=0.0)
    assert result["pass"]
    assert result["metrics"]["n_samples"] == len(frame)

    bad = PhysicsValidator(OverLimitLateralModel(), data_frame=frame)
    result = bad.test_measured_friction_circle(max_violation_rate=0.0, mu_cap=2.0)
    assert not result["pass"]
    assert result["metrics"]["violation_rate"] == 1.0


def test_deployable_gate_requires_mz_r2():
    physics = {
        "friction_circle": {
            "pass": True,
            "metrics": {},
            "message": "ok",
        }
    }
    summary = summarize_validation_results(
        tire_name="test",
        dataset_info={"has_db": True, "has_cr": True},
        r2_dict={"r2_fx": 0.95, "r2_fy": 0.95, "r2_mx": 0.95, "r2_mz": 0.50},
        physics_results=physics,
        r2_target=0.90,
        mz_r2_target=0.80,
    )
    assert not summary["deployable"]
    assert summary["mz_r2"] == 0.50
    assert any("Mz R²" in reason for reason in summary["deployment_reasons"])


class CombinedBoostModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros(x.shape[0], dtype=x.dtype, device=x.device)
        fy = x[:, 2] * (1.0 + 2.0 * torch.abs(x[:, 0]))
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


def test_measured_combined_slip_uses_holdout_rows():
    frame = _measured_frame()
    good = PhysicsValidator(ZeroForceModel(), data_frame=frame)
    result = good.test_measured_combined_slip(max_violation_rate=0.0)
    assert result["pass"]
    assert result["metrics"]["n_samples"] == 3

    bad = PhysicsValidator(CombinedBoostModel(), data_frame=frame)
    result = bad.test_measured_combined_slip(max_violation_rate=0.0, margin=0.0, mu_cap=2.0)
    assert not result["pass"]
    assert result["metrics"]["violation_rate"] > 0.0
