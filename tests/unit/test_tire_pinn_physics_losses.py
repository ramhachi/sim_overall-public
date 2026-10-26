import pytest
import torch

from tire_pinn.losses.physics_losses import (
    CamberThrustLoss,
    CombinedSlipLoss,
    FrictionCircleLoss,
    FySaZeroLoss,
    HighSaLateralLoadSensitivityLoss,
    MonotonicityLoss,
    MzPneumaticTrailLoss,
    PhysicsLossManager,
    SectionSmoothnessLoss,
    StiffnessLoadSensitivityLoss,
    SymmetryLoss,
    ZeroSlipLoss,
)
from tire_pinn.training.collocation_sampler import CollocationSampler


class CombinedFriendlyModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 20_000.0 * x[:, 0] / (1.0 + 0.25 * torch.abs(x[:, 1]))
        fy = 100.0 * x[:, 1] / (1.0 + 4.0 * torch.abs(x[:, 0]))
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class CombinedViolatingModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 20_000.0 * x[:, 0] + 5_000.0 * torch.abs(x[:, 1])
        fy = 100.0 * x[:, 1] + 10_000.0 * torch.abs(x[:, 0])
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class LoadSensitiveGoodModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 20.0 * x[:, 2] * x[:, 0]
        fy = 4.0 * x[:, 2] * x[:, 1]
        mx = torch.zeros_like(fx)
        trail = 0.08 / (1.0 + torch.abs(x[:, 1]) / 3.0)
        mz = -trail * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class LoadSensitiveBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = (60_000.0 - 10.0 * x[:, 2]) * x[:, 0]
        fy = (30_000.0 - 5.0 * x[:, 2]) * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class HighSaLateralBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        load_factor = torch.relu(3000.0 - x[:, 2])
        fy = 3.0 * load_factor * torch.tanh(x[:, 1] / 4.0)
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class BadTrailModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 20.0 * x[:, 2] * x[:, 0]
        fy = 4.0 * x[:, 2] * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = 0.08 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class RuntimeProjectedBadTrailModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 20.0 * x[:, 2] * x[:, 0]
        fy = 4.0 * x[:, 2] * x[:, 1]
        mx = torch.zeros_like(fx)
        mz_good = -0.04 * fy
        mz_bad_mirror = 0.08 * fy
        mz = torch.where(x[:, 1] >= 0.0, mz_good, mz_bad_mirror)
        return torch.stack([fx, fy, mx, mz], dim=1)


class CamberGoodModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        fy = 0.04 * x[:, 3] * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


class CamberBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        fy = 0.20 * x[:, 2] + 0.50 * x[:, 3] * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


class SmoothSectionModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 1000.0 * x[:, 0]
        fy = 100.0 * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = -4.0 * x[:, 1]
        return torch.stack([fx, fy, mx, mz], dim=1)


class WavySectionModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 1000.0 * x[:, 0] + 500.0 * torch.sin(80.0 * x[:, 0])
        fy = 100.0 * x[:, 1] + 600.0 * torch.sin(2.5 * x[:, 1])
        mx = torch.zeros_like(fx)
        mz = -4.0 * x[:, 1] + 25.0 * torch.sin(2.0 * x[:, 1])
        return torch.stack([fx, fy, mx, mz], dim=1)


def _base_collocation() -> torch.Tensor:
    return torch.tensor(
        [
            [0.10, 5.0, 1000.0, 0.0, 60.0],
            [-0.08, -4.0, 1200.0, 0.0, 70.0],
            [0.06, -6.0, 1500.0, 0.0, 50.0],
        ],
        dtype=torch.float32,
    )


def test_combined_slip_loss_penalizes_force_growth_beyond_pure_slip():
    loss_fn = CombinedSlipLoss(mu_cap=2.6, margin=0.03)
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(CombinedFriendlyModel(), colloc)
    bad_loss, bad_metrics = loss_fn(CombinedViolatingModel(), colloc)

    assert good_loss.item() == 0.0
    assert bad_loss.item() > 1.0
    assert good_metrics["combined_slip_violation_rate_fx"] == 0.0
    assert bad_metrics["combined_slip_violation_rate_fx"] == 1.0


def test_stiffness_load_loss_penalizes_load_decreasing_small_slip_stiffness():
    loss_fn = StiffnessLoadSensitivityLoss()
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(LoadSensitiveGoodModel(), colloc)
    bad_loss, bad_metrics = loss_fn(LoadSensitiveBadModel(), colloc)

    assert good_loss.item() == 0.0
    assert bad_loss.item() > 1.0
    assert good_metrics["stiffness_load_violation_rate_kx"] == 0.0
    assert bad_metrics["stiffness_load_violation_rate_kx"] == 1.0


def test_high_sa_lateral_load_loss_penalizes_saturated_fy_load_drop():
    loss_fn = HighSaLateralLoadSensitivityLoss()
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(LoadSensitiveGoodModel(), colloc)
    bad_loss, bad_metrics = loss_fn(HighSaLateralBadModel(), colloc)

    assert good_loss.item() == 0.0
    assert bad_loss.item() > 0.0
    assert good_metrics["high_sa_lateral_load_violation_rate"] == 0.0
    assert bad_metrics["high_sa_lateral_load_violation_rate"] == 1.0


def test_mz_trail_loss_penalizes_negative_pneumatic_trail():
    loss_fn = MzPneumaticTrailLoss()
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(LoadSensitiveGoodModel(), colloc)
    bad_loss, bad_metrics = loss_fn(BadTrailModel(), colloc)

    assert good_loss.item() == 0.0
    assert bad_loss.item() > 0.0
    assert good_metrics["mz_negative_trail_rate"] == 0.0
    assert bad_metrics["mz_negative_trail_rate"] == 1.0


def test_mz_trail_loss_matches_runtime_odd_projection():
    loss_fn = MzPneumaticTrailLoss(odd_loss_weight=0.0)
    colloc = torch.tensor(
        [
            [0.0, 3.0, 800.0, 0.0, 50.0],
            [0.0, 6.0, 1000.0, 0.0, 50.0],
        ],
        dtype=torch.float32,
    )

    bad_loss, bad_metrics = loss_fn(RuntimeProjectedBadTrailModel(), colloc)

    assert bad_loss.item() > 0.0
    assert bad_metrics["mz_shape_negative_trail_rate"] == 1.0


def test_camber_thrust_loss_enforces_odd_response_and_force_cap():
    loss_fn = CamberThrustLoss()
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(CamberGoodModel(), colloc)
    bad_loss, bad_metrics = loss_fn(CamberBadModel(), colloc)

    assert good_loss.item() == 0.0
    assert bad_loss.item() > 1.0
    assert good_metrics["camber_cap_violation_rate"] == 0.0
    assert bad_metrics["camber_cap_violation_rate"] == 1.0


def test_section_smoothness_loss_penalizes_wavy_pure_sections():
    loss_fn = SectionSmoothnessLoss(force_deadband=1.0e-8, mz_deadband=1.0e-8)
    colloc = _base_collocation()

    good_loss, good_metrics = loss_fn(SmoothSectionModel(), colloc)
    bad_loss, bad_metrics = loss_fn(WavySectionModel(), colloc)

    assert good_loss.item() < 1.0e-12
    assert bad_loss.item() > 0.0
    assert good_metrics["smoothness_fx_violation_rate"] == 0.0
    assert bad_metrics["smoothness_fx_violation_rate"] > 0.0


def test_physics_loss_manager_includes_new_constraint_metrics():
    manager = PhysicsLossManager(
        zero_slip_loss=ZeroSlipLoss(),
        symmetry_loss=SymmetryLoss(),
        friction_loss=FrictionCircleLoss(),
        monotonicity_loss=MonotonicityLoss(),
        fy_sa_zero_loss=FySaZeroLoss(),
        combined_slip_loss=CombinedSlipLoss(),
        stiffness_load_loss=StiffnessLoadSensitivityLoss(),
        high_sa_lateral_load_loss=HighSaLateralLoadSensitivityLoss(),
        mz_trail_loss=MzPneumaticTrailLoss(),
        camber_thrust_loss=CamberThrustLoss(),
        section_smoothness_loss=SectionSmoothnessLoss(),
    )
    colloc = _base_collocation()
    zero = colloc.clone()
    zero[:, 0] = 0.0
    zero[:, 1] = 0.0
    zero[:, 3] = 0.0

    _, metrics = manager.compute_all(
        model=LoadSensitiveGoodModel(),
        colloc_zero=zero,
        colloc_sym=colloc,
        colloc_fric=colloc,
        colloc_mono=colloc.clone().requires_grad_(True),
        colloc_fy_sa_zero=zero,
        colloc_combined=colloc,
        colloc_stiffness=colloc,
        colloc_high_sa_lateral_load=colloc,
        colloc_mz_trail=colloc,
        colloc_camber=colloc,
        colloc_smoothness=colloc,
        weights={
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
        },
    )

    assert "combined_slip_loss" in metrics
    assert "stiffness_load_loss" in metrics
    assert "high_sa_lateral_load_loss" in metrics
    assert "mz_trail_loss" in metrics
    assert "camber_thrust_loss" in metrics
    assert "section_smoothness_loss" in metrics
    assert "physics_loss_total" in metrics


def test_collocation_sampler_returns_new_constraint_batches():
    sampler = CollocationSampler(
        input_mean=torch.zeros(5),
        input_std=torch.ones(5),
        device="cpu",
    )

    batch = sampler.sample_batch(
        n_zero=2,
        n_sym=3,
        n_fric=4,
        n_mono=5,
        n_fy_sa_zero=6,
        n_combined=7,
        n_stiffness=8,
        n_high_sa_lateral=9,
        n_mz_trail=9,
        n_camber=10,
        n_smoothness=11,
    )

    assert batch["combined_slip"].shape == (7, 5)
    assert batch["stiffness"].shape == (8, 5)
    assert batch["high_sa_lateral"].shape == (9, 5)
    assert batch["mz_trail"].shape == (9, 5)
    assert batch["camber"].shape == (10, 5)
    assert batch["smoothness"].shape == (11, 5)
    assert torch.all(batch["zero"][:, 3] == 0.0)
    assert torch.all(batch["sa_zero_fy"][:, 3] == 0.0)
    assert torch.all(batch["combined_slip"][:, 0].abs() > 0.0)
    assert torch.all(batch["combined_slip"][:, 1].abs() > 0.0)
    assert torch.all(batch["high_sa_lateral"][:, 0] == 0.0)
    assert torch.all(batch["high_sa_lateral"][:, 1].abs() >= 6.0)
    assert torch.all(batch["smoothness"][:, 3] == 0.0)


class ZeroSlipSeparatedModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        n = x.shape[0]
        fx = torch.full((n,), 4.0)
        fy = torch.full((n,), 2.0)
        mx = torch.zeros(n)
        mz = torch.full((n,), 20.0)
        return torch.stack([fx, fy, mx, mz], dim=1)


def test_zero_slip_loss_uses_separate_force_and_moment_scales():
    loss_fn = ZeroSlipLoss(force_threshold=2.0, moment_threshold=5.0)
    batch = torch.tensor([[0.0, 0.0, 1000.0, 0.0, 25.0]])
    loss, metrics = loss_fn(ZeroSlipSeparatedModel(), batch)

    expected = (4.0 / 2.0) ** 2 + (2.0 / 2.0) ** 2 + (20.0 / 5.0) ** 2
    assert torch.isclose(loss, torch.tensor(expected))
    assert metrics["zero_slip_force_threshold"] == 2.0
    assert metrics["zero_slip_moment_threshold"] == 5.0


class CamberFxModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = x[:, 3] * 100.0
        zero = torch.zeros_like(fx)
        return torch.stack([fx, zero, zero, zero], dim=1)


def test_camber_thrust_loss_penalizes_zero_slip_fx():
    loss_fn = CamberThrustLoss(ia_deg=3.0, fx_threshold_n=50.0)
    batch = torch.tensor([[0.0, 0.0, 800.0, 0.0, 60.0]])

    loss, metrics = loss_fn(CamberFxModel(), batch)

    assert metrics["camber_fx_max"] == pytest.approx(300.0)
    assert metrics["camber_fx_loss"] > 0.0
    assert loss > 0.0
