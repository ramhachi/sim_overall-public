import json
from pathlib import Path

import pandas as pd
import torch

from tire_pinn.export.cpp_exporter import CPPExporter, _json_safe
from tire_pinn.models.pinn_model import (
    NormalizedTirePINN,
    PhysicsEnforcedTireModel,
    TirePINN,
)
from tire_pinn.validation.physics_tests import PhysicsValidator


class AffineStubModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 3000.0 + 5000.0 * x[:, 0]
        fy = 1000.0 + 1000.0 * x[:, 1]
        mx = torch.full_like(fx, 10.0)
        mz = 200.0 + 50.0 * x[:, 1]
        return torch.stack([fx, fy, mx, mz], dim=1)


class MildNegativeGradientModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 300.0 - 0.05 * x[:, 2]
        fy = 400.0 - 0.05 * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


class StrongNegativeGradientModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 300.0 - 0.3 * x[:, 2]
        fy = 400.0 - 0.3 * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


class SignSymmetricStubModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 1000.0 * x[:, 0]
        fy = 100.0 * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = 10.0 * x[:, 1]
        return torch.stack([fx, fy, mx, mz], dim=1)


class CamberRuntimeStubModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        fy = 120.0 * x[:, 3]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


class ExtendedPhysicsGoodModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 4.0 * x[:, 2] * x[:, 0] / (1.0 + 0.20 * torch.abs(x[:, 1]))
        fy_slip = 0.12 * x[:, 2] * x[:, 1] / (1.0 + 3.0 * torch.abs(x[:, 0]))
        fy = fy_slip + 0.04 * x[:, 3] * x[:, 2]
        mx = torch.zeros_like(fx)
        trail = 0.08 / (1.0 + torch.abs(x[:, 1]) / 3.0)
        mz = -trail * fy_slip
        return torch.stack([fx, fy, mx, mz], dim=1)


class CombinedSlipBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 4.0 * x[:, 2] * x[:, 0] + 2.0 * x[:, 2] * torch.abs(x[:, 1])
        fy = 0.12 * x[:, 2] * x[:, 1] + 2.0 * x[:, 2] * torch.abs(x[:, 0])
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class StiffnessLoadBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = (10_000.0 - 4.0 * x[:, 2]) * x[:, 0]
        fy = (5_000.0 - 2.0 * x[:, 2]) * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class HighSaLateralLoadBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        load_factor = torch.relu(1500.0 - x[:, 2])
        fy = 3.0 * load_factor * torch.tanh(x[:, 1] / 4.0)
        mx = torch.zeros_like(fx)
        mz = -0.04 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class TrailBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = 4.0 * x[:, 2] * x[:, 0]
        fy = 0.12 * x[:, 2] * x[:, 1]
        mx = torch.zeros_like(fx)
        mz = 0.08 * fy
        return torch.stack([fx, fy, mx, mz], dim=1)


class CamberBadModel(torch.nn.Module):
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        fx = torch.zeros_like(x[:, 0])
        fy = 0.20 * x[:, 2] + 0.50 * x[:, 3] * x[:, 2]
        mx = torch.zeros_like(fx)
        mz = torch.zeros_like(fx)
        return torch.stack([fx, fy, mx, mz], dim=1)


def test_tire_pinn_default_architecture_is_low_depth_multi_head():
    model = TirePINN()

    assert model.architecture == "multi_head"
    assert model.hidden_dims == []
    assert list(model.heads.keys()) == ["fx", "fy_mz", "mx"]
    assert model.head_hidden_dims["fx"] == [128, 128, 64, 64, 32]
    assert model.head_hidden_dims["fy_mz"] == [128, 128, 64, 64, 32]
    assert model.head_hidden_dims["mx"] == [64, 64, 32]


def test_physics_enforced_tire_model_enforces_zero_slip_and_friction_cap():
    wrapped = PhysicsEnforcedTireModel(
        AffineStubModel(),
        friction_cap_mu=2.0,
        zero_slip_sr_scale=0.03,
        zero_slip_sa_scale_deg=0.5,
        symmetry_sr_scale=0.05,
        symmetry_ia_scale_deg=0.75,
    )

    zero_input = torch.tensor([[0.0, 0.0, 1000.0, 0.0, 60.0]], dtype=torch.float32)
    zero_output = wrapped(zero_input)[0]
    assert torch.isclose(zero_output[0], torch.tensor(0.0))
    assert torch.isclose(zero_output[1], torch.tensor(0.0))
    assert torch.isclose(zero_output[3], torch.tensor(0.0))
    assert torch.isclose(zero_output[2], torch.tensor(10.0))

    high_slip_input = torch.tensor([[0.2, 2.0, 1000.0, 0.0, 60.0]], dtype=torch.float32)
    high_slip_output = wrapped(high_slip_input)[0]
    combined_force = torch.sqrt(high_slip_output[0] ** 2 + high_slip_output[1] ** 2)
    assert torch.isclose(combined_force, torch.tensor(2000.0), atol=1e-4)


def test_physics_enforced_tire_model_projects_symmetry_near_zero_sr_and_ia():
    wrapped = PhysicsEnforcedTireModel(
        AffineStubModel(),
        symmetry_sr_scale=0.05,
        symmetry_ia_scale_deg=0.75,
    )

    pos = torch.tensor([[0.0, 2.0, 1000.0, 0.0, 60.0]], dtype=torch.float32)
    neg = torch.tensor([[0.0, -2.0, 1000.0, 0.0, 60.0]], dtype=torch.float32)
    pos_out = wrapped(pos)[0]
    neg_out = wrapped(neg)[0]

    assert torch.isclose(pos_out[0], neg_out[0], atol=1e-4)
    assert torch.isclose(pos_out[1], -neg_out[1], atol=1e-4)
    assert torch.isclose(pos_out[3], -neg_out[3], atol=1e-4)


def test_cpp_exporter_runtime_json_contains_postprocess(tmp_path: Path):
    base_model = NormalizedTirePINN(TirePINN(hidden_dims=[8], input_dim=5, output_dim=4))
    base_model.set_normalization_params(
        input_mean=torch.zeros(5),
        input_std=torch.ones(5),
        output_mean=torch.zeros(4),
        output_std=torch.ones(4),
    )
    wrapped = PhysicsEnforcedTireModel(
        base_model,
        friction_cap_mu=2.6,
        zero_slip_sr_scale=0.03,
        zero_slip_sa_scale_deg=0.5,
    )

    exporter = CPPExporter(wrapped, tmp_path)
    exporter._export_runtime_model(tmp_path / "pinn_runtime.json")

    runtime_json = json.loads((tmp_path / "pinn_runtime.json").read_text())
    assert runtime_json["postprocess"]["enabled"] is True
    assert runtime_json["postprocess"]["friction_cap_mu"] == 2.6
    assert runtime_json["postprocess"]["zero_slip_sr_scale"] == 0.03
    assert runtime_json["postprocess"]["zero_slip_sa_scale_deg"] == 0.5
    assert runtime_json["postprocess"]["symmetry_sr_scale"] == 0.05
    assert runtime_json["postprocess"]["symmetry_ia_scale_deg"] == 0.75
    assert runtime_json["input"]["units"] == ["ratio", "deg", "N", "deg", "degC"]
    assert runtime_json["input_scope"]["not_modeled_inputs"] == [
        "inflation_pressure",
        "road_surface",
        "wear_state",
    ]


def test_cpp_exporter_runtime_json_supports_multi_head(tmp_path: Path):
    base_model = NormalizedTirePINN(
        TirePINN(
            hidden_dims=[],
            input_dim=5,
            output_dim=4,
            architecture="multi_head",
            head_hidden_dims={
                "fx": [8],
                "fy_mz": [8],
                "mx": [4],
            },
        )
    )
    base_model.set_normalization_params(
        input_mean=torch.zeros(5),
        input_std=torch.ones(5),
        output_mean=torch.zeros(4),
        output_std=torch.ones(4),
    )

    exporter = CPPExporter(base_model, tmp_path)
    exporter._export_runtime_model(tmp_path / "pinn_runtime.json")

    runtime_json = json.loads((tmp_path / "pinn_runtime.json").read_text())
    assert runtime_json["format"] == "pinn_runtime_v2"
    assert runtime_json["architecture"]["type"] == "multi_head"
    assert runtime_json["trunk_layers"] == []
    assert [head["name"] for head in runtime_json["heads"]] == ["fx", "fy_mz", "mx"]
    assert [head["output_indices"] for head in runtime_json["heads"]] == [[0], [1, 3], [2]]
    assert runtime_json["heads"][0]["layers"][-1]["type"] == "linear"


def test_cpp_exporter_test_vectors_respect_input_bounds(tmp_path: Path):
    base_model = NormalizedTirePINN(TirePINN(hidden_dims=[8], input_dim=5, output_dim=4))
    base_model.set_normalization_params(
        input_mean=torch.zeros(5),
        input_std=torch.ones(5),
        output_mean=torch.zeros(4),
        output_std=torch.ones(4),
    )
    exporter = CPPExporter(base_model, tmp_path)
    input_bounds = {
        "SR": {"min": -0.1, "max": 0.1},
        "SA": {"min": -2.0, "max": 2.0},
        "Fz": {"min": 500.0, "max": 1500.0},
        "IA": {"min": -1.0, "max": 1.0},
        "TSTC": {"min": 30.0, "max": 70.0},
    }

    exporter.export_test_vectors(
        n_samples=64,
        output_path=tmp_path / "test_vectors.json",
        input_bounds=input_bounds,
    )

    test_vectors = json.loads((tmp_path / "test_vectors.json").read_text())
    tolerance = 1e-6
    for row in test_vectors["inputs"]:
        assert (
            input_bounds["SR"]["min"] - tolerance <= row[0] <= input_bounds["SR"]["max"] + tolerance
        )
        assert (
            input_bounds["SA"]["min"] - tolerance <= row[1] <= input_bounds["SA"]["max"] + tolerance
        )
        assert (
            input_bounds["Fz"]["min"] - tolerance <= row[2] <= input_bounds["Fz"]["max"] + tolerance
        )
        assert (
            input_bounds["IA"]["min"] - tolerance <= row[3] <= input_bounds["IA"]["max"] + tolerance
        )
        assert (
            input_bounds["TSTC"]["min"] - tolerance
            <= row[4]
            <= input_bounds["TSTC"]["max"] + tolerance
        )


def test_cpp_exporter_default_test_vectors_include_units_and_dense_coverage(tmp_path: Path):
    base_model = NormalizedTirePINN(TirePINN(hidden_dims=[8], input_dim=5, output_dim=4))
    base_model.set_normalization_params(
        input_mean=torch.zeros(5),
        input_std=torch.ones(5),
        output_mean=torch.zeros(4),
        output_std=torch.ones(4),
    )
    exporter = CPPExporter(base_model, tmp_path)

    exporter.export_test_vectors(output_path=tmp_path / "test_vectors.json")

    test_vectors = json.loads((tmp_path / "test_vectors.json").read_text())
    assert test_vectors["n_samples"] == CPPExporter.DEFAULT_TEST_VECTOR_COUNT
    assert len(test_vectors["inputs"]) == CPPExporter.DEFAULT_TEST_VECTOR_COUNT
    assert test_vectors["input_units"]["SA"] == "deg"
    assert test_vectors["input_units"]["IA"] == "deg"
    assert (
        test_vectors["generation"]["strategy"] == "fixed_zero_slip_sign_symmetry_bounds_plus_random"
    )


def test_cpp_exporter_registry_preserves_export_history(tmp_path: Path):
    base_model = NormalizedTirePINN(TirePINN(hidden_dims=[8], input_dim=5, output_dim=4))
    exporter = CPPExporter(base_model, tmp_path / "TestTire")
    manifest = {
        "tire_name": "TestTire",
        "export_id": "20260515T000000Z",
        "deployable": True,
        "exported_at": "2026-05-15T00:00:00+00:00",
        "runtime_files": {"version_manifest": "manifests/manifest_20260515T000000Z.json"},
        "validation": {
            "r2": {"r2_fx": 0.96, "r2_fy": 0.97},
            "r2_mean": 0.965,
        },
    }
    exporter._update_registry(manifest)
    second_manifest = {
        **manifest,
        "export_id": "20260515T010000Z",
        "exported_at": "2026-05-15T01:00:00+00:00",
        "runtime_files": {"version_manifest": "manifests/manifest_20260515T010000Z.json"},
    }
    exporter._update_registry(second_manifest)

    registry = json.loads((tmp_path / "registry.json").read_text())
    assert registry["models"]["TestTire"]["export_id"] == "20260515T010000Z"
    assert [entry["export_id"] for entry in registry["history"]["TestTire"]] == [
        "20260515T000000Z",
        "20260515T010000Z",
    ]


def test_physics_enforced_tire_model_preserves_force_sign_conventions():
    wrapped = PhysicsEnforcedTireModel(
        SignSymmetricStubModel(),
        friction_cap_mu=10.0,
        zero_slip_sr_scale=0.03,
        zero_slip_sa_scale_deg=0.5,
        symmetry_sr_scale=0.05,
        symmetry_ia_scale_deg=0.75,
        sa_zero_fy_scale_deg=0.3,
    )

    sr_pair = torch.tensor(
        [[0.10, 0.0, 1000.0, 0.0, 60.0], [-0.10, 0.0, 1000.0, 0.0, 60.0]],
        dtype=torch.float32,
    )
    sa_pair = torch.tensor(
        [[0.0, 3.0, 1000.0, 0.0, 60.0], [0.0, -3.0, 1000.0, 0.0, 60.0]],
        dtype=torch.float32,
    )

    sr_out = wrapped(sr_pair)
    sa_out = wrapped(sa_pair)

    assert sr_out[0, 0] > 0.0
    assert sr_out[1, 0] < 0.0
    assert sa_out[0, 1] > 0.0
    assert sa_out[1, 1] < 0.0
    assert sa_out[0, 3] > 0.0
    assert sa_out[1, 3] < 0.0
    assert torch.isclose(sa_out[0, 1], -sa_out[1, 1], atol=1e-4)


def test_runtime_postprocess_preserves_camber_thrust_at_zero_slip_angle():
    wrapped = PhysicsEnforcedTireModel(
        CamberRuntimeStubModel(),
        friction_cap_mu=10.0,
        zero_slip_sr_scale=0.03,
        zero_slip_sa_scale_deg=0.5,
        symmetry_sr_scale=0.05,
        symmetry_ia_scale_deg=0.75,
        sa_zero_fy_scale_deg=0.3,
    )

    zero_camber = torch.tensor([[0.0, 0.0, 1000.0, 0.0, 60.0]], dtype=torch.float32)
    camber_pair = torch.tensor(
        [[0.0, 0.0, 1000.0, 3.0, 60.0], [0.0, 0.0, 1000.0, -3.0, 60.0]],
        dtype=torch.float32,
    )

    zero_out = wrapped(zero_camber)
    camber_out = wrapped(camber_pair)

    assert torch.isclose(zero_out[0, 1], torch.tensor(0.0), atol=1e-4)
    assert camber_out[0, 1] > 300.0
    assert camber_out[1, 1] < -300.0
    assert torch.isclose(camber_out[0, 1], -camber_out[1, 1], atol=1e-3)


def test_runtime_sa_zero_fy_correction_preserves_cornering_stiffness():
    wrapped = PhysicsEnforcedTireModel(
        AffineStubModel(),
        friction_cap_mu=10.0,
        zero_slip_sr_scale=0.03,
        zero_slip_sa_scale_deg=0.5,
        symmetry_sr_scale=0.05,
        symmetry_ia_scale_deg=0.75,
        sa_zero_fy_scale_deg=0.3,
    )

    h = 0.05
    pair = torch.tensor(
        [[0.0, h, 1000.0, 0.0, 60.0], [0.0, -h, 1000.0, 0.0, 60.0]],
        dtype=torch.float32,
    )

    out = wrapped(pair)
    slope = (out[0, 1] - out[1, 1]) / (2.0 * h)

    assert abs(slope.item()) > 900.0
    assert torch.isclose(out[0, 1], -out[1, 1], atol=1e-4)


def test_monotonicity_validator_respects_negative_gradient_tolerance():
    validator = PhysicsValidator(MildNegativeGradientModel(), device="cpu")
    result = validator.test_monotonicity(
        n_samples=64,
        max_violation_rate=0.10,
        negative_gradient_tolerance=0.10,
    )

    assert result["pass"] is True
    assert result["metrics"]["raw_negative_rate_fx"] == 1.0
    assert result["metrics"]["raw_negative_rate_fy"] == 1.0
    assert result["metrics"]["violation_rate_fx"] == 0.0
    assert result["metrics"]["violation_rate_fy"] == 0.0


def test_monotonicity_validator_uses_empirical_baseline_from_data():
    data_frame = pd.DataFrame(
        {
            "SR": [0.05, 0.05, 0.05, 0.05],
            "SA": [2.0, 2.0, 2.0, 2.0],
            "Fz": [500.0, 600.0, 700.0, 800.0],
            "IA": [0.0, 0.0, 0.0, 0.0],
            "TSTC": [60.0, 60.0, 60.0, 60.0],
            "Fx": [300.0, 250.0, 200.0, 150.0],
            "Fy": [400.0, 350.0, 300.0, 250.0],
            "Mx": [0.0, 0.0, 0.0, 0.0],
            "Mz": [0.0, 0.0, 0.0, 0.0],
        }
    )
    validator = PhysicsValidator(StrongNegativeGradientModel(), data_frame=data_frame, device="cpu")
    result = validator.test_monotonicity(
        n_samples=32,
        max_violation_rate=0.10,
        negative_gradient_tolerance=0.10,
    )

    assert result["pass"] is True
    assert result["metrics"]["empirical_baseline"] == 1.0
    assert result["metrics"]["effective_threshold"] == 1.0


def test_physics_validator_run_all_includes_extended_checks():
    validator = PhysicsValidator(ExtendedPhysicsGoodModel(), device="cpu")

    results = validator.run_all_tests(
        max_combined_slip_violation_rate=0.05,
        max_stiffness_load_violation_rate=0.05,
        max_high_sa_lateral_load_violation_rate=0.05,
        max_mz_negative_trail_rate=0.05,
        max_mz_trail_cap_rate=0.05,
        max_mz_trail_growth_rate=0.05,
        max_camber_asymmetry=0.05,
        max_camber_cap_violation_rate=0.05,
    )

    assert "combined_slip" in results
    assert "stiffness_load" in results
    assert "high_sa_lateral_load" in results
    assert "mz_trail" in results
    assert "camber_thrust" in results
    assert all(result["pass"] for result in results.values())


def test_extended_physics_validators_detect_targeted_failures():
    assert (
        PhysicsValidator(CombinedSlipBadModel(), device="cpu").test_combined_slip(
            n_samples=64, max_violation_rate=0.10
        )["pass"]
        is False
    )
    assert (
        PhysicsValidator(StiffnessLoadBadModel(), device="cpu").test_stiffness_load_sensitivity(
            n_samples=64, max_violation_rate=0.10
        )["pass"]
        is False
    )
    assert (
        PhysicsValidator(
            HighSaLateralLoadBadModel(), device="cpu"
        ).test_high_sa_lateral_load_sensitivity(n_samples=64, max_violation_rate=0.10)["pass"]
        is False
    )
    assert (
        PhysicsValidator(TrailBadModel(), device="cpu").test_mz_pneumatic_trail(
            n_samples=64, max_negative_trail_rate=0.10
        )["pass"]
        is False
    )
    assert (
        PhysicsValidator(CamberBadModel(), device="cpu").test_camber_thrust(
            n_samples=64, max_mean_asymmetry=0.10
        )["pass"]
        is False
    )


def test_camber_validator_marks_one_sided_measured_camber_as_coverage_limited():
    data_frame = pd.DataFrame(
        {
            "SR": [0.0] * 64,
            "SA": [0.0] * 64,
            "Fz": [1000.0] * 64,
            "IA": [3.0] * 64,
            "TSTC": [60.0] * 64,
            "Fx": [0.0] * 64,
            "Fy": [0.0] * 64,
            "Mx": [0.0] * 64,
            "Mz": [0.0] * 64,
        }
    )

    result = PhysicsValidator(CamberBadModel(), data_frame=data_frame, device="cpu").test_camber_thrust(
        n_samples=64,
        max_mean_asymmetry=0.10,
        min_coverage_per_side=20,
    )

    assert result["pass"] is True
    assert result["metrics"]["coverage_limited"] is True
    assert result["metrics"]["has_positive_ia"] is True
    assert result["metrics"]["has_negative_ia"] is False


def test_manifest_metrics_are_strict_json():
    manifest = {"validation": {"metrics": {"violation_rate": float("nan"), "rates": [1.0, float("inf")]}}}

    text = json.dumps(_json_safe(manifest), allow_nan=False)

    assert json.loads(text) == {"validation": {"metrics": {"violation_rate": None, "rates": [1.0, None]}}}
