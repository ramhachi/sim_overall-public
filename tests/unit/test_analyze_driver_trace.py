import importlib.util
import math
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[2] / "scripts" / "analyze_driver_trace.py"
SPEC = importlib.util.spec_from_file_location("analyze_driver_trace", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_actuator_diagnostics_deduplicate_outer_sequence_and_align_errors() -> None:
    rows = []
    for sequence in range(1, 121):
        error = 0.001 if sequence % 2 else -0.001
        actual = -0.10 if sequence % 2 else 0.10
        for repeat in range(2):
            rows.append(
                {
                    "time": 0.001 * len(rows),
                    "lpv_outer_sequence": float(sequence),
                    "steering_prediction_error_valid": 1.0,
                    "lpv_prediction_error": error,
                    "actual_delta_tire": actual,
                    "inner_target_delta_tire": actual,
                }
            )

    result = MODULE.actuator_model_diagnostics(rows)

    assert result["aligned_sample_count"] == 120
    assert math.isclose(result["prediction_bias_rad"], 0.0, abs_tol=1e-12)
    assert math.isclose(result["prediction_rmse_rad"], 0.001, abs_tol=1e-12)
    assert math.isclose(result["prediction_max_abs_error_rad"], 0.001, abs_tol=1e-12)
    assert result["prediction_nrmse"] < 0.006
    assert result["gate_status"] == "pass"


def test_yaw_gain_diagnostics_use_common_left_positive_frame() -> None:
    rows = []
    for sign in (-1.0, 1.0):
        for _ in range(20):
            driver_angle = -sign * 0.10
            map_yaw_rate = sign * 0.50
            rows.append(
                {
                    "kappa": sign * 0.02,
                    "speed": 10.0,
                    "actual_delta_tire": driver_angle,
                    "actual_delta_tire_rate_pre": 0.0,
                    "steering_saturation": 0.0,
                    "actual_yaw_rate_map": map_yaw_rate,
                    "required_delta_preview": driver_angle,
                    "model_yaw_rate_map": map_yaw_rate,
                    "yaw_rate_error_map": 0.0,
                }
            )

    result = MODULE.yaw_gain_diagnostics(rows)

    assert result["steady_sample_count"] == 40
    assert math.isclose(result["actual_yaw_gain_per_s"], 5.0, abs_tol=1e-12)
    assert math.isclose(result["model_yaw_gain_per_s"], 5.0, abs_tol=1e-12)
    assert math.isclose(result["relative_yaw_gain_mismatch"], 0.0, abs_tol=1e-12)
    assert math.isclose(result["yaw_bias_rad_s"], 0.0, abs_tol=1e-12)
    assert result["gate_status"] == "insufficient_steady_state"
