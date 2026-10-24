import numpy as np
import pandas as pd

from tire_fitting.model_selector import ModelSelector, SelectionMode
from tire_fitting.units import ttc_lateral_angles_to_radians
from tire_fitting.validation import ValidationStatus, ValidationThresholds, validate_direction


def test_ttc_lateral_angles_to_radians_makes_unit_conversion_explicit():
    frame = pd.DataFrame({"SA": [0.0, 5.0, -10.0], "IA": [0.0, 2.0, -4.0]})

    slip_angle, camber_angle = ttc_lateral_angles_to_radians(frame)

    np.testing.assert_allclose(slip_angle, np.deg2rad(frame["SA"].to_numpy()))
    np.testing.assert_allclose(camber_angle, np.deg2rad(frame["IA"].to_numpy()))


def test_model_selector_prefers_balanced_vehicle_metrics_over_raw_rmse_only():
    selector = ModelSelector(SelectionMode.AUTO)
    results = {
        "low_rmse_bad_shape": {
            "fx": {
                "rmse": 90.0,
                "nrmse": 0.13,
                "tic": 0.22,
                "r2": 0.90,
                "peak_error_fraction": 0.35,
            }
        },
        "balanced": {
            "fx": {
                "rmse": 95.0,
                "nrmse": 0.06,
                "tic": 0.05,
                "r2": 0.98,
                "peak_error_fraction": 0.12,
            }
        },
    }

    assert selector._select_best_for_direction(results, "fx") == "balanced"


def test_validation_uses_force_scale_adjusted_rmse_threshold_when_available():
    thresholds = ValidationThresholds()
    metrics = {
        "rmse": 140.0,
        "nrmse": 0.07,
        "tic": 0.05,
        "r2": 0.97,
        "force_scale": 2000.0,
    }

    result = validate_direction("fx", "magic_formula", metrics, thresholds)

    assert result.overall_status == ValidationStatus.PASS
    assert result.rmse_validation.threshold_value == 240.0
    assert result.nrmse_validation is not None
    assert result.nrmse_validation.status == ValidationStatus.PASS
