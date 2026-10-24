"""Tests for attack speed-cap relaxation tuning."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "tune_attack_speed_caps.py"
)
SPEC = importlib.util.spec_from_file_location("tune_attack_speed_caps", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
tune = importlib.util.module_from_spec(SPEC)
sys.modules["tune_attack_speed_caps"] = tune
SPEC.loader.exec_module(tune)


def test_scaled_cap_payload_scales_speed_and_preserves_metadata() -> None:
    payload = {
        "metadata": {"schema": "attack_speed_caps_v1"},
        "speed_caps": [
            {"s": 10.0, "speed": 5.0, "radius": 3.0},
            [20.0, 8.0, 4.0],
        ],
        "window_caps": [
            {"s_start": 30.0, "s_end": 45.0, "speed": 6.0},
            [50.0, 70.0, 9.0],
        ],
    }

    scaled = tune.scaled_cap_payload(payload, scale=1.5, max_cap_speed=10.0)

    assert scaled["metadata"]["relaxation_scale"] == 1.5
    assert scaled["speed_caps"][0]["speed"] == 7.5
    assert scaled["speed_caps"][0]["base_speed"] == 5.0
    assert scaled["speed_caps"][1]["speed"] == 10.0
    assert scaled["speed_caps"][1]["s"] == 20.0
    assert scaled["window_caps"][0]["speed"] == 9.0
    assert scaled["window_caps"][0]["base_speed"] == 6.0
    assert scaled["window_caps"][1]["speed"] == 10.0
    assert scaled["window_caps"][1]["s_start"] == 50.0


def test_is_safe_result_requires_completion_no_excess_and_no_mpc_fail() -> None:
    safe = {
        "lap_completed": True,
        "max_track_limit_excess": 0.0,
        "mpc_fail_count": 0,
    }
    off_track = {
        "lap_completed": True,
        "max_track_limit_excess": 0.2,
        "mpc_fail_count": 0,
    }
    not_completed = {
        "lap_completed": False,
        "max_track_limit_excess": 0.0,
        "mpc_fail_count": 0,
    }

    assert tune.is_safe_result(safe, 1e-6, 0)
    assert not tune.is_safe_result(off_track, 1e-6, 0)
    assert not tune.is_safe_result(not_completed, 1e-6, 0)
