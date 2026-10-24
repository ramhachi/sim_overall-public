"""Tests for the attack-driver sweep helper."""

from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "sweep_attack_driver.py"
)
SPEC = importlib.util.spec_from_file_location("sweep_attack_driver", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
sweep = importlib.util.module_from_spec(SPEC)
sys.modules["sweep_attack_driver"] = sweep
SPEC.loader.exec_module(sweep)


def test_coarse_cases_are_unique_and_expose_attack_cli_knobs() -> None:
    cases = sweep.coarse_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert any("attack_speed_lateral_safety" in case.params for case in cases)
    assert any("attack_floor_exit" in case.params for case in cases)
    assert sweep.cli_name("attack_speed_lateral_safety") == "--attack-speed-lateral-safety"


def test_targeted_weight_cases_are_unique_and_focus_current_bottlenecks() -> None:
    cases = sweep.targeted_weight_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert "target_baseline" in case_ids
    assert len(cases) < 32
    assert any(case.params.get("attack_w_vx") == 0.12 for case in cases)
    assert any(case.params.get("attack_w_delta_rate") == 0.03 for case in cases)
    assert any(case.params.get("attack_soft_boundary_weight") == 80.0 for case in cases)
    assert any(case.params.get("attack_floor_complex") == 0.96 for case in cases)
    assert sweep.cases_for_preset("targeted_weights") == cases


def test_case_filter_selects_reproducible_subsets() -> None:
    cases = sweep.targeted_weight_cases()

    filtered = sweep.filter_cases(cases, r"^(target_baseline|delta_rate_)")

    assert [case.case_id for case in filtered][0] == "target_baseline"
    assert all(
        case.case_id == "target_baseline" or case.case_id.startswith("delta_rate_")
        for case in filtered
    )


def test_smoothness_fine_cases_are_unique_and_centered_on_current_best() -> None:
    cases = sweep.smoothness_fine_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert case_ids[0] == "ax_rate_default_0p0005"
    assert any(case.params.get("attack_w_ax_rate") == 0.0005 for case in cases)
    assert any(case.params.get("attack_w_ax_rate") == 0.0010 for case in cases)
    assert any(case.params.get("attack_w_delta_rate") == 0.02 for case in cases)
    assert sweep.cases_for_preset("smoothness_fine") == cases


def test_tracking_fine_cases_are_unique_and_expose_tracking_weights() -> None:
    cases = sweep.tracking_fine_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert case_ids[0] == "tracking_default_y7_psi3"
    assert any(case.params.get("attack_w_y") == 5.0 for case in cases)
    assert any(case.params.get("attack_w_psi") == 2.0 for case in cases)
    assert any(
        case.params.get("attack_w_y") == 6.0
        and case.params.get("attack_w_psi") == 2.5
        for case in cases
    )
    assert sweep.cases_for_preset("tracking_fine") == cases


def test_understeer_fine_cases_are_unique_and_expose_prediction_gradient() -> None:
    cases = sweep.understeer_fine_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert case_ids[0] == "understeer_default_0p0"
    assert any(
        case.params.get("attack_rollout_understeer_gradient") == 0.0005 for case in cases
    )
    assert any(
        case.params.get("attack_rollout_understeer_gradient") == 0.003 for case in cases
    )
    assert sweep.cases_for_preset("understeer_fine") == cases


def test_generalization_cases_are_unique_and_expose_vehicle_variations() -> None:
    cases = sweep.generalization_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert case_ids[0] == "nominal"
    assert any(case.vehicle_params.get("mass", {}).get("m_total") == 322.0 for case in cases)
    assert any(case.vehicle_params.get("tire", {}).get("mu_y_max") == 1.80 for case in cases)
    assert any(case.vehicle_params.get("steering", {}).get("I_steer") == 0.03 for case in cases)
    assert any(
        case.vehicle_params.get("suspension", {}).get("K_chassis") == 1000.0
        for case in cases
    )
    assert all(not case.params for case in cases)
    assert sweep.cases_for_preset("generalization") == cases


def test_driver_generalization_cases_compare_controller_contracts() -> None:
    cases = sweep.driver_generalization_cases()
    case_ids = [case.case_id for case in cases]

    assert len(case_ids) == len(set(case_ids))
    assert "nominal__attack_residual" in case_ids
    assert "nominal__attack_no_residual" in case_ids
    assert "nominal__attack_legacy_corridor" in case_ids
    assert "nominal__pid" in case_ids
    assert any(case.driver == "pid" for case in cases)
    assert any(case.driver == "attack" and not case.use_residual_profile for case in cases)
    assert any("--attack-braking-corridor-band-target" in case.extra_flags for case in cases)
    assert any(case.vehicle_params.get("steering", {}).get("I_steer") == 0.03 for case in cases)
    assert sweep.cases_for_preset("driver_generalization") == cases


def test_deep_merge_preserves_base_and_overrides_nested_vehicle_params() -> None:
    base = {
        "steering": {"torque_mode": True, "I_steer": 0.02, "T_driver_max": 25.0},
        "suspension": {"K_chassis": 2000.0},
    }
    merged = sweep.deep_merge(base, {"steering": {"I_steer": 0.03}})

    assert merged == {
        "steering": {"torque_mode": True, "I_steer": 0.03, "T_driver_max": 25.0},
        "suspension": {"K_chassis": 2000.0},
    }
    assert base["steering"]["I_steer"] == 0.02


def test_objective_score_prefers_clean_completed_lap() -> None:
    clean = {
        "lap_completed": True,
        "lap_time": 99.5,
        "distance_traveled": 320.0,
        "max_track_limit_excess": 0.0,
        "mpc_fail_count": 0,
    }
    failed = {
        "lap_completed": False,
        "lap_time": math.nan,
        "distance_traveled": 250.0,
        "max_track_limit_excess": 0.0,
        "mpc_fail_count": 0,
    }

    assert sweep.objective_score(clean, {}) == 99.5
    assert sweep.objective_score(failed, {}) > sweep.objective_score(clean, {})


def test_objective_score_penalizes_completed_but_unsafe_rollouts() -> None:
    clean = {
        "lap_completed": True,
        "lap_time": 99.5,
        "distance_traveled": 1000.0,
        "max_track_limit_excess": 0.0,
        "mpc_fail_count": 0,
    }
    unsafe = dict(clean)
    unsafe["lap_time"] = 98.0

    clean_score = sweep.objective_score(
        clean,
        {"all_reject_ratio": 0.0, "min_predicted_margin_m": 0.80},
    )
    unsafe_score = sweep.objective_score(
        unsafe,
        {"all_reject_ratio": 0.02, "min_predicted_margin_m": 0.30},
    )

    assert clean_score == 99.5
    assert unsafe_score > clean_score


def test_csv_last_metrics_recovers_timeout_progress(tmp_path: Path) -> None:
    csv_path = tmp_path / "laptime.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["time", "s", "vx"])
        writer.writeheader()
        writer.writerow({"time": "1.0", "s": "10.0", "vx": "5.0"})
        writer.writerow({"time": "2.5", "s": "31.5", "vx": "8.0"})

    assert sweep.csv_last_metrics(csv_path) == {
        "last_time_s": 2.5,
        "last_distance_m": 31.5,
    }
