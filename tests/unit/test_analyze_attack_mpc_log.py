"""Tests for the attack MPC CSV diagnostic analyzer."""

from __future__ import annotations

import csv
import importlib.util
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "analyze_attack_mpc_log.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_attack_mpc_log", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def test_attack_analyzer_summarizes_new_diagnostic_columns(tmp_path: Path) -> None:
    csv_path = tmp_path / "attack.csv"
    rows = [
        {
            "time": 0.0,
            "s": 0.0,
            "speed": 8.0,
            "throttle": 0.20,
            "d_heading": 0.05,
            "mpc_predicted_min_margin": 1.2,
            "attack_selected_primitive": 0,
            "attack_selected_primitive_kind": "tracking",
            "attack_selected_speed_scale": 0.95,
            "attack_selected_target_speed": 9.0,
            "attack_selected_ax": 1.0,
            "course_v_ref": 10.0,
            "course_n_ref": 0.2,
            "course_kappa_ref": 0.01,
            "course_speed_error": -2.0,
            "course_lateral_error": -0.2,
            "course_a_ref": 0.5,
            "course_target_gap": 2.0,
            "dynamic_cap_gap": 0.0,
            "braking_corridor_active": 0,
            "braking_corridor_v_lower": 0.0,
            "braking_corridor_v_upper": 0.0,
            "longitudinal_mode": "track_speed",
            "healthy_underspeed": 0,
            "accel_headroom": 4.0,
            "track_phase": "straight",
            "phase_speed_bias": 1.02,
            "phase_margin_bias": 0.0,
            "phase_progress_bias": 1.05,
            "attack_segment_id": 0,
            "attack_allowed": 1,
            "attack_static_speed_cap": 13.0,
            "attack_dynamic_speed_cap": 12.0,
            "attack_limiting_cap_reason": "course_ref",
            "attack_rollout_candidate_count": 24,
            "attack_rollout_reject_count": 0,
            "attack_rollout_boundary_reject_count": 0,
            "attack_rollout_nonfinite_reject_count": 0,
            "attack_all_rejected": 0,
            "attack_cost_tracking": 2.0,
            "attack_cost_speed": 1.0,
            "attack_cost_boundary": 0.5,
            "attack_cost_control_smoothness": 0.2,
            "attack_cost_progress": 10.0,
            "attack_cost_primitive_bias": -1.0,
            "attack_cost_overspeed": 0.0,
            "attack_cost_underspeed": 0.4,
            "attack_cost_time_progress": 9.0,
            "attack_best_tracking_cost": 12.0,
            "attack_best_boundary_recovery_cost": 20.0,
            "attack_best_low_speed_pure_pursuit_cost": 30.0,
            "attack_best_speed_scale_065_cost": 18.0,
            "attack_best_speed_scale_080_cost": 14.0,
            "attack_best_speed_scale_095_cost": 12.0,
            "attack_best_speed_scale_105_cost": 16.0,
            "attack_best_speed_scale_095_cost_delta": 0.0,
            "attack_best_speed_scale_095_tracking_delta": 0.0,
            "attack_best_speed_scale_095_speed_delta": 0.0,
            "attack_best_speed_scale_095_boundary_delta": 0.0,
            "attack_best_speed_scale_095_control_smoothness_delta": 0.0,
            "attack_best_speed_scale_095_progress_delta": 0.0,
            "attack_best_speed_scale_095_primitive_bias_delta": 0.0,
        },
        {
            "time": 0.1,
            "s": 0.5,
            "speed": 1.0,
            "throttle": 0.0,
            "d_heading": 0.70,
            "mpc_predicted_min_margin": -0.2,
            "attack_selected_primitive": -1,
            "attack_selected_primitive_kind": "fallback",
            "attack_selected_speed_scale": 0.0,
            "attack_selected_target_speed": 1.0,
            "attack_selected_ax": -2.0,
            "course_v_ref": 2.0,
            "course_n_ref": -0.1,
            "course_kappa_ref": 0.02,
            "course_speed_error": -1.0,
            "course_lateral_error": 0.1,
            "course_a_ref": -1.0,
            "course_target_gap": 1.0,
            "dynamic_cap_gap": 0.5,
            "braking_corridor_active": 1,
            "braking_corridor_v_lower": 7.0,
            "braking_corridor_v_upper": 8.0,
            "longitudinal_mode": "braking_corridor_overspeed",
            "healthy_underspeed": 0,
            "accel_headroom": 6.0,
            "track_phase": "turn_in",
            "phase_speed_bias": 0.97,
            "phase_margin_bias": 0.08,
            "phase_progress_bias": 0.80,
            "attack_segment_id": 1,
            "attack_allowed": 0,
            "attack_static_speed_cap": 8.0,
            "attack_dynamic_speed_cap": 6.0,
            "attack_limiting_cap_reason": "heading",
            "attack_rollout_candidate_count": 24,
            "attack_rollout_reject_count": 24,
            "attack_rollout_boundary_reject_count": 24,
            "attack_rollout_nonfinite_reject_count": 0,
            "attack_all_rejected": 1,
            "attack_cost_tracking": 0.0,
            "attack_cost_speed": 0.0,
            "attack_cost_boundary": 0.0,
            "attack_cost_control_smoothness": 0.0,
            "attack_cost_progress": 20.0,
            "attack_cost_primitive_bias": 0.0,
            "attack_cost_overspeed": 0.0,
            "attack_cost_underspeed": 0.0,
            "attack_cost_time_progress": 20.0,
            "attack_best_tracking_cost": "inf",
            "attack_best_boundary_recovery_cost": "inf",
            "attack_best_low_speed_pure_pursuit_cost": "inf",
            "attack_best_speed_scale_065_cost": "inf",
            "attack_best_speed_scale_080_cost": "inf",
            "attack_best_speed_scale_095_cost": "inf",
            "attack_best_speed_scale_105_cost": "inf",
            "attack_best_speed_scale_095_cost_delta": "inf",
            "attack_best_speed_scale_095_tracking_delta": "inf",
            "attack_best_speed_scale_095_speed_delta": "inf",
            "attack_best_speed_scale_095_boundary_delta": "inf",
            "attack_best_speed_scale_095_control_smoothness_delta": "inf",
            "attack_best_speed_scale_095_progress_delta": "inf",
            "attack_best_speed_scale_095_primitive_bias_delta": "inf",
        },
        {
            "time": 0.2,
            "s": 1.0,
            "speed": 9.0,
            "throttle": 0.40,
            "d_heading": 0.08,
            "mpc_predicted_min_margin": 1.6,
            "attack_selected_primitive": 0,
            "attack_selected_primitive_kind": "tracking",
            "attack_selected_speed_scale": 0.65,
            "attack_selected_target_speed": 11.0,
            "attack_selected_ax": 0.8,
            "course_v_ref": 12.0,
            "course_n_ref": 0.0,
            "course_kappa_ref": 0.0,
            "course_speed_error": -3.0,
            "course_lateral_error": 0.0,
            "course_a_ref": 1.5,
            "course_target_gap": 3.0,
            "dynamic_cap_gap": 1.0,
            "braking_corridor_active": 0,
            "braking_corridor_v_lower": 0.0,
            "braking_corridor_v_upper": 0.0,
            "longitudinal_mode": "healthy_underspeed",
            "healthy_underspeed": 1,
            "accel_headroom": 3.2,
            "track_phase": "exit",
            "phase_speed_bias": 1.03,
            "phase_margin_bias": 0.0,
            "phase_progress_bias": 1.10,
            "attack_segment_id": 2,
            "attack_allowed": 1,
            "attack_static_speed_cap": 14.0,
            "attack_dynamic_speed_cap": 11.0,
            "attack_limiting_cap_reason": "curvature",
            "attack_rollout_candidate_count": 24,
            "attack_rollout_reject_count": 0,
            "attack_rollout_boundary_reject_count": 0,
            "attack_rollout_nonfinite_reject_count": 0,
            "attack_all_rejected": 0,
            "attack_cost_tracking": 1.0,
            "attack_cost_speed": 0.5,
            "attack_cost_boundary": 1.0,
            "attack_cost_control_smoothness": 0.1,
            "attack_cost_progress": 12.0,
            "attack_cost_primitive_bias": -2.0,
            "attack_cost_overspeed": 0.0,
            "attack_cost_underspeed": 0.6,
            "attack_cost_time_progress": 11.0,
            "attack_best_tracking_cost": 15.0,
            "attack_best_boundary_recovery_cost": 9.0,
            "attack_best_low_speed_pure_pursuit_cost": 25.0,
            "attack_best_speed_scale_065_cost": 9.0,
            "attack_best_speed_scale_080_cost": 11.0,
            "attack_best_speed_scale_095_cost": 13.0,
            "attack_best_speed_scale_105_cost": 17.0,
            "attack_best_speed_scale_095_cost_delta": 4.0,
            "attack_best_speed_scale_095_tracking_delta": 1.0,
            "attack_best_speed_scale_095_speed_delta": 0.5,
            "attack_best_speed_scale_095_boundary_delta": 0.0,
            "attack_best_speed_scale_095_control_smoothness_delta": 0.2,
            "attack_best_speed_scale_095_progress_delta": 3.0,
            "attack_best_speed_scale_095_primitive_bias_delta": -0.5,
        },
    ]
    _write_csv(csv_path, rows)

    summary = analyzer.summarize(analyzer.read_csv(csv_path), window_seconds=0.15)

    assert summary["sample_count"] == 3
    assert summary["missing_columns"] == []
    assert summary["primitive_kind_ratio"]["tracking"] == 2 / 3
    assert summary["primitive_ratio"]["-1"] == 1 / 3
    assert summary["track_phase_ratio"]["straight"] == 1 / 3
    assert summary["track_phase_ratio"]["turn_in"] == 1 / 3
    assert summary["cap_reason_ratio"]["course_ref"] == 1 / 3
    assert summary["cap_reason_ratio"]["heading"] == 1 / 3
    assert summary["track_phase_speed_scale_distribution"]["exit"]["0.65"] == 1
    assert summary["track_phase_cap_reason_distribution"]["exit"]["curvature"] == 1
    assert summary["track_phase_speed_scale_distribution"]["straight"]["0.95"] == 1
    assert summary["track_phase_dominant_cost_ratio"]["exit"]["progress"] == 1.0
    assert summary["track_phase_time_s"]["straight"] == 0.1
    assert summary["track_phase_time_ratio"]["straight"] == 0.5
    assert summary["track_phase_mean_speed_mps"]["exit"] == 9.0
    assert summary["phase_bias_means"]["speed"] == (1.02 + 0.97 + 1.03) / 3
    assert summary["track_phase_bias_means"]["turn_in"]["margin"] == 0.08
    assert summary["attack_segment_ratio"]["0"] == 1 / 3
    assert summary["top_bottleneck_segments"][0]["segment_id"] == "2"
    assert summary["top_bottleneck_segments"][0]["dominant_cap_reason"] == "curvature"
    assert summary["top_bottleneck_segments"][0]["mean_dynamic_cap_gap"] == 1.0
    assert summary["attack_allowed_ratio"]["true"] == 2 / 3
    assert summary["attack_planner_means"]["static_speed_cap"] == (13.0 + 8.0 + 14.0) / 3
    assert summary["attack_planner_means"]["underspeed_cost"] == (0.4 + 0.0 + 0.6) / 3
    assert summary["attack_planner_percentiles"]["time_progress_cost"]["p50"] == 11.0
    assert summary["course_reference_means"]["v_ref"] == 8.0
    assert summary["course_reference_means"]["speed_error"] == -2.0
    assert summary["course_reference_means"]["lateral_error"] == pytest.approx(0.1)
    assert summary["course_reference_means"]["a_ref"] == pytest.approx(1.0 / 3.0)
    assert summary["course_reference_means"]["target_gap"] == 2.0
    assert summary["course_reference_means"]["dynamic_cap_gap"] == 0.5
    assert summary["course_reference_means"]["accel_headroom"] == pytest.approx(4.4)
    assert summary["course_reference_percentiles"]["v_ref"]["p90"] == 12.0
    assert summary["track_phase_longitudinal_mode_distribution"]["exit"]["healthy_underspeed"] == 1
    assert summary["braking_corridor_active_ratio"] == 1 / 3
    assert summary["braking_corridor_mode_distribution"]["braking_corridor_overspeed"] == 1
    assert summary["braking_corridor_mode_distribution"]["not_corridor"] == 2
    assert summary["braking_corridor_means"]["v_lower"] == 7.0
    assert summary["braking_corridor_means"]["v_upper"] == 8.0
    assert summary["braking_corridor_means"]["band_width"] == 1.0
    assert summary["braking_corridor_means"]["speed_minus_v_upper"] == -7.0
    assert summary["track_phase_braking_corridor_active_ratio"]["turn_in"] == 1.0
    assert (
        summary["track_phase_braking_corridor_mode_distribution"]["turn_in"][
            "braking_corridor_overspeed"
        ]
        == 1
    )
    assert summary["track_phase_braking_corridor_means"]["turn_in"]["v_lower"] == 7.0
    corridor_segment = next(
        segment
        for segment in summary["top_bottleneck_segments"]
        if segment["segment_id"] == "1"
    )
    assert corridor_segment["braking_corridor_active_ratio"] == 1.0
    assert (
        corridor_segment["braking_corridor_mode_ratio"]["braking_corridor_overspeed"]
        == 1.0
    )
    assert summary["healthy_underspeed_count"] == 1
    assert summary["healthy_underspeed_ratio"] == 1 / 3
    assert summary["healthy_underspeed_throttle_percentiles"]["p50"] == 0.4
    assert summary["under_ref_count"] == 1
    assert summary["under_ref_no_throttle_ratio"] == 0.0
    assert summary["all_reject_ratio"] == 1 / 3
    assert len(summary["all_reject_segments"]) == 1
    assert summary["dominant_cost_term"] == "progress"
    assert summary["estimated_failure_mode"] == "all_reject_stall"
    assert summary["min_predicted_margin_m"] == -0.2
    assert summary["max_abs_heading_error_rad"] == 0.70
    assert summary["row_dominant_cost_ratio"]["progress"] == 1.0
    assert summary["healthy_low_speed_scale_count"] == 1
    assert summary["healthy_low_speed_scale_ratio"] == 1.0
    assert summary["selected_target_speed_error_mean_mps"] == 1.0
    assert summary["speed_scale_best_cost_means"]["0.65"] == 13.5
    assert summary["cost_positive_percentiles"]["progress"]["p90"] == 20.0
    assert summary["speed_scale_095_cost_delta_mean"] == 2.0
    assert summary["speed_scale_095_delta_means"]["progress"] == 1.5
    assert summary["speed_scale_095_loss_term_ratio"]["progress"] == 1.0


def test_attack_analyzer_accepts_legacy_csv_without_new_columns(tmp_path: Path) -> None:
    csv_path = tmp_path / "legacy.csv"
    _write_csv(
        csv_path,
        [
            {
                "time": 0.0,
                "s": 0.0,
                "speed": 6.0,
                "d_heading": 0.10,
                "mpc_selected_plan_index": 3,
            },
            {
                "time": 0.1,
                "s": 0.4,
                "speed": 6.5,
                "d_heading": 0.12,
                "mpc_selected_plan_index": 4,
            },
        ],
    )

    summary = analyzer.summarize(analyzer.read_csv(csv_path))

    assert summary["sample_count"] == 2
    assert "attack_selected_primitive" in summary["missing_columns"]
    assert summary["estimated_failure_mode"] == "unknown"
    assert summary["primitive_kind_ratio"]["unknown"] == 1.0
    assert summary["track_phase_ratio"]["unknown"] == 1.0
    assert summary["cap_reason_ratio"]["unknown"] == 1.0
    assert summary["track_phase_cap_reason_distribution"]["unknown"]["unknown"] == 2
    assert summary["top_bottleneck_segments"][0]["segment_id"] == "unknown"
    assert summary["track_phase_speed_scale_distribution"]["unknown"]["unknown"] == 2
    assert summary["track_phase_longitudinal_mode_distribution"]["unknown"]["unknown"] == 2
    assert summary["braking_corridor_active_ratio"] == 0.0
    assert summary["braking_corridor_mode_distribution"]["not_corridor"] == 2
    assert summary["braking_corridor_means"]["v_lower"] is None
    assert summary["course_reference_means"]["v_ref"] is None
    assert summary["course_reference_means"]["a_ref"] is None
