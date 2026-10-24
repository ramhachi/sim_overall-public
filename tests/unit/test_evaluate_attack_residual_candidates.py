import importlib.util
import sys
from argparse import Namespace
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "evaluate_attack_residual_candidates.py"
)
SPEC = importlib.util.spec_from_file_location("evaluate_attack_residual_candidates", SCRIPT_PATH)
evaluate = importlib.util.module_from_spec(SPEC)
sys.modules["evaluate_attack_residual_candidates"] = evaluate
assert SPEC.loader is not None
SPEC.loader.exec_module(evaluate)


def _row(segment: int, s: float, phase: str, gap: float, throttle: float):
    return {
        "attack_segment_id": str(segment),
        "s": str(s),
        "course_kappa_ref": "0.05",
        "course_target_gap": str(gap),
        "d_heading": "0.03",
        "throttle": str(throttle),
        "track_phase": phase,
    }


def test_candidate_window_selection_targets_under_ref_segments():
    rows = []
    for i in range(6):
        rows.append(_row(81, 700.0 + i, "turn_in", 4.5, 0.0))
        rows.append(_row(97, 850.0 + i, "complex", 3.5, 0.03))
        rows.append(_row(12, 100.0 + i, "straight", 5.0, 0.0))
        rows.append(_row(8, 60.0 + i, "brake", 0.5, 0.0))

    windows = evaluate.select_candidate_windows(
        rows,
        max_segments=3,
        allowed_phases={"turn_in", "brake", "complex"},
        edge_trim_m=1.0,
        min_target_gap=2.0,
        min_under_ref_no_throttle_ratio=0.40,
    )

    assert [window.segment_id for window in windows] == ["81", "97"]
    assert all(window.phase in {"turn_in", "complex"} for window in windows)
    assert windows[0].s_start > 700.0


def test_candidate_window_selection_can_target_explicit_segments():
    rows = []
    for i in range(6):
        rows.append(_row(103, 915.0 + i, "complex", 1.9, 0.12))
        rows.append(_row(97, 831.0 + i, "complex", 1.6, 0.10))
        rows.append(_row(99, 856.0 + i, "complex", 1.1, 0.10))
        rows.append(_row(92, 782.0 + i, "complex", 1.0, 0.10))
        rows.append(_row(61, 524.0 + i, "complex", 1.2, 0.10))

    windows = evaluate.select_candidate_windows(
        rows,
        max_segments=4,
        allowed_phases={"complex"},
        segment_ids={"103", "97", "99", "92"},
        edge_trim_m=1.0,
        min_target_gap=2.0,
        min_under_ref_no_throttle_ratio=0.40,
    )

    assert {window.segment_id for window in windows} == {"103", "97", "99", "92"}
    assert "61" not in {window.segment_id for window in windows}


def test_allowed_phase_defaults_when_not_specified():
    args = Namespace(allowed_phase=None)

    assert evaluate.allowed_phases_from_args(args) == ["turn_in", "brake", "complex"]


def test_allowed_phase_cli_values_override_defaults():
    args = Namespace(allowed_phase=["complex"])

    assert evaluate.allowed_phases_from_args(args) == ["complex"]


def test_apply_candidate_clamps_profile_limits():
    base_profile = {
        "schema": "attack_residual_profile_v1",
        "limits": {
            "max_delta_v_ref_mps": 0.5,
            "max_delta_steer_ff_rad": 0.02,
            "max_delta_ax_bias_mps2": 0.3,
            "max_delta_heading_relax_rad": 0.04,
        },
        "samples": [
            {"s": 0.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.0},
            {"s": 10.0, "delta_v_ref_mps": 0.4, "delta_steer_ff_rad": 0.0},
            {
                "s": 20.0,
                "delta_v_ref_mps": 0.0,
                "delta_steer_ff_rad": 0.0,
                "delta_ax_bias_mps2": 0.2,
                "delta_heading_relax_rad": 0.03,
            },
        ],
    }
    windows = [
        evaluate.CandidateWindow(
            s_start=5.0,
            s_end=25.0,
            segment_id="81",
            phase="turn_in",
            score=10.0,
        )
    ]
    profile = evaluate.apply_candidate(
        base_profile,
        windows,
        evaluate.CandidateSpec(
            "combined",
            delta_v_ref_mps=0.4,
            delta_steer_ff_rad=0.03,
            delta_ax_bias_mps2=0.3,
            delta_heading_relax_rad=0.03,
        ),
        ramp_m=5.0,
    )

    assert profile["metadata"]["candidate_segments"] == ["81"]
    for sample in profile["samples"]:
        assert abs(sample["delta_v_ref_mps"]) <= 0.5
        assert abs(sample["delta_steer_ff_rad"]) <= 0.02
        assert abs(sample["delta_ax_bias_mps2"]) <= 0.3
        assert 0.0 <= sample["delta_heading_relax_rad"] <= 0.04
