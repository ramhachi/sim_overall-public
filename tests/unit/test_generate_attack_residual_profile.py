import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[2]
    / "python"
    / "scripts"
    / "generate_attack_residual_profile.py"
)
SPEC = importlib.util.spec_from_file_location("generate_attack_residual_profile", SCRIPT_PATH)
generate = importlib.util.module_from_spec(SPEC)
sys.modules["generate_attack_residual_profile"] = generate
assert SPEC.loader is not None
SPEC.loader.exec_module(generate)


def _row(segment: int, s: float, kappa: float, gap: float, throttle: float, phase: str = "complex"):
    return {
        "attack_segment_id": str(segment),
        "s": str(s),
        "course_kappa_ref": str(kappa),
        "course_target_gap": str(gap),
        "d_heading": "0.12",
        "throttle": str(throttle),
        "track_phase": phase,
    }


def test_selects_driver_sign_from_curvature_and_merges_nearby_segments():
    rows = []
    for i in range(8):
        rows.append(_row(97, 830.0 + i, -0.12, 5.0, 0.0))
        rows.append(_row(101, 884.0 + i, -0.11, 4.6, 0.0))
    for i in range(8):
        rows.append(_row(8, 60.0 + i, 0.10, 5.0, 0.0, phase="brake"))

    stats = generate.collect_segment_stats(rows)
    windows = generate.select_residual_windows(
        stats,
        max_segments=2,
        steer_mag_rad=0.015,
        edge_trim_m=2.0,
        merge_gap_m=60.0,
        quantize_m=5.0,
        allowed_phases={"complex"},
        min_abs_kappa=0.035,
        min_target_gap=2.0,
        min_under_ref_no_throttle_ratio=0.25,
    )
    profile = generate.build_profile(
        windows,
        ramp_m=15.0,
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=0.035,
        description="test",
    )

    assert len(windows) == 1
    assert windows[0].delta_steer_ff_rad == 0.015
    assert windows[0].source_segments == ["97", "101"]
    assert profile["metadata"]["source_segments"] == ["97", "101"]
    nonzero_samples = [
        sample for sample in profile["samples"] if sample["delta_steer_ff_rad"] != 0.0
    ]
    assert nonzero_samples
    assert all(sample["delta_steer_ff_rad"] == 0.015 for sample in nonzero_samples)


def test_filters_weak_or_wrong_phase_segments():
    rows = []
    for i in range(5):
        rows.append(_row(1, 10.0 + i, 0.01, 5.0, 0.0))
        rows.append(_row(2, 30.0 + i, -0.10, 0.5, 0.0))
        rows.append(_row(3, 50.0 + i, -0.10, 5.0, 1.0))
        rows.append(_row(4, 70.0 + i, -0.10, 5.0, 0.0, phase="straight"))

    stats = generate.collect_segment_stats(rows)
    windows = generate.select_residual_windows(
        stats,
        max_segments=4,
        steer_mag_rad=0.015,
        edge_trim_m=2.0,
        merge_gap_m=20.0,
        quantize_m=5.0,
        allowed_phases={"complex"},
        min_abs_kappa=0.035,
        min_target_gap=2.0,
        min_under_ref_no_throttle_ratio=0.25,
    )

    assert windows == []
