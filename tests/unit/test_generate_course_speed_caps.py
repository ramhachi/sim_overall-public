from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[2] / "python" / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import generate_course_speed_caps as planner  # noqa: E402


def make_arc(radius: float, start_angle: float, end_angle: float, count: int, width: float = 4.5):
    points = []
    for i in range(count):
        alpha = i / max(count - 1, 1)
        theta = start_angle + alpha * (end_angle - start_angle)
        points.append((radius * math.cos(theta), radius * math.sin(theta), width))
    cumulative = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]
        cumulative.append(cumulative[-1] + math.hypot(dx, dy))
    return [
        planner.TrackPoint(x=x, y=y, width=w, s=s)
        for (x, y, w), s in zip(points, cumulative)
    ]


def test_interpolate_radius_speed_uses_v_squared_relation():
    speed = planner.interpolate_radius_speed({10.0: 8.0, 20.0: 12.0}, 15.0)

    assert speed == pytest.approx(math.sqrt(0.5 * (8.0**2 + 12.0**2)))


def test_extract_corner_segments_detects_constant_radius_corner():
    points = planner.with_curvature(make_arc(12.0, 0.0, math.pi / 2.0, 60))

    segments = planner.extract_corner_segments(
        points,
        curvature_threshold=0.02,
        merge_gap_m=4.0,
        min_length_m=4.0,
        slalom_gap_m=55.0,
    )

    assert len(segments) == 1
    assert segments[0].kind == "corner"
    assert segments[0].radius_m == pytest.approx(12.0, rel=0.20)


def test_build_cap_payload_marks_sign_change_segment_as_slalom():
    points = planner.with_curvature(
        make_arc(10.0, 0.0, math.pi / 3.0, 40)
        + make_arc(10.0, math.pi / 3.0, 0.0, 40)
    )
    segments = planner.extract_corner_segments(
        points,
        curvature_threshold=0.02,
        merge_gap_m=10.0,
        min_length_m=4.0,
        slalom_gap_m=80.0,
    )
    assert segments
    segment = planner.CornerSegment(
        s_start=segments[0].s_start,
        s_end=segments[-1].s_end,
        s_apex=segments[0].s_apex,
        radius_m=10.0,
        max_abs_kappa=0.1,
        sign_changes=1,
        kind="slalom",
    )

    payload = planner.build_cap_payload(
        points=points,
        segments=[segment],
        radius_speed={10.0: 10.0},
        slalom_critical_speed=8.0,
        corner_safety=0.95,
        slalom_safety=0.9,
        approach_speed=16.0,
        brake_decel=8.0,
        entry_buffer_m=2.0,
        exit_buffer_m=3.0,
        cap_mode="both",
        point_cap_radius_m=4.0,
        max_capped_radius_m=math.inf,
        min_cap_speed=3.0,
        max_cap_speed=25.0,
    )

    assert payload["window_caps"][0]["speed"] == pytest.approx(7.2)
    assert payload["corner_segments"][0]["kind"] == "slalom"
