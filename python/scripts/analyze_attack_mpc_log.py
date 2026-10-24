#!/usr/bin/env python3
"""Summarize attack MPC diagnostics from a laptime CSV log.

The script is intentionally tolerant of older laptime CSV files. Missing attack
diagnostic columns are reported in the JSON summary rather than treated as
fatal parse errors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


ATTACK_REQUIRED_COLUMNS = [
    "attack_selected_primitive",
    "attack_selected_primitive_kind",
    "attack_selected_speed_scale",
    "attack_selected_target_speed",
    "attack_selected_ax",
    "attack_rollout_candidate_count",
    "attack_rollout_reject_count",
    "attack_rollout_boundary_reject_count",
    "attack_rollout_nonfinite_reject_count",
    "attack_all_rejected",
    "attack_cost_tracking",
    "attack_cost_speed",
    "attack_cost_boundary",
    "attack_cost_control_smoothness",
    "attack_cost_progress",
    "attack_cost_primitive_bias",
    "attack_best_tracking_cost",
    "attack_best_boundary_recovery_cost",
    "attack_best_low_speed_pure_pursuit_cost",
    "attack_best_speed_scale_065_cost",
    "attack_best_speed_scale_080_cost",
    "attack_best_speed_scale_095_cost",
    "attack_best_speed_scale_105_cost",
    "attack_best_speed_scale_095_cost_delta",
    "attack_best_speed_scale_095_tracking_delta",
    "attack_best_speed_scale_095_speed_delta",
    "attack_best_speed_scale_095_boundary_delta",
    "attack_best_speed_scale_095_control_smoothness_delta",
    "attack_best_speed_scale_095_progress_delta",
    "attack_best_speed_scale_095_primitive_bias_delta",
]

COST_COLUMNS = {
    "tracking": "attack_cost_tracking",
    "speed": "attack_cost_speed",
    "boundary": "attack_cost_boundary",
    "control_smoothness": "attack_cost_control_smoothness",
    "progress": "attack_cost_progress",
    "primitive_bias": "attack_cost_primitive_bias",
}

SPEED_SCALE_COST_COLUMNS = {
    "0.65": "attack_best_speed_scale_065_cost",
    "0.80": "attack_best_speed_scale_080_cost",
    "0.95": "attack_best_speed_scale_095_cost",
    "1.05": "attack_best_speed_scale_105_cost",
}

SPEED_SCALE_095_DELTA_COLUMNS = {
    "tracking": "attack_best_speed_scale_095_tracking_delta",
    "speed": "attack_best_speed_scale_095_speed_delta",
    "boundary": "attack_best_speed_scale_095_boundary_delta",
    "control_smoothness": "attack_best_speed_scale_095_control_smoothness_delta",
    "progress": "attack_best_speed_scale_095_progress_delta",
    "primitive_bias": "attack_best_speed_scale_095_primitive_bias_delta",
}

PHASE_BIAS_COLUMNS = {
    "speed": "phase_speed_bias",
    "margin": "phase_margin_bias",
    "progress": "phase_progress_bias",
}

ATTACK_PLANNER_COLUMNS = {
    "static_speed_cap": "attack_static_speed_cap",
    "dynamic_speed_cap": "attack_dynamic_speed_cap",
    "overspeed_cost": "attack_cost_overspeed",
    "underspeed_cost": "attack_cost_underspeed",
    "time_progress_cost": "attack_cost_time_progress",
}

COURSE_REFERENCE_COLUMNS = {
    "v_ref": "course_v_ref",
    "n_ref": "course_n_ref",
    "kappa_ref": "course_kappa_ref",
    "speed_error": "course_speed_error",
    "lateral_error": "course_lateral_error",
    "a_ref": "course_a_ref",
    "target_gap": "course_target_gap",
    "dynamic_cap_gap": "dynamic_cap_gap",
    "accel_headroom": "accel_headroom",
}

BRAKING_CORRIDOR_COLUMNS = {
    "v_lower": "braking_corridor_v_lower",
    "v_upper": "braking_corridor_v_upper",
}


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        number = float(value)
    else:
        text = str(value).strip()
        if not text:
            return None
        try:
            number = float(text)
        except ValueError:
            return None
    if not math.isfinite(number):
        return None
    return number


def _truthy(value: Any) -> bool:
    if value is None:
        return False
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "y"}


def _ratio(counter: Counter[str], total: float) -> dict[str, float]:
    if total <= 0:
        return {}
    return {key: value / total for key, value in sorted(counter.items())}


def _sorted_counter_dict(counter: Counter[str]) -> dict[str, int]:
    return {key: counter[key] for key in sorted(counter)}


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def _percentile(values: list[float], fraction: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    index = round((len(finite) - 1) * min(max(fraction, 0.0), 1.0))
    return finite[index]


def _percentile_summary(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": _percentile(values, 0.50),
        "p90": _percentile(values, 0.90),
        "p99": _percentile(values, 0.99),
    }


def _heading_error_rad(row: dict[str, str]) -> float | None:
    heading_rad = _float_or_none(row.get("d_heading"))
    if heading_rad is not None:
        return heading_rad
    heading_deg = _float_or_none(row.get("heading_error_deg"))
    if heading_deg is None:
        return None
    return math.radians(heading_deg)


def _all_reject_segments(rows: list[dict[str, str]]) -> list[dict[str, float]]:
    segments: list[dict[str, float]] = []
    start: dict[str, str] | None = None
    prev: dict[str, str] | None = None

    for row in rows:
        rejected = _truthy(row.get("attack_all_rejected"))
        if rejected and start is None:
            start = row
        if not rejected and start is not None and prev is not None:
            segments.append(_make_segment(start, prev))
            start = None
        prev = row

    if start is not None and prev is not None:
        segments.append(_make_segment(start, prev))
    return segments


def _make_segment(start: dict[str, str], end: dict[str, str]) -> dict[str, float]:
    start_time = _float_or_none(start.get("time")) or 0.0
    end_time = _float_or_none(end.get("time")) or start_time
    start_s = _float_or_none(start.get("s")) or 0.0
    end_s = _float_or_none(end.get("s")) or start_s
    return {
        "start_time_s": start_time,
        "end_time_s": end_time,
        "duration_s": max(0.0, end_time - start_time),
        "start_s_m": start_s,
        "end_s_m": end_s,
    }


def _worst_heading_window(
    rows: list[dict[str, str]],
    window_seconds: float,
) -> dict[str, float | None]:
    timed_rows: list[tuple[float, float, float | None]] = []
    for row in rows:
        t = _float_or_none(row.get("time"))
        heading = _heading_error_rad(row)
        if t is None or heading is None:
            continue
        timed_rows.append((t, abs(heading), _float_or_none(row.get("speed"))))

    if not timed_rows:
        return {
            "start_time_s": None,
            "end_time_s": None,
            "max_abs_heading_error_rad": None,
            "mean_speed_mps": None,
        }

    best_start = timed_rows[0][0]
    best_end = timed_rows[0][0]
    best_heading = -1.0
    best_mean_speed: float | None = None
    j = 0
    for i, (start_time, _, _) in enumerate(timed_rows):
        while j < len(timed_rows) and timed_rows[j][0] <= start_time + window_seconds:
            j += 1
        window = timed_rows[i:j]
        max_heading = max(item[1] for item in window)
        speeds = [item[2] for item in window if item[2] is not None]
        if max_heading > best_heading:
            best_heading = max_heading
            best_start = start_time
            best_end = window[-1][0]
            best_mean_speed = _mean(speeds) if speeds else None

    return {
        "start_time_s": best_start,
        "end_time_s": best_end,
        "max_abs_heading_error_rad": best_heading,
        "mean_speed_mps": best_mean_speed,
    }


def _dominant_cost(cost_positive_means: dict[str, float | None]) -> str:
    finite = {
        name: value
        for name, value in cost_positive_means.items()
        if value is not None and math.isfinite(value) and value > 0.0
    }
    if not finite:
        return "unknown"
    return max(finite.items(), key=lambda item: item[1])[0]


def _dominant_counter_key(counter: Counter[str]) -> str:
    if not counter:
        return "unknown"
    return max(counter.items(), key=lambda item: item[1])[0]


def _top_segment_bottlenecks(
    segment_stats: dict[str, dict[str, Any]],
    limit: int = 10,
) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for segment_id, stats in segment_stats.items():
        count = int(stats.get("count", 0))
        if count <= 0:
            continue
        cap_counter = stats.get("cap_reason", Counter())
        phase_counter = stats.get("phase", Counter())
        corridor_mode_counter = stats.get("braking_corridor_mode", Counter())
        dynamic_cap_gap = stats.get("dynamic_cap_gap", [])
        target_gap = stats.get("target_gap", [])
        speeds = stats.get("speed", [])
        headings = stats.get("abs_heading", [])
        corridor_lower = stats.get("braking_corridor_v_lower", [])
        corridor_upper = stats.get("braking_corridor_v_upper", [])
        corridor_band_width = stats.get("braking_corridor_band_width", [])
        under_ref_count = int(stats.get("under_ref_count", 0))
        under_ref_no_throttle_count = int(stats.get("under_ref_no_throttle_count", 0))
        corridor_active_count = int(stats.get("braking_corridor_active_count", 0))
        mean_dynamic_cap_gap = _mean(dynamic_cap_gap)
        mean_target_gap = _mean(target_gap)
        summaries.append(
            {
                "segment_id": segment_id,
                "sample_count": count,
                "dominant_phase": _dominant_counter_key(phase_counter),
                "dominant_cap_reason": _dominant_counter_key(cap_counter),
                "cap_reason_ratio": _ratio(cap_counter, sum(cap_counter.values())),
                "mean_dynamic_cap_gap": mean_dynamic_cap_gap,
                "mean_course_target_gap": mean_target_gap,
                "mean_speed_mps": _mean(speeds),
                "mean_abs_heading_rad": _mean(headings),
                "braking_corridor_active_ratio": corridor_active_count / count,
                "braking_corridor_mode_ratio": _ratio(
                    corridor_mode_counter, sum(corridor_mode_counter.values())
                ),
                "mean_braking_corridor_v_lower": _mean(corridor_lower),
                "mean_braking_corridor_v_upper": _mean(corridor_upper),
                "mean_braking_corridor_band_width": _mean(corridor_band_width),
                "under_ref_no_throttle_ratio": (
                    under_ref_no_throttle_count / under_ref_count
                    if under_ref_count > 0
                    else 0.0
                ),
                "bottleneck_score": (
                    (mean_dynamic_cap_gap or 0.0) * count
                    + 0.50 * (mean_target_gap or 0.0) * count
                    + 2.0 * under_ref_no_throttle_count
                ),
            }
        )
    return sorted(
        summaries,
        key=lambda item: item.get("bottleneck_score") or 0.0,
        reverse=True,
    )[:limit]


def _estimate_failure_mode(
    *,
    missing_columns: list[str],
    all_reject_ratio: float,
    primitive_kind_ratio: dict[str, float],
    min_predicted_margin_m: float | None,
    max_abs_heading_error_rad: float | None,
    dominant_cost_term: str,
    mean_speed_mps: float | None,
) -> str:
    if missing_columns:
        return "unknown"
    if all_reject_ratio > 0.10:
        return "all_reject_stall"
    if (
        min_predicted_margin_m is not None
        and min_predicted_margin_m < 0.0
        and max_abs_heading_error_rad is not None
        and max_abs_heading_error_rad > 0.50
    ):
        return "heading_margin_conflict"
    if primitive_kind_ratio.get("boundary_recovery", 0.0) > 0.50:
        if min_predicted_margin_m is None or min_predicted_margin_m > 0.50:
            return "boundary_recovery_overuse"
    if dominant_cost_term == "progress" and mean_speed_mps is not None and mean_speed_mps < 5.0:
        return "progress_starvation"
    return "unknown"


def summarize(rows: list[dict[str, str]], window_seconds: float = 2.0) -> dict[str, Any]:
    fieldnames = set(rows[0].keys()) if rows else set()
    missing_columns = [
        column for column in ATTACK_REQUIRED_COLUMNS if column not in fieldnames
    ]

    primitive_counter: Counter[str] = Counter()
    primitive_kind_counter: Counter[str] = Counter()
    speed_scale_counter: Counter[str] = Counter()
    track_phase_counter: Counter[str] = Counter()
    cap_reason_counter: Counter[str] = Counter()
    track_phase_cap_reason_counter: dict[str, Counter[str]] = defaultdict(Counter)
    track_phase_speed_scale_counter: dict[str, Counter[str]] = defaultdict(Counter)
    track_phase_longitudinal_mode_counter: dict[str, Counter[str]] = defaultdict(Counter)
    track_phase_braking_corridor_mode_counter: dict[str, Counter[str]] = defaultdict(Counter)
    track_phase_braking_corridor_count: Counter[str] = Counter()
    track_phase_speed_values: dict[str, list[float]] = defaultdict(list)
    track_phase_dominant_cost_counter: dict[str, Counter[str]] = defaultdict(Counter)
    track_phase_braking_corridor_values: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {
            "v_lower": [],
            "v_upper": [],
            "band_width": [],
            "speed_minus_v_upper": [],
            "v_lower_minus_speed": [],
        }
    )
    segment_stats: dict[str, dict[str, Any]] = defaultdict(
        lambda: {
            "count": 0,
            "phase": Counter(),
            "cap_reason": Counter(),
            "braking_corridor_mode": Counter(),
            "dynamic_cap_gap": [],
            "target_gap": [],
            "speed": [],
            "abs_heading": [],
            "braking_corridor_v_lower": [],
            "braking_corridor_v_upper": [],
            "braking_corridor_band_width": [],
            "braking_corridor_active_count": 0,
            "under_ref_count": 0,
            "under_ref_no_throttle_count": 0,
        }
    )
    phase_bias_values: dict[str, list[float]] = {
        name: [] for name in PHASE_BIAS_COLUMNS
    }
    track_phase_bias_values: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {name: [] for name in PHASE_BIAS_COLUMNS}
    )
    segment_counter: Counter[str] = Counter()
    attack_allowed_counter: Counter[str] = Counter()
    attack_planner_values: dict[str, list[float]] = {
        name: [] for name in ATTACK_PLANNER_COLUMNS
    }
    course_reference_values: dict[str, list[float]] = {
        name: [] for name in COURSE_REFERENCE_COLUMNS
    }
    braking_corridor_values: dict[str, list[float]] = {
        "v_lower": [],
        "v_upper": [],
        "band_width": [],
        "speed_minus_v_upper": [],
        "v_lower_minus_speed": [],
    }
    braking_corridor_active_count = 0
    braking_corridor_mode_counter: Counter[str] = Counter()
    all_reject_count = 0
    predicted_margins: list[float] = []
    heading_errors: list[float] = []
    speeds: list[float] = []
    selected_target_speed_errors: list[float] = []
    selected_ax_values: list[float] = []
    healthy_underspeed_count = 0
    healthy_underspeed_throttle_values: list[float] = []
    under_ref_count = 0
    under_ref_no_throttle_count = 0
    healthy_low_speed_scale_count = 0
    low_speed_scale_count = 0
    row_dominant_cost_counter: Counter[str] = Counter()
    speed_scale_095_loss_counter: Counter[str] = Counter()
    cost_signed_values: dict[str, list[float]] = {name: [] for name in COST_COLUMNS}
    cost_positive_values: dict[str, list[float]] = {name: [] for name in COST_COLUMNS}
    speed_scale_cost_values: dict[str, list[float]] = {
        name: [] for name in SPEED_SCALE_COST_COLUMNS
    }
    speed_scale_095_delta_values: dict[str, list[float]] = {
        name: [] for name in SPEED_SCALE_095_DELTA_COLUMNS
    }
    speed_scale_095_cost_deltas: list[float] = []

    for row in rows:
        track_phase = row.get("track_phase")
        if track_phase in (None, ""):
            track_phase = "unknown"
        track_phase_counter[track_phase] += 1
        cap_reason = row.get("attack_limiting_cap_reason")
        if cap_reason in (None, ""):
            cap_reason = "unknown"
        cap_reason_counter[cap_reason] += 1
        track_phase_cap_reason_counter[track_phase][cap_reason] += 1
        primitive = row.get("attack_selected_primitive")
        primitive_counter[primitive if primitive not in (None, "") else "unknown"] += 1
        primitive_kind = row.get("attack_selected_primitive_kind")
        primitive_kind_counter[
            primitive_kind if primitive_kind not in (None, "") else "unknown"
        ] += 1
        speed_scale = _float_or_none(row.get("attack_selected_speed_scale"))
        speed_scale_key = f"{speed_scale:.2f}" if speed_scale is not None else "unknown"
        speed_scale_counter[speed_scale_key] += 1
        track_phase_speed_scale_counter[track_phase][speed_scale_key] += 1
        longitudinal_mode = row.get("longitudinal_mode")
        if longitudinal_mode in (None, ""):
            longitudinal_mode = "unknown"
        track_phase_longitudinal_mode_counter[track_phase][longitudinal_mode] += 1
        braking_corridor_mode = (
            longitudinal_mode
            if str(longitudinal_mode).startswith("braking_corridor_")
            else "not_corridor"
        )
        braking_corridor_mode_counter[braking_corridor_mode] += 1
        track_phase_braking_corridor_mode_counter[track_phase][braking_corridor_mode] += 1
        if _truthy(row.get("attack_all_rejected")):
            all_reject_count += 1
        margin = _float_or_none(row.get("mpc_predicted_min_margin"))
        if margin is not None:
            predicted_margins.append(margin)
        heading = _heading_error_rad(row)
        if heading is not None:
            heading_errors.append(abs(heading))
        speed = _float_or_none(row.get("speed"))
        if speed is not None:
            speeds.append(speed)
            track_phase_speed_values[track_phase].append(speed)
        for bias_name, column in PHASE_BIAS_COLUMNS.items():
            bias = _float_or_none(row.get(column))
            if bias is None:
                continue
            phase_bias_values[bias_name].append(bias)
            track_phase_bias_values[track_phase][bias_name].append(bias)
        segment_id = row.get("attack_segment_id")
        segment_key = segment_id if segment_id not in (None, "") else "unknown"
        segment_counter[segment_key] += 1
        segment_stats[segment_key]["count"] += 1
        segment_stats[segment_key]["phase"][track_phase] += 1
        segment_stats[segment_key]["cap_reason"][cap_reason] += 1
        segment_stats[segment_key]["braking_corridor_mode"][braking_corridor_mode] += 1
        if "attack_allowed" in row:
            attack_allowed_counter["true" if _truthy(row.get("attack_allowed")) else "false"] += 1
        for name, column in ATTACK_PLANNER_COLUMNS.items():
            value = _float_or_none(row.get(column))
            if value is not None:
                attack_planner_values[name].append(value)
        for name, column in COURSE_REFERENCE_COLUMNS.items():
            value = _float_or_none(row.get(column))
            if value is not None:
                if name == "lateral_error":
                    value = abs(value)
                course_reference_values[name].append(value)
        selected_target_speed = _float_or_none(row.get("attack_selected_target_speed"))
        if speed is not None and selected_target_speed is not None:
            selected_target_speed_errors.append(selected_target_speed - speed)
        corridor_active = _truthy(row.get("braking_corridor_active"))
        corridor_lower = _float_or_none(row.get("braking_corridor_v_lower"))
        corridor_upper = _float_or_none(row.get("braking_corridor_v_upper"))
        if corridor_active:
            braking_corridor_active_count += 1
            track_phase_braking_corridor_count[track_phase] += 1
            segment_stats[segment_key]["braking_corridor_active_count"] += 1
            if corridor_lower is not None:
                braking_corridor_values["v_lower"].append(corridor_lower)
                track_phase_braking_corridor_values[track_phase]["v_lower"].append(corridor_lower)
                segment_stats[segment_key]["braking_corridor_v_lower"].append(corridor_lower)
            if corridor_upper is not None:
                braking_corridor_values["v_upper"].append(corridor_upper)
                track_phase_braking_corridor_values[track_phase]["v_upper"].append(corridor_upper)
                segment_stats[segment_key]["braking_corridor_v_upper"].append(corridor_upper)
            if corridor_lower is not None and corridor_upper is not None:
                band_width = corridor_upper - corridor_lower
                braking_corridor_values["band_width"].append(band_width)
                track_phase_braking_corridor_values[track_phase]["band_width"].append(band_width)
                segment_stats[segment_key]["braking_corridor_band_width"].append(band_width)
                if speed is not None:
                    braking_corridor_values["speed_minus_v_upper"].append(speed - corridor_upper)
                    braking_corridor_values["v_lower_minus_speed"].append(corridor_lower - speed)
                    track_phase_braking_corridor_values[track_phase]["speed_minus_v_upper"].append(
                        speed - corridor_upper
                    )
                    track_phase_braking_corridor_values[track_phase]["v_lower_minus_speed"].append(
                        corridor_lower - speed
                    )
        selected_ax = _float_or_none(row.get("attack_selected_ax"))
        if selected_ax is not None:
            selected_ax_values.append(selected_ax)
        throttle = _float_or_none(row.get("throttle"))
        target_gap = _float_or_none(row.get("course_target_gap"))
        dynamic_cap_gap = _float_or_none(row.get("dynamic_cap_gap"))
        if speed is not None:
            segment_stats[segment_key]["speed"].append(speed)
        if heading is not None:
            segment_stats[segment_key]["abs_heading"].append(abs(heading))
        if target_gap is not None:
            segment_stats[segment_key]["target_gap"].append(target_gap)
        if dynamic_cap_gap is not None:
            segment_stats[segment_key]["dynamic_cap_gap"].append(dynamic_cap_gap)
        if target_gap is not None and target_gap > 2.0:
            under_ref_count += 1
            segment_stats[segment_key]["under_ref_count"] += 1
            if throttle is not None and throttle < 0.05:
                under_ref_no_throttle_count += 1
                segment_stats[segment_key]["under_ref_no_throttle_count"] += 1
        if _truthy(row.get("healthy_underspeed")):
            healthy_underspeed_count += 1
            if throttle is not None:
                healthy_underspeed_throttle_values.append(throttle)
        if speed_scale is not None and abs(speed_scale - 0.65) < 1e-6:
            boundary_reject_count = (
                _float_or_none(row.get("attack_rollout_boundary_reject_count")) or 0.0
            )
            low_speed_scale_count += 1
            if (
                primitive_kind == "tracking"
                and speed is not None
                and speed > 8.0
                and margin is not None
                and margin > 1.2
                and heading is not None
                and abs(heading) < 0.12
                and not _truthy(row.get("attack_all_rejected"))
                and boundary_reject_count <= 0.0
            ):
                healthy_low_speed_scale_count += 1
        row_costs: dict[str, float] = {}
        for name, column in COST_COLUMNS.items():
            value = _float_or_none(row.get(column))
            if value is None:
                continue
            cost_signed_values[name].append(value)
            positive_value = max(0.0, value)
            cost_positive_values[name].append(positive_value)
            row_costs[name] = positive_value
        if row_costs:
            row_dominant = max(row_costs, key=row_costs.get)
            row_dominant_cost_counter[row_dominant] += 1
            track_phase_dominant_cost_counter[track_phase][row_dominant] += 1
        else:
            row_dominant_cost_counter["unknown"] += 1
            track_phase_dominant_cost_counter[track_phase]["unknown"] += 1
        for name, column in SPEED_SCALE_COST_COLUMNS.items():
            value = _float_or_none(row.get(column))
            if value is not None:
                speed_scale_cost_values[name].append(value)
        cost_delta_095 = _float_or_none(
            row.get("attack_best_speed_scale_095_cost_delta")
        )
        if cost_delta_095 is not None:
            speed_scale_095_cost_deltas.append(cost_delta_095)
            delta_by_term: dict[str, float] = {}
            for name, column in SPEED_SCALE_095_DELTA_COLUMNS.items():
                value = _float_or_none(row.get(column))
                if value is None:
                    continue
                speed_scale_095_delta_values[name].append(value)
                delta_by_term[name] = max(0.0, value)
            if cost_delta_095 > 1e-9 and delta_by_term:
                speed_scale_095_loss_counter[
                    max(delta_by_term, key=delta_by_term.get)
                ] += 1

    total = len(rows)
    cost_means = {
        name: _mean(values) for name, values in cost_signed_values.items()
    }
    cost_positive_means = {
        name: _mean(values) for name, values in cost_positive_values.items()
    }
    cost_positive_percentiles = {
        name: _percentile_summary(values)
        for name, values in cost_positive_values.items()
    }
    speed_scale_best_cost_means = {
        name: _mean(values) for name, values in speed_scale_cost_values.items()
    }
    speed_scale_095_delta_means = {
        name: _mean(values) for name, values in speed_scale_095_delta_values.items()
    }
    dominant_cost_term = _dominant_cost(cost_positive_means)
    all_reject_ratio = all_reject_count / total if total > 0 else 0.0
    primitive_kind_ratio = _ratio(primitive_kind_counter, total)
    duration_s = None
    if rows:
        t0 = _float_or_none(rows[0].get("time"))
        t1 = _float_or_none(rows[-1].get("time"))
        if t0 is not None and t1 is not None:
            duration_s = max(0.0, t1 - t0)

    track_phase_time_s: Counter[str] = Counter()
    for index, row in enumerate(rows):
        phase = row.get("track_phase")
        if phase in (None, ""):
            phase = "unknown"
        t0 = _float_or_none(row.get("time"))
        if t0 is None:
            continue
        if index + 1 < len(rows):
            t1 = _float_or_none(rows[index + 1].get("time"))
        else:
            t1 = None
        if t1 is None:
            continue
        track_phase_time_s[phase] += max(0.0, t1 - t0)
    phase_time_total = sum(track_phase_time_s.values())

    min_predicted_margin_m = min(predicted_margins) if predicted_margins else None
    max_abs_heading_error_rad = max(heading_errors) if heading_errors else None
    mean_speed_mps = _mean(speeds)

    return {
        "sample_count": total,
        "duration_s": duration_s,
        "distance_end_m": _float_or_none(rows[-1].get("s")) if rows else None,
        "primitive_ratio": _ratio(primitive_counter, total),
        "primitive_kind_ratio": primitive_kind_ratio,
        "speed_scale_distribution": dict(sorted(speed_scale_counter.items())),
        "cap_reason_ratio": _ratio(cap_reason_counter, total),
        "track_phase_ratio": _ratio(track_phase_counter, total),
        "track_phase_time_s": dict(sorted(track_phase_time_s.items())),
        "track_phase_time_ratio": _ratio(track_phase_time_s, phase_time_total),
        "track_phase_speed_scale_distribution": {
            phase: _sorted_counter_dict(counter)
            for phase, counter in sorted(track_phase_speed_scale_counter.items())
        },
        "track_phase_cap_reason_distribution": {
            phase: _sorted_counter_dict(counter)
            for phase, counter in sorted(track_phase_cap_reason_counter.items())
        },
        "track_phase_longitudinal_mode_distribution": {
            phase: _sorted_counter_dict(counter)
            for phase, counter in sorted(track_phase_longitudinal_mode_counter.items())
        },
        "braking_corridor_active_ratio": (
            braking_corridor_active_count / total
            if total > 0
            else 0.0
        ),
        "braking_corridor_mode_distribution": _sorted_counter_dict(
            braking_corridor_mode_counter
        ),
        "braking_corridor_mode_ratio": _ratio(braking_corridor_mode_counter, total),
        "braking_corridor_means": {
            name: _mean(values)
            for name, values in braking_corridor_values.items()
        },
        "braking_corridor_percentiles": {
            name: _percentile_summary(values)
            for name, values in braking_corridor_values.items()
        },
        "track_phase_braking_corridor_active_ratio": {
            phase: (
                track_phase_braking_corridor_count[phase] / count
                if count > 0
                else 0.0
            )
            for phase, count in sorted(track_phase_counter.items())
        },
        "track_phase_braking_corridor_mode_distribution": {
            phase: _sorted_counter_dict(counter)
            for phase, counter in sorted(track_phase_braking_corridor_mode_counter.items())
        },
        "track_phase_braking_corridor_means": {
            phase: {
                name: _mean(values)
                for name, values in value_map.items()
            }
            for phase, value_map in sorted(track_phase_braking_corridor_values.items())
        },
        "track_phase_mean_speed_mps": {
            phase: _mean(values)
            for phase, values in sorted(track_phase_speed_values.items())
        },
        "track_phase_dominant_cost_ratio": {
            phase: _ratio(counter, sum(counter.values()))
            for phase, counter in sorted(track_phase_dominant_cost_counter.items())
        },
        "phase_bias_means": {
            name: _mean(values) for name, values in phase_bias_values.items()
        },
        "attack_segment_ratio": _ratio(segment_counter, total),
        "top_bottleneck_segments": _top_segment_bottlenecks(segment_stats),
        "attack_allowed_ratio": _ratio(attack_allowed_counter, sum(attack_allowed_counter.values())),
        "attack_planner_means": {
            name: _mean(values) for name, values in attack_planner_values.items()
        },
        "attack_planner_percentiles": {
            name: _percentile_summary(values)
            for name, values in attack_planner_values.items()
        },
        "course_reference_means": {
            name: _mean(values)
            for name, values in course_reference_values.items()
        },
        "course_reference_percentiles": {
            name: _percentile_summary(values)
            for name, values in course_reference_values.items()
        },
        "track_phase_bias_means": {
            phase: {
                name: _mean(values)
                for name, values in bias_values.items()
            }
            for phase, bias_values in sorted(track_phase_bias_values.items())
        },
        "all_reject_ratio": all_reject_ratio,
        "all_reject_segments": _all_reject_segments(rows),
        "dominant_cost_term": dominant_cost_term,
        "row_dominant_cost_ratio": _ratio(row_dominant_cost_counter, total),
        "cost_means": cost_means,
        "cost_positive_means": cost_positive_means,
        "cost_positive_percentiles": cost_positive_percentiles,
        "speed_scale_best_cost_means": speed_scale_best_cost_means,
        "speed_scale_095_cost_delta_mean": _mean(speed_scale_095_cost_deltas),
        "speed_scale_095_cost_delta_percentiles": _percentile_summary(
            speed_scale_095_cost_deltas
        ),
        "speed_scale_095_delta_means": speed_scale_095_delta_means,
        "speed_scale_095_loss_term_ratio": _ratio(
            speed_scale_095_loss_counter,
            sum(speed_scale_095_loss_counter.values()),
        ),
        "selected_target_speed_error_mean_mps": _mean(selected_target_speed_errors),
        "selected_target_speed_error_percentiles_mps": _percentile_summary(
            selected_target_speed_errors
        ),
        "selected_ax_mean_mps2": _mean(selected_ax_values),
        "selected_ax_percentiles_mps2": _percentile_summary(selected_ax_values),
        "healthy_underspeed_count": healthy_underspeed_count,
        "healthy_underspeed_ratio": (
            healthy_underspeed_count / total
            if total > 0
            else 0.0
        ),
        "healthy_underspeed_throttle_percentiles": _percentile_summary(
            healthy_underspeed_throttle_values
        ),
        "under_ref_count": under_ref_count,
        "under_ref_no_throttle_ratio": (
            under_ref_no_throttle_count / under_ref_count
            if under_ref_count > 0
            else 0.0
        ),
        "healthy_low_speed_scale_count": healthy_low_speed_scale_count,
        "healthy_low_speed_scale_ratio": (
            healthy_low_speed_scale_count / low_speed_scale_count
            if low_speed_scale_count > 0
            else 0.0
        ),
        "min_predicted_margin_m": min_predicted_margin_m,
        "max_abs_heading_error_rad": max_abs_heading_error_rad,
        "worst_heading_window": _worst_heading_window(rows, window_seconds),
        "estimated_failure_mode": _estimate_failure_mode(
            missing_columns=missing_columns,
            all_reject_ratio=all_reject_ratio,
            primitive_kind_ratio=primitive_kind_ratio,
            min_predicted_margin_m=min_predicted_margin_m,
            max_abs_heading_error_rad=max_abs_heading_error_rad,
            dominant_cost_term=dominant_cost_term,
            mean_speed_mps=mean_speed_mps,
        ),
        "missing_columns": missing_columns,
    }


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", dest="csv_path", required=True, type=Path)
    parser.add_argument("--json", dest="json_path", type=Path)
    parser.add_argument("--window-seconds", type=float, default=2.0)
    args = parser.parse_args()

    summary = summarize(read_csv(args.csv_path), window_seconds=args.window_seconds)
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.json_path is not None:
        args.json_path.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
