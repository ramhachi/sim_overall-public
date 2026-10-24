#!/usr/bin/env python3
"""Summarize laptime_sim driver traces by steering-lag-critical sections."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


NUMERIC_COLUMNS = {
    "time",
    "s",
    "n",
    "d_heading",
    "kappa",
    "speed",
    "target_speed_base",
    "target_speed_preview",
    "target_speed_final",
    "steering_preview_distance_used",
    "delta_target",
    "target_sw",
    "delta_sw",
    "ddelta_sw",
    "required_delta_preview",
    "actual_delta_tire",
    "predicted_delta_tire",
    "preview_delta_gap",
    "raw_delta_target",
    "inner_target_delta_tire",
    "actual_delta_tire_rate_pre",
    "lpv_prediction_error",
    "predicted_delta_tire_rate",
    "steering_prediction_horizon",
    "steering_prediction_target_time",
    "steering_prediction_valid",
    "steering_prediction_error_valid",
    "lpv_outer_sequence",
    "required_reference_s",
    "required_reference_time",
    "model_yaw_rate_map",
    "actual_yaw_rate_map",
    "yaw_rate_error_map",
    "sw_error",
    "T_driver_raw",
    "T_driver_clamped",
    "torque_headroom",
    "time_to_reach_delta",
    "steering_saturation",
    "torque_speed_multiplier",
    "torque_speed_recovery_ratio",
    "steering_lag_ratio",
    "chassis_twist_risk",
    "effective_max_tire_steer_rate",
    "steering_actuator_speed_cap",
    "steering_reach_speed_cap",
    "track_margin_speed_cap",
    "track_margin_effective_margin",
    "track_margin_outward_rate",
    "corner_throttle_steer_ratio",
    "corner_throttle_lateral_utilization",
    "corner_throttle_lateral_ratio",
    "corner_throttle_drive_slip",
    "corner_throttle_drive_slip_ratio",
    "corner_throttle_limit_ratio",
    "corner_throttle_scale",
    "phi_twist",
    "dphi_twist",
    "stability_vy_penalty",
    "stability_yaw_penalty",
    "stability_slip_penalty",
    "stability_penalty",
    "estimated_front_slip",
    "throttle",
    "brake",
}


@dataclass(frozen=True)
class Segment:
    label: str
    rows: list[dict[str, float]]


def parse_float(raw: str | None) -> float:
    if raw is None:
        return math.nan
    text = str(raw).strip()
    if text == "":
        return math.nan
    try:
        return float(text)
    except ValueError:
        lowered = text.lower()
        if lowered in {"inf", "+inf", "infinity", "+infinity"}:
            return math.inf
        if lowered in {"-inf", "-infinity"}:
            return -math.inf
        return math.nan


def finite(value: float) -> bool:
    return math.isfinite(value)


def val(row: dict[str, float], key: str, default: float = math.nan) -> float:
    value = row.get(key, default)
    return value if finite(value) else default


def load_trace(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            row: dict[str, float] = {}
            for key in NUMERIC_COLUMNS:
                if key in raw:
                    row[key] = parse_float(raw.get(key))
            rows.append(row)

    if not rows:
        raise ValueError(f"trace is empty: {path}")

    offset = 0.0
    prev_cont = val(rows[0], "s", 0.0)
    max_raw_s = max(prev_cont, 0.0)
    for row in rows:
        raw_s = val(row, "s", prev_cont)
        if raw_s + offset < prev_cont - 50.0:
            offset += max_raw_s
        cont_s = raw_s + offset
        row["_s_cont"] = cont_s
        prev_cont = cont_s
        max_raw_s = max(max_raw_s, raw_s)

    for i, row in enumerate(rows):
        row["_index"] = float(i)
        row["_corner_throttle_loss"] = max(0.0, 1.0 - val(row, "corner_throttle_scale", 1.0))
        preview_speed = val(row, "target_speed_preview", val(row, "target_speed_final", 0.0))
        final_speed = val(row, "target_speed_final", preview_speed)
        row["_target_speed_loss"] = max(0.0, preview_speed - final_speed)
        reach_cap = val(row, "steering_reach_speed_cap", math.inf)
        if finite(reach_cap) and preview_speed > 1e-6:
            row["_steering_reach_speed_loss"] = max(0.0, (preview_speed - reach_cap) / preview_speed)
        else:
            row["_steering_reach_speed_loss"] = 0.0
    return rows


def mean(values: list[float]) -> float:
    finite_values = [x for x in values if finite(x)]
    return statistics.fmean(finite_values) if finite_values else math.nan


def maximum(values: list[float]) -> float:
    finite_values = [x for x in values if finite(x)]
    return max(finite_values) if finite_values else math.nan


def minimum(values: list[float]) -> float:
    finite_values = [x for x in values if finite(x)]
    return min(finite_values) if finite_values else math.nan


def max_abs(values: list[float]) -> float:
    finite_values = [abs(x) for x in values if finite(x)]
    return max(finite_values) if finite_values else math.nan


def percentile(values: list[float], probability: float) -> float:
    finite_values = sorted(x for x in values if finite(x))
    if not finite_values:
        return math.nan
    position = min(max(probability, 0.0), 1.0) * (len(finite_values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite_values[lower]
    alpha = position - lower
    return (1.0 - alpha) * finite_values[lower] + alpha * finite_values[upper]


def pearson(x: list[float], y: list[float]) -> float:
    if len(x) != len(y) or len(x) < 3:
        return math.nan
    mx = statistics.fmean(x)
    my = statistics.fmean(y)
    xx = sum((value - mx) ** 2 for value in x)
    yy = sum((value - my) ** 2 for value in y)
    if xx <= 1e-18 or yy <= 1e-18:
        return math.nan
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / math.sqrt(xx * yy)


def actuator_model_diagnostics(rows: list[dict[str, float]]) -> dict[str, Any]:
    outer_rows: list[dict[str, float]] = []
    seen_sequences: set[int] = set()
    for row in rows:
        sequence_value = val(row, "lpv_outer_sequence")
        if not finite(sequence_value):
            continue
        sequence = int(round(sequence_value))
        if sequence <= 0 or sequence in seen_sequences:
            continue
        seen_sequences.add(sequence)
        if val(row, "steering_prediction_error_valid", 0.0) > 0.5:
            outer_rows.append(row)

    errors = [val(row, "lpv_prediction_error") for row in outer_rows]
    errors = [error for error in errors if finite(error)]
    actual = [val(row, "actual_delta_tire") for row in outer_rows]
    actual = [value for value in actual if finite(value)]
    prediction_rmse = (
        math.sqrt(statistics.fmean(error * error for error in errors))
        if errors
        else math.nan
    )
    robust_span = percentile(actual, 0.95) - percentile(actual, 0.05)
    normalization = max(robust_span, 0.05) if finite(robust_span) else math.nan
    prediction_nrmse = (
        prediction_rmse / normalization
        if finite(prediction_rmse) and finite(normalization) and normalization > 0.0
        else math.nan
    )

    time_values = [val(row, "time") for row in rows]
    finite_times = [value for value in time_values if finite(value)]
    dts = [b - a for a, b in zip(finite_times, finite_times[1:]) if b > a]
    sample_dt = statistics.median(dts) if dts else math.nan
    targets = [val(row, "inner_target_delta_tire") for row in rows]
    measured = [val(row, "actual_delta_tire") for row in rows]
    paired = [
        (target, value)
        for target, value in zip(targets, measured)
        if finite(target) and finite(value)
    ]
    best_lag_samples = 0
    best_correlation = math.nan
    target_excitation = statistics.pstdev([item[0] for item in paired]) if len(paired) >= 2 else 0.0
    if finite(sample_dt) and sample_dt > 0.0 and target_excitation >= 0.01:
        max_lag_samples = min(int(round(0.100 / sample_dt)), max(len(paired) - 3, 0))
        for lag in range(max_lag_samples + 1):
            delayed_target = [item[0] for item in paired[: len(paired) - lag or None]]
            delayed_actual = [item[1] for item in paired[lag:]]
            correlation = pearson(delayed_target, delayed_actual)
            if finite(correlation) and (not finite(best_correlation) or correlation > best_correlation):
                best_correlation = correlation
                best_lag_samples = lag

    result = {
        "aligned_sample_count": len(errors),
        "prediction_bias_rad": mean(errors),
        "prediction_mae_rad": mean([abs(error) for error in errors]),
        "prediction_rmse_rad": prediction_rmse,
        "prediction_max_abs_error_rad": max_abs(errors),
        "actual_p05_rad": percentile(actual, 0.05),
        "actual_p95_rad": percentile(actual, 0.95),
        "normalization_span_rad": normalization,
        "prediction_nrmse": prediction_nrmse,
        "target_excitation_std_rad": target_excitation,
        "target_actual_lag_ms": (
            1000.0 * best_lag_samples * sample_dt
            if finite(best_correlation)
            else math.nan
        ),
        "target_actual_lag_correlation": best_correlation,
        "sample_dt_s": sample_dt,
    }
    if (
        len(errors) < 100
        or not finite(prediction_nrmse)
        or target_excitation < 0.01
    ):
        result["gate_status"] = "insufficient_excitation"
    elif (
        abs(result["prediction_bias_rad"]) <= 0.005
        and result["prediction_rmse_rad"] <= 0.02
        and result["prediction_nrmse"] <= 0.15
        and result["prediction_max_abs_error_rad"] <= 0.10
    ):
        result["gate_status"] = "pass"
    else:
        result["gate_status"] = "fail"
    return result


def yaw_gain_diagnostics(rows: list[dict[str, float]]) -> dict[str, Any]:
    steady_rows = [
        row
        for row in rows
        if abs(val(row, "kappa", 0.0)) >= 0.01
        and val(row, "speed", 0.0) >= 5.0
        and abs(val(row, "actual_delta_tire", 0.0)) >= 0.01
        and abs(val(row, "actual_delta_tire_rate_pre", math.inf)) <= 0.05
        and val(row, "steering_saturation", 0.0) < 0.5
    ]
    actual_input = [-val(row, "actual_delta_tire") for row in steady_rows]
    actual_output = [val(row, "actual_yaw_rate_map") for row in steady_rows]
    model_input = [-val(row, "required_delta_preview") for row in steady_rows]
    model_output = [val(row, "model_yaw_rate_map") for row in steady_rows]

    def slope(inputs: list[float], outputs: list[float]) -> float:
        pairs = [(x, y) for x, y in zip(inputs, outputs) if finite(x) and finite(y)]
        denominator = sum(x * x for x, _ in pairs)
        return sum(x * y for x, y in pairs) / denominator if denominator > 1e-12 else math.nan

    actual_gain = slope(actual_input, actual_output)
    model_gain = slope(model_input, model_output)
    mismatch = (
        abs(actual_gain - model_gain) / abs(model_gain)
        if finite(actual_gain) and finite(model_gain) and abs(model_gain) > 1e-9
        else math.nan
    )
    yaw_bias = mean(
        [
            output - actual_gain * input_value
            for input_value, output in zip(actual_input, actual_output)
        ]
    )
    result = {
        "steady_sample_count": len(steady_rows),
        "actual_yaw_gain_per_s": actual_gain,
        "model_yaw_gain_per_s": model_gain,
        "relative_yaw_gain_mismatch": mismatch,
        "yaw_bias_rad_s": yaw_bias,
        "mean_abs_yaw_rate_error_rad_s": mean(
            [abs(val(row, "yaw_rate_error_map")) for row in steady_rows]
        ),
    }
    if len(steady_rows) < 100 or not finite(mismatch):
        result["gate_status"] = "insufficient_steady_state"
    elif mismatch <= 0.10 and finite(yaw_bias) and abs(yaw_bias) <= 0.02:
        result["gate_status"] = "pass"
    else:
        result["gate_status"] = "fail"
    return result


def row_for(rows: list[dict[str, float]], key: str, mode: str = "max_abs") -> dict[str, float]:
    best = rows[0]
    best_score = -math.inf
    for row in rows:
        value = val(row, key)
        if mode == "min":
            score = -value if finite(value) else -math.inf
        elif mode == "max":
            score = value if finite(value) else -math.inf
        else:
            score = abs(value) if finite(value) else -math.inf
        if score > best_score:
            best = row
            best_score = score
    return best


def summarize(label: str, rows: list[dict[str, float]]) -> dict[str, Any]:
    if not rows:
        raise ValueError("cannot summarize empty rows")

    s0 = val(rows[0], "_s_cont", 0.0)
    s1 = val(rows[-1], "_s_cont", s0)
    t0 = val(rows[0], "time", 0.0)
    t1 = val(rows[-1], "time", t0)
    min_margin = minimum([val(r, "track_margin_effective_margin") for r in rows])
    max_time_to_reach = maximum([val(r, "time_to_reach_delta") for r in rows])
    max_lag = maximum([val(r, "steering_lag_ratio") for r in rows])
    mean_lag = mean([val(r, "steering_lag_ratio") for r in rows])
    max_preview_gap = max_abs([val(r, "preview_delta_gap") for r in rows])
    mean_corner_loss = mean([val(r, "_corner_throttle_loss") for r in rows])
    mean_speed_loss = mean([val(r, "_target_speed_loss") for r in rows])
    mean_reach_cap_loss = mean([val(r, "_steering_reach_speed_loss") for r in rows])
    max_track_excess = max(0.0, -min_margin) if finite(min_margin) else math.nan

    risk_score = 0.0
    if finite(max_lag):
        risk_score += 2.0 * max_lag
    if finite(max_time_to_reach):
        risk_score += 1.5 * max_time_to_reach
    if finite(max_preview_gap):
        risk_score += 2.0 * max_preview_gap
    if finite(max_track_excess):
        risk_score += 0.5 * max_track_excess
    if finite(mean_corner_loss):
        risk_score += 0.5 * mean_corner_loss

    return {
        "label": label,
        "row_count": len(rows),
        "s_start": s0,
        "s_end": s1,
        "length_m": max(0.0, s1 - s0),
        "time_start": t0,
        "time_end": t1,
        "duration_s": max(0.0, t1 - t0),
        "mean_speed": mean([val(r, "speed") for r in rows]),
        "max_speed": maximum([val(r, "speed") for r in rows]),
        "min_speed": minimum([val(r, "speed") for r in rows]),
        "max_abs_lateral_error": max_abs([val(r, "n") for r in rows]),
        "min_projected_track_margin": min_margin,
        "max_outward_rate": maximum([val(r, "track_margin_outward_rate") for r in rows]),
        "mean_steering_lag_ratio": mean_lag,
        "max_steering_lag_ratio": max_lag,
        "max_abs_sw_error": max_abs([val(r, "sw_error") for r in rows]),
        "max_abs_preview_delta_gap": max_preview_gap,
        "mean_abs_preview_delta_gap": mean([abs(val(r, "preview_delta_gap")) for r in rows]),
        "max_time_to_reach_delta": max_time_to_reach,
        "mean_torque_speed_multiplier": mean([val(r, "torque_speed_multiplier") for r in rows]),
        "mean_target_speed_loss": mean_speed_loss,
        "mean_steering_reach_speed_loss": mean_reach_cap_loss,
        "mean_corner_throttle_loss": mean_corner_loss,
        "mean_throttle": mean([val(r, "throttle") for r in rows]),
        "mean_brake": mean([val(r, "brake") for r in rows]),
        "max_steering_saturation": maximum([val(r, "steering_saturation") for r in rows]),
        "max_abs_delta_target": max_abs([val(r, "delta_target") for r in rows]),
        "max_abs_actual_delta_tire": max_abs([val(r, "actual_delta_tire") for r in rows]),
        "max_abs_required_delta_preview": max_abs([val(r, "required_delta_preview") for r in rows]),
        "risk_score": risk_score,
    }


def curvature_demand(row: dict[str, float]) -> float:
    curvature = max(abs(val(row, "kappa", 0.0)), 0.0)
    required_delta = abs(val(row, "required_delta_preview", 0.0))
    target_delta = abs(val(row, "delta_target", 0.0))
    return max(curvature, required_delta / 8.0, target_delta / 10.0)


def build_curvature_segments(
    rows: list[dict[str, float]],
    curvature_threshold: float,
    min_length_m: float,
    merge_gap_m: float,
) -> list[Segment]:
    raw_segments: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []
    for row in rows:
        active = curvature_demand(row) >= curvature_threshold
        if active:
            current.append(row)
        elif current:
            raw_segments.append(current)
            current = []
    if current:
        raw_segments.append(current)

    merged: list[list[dict[str, float]]] = []
    for segment in raw_segments:
        if not merged:
            merged.append(segment)
            continue
        gap = val(segment[0], "_s_cont") - val(merged[-1][-1], "_s_cont")
        if gap <= merge_gap_m:
            merged[-1].extend(segment)
        else:
            merged.append(segment)

    filtered: list[Segment] = []
    for idx, segment in enumerate(merged, start=1):
        length = val(segment[-1], "_s_cont") - val(segment[0], "_s_cont")
        if length >= min_length_m or max_abs([val(r, "required_delta_preview") for r in segment]) >= 0.12:
            filtered.append(Segment(label=f"C{idx:02d}", rows=segment))
    return filtered


def build_distance_windows(rows: list[dict[str, float]], window_m: float) -> list[Segment]:
    if window_m <= 1e-6:
        return []
    start = math.floor(val(rows[0], "_s_cont", 0.0) / window_m) * window_m
    end = val(rows[-1], "_s_cont", start)
    windows: list[Segment] = []
    index = 1
    while start <= end + 1e-6:
        stop = start + window_m
        subset = [r for r in rows if start <= val(r, "_s_cont") < stop]
        if len(subset) >= 2:
            windows.append(Segment(label=f"W{index:02d}", rows=subset))
            index += 1
        start = stop
    return windows


def fmt(value: Any, digits: int = 3) -> str:
    if isinstance(value, str):
        return value
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if not finite(number):
        return ""
    return f"{number:.{digits}f}"


def metric_row(rows: list[dict[str, float]], label: str, key: str, mode: str = "max_abs") -> str:
    row = row_for(rows, key, mode)
    value = val(row, key)
    if mode == "max_abs":
        value = abs(value)
    return (
        f"| {label} | {fmt(value)} | {fmt(val(row, 'time'))} | "
        f"{fmt(val(row, '_s_cont'))} | {fmt(val(row, 'speed'))} | "
        f"{fmt(val(row, 'n'))} | {fmt(val(row, 'target_speed_final'))} | "
        f"{fmt(val(row, 'throttle'))} | {fmt(val(row, 'brake'))} |"
    )


def table_for_summaries(summaries: list[dict[str, Any]], limit: int) -> list[str]:
    lines = [
        "| id | s_range_m | len_m | mean_v | max_v | max_abs_n | min_proj_margin | mean_lag | max_lag | max_reach_s | max_gap_rad | reach_loss | corner_loss | brake | risk |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summaries[:limit]:
        lines.append(
            "| {label} | {s0}-{s1} | {length} | {mean_v} | {max_v} | {max_n} | {margin} | {mean_lag} | {max_lag} | {reach} | {gap} | {reach_loss} | {corner} | {brake} | {risk} |".format(
                label=item["label"],
                s0=fmt(item["s_start"], 1),
                s1=fmt(item["s_end"], 1),
                length=fmt(item["length_m"], 1),
                mean_v=fmt(item["mean_speed"]),
                max_v=fmt(item["max_speed"]),
                max_n=fmt(item["max_abs_lateral_error"]),
                margin=fmt(item["min_projected_track_margin"]),
                mean_lag=fmt(item["mean_steering_lag_ratio"]),
                max_lag=fmt(item["max_steering_lag_ratio"]),
                reach=fmt(item["max_time_to_reach_delta"]),
                gap=fmt(item["max_abs_preview_delta_gap"]),
                reach_loss=fmt(item["mean_steering_reach_speed_loss"]),
                corner=fmt(item["mean_corner_throttle_loss"]),
                brake=fmt(item["mean_brake"]),
                risk=fmt(item["risk_score"]),
            )
        )
    return lines


def write_report(path: Path, trace_path: Path, rows: list[dict[str, float]], summary: dict[str, Any]) -> None:
    global_summary = summary["global"]
    actuator = summary["actuator_model"]
    yaw_gain = summary["yaw_gain"]
    curvature = summary["curvature_segments"]
    windows = summary["top_windows"]

    lines = [
        "# Driver Trace Analysis",
        "",
        f"- trace_csv: `{trace_path}`",
        f"- rows: {len(rows)}",
        f"- time_range_s: {fmt(global_summary['time_start'])} - {fmt(global_summary['time_end'])}",
        f"- distance_range_m: {fmt(global_summary['s_start'], 1)} - {fmt(global_summary['s_end'], 1)}",
        "",
        "## Global Summary",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| mean_speed | {fmt(global_summary['mean_speed'])} |",
        f"| max_speed | {fmt(global_summary['max_speed'])} |",
        f"| max_abs_lateral_error | {fmt(global_summary['max_abs_lateral_error'])} |",
        f"| min_projected_track_margin | {fmt(global_summary['min_projected_track_margin'])} |",
        f"| mean_steering_lag_ratio | {fmt(global_summary['mean_steering_lag_ratio'])} |",
        f"| max_steering_lag_ratio | {fmt(global_summary['max_steering_lag_ratio'])} |",
        f"| max_abs_preview_delta_gap | {fmt(global_summary['max_abs_preview_delta_gap'])} |",
        f"| max_time_to_reach_delta | {fmt(global_summary['max_time_to_reach_delta'])} |",
        f"| mean_torque_speed_multiplier | {fmt(global_summary['mean_torque_speed_multiplier'])} |",
        f"| mean_target_speed_loss | {fmt(global_summary['mean_target_speed_loss'])} |",
        f"| mean_steering_reach_speed_loss | {fmt(global_summary['mean_steering_reach_speed_loss'])} |",
        f"| mean_corner_throttle_loss | {fmt(global_summary['mean_corner_throttle_loss'])} |",
        "",
        "## Actuator Model Diagnostics",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| aligned_outer_samples | {actuator['aligned_sample_count']} |",
        f"| gate_status | {actuator['gate_status']} |",
        f"| prediction_bias_rad | {fmt(actuator['prediction_bias_rad'], 6)} |",
        f"| prediction_mae_rad | {fmt(actuator['prediction_mae_rad'], 6)} |",
        f"| prediction_rmse_rad | {fmt(actuator['prediction_rmse_rad'], 6)} |",
        f"| prediction_nrmse | {fmt(actuator['prediction_nrmse'], 6)} |",
        f"| prediction_max_abs_error_rad | {fmt(actuator['prediction_max_abs_error_rad'], 6)} |",
        f"| inner_target_to_actual_lag_ms | {fmt(actuator['target_actual_lag_ms'])} |",
        f"| lag_correlation | {fmt(actuator['target_actual_lag_correlation'])} |",
        "",
        "## Yaw Gain Diagnostics",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| steady_samples | {yaw_gain['steady_sample_count']} |",
        f"| gate_status | {yaw_gain['gate_status']} |",
        f"| actual_yaw_gain_per_s | {fmt(yaw_gain['actual_yaw_gain_per_s'], 6)} |",
        f"| model_yaw_gain_per_s | {fmt(yaw_gain['model_yaw_gain_per_s'], 6)} |",
        f"| relative_yaw_gain_mismatch | {fmt(yaw_gain['relative_yaw_gain_mismatch'], 6)} |",
        f"| yaw_bias_rad_s | {fmt(yaw_gain['yaw_bias_rad_s'], 6)} |",
        "",
        "## Critical Points",
        "",
        "| metric | value | time | s | speed | n | target_speed | throttle | brake |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        metric_row(rows, "max_abs_lateral_error", "n"),
        metric_row(rows, "min_projected_track_margin", "track_margin_effective_margin", "min"),
        metric_row(rows, "max_steering_lag_ratio", "steering_lag_ratio", "max"),
        metric_row(rows, "max_abs_preview_delta_gap", "preview_delta_gap"),
        metric_row(rows, "max_time_to_reach_delta", "time_to_reach_delta", "max"),
        metric_row(rows, "max_brake", "brake", "max"),
        "",
        "## Curvature Segments",
        "",
    ]
    lines.extend(table_for_summaries(curvature, limit=30))
    lines.extend(["", "## Top Risk Windows", ""])
    lines.extend(table_for_summaries(windows, limit=10))

    if windows:
        top = windows[0]
        lines.extend(
            [
                "",
                "## Notes",
                "",
                f"- Highest-risk window: `{top['label']}` at s={fmt(top['s_start'], 1)}-{fmt(top['s_end'], 1)} m.",
                f"- In that window max steering lag ratio is {fmt(top['max_steering_lag_ratio'])}, "
                f"max preview gap is {fmt(top['max_abs_preview_delta_gap'])} rad, "
                f"and min projected track margin is {fmt(top['min_projected_track_margin'])} m.",
            ]
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(args: argparse.Namespace) -> tuple[dict[str, Any], list[dict[str, float]]]:
    trace_path = Path(args.trace).expanduser().resolve()
    rows = load_trace(trace_path)
    curvature_segments = build_curvature_segments(
        rows,
        curvature_threshold=args.curvature_threshold,
        min_length_m=args.min_segment_length,
        merge_gap_m=args.merge_gap,
    )
    windows = build_distance_windows(rows, args.window_m)

    global_summary = summarize("global", rows)
    curvature_summaries = [summarize(seg.label, seg.rows) for seg in curvature_segments]
    window_summaries = [summarize(seg.label, seg.rows) for seg in windows]
    window_summaries.sort(key=lambda x: x["risk_score"], reverse=True)

    summary = {
        "schema_version": "driver_trace_analysis/v2",
        "trace_csv": str(trace_path),
        "global": global_summary,
        "actuator_model": actuator_model_diagnostics(rows),
        "yaw_gain": yaw_gain_diagnostics(rows),
        "curvature_segments": curvature_summaries,
        "top_windows": window_summaries[: args.top_windows],
    }
    return summary, rows


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, help="CSV trace from laptime_sim --output.")
    parser.add_argument("--output", default="", help="Markdown report path.")
    parser.add_argument("--summary-json", default="", help="Optional machine-readable summary path.")
    parser.add_argument("--window-m", type=float, default=25.0)
    parser.add_argument("--top-windows", type=int, default=10)
    parser.add_argument("--curvature-threshold", type=float, default=0.010)
    parser.add_argument("--min-segment-length", type=float, default=5.0)
    parser.add_argument("--merge-gap", type=float, default=5.0)
    args = parser.parse_args(argv)

    summary, rows = analyze(args)
    trace_path = Path(args.trace).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve() if args.output else trace_path.with_suffix(".driver_report.md")
    write_report(output_path, trace_path, rows, summary)

    if args.summary_json:
        summary_path = Path(args.summary_json).expanduser().resolve()
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"[done] summary: {summary_path}")
    print(f"[done] report:  {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
