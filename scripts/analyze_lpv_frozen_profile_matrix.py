#!/usr/bin/env python3
"""Compute station-weighted metrics for the frozen-profile LPV-LQR matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable


CASE_NAMES = (
    "nominal_bicycle4",
    "nominal_steering6",
    "steering_inertia_plus50_bicycle4",
    "steering_inertia_plus50_steering6",
    "mass_inertia_plus20_bicycle4",
    "mass_inertia_plus20_steering6",
)

SIGNALS = (
    "n",
    "d_heading",
    "sw_error",
    "required_delta_preview",
    "raw_delta_target",
    "delta_target",
    "target_speed_base",
)


def finite_float(value: str, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"non-finite {name}: {value!r}")
    return result


def load_first_lap(path: Path, track_length: float) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    offset = 0.0
    previous_wrapped = 0.0
    previous_progress = -math.inf
    with path.open(newline="", encoding="utf-8") as stream:
        for row_index, row in enumerate(csv.DictReader(stream)):
            wrapped = finite_float(row["s"], "s")
            if row_index > 0 and wrapped < previous_wrapped - 0.5 * track_length:
                offset += track_length
            previous_wrapped = wrapped
            progress = wrapped + offset
            if progress > track_length + 1e-6:
                break
            if progress <= previous_progress + 1e-9:
                continue
            sample = {"s": min(progress, track_length)}
            for signal in SIGNALS:
                sample[signal] = finite_float(row[signal], signal)
            samples.append(sample)
            previous_progress = progress
    if len(samples) < 2 or samples[0]["s"] > 1e-6:
        raise ValueError(f"{path}: trace does not start at station zero")
    if samples[-1]["s"] < track_length - 1.0:
        raise ValueError(
            f"{path}: trace covers only {samples[-1]['s']:.3f} / {track_length:.3f} m"
        )
    return samples


def interpolate(
    samples: list[dict[str, float]], stations: Iterable[float]
) -> list[dict[str, float]]:
    output: list[dict[str, float]] = []
    upper = 1
    for station in stations:
        while upper < len(samples) and samples[upper]["s"] < station:
            upper += 1
        if upper >= len(samples):
            upper = len(samples) - 1
        lower = max(0, upper - 1)
        left = samples[lower]
        right = samples[upper]
        span = right["s"] - left["s"]
        alpha = 0.0 if span <= 1e-12 else (station - left["s"]) / span
        alpha = min(max(alpha, 0.0), 1.0)
        row = {"s": station}
        for signal in SIGNALS:
            row[signal] = left[signal] + alpha * (right[signal] - left[signal])
        output.append(row)
    return output


def rms(values: Iterable[float]) -> float:
    data = list(values)
    return math.sqrt(sum(value * value for value in data) / len(data))


def percentile(values: Iterable[float], probability: float) -> float:
    data = sorted(values)
    position = probability * (len(data) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return data[lower]
    alpha = position - lower
    return data[lower] + alpha * (data[upper] - data[lower])


def summarize(rows: list[dict[str, float]]) -> dict[str, float]:
    lateral_abs = [abs(row["n"]) for row in rows]
    projection = [row["delta_target"] - row["raw_delta_target"] for row in rows]
    feedback = [row["raw_delta_target"] - row["required_delta_preview"] for row in rows]
    return {
        "station_lateral_rms_m": rms(row["n"] for row in rows),
        "station_lateral_p95_abs_m": percentile(lateral_abs, 0.95),
        "station_lateral_max_abs_m": max(lateral_abs),
        "station_heading_rms_deg": math.degrees(rms(row["d_heading"] for row in rows)),
        "station_sw_mean_abs_error_rad": sum(abs(row["sw_error"]) for row in rows)
        / len(rows),
        "station_feedback_rms_rad": rms(feedback),
        "station_safety_projection_rms_rad": rms(projection),
        "station_safety_projection_active_ratio": (
            sum(abs(value) > 1e-4 for value in projection) / len(projection)
        ),
    }


def analyze(
    matrix_dir: Path, profile_path: Path, station_step: float
) -> dict[str, object]:
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    track_length = float(profile["track"]["length_m"])
    fingerprint = str(profile["profile_fingerprint"])
    stations = [
        min(index * station_step, track_length)
        for index in range(math.ceil(track_length / station_step) + 1)
    ]
    if stations[-1] != track_length:
        stations[-1] = track_length

    interpolated: dict[str, list[dict[str, float]]] = {}
    metrics: dict[str, dict[str, float]] = {}
    for case_name in CASE_NAMES:
        trace_path = matrix_dir / f"{case_name}.csv"
        rows = interpolate(load_first_lap(trace_path, track_length), stations)
        interpolated[case_name] = rows
        metrics[case_name] = summarize(rows)

    paired: dict[str, dict[str, float]] = {}
    for vehicle_name in ("nominal", "steering_inertia_plus50", "mass_inertia_plus20"):
        base_name = f"{vehicle_name}_bicycle4"
        candidate_name = f"{vehicle_name}_steering6"
        base_rows = interpolated[base_name]
        candidate_rows = interpolated[candidate_name]
        paired[vehicle_name] = {
            "max_abs_base_speed_difference_mps": max(
                abs(left["target_speed_base"] - right["target_speed_base"])
                for left, right in zip(base_rows, candidate_rows)
            ),
            "lateral_rms_delta_m": (
                metrics[candidate_name]["station_lateral_rms_m"]
                - metrics[base_name]["station_lateral_rms_m"]
            ),
            "lateral_rms_ratio": (
                metrics[candidate_name]["station_lateral_rms_m"]
                / metrics[base_name]["station_lateral_rms_m"]
            ),
            "safety_projection_rms_delta_rad": (
                metrics[candidate_name]["station_safety_projection_rms_rad"]
                - metrics[base_name]["station_safety_projection_rms_rad"]
            ),
            "safety_projection_active_delta": (
                metrics[candidate_name]["station_safety_projection_active_ratio"]
                - metrics[base_name]["station_safety_projection_active_ratio"]
            ),
        }

    return {
        "schema": "fsae_lpv_frozen_profile_matrix_v1",
        "profile_path": str(profile_path),
        "profile_fingerprint": fingerprint,
        "track_length_m": track_length,
        "station_step_m": station_step,
        "station_count": len(stations),
        "cases": metrics,
        "paired": paired,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-dir", required=True, type=Path)
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--station-step", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.station_step) or args.station_step <= 0.0:
        parser.error("--station-step must be finite and positive")
    result = analyze(args.matrix_dir, args.profile, args.station_step)
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
