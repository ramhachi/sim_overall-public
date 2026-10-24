#!/usr/bin/env python3
"""Generate a small attack-driver residual profile from an attack CSV log.

The profile is intentionally simple: it finds high-curvature bottleneck
segments where the car is under the fixed-course reference speed without
consistent throttle, then adds a bounded steering feed-forward correction with
the driver convention used by the C++ attack controller.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Iterable


@dataclass
class SegmentStats:
    segment_id: str
    phase: str
    s_min: float
    s_max: float
    count: int
    mean_kappa: float
    mean_abs_kappa: float
    mean_target_gap: float
    mean_abs_heading: float
    under_ref_no_throttle_ratio: float
    score: float


@dataclass
class ResidualWindow:
    s_start: float
    s_end: float
    delta_steer_ff_rad: float
    source_segments: list[str]


def _finite_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(key, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def collect_segment_stats(rows: Iterable[dict[str, str]]) -> list[SegmentStats]:
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        segment_id = row.get("attack_segment_id", "-1")
        if segment_id in {"", "-1", "unknown"}:
            continue
        grouped.setdefault(segment_id, []).append(row)

    stats: list[SegmentStats] = []
    for segment_id, segment_rows in grouped.items():
        if not segment_rows:
            continue
        s_values = [_finite_float(row, "s") for row in segment_rows]
        kappa_values = [_finite_float(row, "course_kappa_ref") for row in segment_rows]
        target_gaps = [_finite_float(row, "course_target_gap") for row in segment_rows]
        headings = [_finite_float(row, "d_heading") for row in segment_rows]
        throttle_values = [_finite_float(row, "throttle") for row in segment_rows]
        phases = [row.get("track_phase", "unknown") for row in segment_rows]
        phase = max(set(phases), key=phases.count) if phases else "unknown"
        under_ref_no_throttle = [
            1.0 if gap > 1.5 and throttle < 0.08 else 0.0
            for gap, throttle in zip(target_gaps, throttle_values)
        ]
        mean_abs_kappa = fmean(abs(value) for value in kappa_values)
        mean_target_gap = fmean(target_gaps)
        under_ratio = fmean(under_ref_no_throttle)
        mean_abs_heading = fmean(abs(value) for value in headings)
        score = (
            len(segment_rows)
            * max(0.0, mean_target_gap)
            * (0.35 + under_ratio)
            * (1.0 + 8.0 * mean_abs_kappa)
        )
        stats.append(
            SegmentStats(
                segment_id=segment_id,
                phase=phase,
                s_min=min(s_values),
                s_max=max(s_values),
                count=len(segment_rows),
                mean_kappa=fmean(kappa_values),
                mean_abs_kappa=mean_abs_kappa,
                mean_target_gap=mean_target_gap,
                mean_abs_heading=mean_abs_heading,
                under_ref_no_throttle_ratio=under_ratio,
                score=score,
            )
        )
    return sorted(stats, key=lambda item: item.score, reverse=True)


def select_residual_windows(
    stats: Iterable[SegmentStats],
    *,
    max_segments: int,
    steer_mag_rad: float,
    edge_trim_m: float,
    merge_gap_m: float,
    quantize_m: float,
    allowed_phases: set[str],
    min_abs_kappa: float,
    min_target_gap: float,
    min_under_ref_no_throttle_ratio: float,
) -> list[ResidualWindow]:
    selected: list[ResidualWindow] = []
    for item in stats:
        if len(selected) >= max_segments:
            break
        if item.phase not in allowed_phases:
            continue
        if item.mean_abs_kappa < min_abs_kappa:
            continue
        if item.mean_target_gap < min_target_gap:
            continue
        if item.under_ref_no_throttle_ratio < min_under_ref_no_throttle_ratio:
            continue
        sign = -1.0 if item.mean_kappa > 0.0 else 1.0
        window_start = max(0.0, item.s_min + edge_trim_m)
        window_end = max(window_start, item.s_max - edge_trim_m)
        selected.append(
            ResidualWindow(
                s_start=window_start,
                s_end=window_end,
                delta_steer_ff_rad=sign * abs(steer_mag_rad),
                source_segments=[item.segment_id],
            )
        )

    if not selected:
        return []

    selected.sort(key=lambda window: window.s_start)
    merged: list[ResidualWindow] = [selected[0]]
    for window in selected[1:]:
        last = merged[-1]
        same_sign = last.delta_steer_ff_rad * window.delta_steer_ff_rad > 0.0
        if same_sign and window.s_start <= last.s_end + merge_gap_m:
            last.s_end = max(last.s_end, window.s_end)
            last.source_segments.extend(window.source_segments)
        else:
            merged.append(window)
    if quantize_m > 1e-9:
        for window in merged:
            window.s_start = math.floor(window.s_start / quantize_m) * quantize_m
            window.s_end = math.floor(window.s_end / quantize_m) * quantize_m
            if window.s_end <= window.s_start:
                window.s_end = window.s_start + quantize_m
    return merged


def build_profile(
    windows: Iterable[ResidualWindow],
    *,
    ramp_m: float,
    max_delta_v_ref_mps: float,
    max_delta_steer_ff_rad: float,
    description: str,
) -> dict[str, object]:
    samples: list[dict[str, float]] = [
        {"s": 0.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.0}
    ]
    for window in windows:
        start_ramp = max(0.0, window.s_start - ramp_m)
        end_ramp = window.s_end + ramp_m
        samples.extend(
            [
                {
                    "s": start_ramp,
                    "delta_v_ref_mps": 0.0,
                    "delta_steer_ff_rad": 0.0,
                },
                {
                    "s": window.s_start,
                    "delta_v_ref_mps": 0.0,
                    "delta_steer_ff_rad": window.delta_steer_ff_rad,
                },
                {
                    "s": window.s_end,
                    "delta_v_ref_mps": 0.0,
                    "delta_steer_ff_rad": window.delta_steer_ff_rad,
                },
                {
                    "s": end_ramp,
                    "delta_v_ref_mps": 0.0,
                    "delta_steer_ff_rad": 0.0,
                },
            ]
        )
    samples.sort(key=lambda item: item["s"])
    deduped: list[dict[str, float]] = []
    for sample in samples:
        if deduped and abs(sample["s"] - deduped[-1]["s"]) < 1e-9:
            deduped[-1] = sample
        else:
            deduped.append(sample)

    return {
        "schema": "attack_residual_profile_v1",
        "description": description,
        "limits": {
            "max_delta_v_ref_mps": max_delta_v_ref_mps,
            "max_delta_steer_ff_rad": max_delta_steer_ff_rad,
        },
        "samples": deduped,
        "metadata": {
            "generator": "python/scripts/generate_attack_residual_profile.py",
            "source_segments": [
                segment for window in windows for segment in window.source_segments
            ],
        },
    }


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="Attack driver CSV log")
    parser.add_argument("--output", required=True, type=Path, help="Output residual JSON")
    parser.add_argument("--max-segments", type=int, default=2)
    parser.add_argument("--steer-mag-rad", type=float, default=0.015)
    parser.add_argument(
        "--edge-trim-m",
        type=float,
        default=5.0,
        help="Shrink each selected segment window at both ends before merging",
    )
    parser.add_argument("--ramp-m", type=float, default=25.0)
    parser.add_argument("--merge-gap-m", type=float, default=40.0)
    parser.add_argument(
        "--quantize-m",
        type=float,
        default=5.0,
        help="Quantize generated window edges to this station spacing; <=0 disables",
    )
    parser.add_argument("--min-abs-kappa", type=float, default=0.035)
    parser.add_argument("--min-target-gap", type=float, default=2.0)
    parser.add_argument("--min-under-ref-no-throttle-ratio", type=float, default=0.25)
    parser.add_argument(
        "--allowed-phase",
        action="append",
        default=["complex"],
        help="Phase allowed for steering residual generation; repeatable",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_rows(args.csv)
    stats = collect_segment_stats(rows)
    windows = select_residual_windows(
        stats,
        max_segments=max(0, args.max_segments),
        steer_mag_rad=args.steer_mag_rad,
        edge_trim_m=max(0.0, args.edge_trim_m),
        merge_gap_m=max(0.0, args.merge_gap_m),
        quantize_m=max(0.0, args.quantize_m),
        allowed_phases=set(args.allowed_phase),
        min_abs_kappa=max(0.0, args.min_abs_kappa),
        min_target_gap=args.min_target_gap,
        min_under_ref_no_throttle_ratio=max(0.0, args.min_under_ref_no_throttle_ratio),
    )
    profile = build_profile(
        windows,
        ramp_m=max(0.0, args.ramp_m),
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=max(abs(args.steer_mag_rad), 0.035),
        description=(
            "Generated steering residual profile from attack bottleneck segments. "
            "Use as an optional residual baseline or RL teacher, not as a plant parameter."
        ),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(profile, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "selected_segments": profile["metadata"]["source_segments"],
                "window_count": len(windows),
                "top_segments": [item.__dict__ for item in stats[:5]],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
