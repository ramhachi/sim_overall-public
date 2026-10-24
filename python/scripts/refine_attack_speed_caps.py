#!/usr/bin/env python3
"""Generate local attack-driver speed caps from failed sweep logs.

This is a reproducibility tool: it converts observed heading/margin failures
into explicit ``s/speed/radius`` speed caps that can be fed back to
``laptime_sim --attack-speed-cap-file``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


@dataclass
class FailureCap:
    s: float
    speed: float
    radius: float
    source_case: str
    source_csv: str
    reason: str
    source_speed: float
    source_time: float

    def to_json(self) -> dict[str, Any]:
        return {
            "s": round(self.s, 6),
            "speed": round(self.speed, 6),
            "radius": round(self.radius, 6),
            "source_case": self.source_case,
            "source_csv": self.source_csv,
            "reason": self.reason,
            "source_speed": round(self.source_speed, 6),
            "source_time": round(self.source_time, 6),
        }


@dataclass
class FailureWindow:
    s_start: float
    s_end: float
    speed: float
    source_case: str
    source_csv: str
    reason: str
    source_s: float
    source_speed: float
    source_time: float

    def to_json(self) -> dict[str, Any]:
        return {
            "s_start": round(self.s_start, 6),
            "s_end": round(self.s_end, 6),
            "speed": round(self.speed, 6),
            "source_case": self.source_case,
            "source_csv": self.source_csv,
            "reason": self.reason,
            "source_s": round(self.source_s, 6),
            "source_speed": round(self.source_speed, 6),
            "source_time": round(self.source_time, 6),
        }


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "completed"}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _heading_abs(row: dict[str, str]) -> float:
    heading = _float_or_none(row.get("d_heading"))
    if heading is not None:
        return abs(heading)
    heading_deg = _float_or_none(row.get("heading_error_deg"))
    if heading_deg is not None:
        return abs(math.radians(heading_deg))
    return 0.0


def _row_time(row: dict[str, str]) -> float:
    return _float_or_none(row.get("time")) or 0.0


def _row_s(row: dict[str, str]) -> float | None:
    return _float_or_none(row.get("s"))


def _row_speed(row: dict[str, str]) -> float | None:
    actual = _float_or_none(row.get("speed"))
    if actual is None:
        actual = _float_or_none(row.get("vx"))
    target_candidates = [
        _float_or_none(row.get("attack_selected_target_speed")),
        _float_or_none(row.get("target_speed")),
    ]
    target_candidates = [
        value for value in target_candidates
        if value is not None and value > 0.0
    ]
    target = min(target_candidates) if target_candidates else None
    if actual is not None and actual > 0.0:
        if target is not None:
            return max(actual, 0.75 * target)
        return actual
    if target is not None:
        return target
    return None


def _risk_score(row: dict[str, str], margin_threshold: float) -> tuple[float, str]:
    margin = _float_or_none(row.get("mpc_predicted_min_margin"))
    heading = _heading_abs(row)
    margin_deficit = max(0.0, margin_threshold - margin) if margin is not None else 0.0
    if margin_deficit > 0.0 and heading > 0.25:
        return 5.0 * margin_deficit + heading, "heading_margin_conflict"
    if margin_deficit > 0.0:
        return 5.0 * margin_deficit + 0.25 * heading, "predicted_margin"
    return heading, "heading_error"


def select_failure_row(
    rows: list[dict[str, str]],
    window_seconds: float,
    margin_threshold: float,
) -> tuple[dict[str, str] | None, str]:
    if not rows:
        return None, "empty_log"
    end_time = max(_row_time(row) for row in rows)
    window_start = max(0.0, end_time - max(window_seconds, 0.0))
    candidates = [row for row in rows if _row_time(row) >= window_start]
    if not candidates:
        candidates = rows
    scored = [(_risk_score(row, margin_threshold), row) for row in candidates]
    scored.sort(key=lambda item: item[0][0], reverse=True)
    score_reason, row = scored[0]
    return row, score_reason[1]


def _is_failed_summary_row(row: dict[str, str]) -> bool:
    if not _truthy(row.get("lap_completed")):
        return True
    max_excess = _float_or_none(row.get("max_track_limit_excess")) or 0.0
    mpc_fail_count = _float_or_none(row.get("mpc_fail_count")) or 0.0
    return max_excess > 1e-6 or mpc_fail_count > 0.0


def generate_caps_from_summary(
    summary_path: Path,
    speed_factor: float = 0.85,
    radius: float = 18.0,
    min_speed: float = 3.0,
    window_seconds: float = 5.0,
    margin_threshold: float = 1.05,
) -> list[FailureCap]:
    summary_rows = read_csv(summary_path)
    caps: list[FailureCap] = []
    for summary_row in summary_rows:
        if not _is_failed_summary_row(summary_row):
            continue
        csv_path_text = summary_row.get("csv_path", "")
        if not csv_path_text:
            continue
        csv_path = Path(csv_path_text)
        if not csv_path.is_file():
            continue
        log_rows = read_csv(csv_path)
        row, reason = select_failure_row(log_rows, window_seconds, margin_threshold)
        if row is None:
            continue
        s = _row_s(row)
        speed = _row_speed(row)
        if s is None or speed is None:
            continue
        cap_speed = max(min_speed, speed * max(0.1, min(speed_factor, 1.0)))
        caps.append(
            FailureCap(
                s=s,
                speed=cap_speed,
                radius=radius,
                source_case=summary_row.get("case_id", "unknown"),
                source_csv=str(csv_path),
                reason=reason,
                source_speed=speed,
                source_time=_row_time(row),
            )
        )
    return merge_caps(caps)


def generate_windows_from_summary(
    summary_path: Path,
    speed_factor: float = 0.85,
    min_speed: float = 3.0,
    window_seconds: float = 5.0,
    margin_threshold: float = 1.05,
    prefix_distance: float = 25.0,
    post_distance: float = 8.0,
) -> list[FailureWindow]:
    summary_rows = read_csv(summary_path)
    windows: list[FailureWindow] = []
    for summary_row in summary_rows:
        if not _is_failed_summary_row(summary_row):
            continue
        csv_path_text = summary_row.get("csv_path", "")
        if not csv_path_text:
            continue
        csv_path = Path(csv_path_text)
        if not csv_path.is_file():
            continue
        log_rows = read_csv(csv_path)
        row, reason = select_failure_row(log_rows, window_seconds, margin_threshold)
        if row is None:
            continue
        s = _row_s(row)
        speed = _row_speed(row)
        if s is None or speed is None:
            continue
        cap_speed = max(min_speed, speed * max(0.1, min(speed_factor, 1.0)))
        windows.append(
            FailureWindow(
                s_start=max(0.0, s - max(prefix_distance, 0.0)),
                s_end=max(0.0, s + max(post_distance, 0.0)),
                speed=cap_speed,
                source_case=summary_row.get("case_id", "unknown"),
                source_csv=str(csv_path),
                reason=reason,
                source_s=s,
                source_speed=speed,
                source_time=_row_time(row),
            )
        )
    return windows


def merge_caps(caps: list[FailureCap], merge_distance: float = 8.0) -> list[FailureCap]:
    if not caps:
        return []
    caps = sorted(caps, key=lambda cap: cap.s)
    merged: list[FailureCap] = []
    for cap in caps:
        if not merged or abs(cap.s - merged[-1].s) > merge_distance:
            merged.append(cap)
            continue
        prev = merged[-1]
        if cap.speed < prev.speed:
            prev.speed = cap.speed
            prev.source_case = f"{prev.source_case},{cap.source_case}"
            prev.source_csv = f"{prev.source_csv},{cap.source_csv}"
            prev.reason = cap.reason
            prev.source_speed = cap.source_speed
            prev.source_time = cap.source_time
        prev.radius = max(prev.radius, cap.radius)
    return merged


def write_caps_json(
    caps: list[FailureCap],
    windows: list[FailureWindow],
    output_path: Path,
    summary_path: Path,
) -> None:
    payload = {
        "metadata": {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "source_summary": str(summary_path),
            "cap_count": len(caps),
            "window_count": len(windows),
            "schema": "attack_speed_caps_v2",
        },
        "speed_caps": [cap.to_json() for cap in caps],
        "window_caps": [window.to_json() for window in windows],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--speed-factor", type=float, default=0.85)
    parser.add_argument("--radius", type=float, default=18.0)
    parser.add_argument("--min-speed", type=float, default=3.0)
    parser.add_argument("--window-seconds", type=float, default=5.0)
    parser.add_argument("--margin-threshold", type=float, default=1.05)
    parser.add_argument("--prefix-distance", type=float, default=25.0)
    parser.add_argument("--post-distance", type=float, default=8.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    caps = generate_caps_from_summary(
        args.summary,
        speed_factor=args.speed_factor,
        radius=args.radius,
        min_speed=args.min_speed,
        window_seconds=args.window_seconds,
        margin_threshold=args.margin_threshold,
    )
    windows = generate_windows_from_summary(
        args.summary,
        speed_factor=args.speed_factor,
        min_speed=args.min_speed,
        window_seconds=args.window_seconds,
        margin_threshold=args.margin_threshold,
        prefix_distance=args.prefix_distance,
        post_distance=args.post_distance,
    )
    write_caps_json(caps, windows, args.out, args.summary)
    print(f"Wrote {len(caps)} attack speed caps and {len(windows)} windows to {args.out}")
    for cap in caps:
        print(
            f"  s={cap.s:.2f} speed={cap.speed:.2f} radius={cap.radius:.1f} "
            f"case={cap.source_case} reason={cap.reason}"
        )
    for window in windows:
        print(
            f"  window={window.s_start:.2f}-{window.s_end:.2f} speed={window.speed:.2f} "
            f"case={window.source_case} reason={window.reason}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
