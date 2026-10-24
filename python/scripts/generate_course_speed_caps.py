#!/usr/bin/env python3
"""Generate fixed-course attack speed caps from simple vehicle limit tests.

The workflow is intentionally direct:

1. Run PID angle-steering circle tests for several radii and binary-search the
   highest speed that completes without leaving the track.
2. Estimate one slalom critical speed using the lightweight driver benchmark.
3. Analyze the fixed autocross CSV curvature and assign each corner or chicane
   a speed cap from the measured radius map.
4. Write an ``attack_speed_caps`` JSON file consumable by ``laptime_sim``.

This is not a full racing-line optimizer. It is a reproducible way to seed the
attack MPC with course-specific entry/apex speed constraints before tuning MPC
weights.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
JSON_BLOCK_RE = re.compile(r"%%JSON_BEGIN%%\s*(\{.*?\})\s*%%JSON_END%%", re.S)


@dataclass(frozen=True)
class TrackPoint:
    x: float
    y: float
    width: float
    s: float
    kappa: float = 0.0


@dataclass(frozen=True)
class CornerSegment:
    s_start: float
    s_end: float
    s_apex: float
    radius_m: float
    max_abs_kappa: float
    sign_changes: int
    kind: str


@dataclass(frozen=True)
class CircleTrial:
    radius_m: float
    speed_mps: float
    passed: bool
    lap_completed: bool
    failure_code: str
    max_track_limit_excess: float
    max_lateral_error: float
    max_speed: float
    avg_speed: float
    lap_time: float
    distance_traveled: float
    elapsed_wall_s: float
    command: list[str]


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "completed", "success"}


def parse_json_result(stdout: str) -> dict[str, Any]:
    match = JSON_BLOCK_RE.search(stdout)
    if not match:
        return {}
    try:
        return json.loads(match.group(1))
    except json.JSONDecodeError:
        return {}


def parse_float_list(text: str) -> list[float]:
    values: list[float] = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(float(item))
    if not values:
        raise ValueError("list must contain at least one number")
    return values


def load_partial_params_json(path: Path, *, force_angle_steering: bool) -> str:
    payload: dict[str, Any]
    if path.exists():
        payload = json.loads(path.read_text())
    else:
        payload = {}
    steering = dict(payload.get("steering", {}))
    if force_angle_steering:
        steering["torque_mode"] = False
    payload["steering"] = steering
    return json.dumps(payload, separators=(",", ":"))


def read_track_csv(path: Path) -> list[TrackPoint]:
    rows: list[tuple[float, float, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((
                float(row["x"]),
                float(row["y"]),
                float(row.get("width", 3.0)),
            ))
    if len(rows) < 4:
        raise ValueError(f"track CSV has too few rows: {path}")

    cumulative = [0.0]
    for i in range(1, len(rows)):
        dx = rows[i][0] - rows[i - 1][0]
        dy = rows[i][1] - rows[i - 1][1]
        cumulative.append(cumulative[-1] + math.hypot(dx, dy))

    points = [
        TrackPoint(x=x, y=y, width=w, s=s)
        for (x, y, w), s in zip(rows, cumulative)
    ]
    return with_curvature(points)


def signed_curvature(prev: TrackPoint, cur: TrackPoint, nxt: TrackPoint) -> float:
    ax = cur.x - prev.x
    ay = cur.y - prev.y
    bx = nxt.x - cur.x
    by = nxt.y - cur.y
    a = math.hypot(ax, ay)
    b = math.hypot(bx, by)
    cx = nxt.x - prev.x
    cy = nxt.y - prev.y
    c = math.hypot(cx, cy)
    denom = a * b * c
    if denom <= 1e-9:
        return 0.0
    cross = ax * by - ay * bx
    return 2.0 * cross / denom


def with_curvature(points: list[TrackPoint], smooth_half_window: int = 2) -> list[TrackPoint]:
    raw = [0.0] * len(points)
    for i in range(1, len(points) - 1):
        raw[i] = signed_curvature(points[i - 1], points[i], points[i + 1])
    raw[0] = raw[1]
    raw[-1] = raw[-2]

    smoothed: list[float] = []
    for i in range(len(raw)):
        start = max(0, i - smooth_half_window)
        end = min(len(raw), i + smooth_half_window + 1)
        smoothed.append(sum(raw[start:end]) / max(end - start, 1))

    return [
        TrackPoint(x=p.x, y=p.y, width=p.width, s=p.s, kappa=k)
        for p, k in zip(points, smoothed)
    ]


def extract_corner_segments(
    points: list[TrackPoint],
    *,
    curvature_threshold: float,
    merge_gap_m: float,
    min_length_m: float,
    slalom_gap_m: float,
    peak_min_distance_m: float = 28.0,
    max_corner_window_m: float = 48.0,
) -> list[CornerSegment]:
    del merge_gap_m  # peak-based extraction supersedes contiguous cluster merging.
    abs_kappa = [abs(p.kappa) for p in points]
    candidates: list[int] = []
    for i in range(1, len(points) - 1):
        if abs_kappa[i] < curvature_threshold:
            continue
        if abs_kappa[i] >= abs_kappa[i - 1] and abs_kappa[i] >= abs_kappa[i + 1]:
            candidates.append(i)
    if not candidates:
        return []

    selected: list[int] = []
    for idx in sorted(candidates, key=lambda i: abs_kappa[i], reverse=True):
        if all(abs(points[idx].s - points[other].s) >= peak_min_distance_m for other in selected):
            selected.append(idx)
    selected.sort(key=lambda i: points[i].s)

    segments: list[CornerSegment] = []
    half_window = max(max_corner_window_m, min_length_m) * 0.5
    for apex_idx in selected:
        peak = abs_kappa[apex_idx]
        local_threshold = max(curvature_threshold, 0.45 * peak)
        s_apex = points[apex_idx].s
        seg_start = apex_idx
        while seg_start > 0:
            if s_apex - points[seg_start - 1].s > half_window:
                break
            if abs_kappa[seg_start - 1] < local_threshold:
                break
            seg_start -= 1
        seg_end = apex_idx
        while seg_end + 1 < len(points):
            if points[seg_end + 1].s - s_apex > half_window:
                break
            if abs_kappa[seg_end + 1] < local_threshold:
                break
            seg_end += 1

        length = points[seg_end].s - points[seg_start].s
        if length < min_length_m:
            continue
        local = points[seg_start:seg_end + 1]
        max_abs_kappa = max(abs(p.kappa) for p in local)
        if max_abs_kappa <= 1e-9:
            continue
        signs: list[int] = []
        for p in local:
            if abs(p.kappa) < 0.5 * curvature_threshold:
                continue
            sign = 1 if p.kappa > 0.0 else -1
            if not signs or signs[-1] != sign:
                signs.append(sign)
        sign_changes = max(0, len(signs) - 1)
        kind = "slalom" if sign_changes > 0 and length <= slalom_gap_m else "corner"
        segments.append(CornerSegment(
            s_start=points[seg_start].s,
            s_end=points[seg_end].s,
            s_apex=points[apex_idx].s,
            radius_m=1.0 / max_abs_kappa,
            max_abs_kappa=max_abs_kappa,
            sign_changes=sign_changes,
            kind=kind,
        ))
    return segments


def interpolate_radius_speed(radius_speed: dict[float, float], radius: float) -> float:
    if not radius_speed:
        raise ValueError("radius speed map is empty")
    items = sorted(radius_speed.items())
    if radius <= items[0][0]:
        r0, v0 = items[0]
        return v0 * math.sqrt(max(radius, 1e-6) / r0)
    if radius >= items[-1][0]:
        r0, v0 = items[-1]
        return v0 * math.sqrt(radius / r0)
    for (r0, v0), (r1, v1) in zip(items, items[1:]):
        if r0 <= radius <= r1:
            alpha = (radius - r0) / max(r1 - r0, 1e-9)
            # Interpolate v^2 versus radius because lateral acceleration is v^2/R.
            v_sq = v0 * v0 + alpha * (v1 * v1 - v0 * v0)
            return math.sqrt(max(v_sq, 0.0))
    return items[-1][1]


def run_command(cmd: list[str], *, cwd: Path, timeout: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def run_circle_trial(
    *,
    executable: Path,
    params_json: str,
    radius: float,
    speed: float,
    track_width: float,
    timeout: float,
    output_dir: Path,
    trial_name: str,
) -> CircleTrial:
    output_dir.mkdir(parents=True, exist_ok=True)
    circumference = 2.0 * math.pi * radius
    sim_time = max(18.0, 1.45 * circumference / max(speed, 1.0))
    output_csv = output_dir / f"{trial_name}_r{radius:g}_v{speed:.3f}.csv"
    cmd = [
        str(executable),
        "--track", "circle",
        "--radius", f"{radius:.6g}",
        "--track-width", f"{track_width:.6g}",
        "--driver", "pid",
        "--gain-preset", "optimized_autocross",
        "--params-json", params_json,
        "--initial-speed", f"{speed:.6g}",
        "--max-speed", f"{speed:.6g}",
        "--max-lat-accel", "40.0",
        "--time", f"{sim_time:.6g}",
        "--output", str(output_csv),
        "--output_json",
    ]
    start = time.monotonic()
    completed = run_command(cmd, cwd=REPO_ROOT, timeout=timeout)
    elapsed = time.monotonic() - start
    result = parse_json_result(completed.stdout)
    max_excess = finite_float(result.get("max_track_limit_excess"), math.inf)
    max_speed = finite_float(result.get("max_speed"))
    avg_speed = finite_float(result.get("avg_speed"))
    lap_completed = truthy(result.get("lap_completed"))
    failure_code = str(result.get("failure_code", ""))
    passed = (
        completed.returncode == 0
        and lap_completed
        and max_excess <= 0.0
        and max_speed >= 0.98 * speed
        and avg_speed >= 0.90 * speed
        and failure_code == ""
    )
    return CircleTrial(
        radius_m=radius,
        speed_mps=speed,
        passed=passed,
        lap_completed=lap_completed,
        failure_code=failure_code,
        max_track_limit_excess=max_excess,
        max_lateral_error=finite_float(result.get("max_lateral_error")),
        max_speed=max_speed,
        avg_speed=avg_speed,
        lap_time=finite_float(result.get("lap_time")),
        distance_traveled=finite_float(result.get("distance_traveled")),
        elapsed_wall_s=elapsed,
        command=cmd,
    )


def find_radius_critical_speed(
    *,
    executable: Path,
    params_json: str,
    radius: float,
    track_width: float,
    low_speed: float,
    high_speed: float,
    iterations: int,
    timeout: float,
    output_dir: Path,
) -> tuple[float, list[CircleTrial]]:
    trials: list[CircleTrial] = []

    low_trial = run_circle_trial(
        executable=executable,
        params_json=params_json,
        radius=radius,
        speed=low_speed,
        track_width=track_width,
        timeout=timeout,
        output_dir=output_dir,
        trial_name="low",
    )
    trials.append(low_trial)
    if not low_trial.passed:
        return 0.0, trials

    high = max(high_speed, low_speed + 0.5)
    high_trial = run_circle_trial(
        executable=executable,
        params_json=params_json,
        radius=radius,
        speed=high,
        track_width=track_width,
        timeout=timeout,
        output_dir=output_dir,
        trial_name="high",
    )
    trials.append(high_trial)
    while high_trial.passed and high < 30.0:
        low_speed = high
        high = min(30.0, high + 2.0)
        high_trial = run_circle_trial(
            executable=executable,
            params_json=params_json,
            radius=radius,
            speed=high,
            track_width=track_width,
            timeout=timeout,
            output_dir=output_dir,
            trial_name="high_expand",
        )
        trials.append(high_trial)

    if high_trial.passed:
        return high_trial.speed_mps, trials

    low = low_speed
    for index in range(iterations):
        mid = 0.5 * (low + high)
        trial = run_circle_trial(
            executable=executable,
            params_json=params_json,
            radius=radius,
            speed=mid,
            track_width=track_width,
            timeout=timeout,
            output_dir=output_dir,
            trial_name=f"bisect_{index}",
        )
        trials.append(trial)
        if trial.passed:
            low = mid
        else:
            high = mid
    return low, trials


def estimate_slalom_critical_speed(
    *,
    executable: Path,
    params_file: Path,
    driver_torque_max: float,
    output_dir: Path,
    timeout: float,
) -> float:
    output_path = output_dir / "slalom_capability_profile.json"
    cmd = [
        str(executable),
        "--quick",
        "--params_file", str(params_file),
        "--driver-torque-max", f"{driver_torque_max:.6g}",
        "--output", str(output_path),
    ]
    completed = run_command(cmd, cwd=REPO_ROOT, timeout=timeout)
    result = parse_json_result(completed.stdout)
    if not result and output_path.exists():
        result = json.loads(output_path.read_text())
    speed = finite_float(result.get("metrics", {}).get("slalom_stable_speed_mps"))
    return speed if speed > 0.0 else math.nan


def build_cap_payload(
    *,
    points: list[TrackPoint],
    segments: list[CornerSegment],
    radius_speed: dict[float, float],
    slalom_critical_speed: float,
    corner_safety: float,
    slalom_safety: float,
    approach_speed: float,
    brake_decel: float,
    entry_buffer_m: float,
    exit_buffer_m: float,
    cap_mode: str,
    point_cap_radius_m: float,
    max_capped_radius_m: float,
    min_cap_speed: float,
    max_cap_speed: float,
) -> dict[str, Any]:
    track_length = points[-1].s if points else 0.0
    speed_caps: list[dict[str, Any]] = []
    window_caps: list[dict[str, Any]] = []
    segment_rows: list[dict[str, Any]] = []

    for idx, segment in enumerate(segments):
        if segment.radius_m > max_capped_radius_m:
            continue
        radius_speed_limit = interpolate_radius_speed(radius_speed, segment.radius_m)
        if segment.kind == "slalom" and math.isfinite(slalom_critical_speed):
            cap_speed = min(radius_speed_limit, slalom_critical_speed) * slalom_safety
        else:
            cap_speed = radius_speed_limit * corner_safety
        cap_speed = max(min(cap_speed, max_cap_speed), min_cap_speed)
        d_brake = max(0.0, (approach_speed * approach_speed - cap_speed * cap_speed)
                      / max(2.0 * brake_decel, 1e-6))
        s_start = max(0.0, segment.s_start - d_brake - entry_buffer_m)
        s_end = min(track_length, segment.s_end + exit_buffer_m)
        radius_window = max(6.0, 0.5 * (s_end - s_start))

        speed_caps.append({
            "s": round(segment.s_apex, 4),
            "speed": round(cap_speed, 4),
            "radius": round(
                point_cap_radius_m if cap_mode == "point" else radius_window,
                4,
            ),
            "reason": f"{segment.kind}_critical_speed",
            "source_radius_m": round(segment.radius_m, 4),
            "radius_speed_mps": round(radius_speed_limit, 4),
        })
        if cap_mode in {"window", "both"}:
            window_caps.append({
                "s_start": round(s_start, 4),
                "s_end": round(s_end, 4),
                "speed": round(cap_speed, 4),
                "reason": f"{segment.kind}_entry_apex_cap",
                "source_s_apex": round(segment.s_apex, 4),
                "source_radius_m": round(segment.radius_m, 4),
            })
        segment_rows.append({
            "index": idx,
            "kind": segment.kind,
            "s_start": segment.s_start,
            "s_end": segment.s_end,
            "s_apex": segment.s_apex,
            "radius_m": segment.radius_m,
            "max_abs_kappa": segment.max_abs_kappa,
            "sign_changes": segment.sign_changes,
            "radius_speed_mps": radius_speed_limit,
            "cap_speed_mps": cap_speed,
            "window_start": s_start,
            "window_end": s_end,
        })

    return {
        "metadata": {
            "schema": "attack_speed_caps_course_limit_v1",
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "track_length_m": track_length,
            "corner_count": len(segments),
            "corner_safety": corner_safety,
            "slalom_safety": slalom_safety,
            "approach_speed_mps": approach_speed,
            "brake_decel_mps2": brake_decel,
            "cap_mode": cap_mode,
            "point_cap_radius_m": point_cap_radius_m,
            "max_capped_radius_m": max_capped_radius_m,
        },
        "radius_speed_map": [
            {"radius_m": r, "critical_speed_mps": v}
            for r, v in sorted(radius_speed.items())
        ],
        "slalom_critical_speed_mps": slalom_critical_speed,
        "corner_segments": segment_rows,
        "speed_caps": speed_caps,
        "window_caps": window_caps,
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def write_circle_trials(path: Path, trials: list[CircleTrial]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "radius_m",
            "speed_mps",
            "passed",
            "lap_completed",
            "failure_code",
            "max_track_limit_excess",
            "max_lateral_error",
            "max_speed",
            "avg_speed",
            "lap_time",
            "distance_traveled",
            "elapsed_wall_s",
            "command",
        ])
        writer.writeheader()
        for trial in trials:
            writer.writerow({
                "radius_m": trial.radius_m,
                "speed_mps": trial.speed_mps,
                "passed": trial.passed,
                "lap_completed": trial.lap_completed,
                "failure_code": trial.failure_code,
                "max_track_limit_excess": trial.max_track_limit_excess,
                "max_lateral_error": trial.max_lateral_error,
                "max_speed": trial.max_speed,
                "avg_speed": trial.avg_speed,
                "lap_time": trial.lap_time,
                "distance_traveled": trial.distance_traveled,
                "elapsed_wall_s": trial.elapsed_wall_s,
                "command": " ".join(trial.command),
            })


def run_attack_validation(args: argparse.Namespace, cap_file: Path, out_dir: Path) -> dict[str, Any]:
    csv_path = out_dir / "attack_laptime.csv"
    stdout_path = out_dir / "attack_stdout.txt"
    stderr_path = out_dir / "attack_stderr.txt"
    cmd = [
        str(args.executable),
        "--track", args.track,
        "--driver", "attack",
        "--gain-preset", "optimized_autocross",
        "--time", str(args.attack_time),
        "--params_file", str(args.params_file),
        "--driver-torque-max", str(args.driver_torque_max),
        "--attack-speed-cap-file", str(cap_file),
        "--output", str(csv_path),
        "--output_json",
    ]
    if args.driver_capability_profile:
        cmd.extend(["--driver-capability-profile", str(args.driver_capability_profile)])
    start = time.monotonic()
    completed = run_command(cmd, cwd=REPO_ROOT, timeout=args.attack_timeout)
    elapsed = time.monotonic() - start
    stdout_path.write_text(completed.stdout)
    stderr_path.write_text(completed.stderr)
    result = parse_json_result(completed.stdout)
    result.update({
        "returncode": completed.returncode,
        "elapsed_wall_s": elapsed,
        "command": " ".join(cmd),
        "csv_path": str(csv_path),
        "stdout_path": str(stdout_path),
        "stderr_path": str(stderr_path),
    })
    write_json(out_dir / "attack_result.json", result)
    return result


def make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=REPO_ROOT / "build/cpp/laptime_sim")
    parser.add_argument(
        "--driver-benchmark-executable",
        type=Path,
        default=REPO_ROOT / "build/cpp/driver_benchmark",
    )
    parser.add_argument("--track", default="ASE_autocross")
    parser.add_argument("--track-csv", type=Path, default=REPO_ROOT / "data/ASE_autocross_sim.csv")
    parser.add_argument(
        "--params-file",
        type=Path,
        default=REPO_ROOT / "matlab/parameter_presets/params_flexible_torque.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/course_speed_caps_experiment"))
    parser.add_argument("--radii", default="5,6,8,10,12,15,20,30,45")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--radius-iterations", type=int, default=5)
    parser.add_argument("--low-speed", type=float, default=3.0)
    parser.add_argument("--high-speed", type=float, default=18.0)
    parser.add_argument("--track-width", type=float, default=4.5)
    parser.add_argument("--corner-safety", type=float, default=0.92)
    parser.add_argument("--slalom-safety", type=float, default=0.88)
    parser.add_argument("--slalom-critical-speed", type=float, default=math.nan)
    parser.add_argument("--approach-speed", type=float, default=18.0)
    parser.add_argument("--brake-decel", type=float, default=8.0)
    parser.add_argument("--entry-buffer", type=float, default=2.0)
    parser.add_argument("--exit-buffer", type=float, default=3.0)
    parser.add_argument("--cap-mode", choices=["point", "window", "both"], default="point")
    parser.add_argument("--point-cap-radius", type=float, default=4.0)
    parser.add_argument("--max-capped-radius", type=float, default=math.inf)
    parser.add_argument("--curvature-threshold", type=float, default=0.018)
    parser.add_argument("--merge-gap", type=float, default=8.0)
    parser.add_argument("--min-corner-length", type=float, default=5.0)
    parser.add_argument("--slalom-gap", type=float, default=55.0)
    parser.add_argument("--peak-min-distance", type=float, default=28.0)
    parser.add_argument("--max-corner-window", type=float, default=48.0)
    parser.add_argument("--min-cap-speed", type=float, default=3.5)
    parser.add_argument("--max-cap-speed", type=float, default=28.0)
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--attack-time", type=float, default=180.0)
    parser.add_argument("--attack-timeout", type=float, default=180.0)
    parser.add_argument("--run-attack", action="store_true")
    parser.add_argument("--driver-capability-profile", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = make_arg_parser().parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    radii = parse_float_list(args.radii)
    if args.quick:
        radii = radii[: min(len(radii), 5)]
        args.radius_iterations = min(args.radius_iterations, 3)

    params_json = load_partial_params_json(args.params_file, force_angle_steering=True)
    radius_speed: dict[float, float] = {}
    all_trials: list[CircleTrial] = []
    for radius in radii:
        radius_dir = args.out_dir / "circle_trials" / f"r{radius:g}"
        high = min(args.high_speed, math.sqrt(max(0.1, 1.7 * 9.81 * radius)))
        critical, trials = find_radius_critical_speed(
            executable=args.executable,
            params_json=params_json,
            radius=radius,
            track_width=args.track_width,
            low_speed=args.low_speed,
            high_speed=high,
            iterations=args.radius_iterations,
            timeout=args.timeout,
            output_dir=radius_dir,
        )
        radius_speed[radius] = critical
        all_trials.extend(trials)
        print(f"radius={radius:g}m critical_speed={critical:.3f} m/s", flush=True)

    write_circle_trials(args.out_dir / "circle_trials.csv", all_trials)

    slalom_critical = args.slalom_critical_speed
    if not math.isfinite(slalom_critical) or slalom_critical <= 0.0:
        slalom_critical = estimate_slalom_critical_speed(
            executable=args.driver_benchmark_executable,
            params_file=args.params_file,
            driver_torque_max=args.driver_torque_max,
            output_dir=args.out_dir,
            timeout=args.timeout,
        )
    print(f"slalom_critical_speed={slalom_critical:.3f} m/s", flush=True)

    points = read_track_csv(args.track_csv)
    segments = extract_corner_segments(
        points,
        curvature_threshold=args.curvature_threshold,
        merge_gap_m=args.merge_gap,
        min_length_m=args.min_corner_length,
        slalom_gap_m=args.slalom_gap,
        peak_min_distance_m=args.peak_min_distance,
        max_corner_window_m=args.max_corner_window,
    )
    payload = build_cap_payload(
        points=points,
        segments=segments,
        radius_speed=radius_speed,
        slalom_critical_speed=slalom_critical,
        corner_safety=args.corner_safety,
        slalom_safety=args.slalom_safety,
        approach_speed=args.approach_speed,
        brake_decel=args.brake_decel,
        entry_buffer_m=args.entry_buffer,
        exit_buffer_m=args.exit_buffer,
        cap_mode=args.cap_mode,
        point_cap_radius_m=args.point_cap_radius,
        max_capped_radius_m=args.max_capped_radius,
        min_cap_speed=args.min_cap_speed,
        max_cap_speed=args.max_cap_speed,
    )
    write_json(args.output, payload)
    write_json(args.out_dir / "course_speed_caps.json", payload)
    print(f"wrote {args.output}", flush=True)

    if args.run_attack:
        result = run_attack_validation(args, args.output, args.out_dir)
        lap_time = finite_float(result.get("lap_time"))
        completed = truthy(result.get("lap_completed"))
        failure = result.get("failure_code", "")
        print(
            "attack_result "
            f"completed={completed} lap_time={lap_time:.3f} failure={failure}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
