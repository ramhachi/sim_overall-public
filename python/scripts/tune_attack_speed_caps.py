#!/usr/bin/env python3
"""Tune generated attack speed caps by binary-searching a safe relaxation scale."""

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
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_attack_mpc_log  # noqa: E402


JSON_BLOCK_RE = re.compile(r"%%JSON_BEGIN%%\s*(\{.*?\})\s*%%JSON_END%%", re.S)


BALANCED_FLOOR_ARGS = [
    "--attack-floor-straight", "0.94",
    "--attack-floor-exit", "0.95",
    "--attack-floor-complex", "0.88",
    "--attack-floor-turn-in", "0.84",
    "--attack-floor-apex", "0.84",
    "--attack-floor-brake", "0.78",
]

FAST_EXIT_ARGS = [
    "--attack-floor-straight", "0.96",
    "--attack-floor-exit", "0.98",
    "--attack-floor-complex", "0.88",
    "--attack-floor-turn-in", "0.82",
    "--attack-floor-apex", "0.82",
    "--attack-floor-brake", "0.76",
]


@dataclass
class TrialResult:
    scale: float
    safe: bool
    lap_completed: bool
    lap_time: float
    distance_traveled: float
    max_track_limit_excess: float
    mpc_fail_count: int
    returncode: int
    elapsed_wall_s: float
    estimated_failure_mode: str
    min_predicted_margin_m: float | None
    command: str
    cap_file: str
    csv_path: str
    stdout_path: str
    stderr_path: str
    analysis_path: str

    def to_row(self) -> dict[str, Any]:
        return {
            "scale": self.scale,
            "safe": self.safe,
            "lap_completed": self.lap_completed,
            "lap_time": self.lap_time,
            "distance_traveled": self.distance_traveled,
            "max_track_limit_excess": self.max_track_limit_excess,
            "mpc_fail_count": self.mpc_fail_count,
            "returncode": self.returncode,
            "elapsed_wall_s": self.elapsed_wall_s,
            "estimated_failure_mode": self.estimated_failure_mode,
            "min_predicted_margin_m": self.min_predicted_margin_m,
            "command": self.command,
            "cap_file": self.cap_file,
            "csv_path": self.csv_path,
            "stdout_path": self.stdout_path,
            "stderr_path": self.stderr_path,
            "analysis_path": self.analysis_path,
        }


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "completed"}


def extract_json_result(stdout: str) -> dict[str, Any]:
    match = JSON_BLOCK_RE.search(stdout)
    if not match:
        return {}
    try:
        return json.loads(match.group(1))
    except json.JSONDecodeError:
        return {}


def load_cap_payload(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text())
    if isinstance(payload, list):
        payload = {"speed_caps": payload}
    if not isinstance(payload, dict):
        raise ValueError(f"cap file must be an object or speed_caps array: {path}")
    has_point_caps = isinstance(payload.get("speed_caps"), list)
    has_window_caps = isinstance(payload.get("window_caps"), list)
    if not has_point_caps and not has_window_caps:
        raise ValueError(f"cap file must contain speed_caps or window_caps array: {path}")
    return payload


def scaled_cap_payload(
    payload: dict[str, Any],
    scale: float,
    max_cap_speed: float,
) -> dict[str, Any]:
    if scale <= 0.0 or not math.isfinite(scale):
        raise ValueError("scale must be positive and finite")
    scaled_caps: list[dict[str, Any]] = []
    for item in payload.get("speed_caps", []):
        if isinstance(item, list):
            if len(item) < 2:
                continue
            cap = {
                "s": float(item[0]),
                "speed": float(item[1]),
                "radius": float(item[2]) if len(item) >= 3 else 15.0,
            }
        else:
            cap = dict(item)
        speed = finite_float(cap.get("speed", cap.get("speed_mps")))
        if not math.isfinite(speed):
            continue
        cap["speed"] = min(speed * scale, max_cap_speed)
        cap["base_speed"] = speed
        cap["relaxation_scale"] = scale
        scaled_caps.append(cap)
    scaled_windows: list[dict[str, Any]] = []
    for item in payload.get("window_caps", []):
        if isinstance(item, list):
            if len(item) < 3:
                continue
            cap = {
                "s_start": float(item[0]),
                "s_end": float(item[1]),
                "speed": float(item[2]),
            }
        else:
            cap = dict(item)
        speed = finite_float(cap.get("speed", cap.get("speed_mps")))
        if not math.isfinite(speed):
            continue
        cap["speed"] = min(speed * scale, max_cap_speed)
        cap["base_speed"] = speed
        cap["relaxation_scale"] = scale
        scaled_windows.append(cap)
    metadata = dict(payload.get("metadata", {}))
    metadata.update(
        {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "schema": "attack_speed_caps_v2_scaled",
            "relaxation_scale": scale,
            "max_cap_speed": max_cap_speed,
        }
    )
    return {"metadata": metadata, "speed_caps": scaled_caps, "window_caps": scaled_windows}


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def case_preset_args(preset: str) -> list[str]:
    if preset == "balanced":
        return list(BALANCED_FLOOR_ARGS)
    if preset == "fast_exit":
        return list(FAST_EXIT_ARGS)
    if preset == "current":
        return []
    raise ValueError(f"unknown case preset: {preset}")


def is_safe_result(
    result: dict[str, Any],
    max_track_limit_excess: float,
    max_mpc_fail_count: int,
) -> bool:
    return (
        truthy(result.get("lap_completed"))
        and finite_float(result.get("max_track_limit_excess"), math.inf) <= max_track_limit_excess
        and int(finite_float(result.get("mpc_fail_count"), math.inf)) <= max_mpc_fail_count
    )


def run_trial(
    args: argparse.Namespace,
    base_payload: dict[str, Any],
    scale: float,
    trial_index: int,
) -> TrialResult:
    trial_dir = args.out_dir / f"trial_{trial_index:02d}_scale_{scale:.4f}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    cap_file = trial_dir / "attack_speed_caps.json"
    csv_path = trial_dir / "laptime.csv"
    stdout_path = trial_dir / "stdout.txt"
    stderr_path = trial_dir / "stderr.txt"
    analysis_path = trial_dir / "analysis.json"
    write_json(cap_file, scaled_cap_payload(base_payload, scale, args.max_cap_speed))

    cmd = [
        str(args.executable),
        "--track",
        args.track,
        "--driver",
        "attack",
        "--gain-preset",
        "optimized_autocross",
        "--time",
        str(args.time),
        "--driver-torque-max",
        str(args.driver_torque_max),
        "--params_file",
        str(args.params_file),
        "--attack-speed-cap-file",
        str(cap_file),
        "--output",
        str(csv_path),
        "--output_json",
    ]
    cmd.extend(case_preset_args(args.case_preset))
    cmd.extend(args.extra_arg or [])

    start = time.monotonic()
    completed = subprocess.run(
        cmd,
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=args.timeout,
        check=False,
    )
    elapsed = time.monotonic() - start
    stdout_path.write_text(completed.stdout)
    stderr_path.write_text(completed.stderr)
    result = extract_json_result(completed.stdout)

    analysis: dict[str, Any] = {}
    if csv_path.exists():
        try:
            analysis = analyze_attack_mpc_log.summarize(
                analyze_attack_mpc_log.read_csv(csv_path),
                window_seconds=2.0,
            )
            write_json(analysis_path, analysis)
        except Exception as exc:  # pragma: no cover - diagnostic path
            analysis = {"analysis_error": str(exc)}
            write_json(analysis_path, analysis)

    safe = is_safe_result(result, args.max_track_limit_excess, args.max_mpc_fail_count)
    return TrialResult(
        scale=scale,
        safe=safe,
        lap_completed=truthy(result.get("lap_completed")),
        lap_time=finite_float(result.get("lap_time")),
        distance_traveled=finite_float(result.get("distance_traveled"), 0.0),
        max_track_limit_excess=finite_float(result.get("max_track_limit_excess"), math.inf),
        mpc_fail_count=int(finite_float(result.get("mpc_fail_count"), math.inf)),
        returncode=completed.returncode,
        elapsed_wall_s=elapsed,
        estimated_failure_mode=str(analysis.get("estimated_failure_mode", "unknown")),
        min_predicted_margin_m=analysis.get("min_predicted_margin_m"),
        command=" ".join(cmd),
        cap_file=str(cap_file),
        csv_path=str(csv_path),
        stdout_path=str(stdout_path),
        stderr_path=str(stderr_path),
        analysis_path=str(analysis_path),
    )


def write_trial_summary(out_dir: Path, trials: list[TrialResult]) -> None:
    rows = [trial.to_row() for trial in trials]
    write_json(out_dir / "trials.json", {"trials": rows})
    if not rows:
        (out_dir / "trials.csv").write_text("")
        return
    with (out_dir / "trials.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def choose_best_trial(trials: list[TrialResult]) -> TrialResult | None:
    safe_trials = [trial for trial in trials if trial.safe and math.isfinite(trial.lap_time)]
    if not safe_trials:
        return None
    return min(safe_trials, key=lambda trial: trial.lap_time)


def choose_max_safe_trial(trials: list[TrialResult]) -> TrialResult | None:
    safe_trials = [trial for trial in trials if trial.safe]
    if not safe_trials:
        return None
    return max(safe_trials, key=lambda trial: trial.scale)


def tune(args: argparse.Namespace) -> list[TrialResult]:
    base_payload = load_cap_payload(args.cap_file)
    trials: list[TrialResult] = []

    low = args.initial_scale
    high = args.max_scale
    trial_index = 0

    first = run_trial(args, base_payload, low, trial_index)
    trials.append(first)
    write_trial_summary(args.out_dir, trials)
    print(
        f"trial {trial_index}: scale={first.scale:.4f} safe={first.safe} "
        f"lap={first.lap_time} dist={first.distance_traveled:.1f} "
        f"excess={first.max_track_limit_excess}"
    )
    if not first.safe:
        return trials

    trial_index += 1
    high_trial = run_trial(args, base_payload, high, trial_index)
    trials.append(high_trial)
    write_trial_summary(args.out_dir, trials)
    print(
        f"trial {trial_index}: scale={high_trial.scale:.4f} safe={high_trial.safe} "
        f"lap={high_trial.lap_time} dist={high_trial.distance_traveled:.1f} "
        f"excess={high_trial.max_track_limit_excess}"
    )
    if high_trial.safe:
        return trials

    for _ in range(max(args.iterations, 0)):
        trial_index += 1
        mid = 0.5 * (low + high)
        trial = run_trial(args, base_payload, mid, trial_index)
        trials.append(trial)
        write_trial_summary(args.out_dir, trials)
        print(
            f"trial {trial_index}: scale={trial.scale:.4f} safe={trial.safe} "
            f"lap={trial.lap_time} dist={trial.distance_traveled:.1f} "
            f"excess={trial.max_track_limit_excess}"
        )
        if trial.safe:
            low = mid
        else:
            high = mid

    return trials


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cap-file", type=Path, required=True)
    parser.add_argument("--executable", type=Path, default=REPO_ROOT / "build/cpp/laptime_sim")
    parser.add_argument("--track", default="ASE_autocross")
    parser.add_argument("--params-file", type=Path, default=Path("/tmp/winged_canonical_torque_flex.json"))
    parser.add_argument("--time", type=float, default=180.0)
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--case-preset", choices=["current", "balanced", "fast_exit"], default="balanced")
    parser.add_argument("--initial-scale", type=float, default=1.0)
    parser.add_argument("--max-scale", type=float, default=2.0)
    parser.add_argument("--max-cap-speed", type=float, default=30.0)
    parser.add_argument("--iterations", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--max-track-limit-excess", type=float, default=1e-6)
    parser.add_argument("--max-mpc-fail-count", type=int, default=0)
    parser.add_argument("--extra-arg", action="append", default=[])
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp") / "attack_cap_tuning")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.executable.is_file():
        raise FileNotFoundError(f"laptime executable not found: {args.executable}")
    if not args.params_file.is_file():
        raise FileNotFoundError(f"params file not found: {args.params_file}")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    trials = tune(args)
    base_payload = load_cap_payload(args.cap_file)
    best = choose_best_trial(trials)
    if best is None:
        print("No safe cap scale found.")
        return 1
    max_safe = choose_max_safe_trial(trials)
    write_json(
        args.out_dir / "best_caps.json",
        scaled_cap_payload(base_payload, best.scale, args.max_cap_speed),
    )
    if max_safe is not None:
        write_json(
            args.out_dir / "max_safe_caps.json",
            scaled_cap_payload(base_payload, max_safe.scale, args.max_cap_speed),
        )
    print(
        f"best_safe_by_lap: scale={best.scale:.4f} lap={best.lap_time} "
        f"distance={best.distance_traveled:.1f} caps={args.out_dir / 'best_caps.json'}"
    )
    if max_safe is not None:
        print(
            f"max_safe_scale: scale={max_safe.scale:.4f} lap={max_safe.lap_time} "
            f"caps={args.out_dir / 'max_safe_caps.json'}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
