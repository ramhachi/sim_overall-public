#!/usr/bin/env python3
"""Generate and optionally evaluate local residual candidates for attack driver.

This is a small pre-CEM tool.  It reads an attack-driver CSV log, selects the
segments where the car is far below the course reference without throttle, and
creates bounded residual-profile variants around those windows.  The candidates
can be evaluated with the same laptime wrapper used by CEM training.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_attack_mpc_log  # noqa: E402
import generate_attack_residual_profile as residual_segments  # noqa: E402

from fsae_sim.rl import AttackResidualEnvConfig, AttackResidualProfileEnv  # noqa: E402


@dataclass(frozen=True)
class CandidateWindow:
    s_start: float
    s_end: float
    segment_id: str
    phase: str
    score: float


@dataclass(frozen=True)
class CandidateSpec:
    name: str
    delta_v_ref_mps: float = 0.0
    delta_steer_ff_rad: float = 0.0
    delta_ax_bias_mps2: float = 0.0
    delta_heading_relax_rad: float = 0.0


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _limit_from_profile(
    profile: Mapping[str, Any],
    key: str,
    fallback: float,
) -> float:
    try:
        value = float(profile.get("limits", {}).get(key, fallback))
    except (TypeError, ValueError):
        value = fallback
    return abs(value) if math.isfinite(value) and value > 0.0 else fallback


def _window_weight(s: float, window: CandidateWindow, ramp_m: float) -> float:
    if window.s_end <= window.s_start:
        return 0.0
    ramp = max(ramp_m, 1e-9)
    if s < window.s_start - ramp or s > window.s_end + ramp:
        return 0.0
    if s < window.s_start:
        return (s - (window.s_start - ramp)) / ramp
    if s > window.s_end:
        return ((window.s_end + ramp) - s) / ramp
    return 1.0


def select_candidate_windows(
    rows: Iterable[dict[str, str]],
    *,
    max_segments: int,
    allowed_phases: set[str],
    segment_ids: set[str] | None = None,
    edge_trim_m: float,
    min_target_gap: float,
    min_under_ref_no_throttle_ratio: float,
) -> list[CandidateWindow]:
    stats = residual_segments.collect_segment_stats(rows)
    windows: list[CandidateWindow] = []
    for item in stats:
        if len(windows) >= max_segments:
            break
        if segment_ids is not None and item.segment_id not in segment_ids:
            continue
        if item.phase not in allowed_phases:
            continue
        if segment_ids is None:
            if item.mean_target_gap < min_target_gap:
                continue
            if item.under_ref_no_throttle_ratio < min_under_ref_no_throttle_ratio:
                continue
        start = max(0.0, item.s_min + edge_trim_m)
        end = max(start + 1e-6, item.s_max - edge_trim_m)
        windows.append(
            CandidateWindow(
                s_start=start,
                s_end=end,
                segment_id=item.segment_id,
                phase=item.phase,
                score=item.score,
            )
        )
    return windows


def apply_candidate(
    base_profile: Mapping[str, Any],
    windows: list[CandidateWindow],
    spec: CandidateSpec,
    *,
    ramp_m: float,
) -> dict[str, Any]:
    max_delta_v = _limit_from_profile(base_profile, "max_delta_v_ref_mps", 2.0)
    max_delta_steer = _limit_from_profile(base_profile, "max_delta_steer_ff_rad", 0.035)
    max_delta_ax = _limit_from_profile(base_profile, "max_delta_ax_bias_mps2", 1.0)
    max_delta_heading = _limit_from_profile(base_profile, "max_delta_heading_relax_rad", 0.08)
    samples = []
    for sample in base_profile.get("samples", []):
        s = float(sample.get("s", sample.get("s_m", 0.0)))
        weight = max((_window_weight(s, window, ramp_m) for window in windows), default=0.0)
        next_sample = dict(sample)
        next_sample["delta_v_ref_mps"] = float(
            max(
                -max_delta_v,
                min(max_delta_v, float(sample.get("delta_v_ref_mps", 0.0))
                    + spec.delta_v_ref_mps * weight),
            )
        )
        next_sample["delta_steer_ff_rad"] = float(
            max(
                -max_delta_steer,
                min(max_delta_steer, float(sample.get("delta_steer_ff_rad", 0.0))
                    + spec.delta_steer_ff_rad * weight),
            )
        )
        next_sample["delta_ax_bias_mps2"] = float(
            max(
                -max_delta_ax,
                min(max_delta_ax, float(sample.get("delta_ax_bias_mps2", 0.0))
                    + spec.delta_ax_bias_mps2 * weight),
            )
        )
        next_sample["delta_heading_relax_rad"] = float(
            max(
                0.0,
                min(max_delta_heading, float(sample.get("delta_heading_relax_rad", 0.0))
                    + spec.delta_heading_relax_rad * weight),
            )
        )
        samples.append(next_sample)

    output = dict(base_profile)
    output["samples"] = samples
    output["description"] = f"Attack residual local candidate: {spec.name}"
    metadata = dict(base_profile.get("metadata", {}))
    metadata["candidate_name"] = spec.name
    metadata["candidate_segments"] = [window.segment_id for window in windows]
    metadata["candidate_phases"] = sorted({window.phase for window in windows})
    metadata["generator"] = "python/scripts/evaluate_attack_residual_candidates.py"
    output["metadata"] = metadata
    return output


def candidate_specs() -> list[CandidateSpec]:
    return [
        CandidateSpec("ax_plus_020", delta_ax_bias_mps2=0.20),
        CandidateSpec("ax_plus_035", delta_ax_bias_mps2=0.35),
        CandidateSpec("vref_plus_030", delta_v_ref_mps=0.30),
        CandidateSpec("vref_plus_050", delta_v_ref_mps=0.50),
        CandidateSpec("combined_mild", delta_v_ref_mps=0.25, delta_ax_bias_mps2=0.20),
        CandidateSpec("heading_relax_010", delta_heading_relax_rad=0.010),
    ]


def analyze_csv(csv_path: Path) -> dict[str, Any]:
    rows = analyze_attack_mpc_log.read_csv(csv_path)
    summary = analyze_attack_mpc_log.summarize(rows)
    course_means = summary.get("course_reference_means", {})
    return {
        "mean_speed_mps": course_means.get("v_ref"),
        "mean_course_target_gap_mps": course_means.get("target_gap"),
        "attack_summary": summary,
    }


def evaluate_candidates(
    candidate_paths: list[Path],
    args: argparse.Namespace,
) -> list[dict[str, Any]]:
    env = AttackResidualProfileEnv(
        AttackResidualEnvConfig(
            executable=args.executable,
            repo_root=REPO_ROOT,
            out_dir=args.out_dir / "episodes",
            track=args.track,
            params_file=args.params_file,
            time_limit_s=args.time,
            driver_torque_max_nm=args.driver_torque_max,
            lap_time_target_s=args.lap_time_target,
        ),
        analyze_csv=analyze_csv,
        timeout_s=args.timeout,
    )
    results = []
    for path in candidate_paths:
        result = env.evaluate_profile_file(path)
        results.append(
            {
                "candidate": path.stem,
                "profile_path": str(path),
                "lap_completed": result.lap_completed,
                "lap_time": result.lap_time,
                "reward": result.reward,
                "distance_traveled": result.distance_traveled,
                "max_track_limit_excess": result.max_track_limit_excess,
                "mpc_fail_count": result.mpc_fail_count,
                "csv_path": str(result.csv_path),
                "analysis": result.analysis,
            }
        )
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--base-profile", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--max-segments", type=int, default=4)
    parser.add_argument("--max-candidates", type=int, default=6)
    parser.add_argument("--edge-trim-m", type=float, default=2.0)
    parser.add_argument("--ramp-m", type=float, default=18.0)
    parser.add_argument("--min-target-gap", type=float, default=2.0)
    parser.add_argument("--min-under-ref-no-throttle-ratio", type=float, default=0.40)
    parser.add_argument(
        "--segment-id",
        action="append",
        default=None,
        help="Restrict candidate windows to explicit attack_segment_id values; repeatable",
    )
    parser.add_argument(
        "--allowed-phase",
        action="append",
        default=None,
    )
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--executable", type=Path, default=REPO_ROOT / "build/cpp/laptime_sim")
    parser.add_argument("--track", default="autocross")
    parser.add_argument(
        "--params-file",
        type=Path,
        default=REPO_ROOT / "matlab/parameter_presets/params_flexible_torque.json",
    )
    parser.add_argument("--time", type=float, default=120.0)
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--lap-time-target", type=float, default=66.0)
    parser.add_argument("--timeout", type=float, default=420.0)
    return parser.parse_args()


def allowed_phases_from_args(args: argparse.Namespace) -> list[str]:
    return args.allowed_phase or ["turn_in", "brake", "complex"]


def main() -> int:
    args = parse_args()
    rows = load_csv_rows(args.csv)
    base_profile = json.loads(args.base_profile.read_text(encoding="utf-8"))
    allowed_phases = allowed_phases_from_args(args)
    windows = select_candidate_windows(
        rows,
        max_segments=max(1, args.max_segments),
        allowed_phases=set(allowed_phases),
        segment_ids=set(args.segment_id) if args.segment_id else None,
        edge_trim_m=max(0.0, args.edge_trim_m),
        min_target_gap=args.min_target_gap,
        min_under_ref_no_throttle_ratio=max(0.0, args.min_under_ref_no_throttle_ratio),
    )
    args.out_dir.mkdir(parents=True, exist_ok=True)
    candidate_paths: list[Path] = []
    for spec in candidate_specs()[: max(1, args.max_candidates)]:
        profile = apply_candidate(base_profile, windows, spec, ramp_m=max(0.0, args.ramp_m))
        path = args.out_dir / f"{spec.name}.json"
        path.write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        candidate_paths.append(path)

    results = evaluate_candidates(candidate_paths, args) if args.evaluate else []
    summary = {
        "base_profile": str(args.base_profile),
        "source_csv": str(args.csv),
        "candidate_dir": str(args.out_dir),
        "selected_windows": [window.__dict__ for window in windows],
        "candidate_profiles": [str(path) for path in candidate_paths],
        "results": results,
    }
    (args.out_dir / "candidate_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
