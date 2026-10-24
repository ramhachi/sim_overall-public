#!/usr/bin/env python3
"""Run reproducible parameter sweeps for the attack driver family.

Most presets exercise ``--driver attack`` knobs added for the course-aware
speed planner. The ``driver_generalization`` preset additionally compares PID
and attack residual/legacy contracts across compact vehicle-parameter
variations. Runtime artifacts are written to an explicit output directory so
code changes and benchmark data stay separated.
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
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_attack_mpc_log  # noqa: E402,I001


JSON_BLOCK_RE = re.compile(r"%%JSON_BEGIN%%\s*(\{.*?\})\s*%%JSON_END%%", re.S)


@dataclass(frozen=True)
class AttackSweepCase:
    case_id: str
    description: str
    params: dict[str, float] = field(default_factory=dict)
    vehicle_params: dict[str, Any] = field(default_factory=dict)
    driver: str = "attack"
    use_residual_profile: bool = True
    extra_flags: tuple[str, ...] = ()


FAST_EXIT_FLOORS = {
    "attack_floor_straight": 0.96,
    "attack_floor_exit": 0.98,
    "attack_floor_complex": 0.88,
    "attack_floor_turn_in": 0.82,
    "attack_floor_apex": 0.82,
    "attack_floor_brake": 0.76,
}


def coarse_cases() -> list[AttackSweepCase]:
    floor_current = {
        "attack_speed_lateral_safety": 0.66,
        "attack_floor_straight": 0.92,
        "attack_floor_exit": 0.92,
        "attack_floor_complex": 0.86,
        "attack_floor_turn_in": 0.84,
        "attack_floor_apex": 0.84,
        "attack_floor_brake": 0.78,
    }
    floor_fast_exit = {
        "attack_floor_straight": 0.96,
        "attack_floor_exit": 0.98,
        "attack_floor_complex": 0.88,
        "attack_floor_turn_in": 0.82,
        "attack_floor_apex": 0.82,
        "attack_floor_brake": 0.76,
    }
    floor_balanced = {
        "attack_floor_straight": 0.94,
        "attack_floor_exit": 0.95,
        "attack_floor_complex": 0.88,
        "attack_floor_turn_in": 0.84,
        "attack_floor_apex": 0.84,
        "attack_floor_brake": 0.78,
    }

    return [
        AttackSweepCase("baseline_current", "current course-aware planner", dict(floor_current)),
        AttackSweepCase(
            "lat070_current",
            "previous lateral safety baseline",
            {**floor_current, "attack_speed_lateral_safety": 0.70},
        ),
        AttackSweepCase(
            "lat074_current",
            "higher lateral safety",
            {**floor_current, "attack_speed_lateral_safety": 0.74},
        ),
        AttackSweepCase(
            "brake084_current",
            "later braking feasibility pass",
            {**floor_current, "attack_speed_braking_safety": 0.84},
        ),
        AttackSweepCase(
            "fast_exit",
            "higher straight/exit floor, safer corner entry",
            dict(floor_fast_exit),
        ),
        AttackSweepCase(
            "lat066_fast_exit",
            "fast exit with lower lateral safety",
            {**floor_fast_exit, "attack_speed_lateral_safety": 0.66},
        ),
        AttackSweepCase(
            "lat074_fast_exit",
            "fast exit with higher lateral safety",
            {**floor_fast_exit, "attack_speed_lateral_safety": 0.74},
        ),
        AttackSweepCase(
            "balanced_floor",
            "balanced floor profile",
            dict(floor_balanced),
        ),
        AttackSweepCase(
            "balanced_low_vx_cost",
            "balanced floor with weaker speed regularizer",
            {**floor_balanced, "attack_w_vx": 0.18},
        ),
        AttackSweepCase(
            "balanced_progress12",
            "balanced floor with weaker terminal progress pressure",
            {**floor_balanced, "attack_terminal_progress_weight": 1.2},
        ),
        AttackSweepCase(
            "boundary120",
            "lower soft-boundary penalty",
            {**floor_balanced, "attack_soft_boundary_weight": 120.0},
        ),
        AttackSweepCase(
            "boundary100_outward20",
            "lower boundary and outward-rate penalties",
            {
                **floor_balanced,
                "attack_soft_boundary_weight": 100.0,
                "attack_outward_rate_weight": 20.0,
            },
        ),
        AttackSweepCase(
            "fast_exit_boundary120",
            "fast exit with lower soft-boundary penalty",
            {**floor_fast_exit, "attack_soft_boundary_weight": 120.0},
        ),
        AttackSweepCase(
            "fast_exit_progress12",
            "fast exit with weaker terminal progress pressure",
            {**floor_fast_exit, "attack_terminal_progress_weight": 1.2},
        ),
        AttackSweepCase(
            "lat070_accel65",
            "higher planner acceleration limit",
            {**floor_balanced, "attack_speed_accel_limit": 6.5},
        ),
        AttackSweepCase(
            "lat066_accel65",
            "lower lateral safety with higher acceleration limit",
            {
                **floor_balanced,
                "attack_speed_lateral_safety": 0.66,
                "attack_speed_accel_limit": 6.5,
            },
        ),
    ]


def targeted_weight_cases() -> list[AttackSweepCase]:
    """Return a compact sweep around the current attack-driver bottlenecks.

    The cases intentionally avoid a full factorial grid. Recent diagnostics show
    that the relevant knobs are speed regularization, progress pressure, soft
    boundary costs, steering-rate smoothness, and complex-section floor speed.
    One-at-a-time cases plus a few coupled cases are enough to determine whether
    each knob is causal before spending compute on a larger search.
    """

    base = dict(FAST_EXIT_FLOORS)
    cases: list[AttackSweepCase] = [
        AttackSweepCase(
            "target_baseline",
            "fast-exit floors with current cost weights",
            dict(base),
        )
    ]

    def add(case_id: str, description: str, **updates: float) -> None:
        cases.append(AttackSweepCase(case_id, description, {**base, **updates}))

    for value in [0.12, 0.18, 0.35]:
        add(
            f"w_vx_{str(value).replace('.', 'p')}",
            f"speed regularization weight {value}",
            attack_w_vx=value,
        )
    for value in [1.2, 2.0]:
        add(
            f"progress_w_{str(value).replace('.', 'p')}",
            f"terminal progress weight {value}",
            attack_terminal_progress_weight=value,
        )
    for value in [0.82, 0.90, 0.98]:
        add(
            f"progress_target_{str(value).replace('.', 'p')}",
            f"progress target fraction {value}",
            attack_progress_target_fraction=value,
        )
    for value in [80.0, 110.0]:
        add(
            f"soft_boundary_{int(value)}",
            f"soft boundary weight {value:g}",
            attack_soft_boundary_weight=value,
        )
    for value in [10.0, 20.0]:
        add(
            f"outward_{int(value)}",
            f"outward-rate weight {value:g}",
            attack_outward_rate_weight=value,
        )
    for value in [0.92, 0.96]:
        add(
            f"complex_floor_{str(value).replace('.', 'p')}",
            f"complex-section speed floor {value}",
            attack_floor_complex=value,
        )
    for value in [0.02, 0.03, 0.045]:
        add(
            f"delta_rate_{str(value).replace('.', 'p')}",
            f"steering-rate smoothness weight {value}",
            attack_w_delta_rate=value,
        )
    for value in [0.001, 0.006]:
        add(
            f"ax_rate_{str(value).replace('.', 'p')}",
            f"acceleration-rate smoothness weight {value}",
            attack_w_ax_rate=value,
        )

    coupled_cases = [
        (
            "w_vx_0p12_progress_2p0",
            "weaker speed regularization with stronger progress",
            {"attack_w_vx": 0.12, "attack_terminal_progress_weight": 2.0},
        ),
        (
            "w_vx_0p12_progress_target_0p98",
            "weaker speed regularization with stricter progress target",
            {"attack_w_vx": 0.12, "attack_progress_target_fraction": 0.98},
        ),
        (
            "w_vx_0p18_soft_80_outward_10",
            "weaker speed regularization with relaxed boundary costs",
            {
                "attack_w_vx": 0.18,
                "attack_soft_boundary_weight": 80.0,
                "attack_outward_rate_weight": 10.0,
            },
        ),
        (
            "complex_0p92_w_vx_0p18",
            "higher complex floor with weaker speed regularization",
            {"attack_floor_complex": 0.92, "attack_w_vx": 0.18},
        ),
        (
            "complex_0p96_soft_110",
            "higher complex floor with moderately relaxed boundary cost",
            {"attack_floor_complex": 0.96, "attack_soft_boundary_weight": 110.0},
        ),
        (
            "progress_2p0_soft_80",
            "stronger progress with relaxed soft boundary cost",
            {"attack_terminal_progress_weight": 2.0, "attack_soft_boundary_weight": 80.0},
        ),
        (
            "progress_2p0_delta_rate_0p03",
            "stronger progress with weaker steering-rate smoothness",
            {
                "attack_terminal_progress_weight": 2.0,
                "attack_w_delta_rate": 0.03,
            },
        ),
        (
            "w_vx_0p12_delta_rate_0p03",
            "weaker speed regularization with weaker steering-rate smoothness",
            {
                "attack_w_vx": 0.12,
                "attack_w_delta_rate": 0.03,
            },
        ),
        (
            "progress_2p0_ax_rate_0p001",
            "stronger progress with weaker acceleration-rate smoothness",
            {
                "attack_terminal_progress_weight": 2.0,
                "attack_w_ax_rate": 0.001,
            },
        ),
        (
            "w_vx_0p12_ax_rate_0p001",
            "weaker speed regularization with weaker acceleration-rate smoothness",
            {
                "attack_w_vx": 0.12,
                "attack_w_ax_rate": 0.001,
            },
        ),
        (
            "progress_target_0p98_soft_80",
            "stricter progress target with relaxed soft boundary cost",
            {"attack_progress_target_fraction": 0.98, "attack_soft_boundary_weight": 80.0},
        ),
    ]
    for case_id, description, updates in coupled_cases:
        add(case_id, description, **updates)

    return cases


def smoothness_fine_cases() -> list[AttackSweepCase]:
    """Return a focused sweep for the attack-driver smoothness bottleneck."""

    base = dict(FAST_EXIT_FLOORS)
    cases: list[AttackSweepCase] = [
        AttackSweepCase(
            "ax_rate_default_0p0005",
            "current attack default acceleration-rate smoothness",
            {**base, "attack_w_ax_rate": 0.0005},
        )
    ]

    def add(case_id: str, description: str, **updates: float) -> None:
        cases.append(AttackSweepCase(case_id, description, {**base, **updates}))

    for value in [0.0003, 0.0008, 0.0010, 0.0012, 0.0015, 0.0020]:
        add(
            f"ax_rate_{str(value).replace('.', 'p')}",
            f"acceleration-rate smoothness weight {value}",
            attack_w_ax_rate=value,
        )
    for value in [0.015, 0.02, 0.03]:
        add(
            f"ax_0p001_delta_rate_{str(value).replace('.', 'p')}",
            f"current acceleration-rate weight with steering-rate weight {value}",
            attack_w_ax_rate=0.001,
            attack_w_delta_rate=value,
        )

    return cases


def tracking_fine_cases() -> list[AttackSweepCase]:
    """Return a focused sweep for lateral/heading tracking weights."""

    base = {**FAST_EXIT_FLOORS, "attack_w_ax_rate": 0.0005}
    cases: list[AttackSweepCase] = [
        AttackSweepCase(
            "tracking_default_y7_psi3",
            "current attack tracking weights",
            dict(base),
        )
    ]

    def add(case_id: str, description: str, **updates: float) -> None:
        cases.append(AttackSweepCase(case_id, description, {**base, **updates}))

    for value in [5.0, 6.0, 8.0, 9.0]:
        add(
            f"w_y_{str(value).replace('.', 'p')}",
            f"lateral tracking weight {value:g}",
            attack_w_y=value,
        )
    for value in [2.0, 2.5, 3.5, 4.0]:
        add(
            f"w_psi_{str(value).replace('.', 'p')}",
            f"heading tracking weight {value:g}",
            attack_w_psi=value,
        )

    coupled_cases = [
        (
            "w_y_5_psi_2",
            "relaxed lateral and heading tracking",
            {"attack_w_y": 5.0, "attack_w_psi": 2.0},
        ),
        (
            "w_y_6_psi_2p5",
            "moderately relaxed tracking",
            {"attack_w_y": 6.0, "attack_w_psi": 2.5},
        ),
        (
            "w_y_8_psi_3p5",
            "moderately stronger tracking",
            {"attack_w_y": 8.0, "attack_w_psi": 3.5},
        ),
        (
            "w_y_9_psi_4",
            "stronger lateral and heading tracking",
            {"attack_w_y": 9.0, "attack_w_psi": 4.0},
        ),
    ]
    for case_id, description, updates in coupled_cases:
        add(case_id, description, **updates)

    return cases


def understeer_fine_cases() -> list[AttackSweepCase]:
    """Return a focused sweep for attack rollout yaw-rate understeer prediction."""

    base = {**FAST_EXIT_FLOORS, "attack_w_ax_rate": 0.0005}
    cases = [
        AttackSweepCase(
            "understeer_default_0p0",
            "current attack rollout yaw-rate prediction",
            dict(base),
        )
    ]
    for value in [0.0005, 0.001, 0.0015, 0.002, 0.003, 0.004]:
        case_id = f"understeer_{str(value).replace('.', 'p')}"
        cases.append(
            AttackSweepCase(
                case_id,
                f"attack rollout yaw-rate understeer gradient {value:g}",
                {**base, "attack_rollout_understeer_gradient": value},
            )
        )
    return cases


def generalization_cases() -> list[AttackSweepCase]:
    """Return compact vehicle-parameter variations for generic-driver checks.

    These cases vary plant parameters that the attack driver should account for
    through the vehicle params and fixed-course planner, not through case-specific
    driver tuning.  They intentionally avoid residual profiles so this preset
    evaluates the generic controller skeleton.
    """

    return [
        AttackSweepCase(
            "nominal",
            "nominal flexible torque vehicle",
        ),
        AttackSweepCase(
            "mass_plus_15",
            "15 percent heavier car",
            vehicle_params={"mass": {"m_total": 322.0}},
        ),
        AttackSweepCase(
            "mass_minus_10",
            "10 percent lighter car",
            vehicle_params={"mass": {"m_total": 252.0}},
        ),
        AttackSweepCase(
            "mu_y_minus_10",
            "10 percent lower lateral friction",
            vehicle_params={"tire": {"mu_y_max": 1.80, "mu_x_max": 1.44}},
        ),
        AttackSweepCase(
            "mu_y_plus_05",
            "5 percent higher lateral friction",
            vehicle_params={"tire": {"mu_y_max": 2.10, "mu_x_max": 1.68}},
        ),
        AttackSweepCase(
            "aero_minus_20",
            "20 percent lower downforce and drag",
            vehicle_params={"aero": {"CL_A": 2.24, "CD_A": 0.88}},
        ),
        AttackSweepCase(
            "steer_inertia_plus_50",
            "50 percent higher steering inertia",
            vehicle_params={"steering": {"I_steer": 0.03}},
        ),
        AttackSweepCase(
            "chassis_stiffness_half",
            "half chassis torsional stiffness",
            vehicle_params={"suspension": {"K_chassis": 1000.0}},
        ),
    ]


def driver_generalization_cases() -> list[AttackSweepCase]:
    """Return driver/vehicle matrix cases for generic-driver regressions.

    The matrix keeps vehicle variations compact and compares four controller
    contracts on each plant:

    - attack + residual profile supplied by ``--attack-residual-profile``
    - attack without residual profile
    - attack with the legacy braking-corridor target contract
    - PID baseline

    This preset is intentionally broader than ``generalization`` because it
    checks whether a speedup came from a robust base-controller change or from a
    residual/contract that overfits the nominal plant.
    """

    vehicle_cases = generalization_cases()
    variants = [
        {
            "suffix": "attack_residual",
            "description": "attack default with residual profile",
            "driver": "attack",
            "use_residual_profile": True,
            "extra_flags": (),
        },
        {
            "suffix": "attack_no_residual",
            "description": "attack default without residual profile",
            "driver": "attack",
            "use_residual_profile": False,
            "extra_flags": (),
        },
        {
            "suffix": "attack_legacy_corridor",
            "description": "attack with legacy braking-corridor target contract",
            "driver": "attack",
            "use_residual_profile": True,
            "extra_flags": ("--attack-braking-corridor-band-target",),
        },
        {
            "suffix": "pid",
            "description": "PID baseline",
            "driver": "pid",
            "use_residual_profile": False,
            "extra_flags": (),
        },
    ]
    cases: list[AttackSweepCase] = []
    for vehicle_case in vehicle_cases:
        for variant in variants:
            cases.append(
                AttackSweepCase(
                    case_id=f"{vehicle_case.case_id}__{variant['suffix']}",
                    description=f"{vehicle_case.description}; {variant['description']}",
                    vehicle_params=vehicle_case.vehicle_params,
                    driver=str(variant["driver"]),
                    use_residual_profile=bool(variant["use_residual_profile"]),
                    extra_flags=tuple(variant["extra_flags"]),
                )
            )
    return cases


def cases_for_preset(preset: str) -> list[AttackSweepCase]:
    if preset == "coarse":
        return coarse_cases()
    if preset == "targeted_weights":
        return targeted_weight_cases()
    if preset == "smoothness_fine":
        return smoothness_fine_cases()
    if preset == "tracking_fine":
        return tracking_fine_cases()
    if preset == "understeer_fine":
        return understeer_fine_cases()
    if preset == "generalization":
        return generalization_cases()
    if preset == "driver_generalization":
        return driver_generalization_cases()
    raise ValueError(f"unknown attack sweep preset: {preset}")


def filter_cases(cases: list[AttackSweepCase], pattern: str | None) -> list[AttackSweepCase]:
    if not pattern:
        return list(cases)
    case_re = re.compile(pattern)
    return [case for case in cases if case_re.search(case.case_id)]


def cli_name(param_name: str) -> str:
    return "--" + param_name.replace("_", "-")


def deep_merge(base: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    merged = json.loads(json.dumps(base))
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_base_params(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    with path.open() as handle:
        return json.load(handle)


def extract_json_result(stdout: str) -> dict[str, Any]:
    match = JSON_BLOCK_RE.search(stdout)
    if not match:
        return {}
    try:
        return json.loads(match.group(1))
    except json.JSONDecodeError:
        return {}


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def bool_from_result(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "completed"}


def csv_last_metrics(csv_path: Path) -> dict[str, float]:
    """Return last recorded time and distance from a laptime CSV if present."""

    if not csv_path.exists():
        return {}
    try:
        with csv_path.open(newline="") as handle:
            last_row: dict[str, str] | None = None
            for row in csv.DictReader(handle):
                last_row = row
    except OSError:
        return {}
    if not last_row:
        return {}
    metrics: dict[str, float] = {}
    if "time" in last_row:
        metrics["last_time_s"] = finite_float(last_row.get("time"))
    if "s" in last_row:
        metrics["last_distance_m"] = finite_float(last_row.get("s"))
    return metrics


def objective_score(result: dict[str, Any], analysis: dict[str, Any]) -> float:
    completed = bool_from_result(result.get("lap_completed"))
    lap_time = finite_float(result.get("lap_time"))
    distance = finite_float(result.get("distance_traveled"), 0.0)
    max_excess = finite_float(result.get("max_track_limit_excess"), 0.0)
    mpc_fail_count = finite_float(result.get("mpc_fail_count"), 0.0)
    all_reject_ratio = finite_float(analysis.get("all_reject_ratio"), 0.0)
    min_margin = finite_float(analysis.get("min_predicted_margin_m"), math.inf)
    strict_safe = (
        max_excess <= 1e-6
        and mpc_fail_count <= 0.0
        and all_reject_ratio <= 1e-9
        and min_margin >= 0.60
    )
    if completed and strict_safe and math.isfinite(lap_time):
        return lap_time
    if completed and math.isfinite(lap_time):
        margin_penalty = max(0.0, 0.60 - min_margin) if math.isfinite(min_margin) else 0.0
        return (
            1000.0
            + lap_time
            + 100.0 * max(0.0, max_excess)
            + 5.0 * mpc_fail_count
            + 500.0 * all_reject_ratio
            + 100.0 * margin_penalty
        )
    return 10000.0 - min(distance, 9999.0)


def run_case(
    case: AttackSweepCase,
    args: argparse.Namespace,
    out_dir: Path,
) -> dict[str, Any]:
    case_dir = out_dir / case.case_id
    case_dir.mkdir(parents=True, exist_ok=True)
    csv_path = case_dir / "laptime.csv"
    stdout_path = case_dir / "stdout.txt"
    stderr_path = case_dir / "stderr.txt"
    analysis_path = case_dir / "analysis.json"

    cmd = [
        str(args.executable),
        "--track",
        args.track,
        "--driver",
        case.driver,
        "--time",
        str(args.time),
        "--driver-torque-max",
        str(args.driver_torque_max),
        "--output",
        str(csv_path),
        "--output_json",
    ]
    if case.vehicle_params:
        params_json = deep_merge(load_base_params(args.params_file), case.vehicle_params)
        cmd.extend(["--params-json", json.dumps(params_json, separators=(",", ":"))])
    elif args.params_file:
        cmd.extend(["--params_file", str(args.params_file)])
    if args.attack_speed_cap_file:
        cmd.extend(["--attack-speed-cap-file", str(args.attack_speed_cap_file)])
    if case.driver == "attack" and case.use_residual_profile and args.attack_residual_profile:
        cmd.extend(["--attack-residual-profile", str(args.attack_residual_profile)])
    for key, value in sorted(case.params.items()):
        cmd.extend([cli_name(key), f"{value:.12g}"])
    cmd.extend(case.extra_flags)

    start = time.monotonic()
    timed_out = False
    try:
        completed_process = subprocess.run(
            cmd,
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            timeout=args.timeout,
            check=False,
        )
        returncode = completed_process.returncode
        stdout = completed_process.stdout
        stderr = completed_process.stderr
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        returncode = 124
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
    elapsed = time.monotonic() - start
    stdout_path.write_text(stdout)
    stderr_path.write_text(stderr)

    result = extract_json_result(stdout)
    analysis: dict[str, Any] = {}
    if csv_path.exists():
        try:
            analysis = analyze_attack_mpc_log.summarize(
                analyze_attack_mpc_log.read_csv(csv_path),
                window_seconds=2.0,
            )
            analysis_path.write_text(json.dumps(analysis, indent=2, sort_keys=True) + "\n")
        except Exception as exc:  # pragma: no cover - runtime diagnostics path
            analysis = {"analysis_error": str(exc)}

    last_metrics = csv_last_metrics(csv_path)
    result_for_score = dict(result)
    if "distance_traveled" not in result_for_score and "last_distance_m" in last_metrics:
        result_for_score["distance_traveled"] = last_metrics["last_distance_m"]
    score = objective_score(result_for_score, analysis)
    lap_completed = bool_from_result(result.get("lap_completed"))
    lap_time = finite_float(result.get("lap_time"))
    distance_traveled = finite_float(
        result.get("distance_traveled"),
        finite_float(last_metrics.get("last_distance_m"), 0.0),
    )
    estimated_failure_mode = analysis.get("estimated_failure_mode")
    if timed_out and not estimated_failure_mode:
        estimated_failure_mode = "timeout"
    return {
        "case_id": case.case_id,
        "description": case.description,
        "returncode": returncode,
        "timed_out": timed_out,
        "elapsed_wall_s": elapsed,
        "score": score,
        "lap_completed": lap_completed,
        "lap_time": lap_time,
        "distance_traveled": distance_traveled,
        "last_csv_time_s": last_metrics.get("last_time_s"),
        "max_track_limit_excess": finite_float(result.get("max_track_limit_excess"), 0.0),
        "mpc_fail_count": int(finite_float(result.get("mpc_fail_count"), 0.0)),
        "all_reject_ratio": analysis.get("all_reject_ratio"),
        "estimated_failure_mode": estimated_failure_mode,
        "primitive_kind_ratio": json.dumps(
            analysis.get("primitive_kind_ratio", {}),
            sort_keys=True,
        ),
        "speed_scale_distribution": json.dumps(
            analysis.get("speed_scale_distribution", {}),
            sort_keys=True,
        ),
        "track_phase_ratio": json.dumps(analysis.get("track_phase_ratio", {}), sort_keys=True),
        "row_dominant_cost_ratio": json.dumps(
            analysis.get("row_dominant_cost_ratio", {}),
            sort_keys=True,
        ),
        "speed_scale_095_loss_term_ratio": json.dumps(
            analysis.get("speed_scale_095_loss_term_ratio", {}),
            sort_keys=True,
        ),
        "track_phase_mean_speed_mps": json.dumps(
            analysis.get("track_phase_mean_speed_mps", {}),
            sort_keys=True,
        ),
        "min_predicted_margin_m": analysis.get("min_predicted_margin_m"),
        "params": json.dumps(case.params, sort_keys=True),
        "vehicle_params": json.dumps(case.vehicle_params, sort_keys=True),
        "driver": case.driver,
        "use_residual_profile": case.use_residual_profile,
        "extra_flags": " ".join(case.extra_flags),
        "csv_path": str(csv_path),
        "analysis_path": str(analysis_path) if analysis_path.exists() else "",
        "stdout_path": str(stdout_path),
        "stderr_path": str(stderr_path),
        "command": " ".join(cmd),
    }


def write_summary(out_dir: Path, results: list[dict[str, Any]]) -> None:
    rows = sorted(results, key=lambda row: finite_float(row.get("score"), float("inf")))
    json_path = out_dir / "summary.json"
    csv_path = out_dir / "summary.csv"
    json_path.write_text(json.dumps(rows, indent=2, sort_keys=True) + "\n")
    if not rows:
        csv_path.write_text("")
        return
    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def default_out_dir() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("/tmp") / f"attack_grid_search_{stamp}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=REPO_ROOT / "build/cpp/laptime_sim")
    parser.add_argument("--track", default="ASE_autocross")
    parser.add_argument(
        "--params-file",
        type=Path,
        default=REPO_ROOT / "matlab/parameter_presets/params_flexible_torque.json",
    )
    parser.add_argument("--time", type=float, default=180.0)
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--attack-speed-cap-file", type=Path)
    parser.add_argument("--attack-residual-profile", type=Path)
    parser.add_argument("--out-dir", type=Path, default=default_out_dir())
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument(
        "--preset",
        choices=[
            "coarse",
            "targeted_weights",
            "smoothness_fine",
            "tracking_fine",
            "understeer_fine",
            "generalization",
            "driver_generalization",
        ],
        default="coarse",
    )
    parser.add_argument(
        "--case-filter",
        help="Optional regular expression used to keep matching case IDs only.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.executable.is_file():
        raise FileNotFoundError(f"laptime executable not found: {args.executable}")
    if args.params_file and not args.params_file.is_file():
        raise FileNotFoundError(f"params file not found: {args.params_file}")

    cases = cases_for_preset(args.preset)
    cases = filter_cases(cases, args.case_filter)
    if args.max_cases > 0:
        cases = cases[: args.max_cases]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "cases.json").write_text(
        json.dumps([case.__dict__ for case in cases], indent=2, sort_keys=True) + "\n"
    )

    results: list[dict[str, Any]] = []
    max_workers = max(1, args.jobs)
    print(f"Running {len(cases)} attack sweep cases with jobs={max_workers}")
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_case = {
            executor.submit(run_case, case, args, args.out_dir): case
            for case in cases
        }
        for future in as_completed(future_to_case):
            case = future_to_case[future]
            try:
                result = future.result()
            except Exception as exc:
                result = {
                    "case_id": case.case_id,
                    "description": case.description,
                    "returncode": -1,
                    "timed_out": False,
                    "elapsed_wall_s": math.nan,
                    "score": float("inf"),
                    "lap_completed": False,
                    "lap_time": math.nan,
                    "distance_traveled": 0.0,
                    "last_csv_time_s": None,
                    "max_track_limit_excess": math.nan,
                    "mpc_fail_count": -1,
                    "all_reject_ratio": None,
                    "estimated_failure_mode": f"runner_error: {exc}",
                    "primitive_kind_ratio": "{}",
                    "speed_scale_distribution": "{}",
                    "track_phase_ratio": "{}",
                    "row_dominant_cost_ratio": "{}",
                    "speed_scale_095_loss_term_ratio": "{}",
                    "track_phase_mean_speed_mps": "{}",
                    "min_predicted_margin_m": None,
                    "params": json.dumps(case.params, sort_keys=True),
                    "vehicle_params": json.dumps(case.vehicle_params, sort_keys=True),
                    "driver": case.driver,
                    "use_residual_profile": case.use_residual_profile,
                    "extra_flags": " ".join(case.extra_flags),
                    "csv_path": "",
                    "analysis_path": "",
                    "stdout_path": "",
                    "stderr_path": "",
                    "command": "",
                }
            results.append(result)
            write_summary(args.out_dir, results)
            print(
                f"{result['case_id']}: completed={result['lap_completed']} "
                f"lap={result['lap_time']} dist={result['distance_traveled']:.1f} "
                f"excess={result['max_track_limit_excess']} score={result['score']}"
            )

    write_summary(args.out_dir, results)
    ranked = sorted(results, key=lambda row: finite_float(row.get("score"), float("inf")))
    print("\nTop results:")
    for row in ranked[:5]:
        print(
            f"  {row['case_id']}: completed={row['lap_completed']} "
            f"lap={row['lap_time']} excess={row['max_track_limit_excess']} "
            f"failure={row['estimated_failure_mode']}"
        )
    print(f"\nSummary: {args.out_dir / 'summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
