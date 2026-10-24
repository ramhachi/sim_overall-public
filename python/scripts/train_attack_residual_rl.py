#!/usr/bin/env python3
"""Train an attack-driver residual profile with black-box policy search.

This script is deliberately dependency-light.  It uses CEM (cross-entropy
method) over the fixed-course residual profile consumed by the C++ attack
driver.  The learned artifact is still a residual policy: it maps station
``s`` to ``delta_v_ref`` and ``delta_steer_ff`` via the existing profile JSON.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_attack_mpc_log  # noqa: E402

from fsae_sim.rl import (  # noqa: E402
    AttackResidualEnvConfig,
    AttackResidualProfileEnv,
    ResidualBasisConfig,
    action_to_profile,
    basis_action_to_profile,
    basis_reconstruction_rmse,
    delta_basis_action_to_profile,
    profile_to_action,
    profile_to_basis_action,
    residual_basis_dim,
)


@dataclass
class CEMState:
    mean: np.ndarray
    std: np.ndarray
    best_action: np.ndarray
    best_reward: float = -math.inf
    best_info: dict[str, Any] | None = None


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, np.floating):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def dumps_json(value: Any, *, indent: int | None = None) -> str:
    return json.dumps(json_safe(value), indent=indent, sort_keys=True, allow_nan=False)


def analyze_csv(csv_path: Path) -> dict[str, Any]:
    rows = analyze_attack_mpc_log.read_csv(csv_path)
    summary = analyze_attack_mpc_log.summarize(rows)
    course_means = summary.get("course_reference_means", {})
    return {
        "mean_speed_mps": course_means.get("v_ref"),
        "mean_course_target_gap_mps": course_means.get("target_gap"),
        "attack_summary": summary,
    }


def load_initial_action(
    profile_path: Path | None,
    *,
    policy_mode: str,
    basis_config: ResidualBasisConfig | None,
    knot_count: int,
    track_length_m: float,
    max_delta_v_ref_mps: float,
    max_delta_steer_ff_rad: float,
) -> np.ndarray:
    if profile_path is None:
        if policy_mode in {"basis", "delta-basis"}:
            if basis_config is None:
                raise ValueError("basis_config is required for basis policy modes")
            return np.zeros(residual_basis_dim(basis_config), dtype=np.float64)
        return np.zeros(2 * knot_count, dtype=np.float64)
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    if policy_mode == "delta-basis":
        if basis_config is None:
            raise ValueError("basis_config is required for delta-basis policy mode")
        return np.zeros(residual_basis_dim(basis_config), dtype=np.float64)
    if policy_mode == "basis":
        if basis_config is None:
            raise ValueError("basis_config is required for basis policy mode")
        return profile_to_basis_action(profile, basis_config)
    return profile_to_action(
        profile,
        knot_count=knot_count,
        track_length_m=track_length_m,
        max_delta_v_ref_mps=max_delta_v_ref_mps,
        max_delta_steer_ff_rad=max_delta_steer_ff_rad,
    )


def materialize_action_profile(
    action: np.ndarray,
    *,
    policy_mode: str,
    basis_config: ResidualBasisConfig | None,
    base_profile: dict[str, Any] | None,
    knot_count: int,
    track_length_m: float,
    max_delta_v_ref_mps: float,
    max_delta_steer_ff_rad: float,
    smoothing_passes: int,
    description: str,
) -> dict[str, Any]:
    if policy_mode == "basis":
        if basis_config is None:
            raise ValueError("basis_config is required for basis policy mode")
        return basis_action_to_profile(action, basis_config, description=description)
    if policy_mode == "delta-basis":
        if basis_config is None or base_profile is None:
            raise ValueError("basis_config and base_profile are required for delta-basis")
        return delta_basis_action_to_profile(
            action,
            base_profile,
            basis_config,
            description=description,
        )
    return action_to_profile(
        action,
        knot_count=knot_count,
        track_length_m=track_length_m,
        max_delta_v_ref_mps=max_delta_v_ref_mps,
        max_delta_steer_ff_rad=max_delta_steer_ff_rad,
        smoothing_passes=smoothing_passes,
        description=description,
    )


def write_profile(
    path: Path,
    action: np.ndarray,
    *,
    policy_mode: str,
    basis_config: ResidualBasisConfig | None,
    base_profile: dict[str, Any] | None,
    knot_count: int,
    track_length_m: float,
    max_delta_v_ref_mps: float,
    max_delta_steer_ff_rad: float,
    smoothing_passes: int,
    description: str,
) -> None:
    profile = materialize_action_profile(
        action,
        policy_mode=policy_mode,
        basis_config=basis_config,
        base_profile=base_profile,
        knot_count=knot_count,
        track_length_m=track_length_m,
        max_delta_v_ref_mps=max_delta_v_ref_mps,
        max_delta_steer_ff_rad=max_delta_steer_ff_rad,
        smoothing_passes=smoothing_passes,
        description=description,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def evaluate_action(
    env: AttackResidualProfileEnv,
    action: np.ndarray,
    *,
    args: argparse.Namespace,
    basis_config: ResidualBasisConfig | None,
    base_profile: dict[str, Any] | None,
):
    if args.policy_mode == "basis":
        profile = materialize_action_profile(
            action,
            policy_mode=args.policy_mode,
            basis_config=basis_config,
            base_profile=base_profile,
            knot_count=args.knots,
            track_length_m=args.track_length_m,
            max_delta_v_ref_mps=args.max_delta_v_ref_mps,
            max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
            smoothing_passes=args.smoothing_passes,
            description="CEM basis-policy residual profile sample.",
        )
        return env.evaluate_profile_dict(profile)
    if args.policy_mode == "delta-basis":
        if args.initial_profile is not None and np.allclose(action, 0.0, atol=1e-12):
            return env.evaluate_profile_file(args.initial_profile)
        profile = materialize_action_profile(
            action,
            policy_mode=args.policy_mode,
            basis_config=basis_config,
            base_profile=base_profile,
            knot_count=args.knots,
            track_length_m=args.track_length_m,
            max_delta_v_ref_mps=args.max_delta_v_ref_mps,
            max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
            smoothing_passes=args.smoothing_passes,
            description="CEM delta-basis residual profile sample.",
        )
        return env.evaluate_profile_dict(profile)
    return env.evaluate_action(action)


def run_cem(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    env_config = AttackResidualEnvConfig(
        executable=args.executable,
        repo_root=REPO_ROOT,
        out_dir=out_dir / "episodes",
        track=args.track,
        params_file=args.params_file,
        time_limit_s=args.time,
        driver_torque_max_nm=args.driver_torque_max,
        knot_count=args.knots,
        track_length_m=args.track_length_m,
        max_delta_v_ref_mps=args.max_delta_v_ref_mps,
        max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
        action_smoothing_passes=args.smoothing_passes,
        lap_time_target_s=args.lap_time_target,
    )
    env = AttackResidualProfileEnv(
        env_config,
        analyze_csv=analyze_csv,
        timeout_s=args.timeout,
    )
    basis_config = None
    if args.policy_mode == "basis":
        basis_config = ResidualBasisConfig(
            track_length_m=args.track_length_m,
            basis_count=args.basis_count,
            output_samples=args.output_samples,
            basis_width_m=args.basis_width_m,
            max_delta_v_ref_mps=args.max_delta_v_ref_mps,
            max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
            max_delta_ax_bias_mps2=args.max_delta_ax_bias_mps2,
            max_delta_heading_relax_rad=args.max_delta_heading_relax_rad,
            basis_delta_v_ref_mps=args.basis_delta_v_ref_mps,
            basis_delta_steer_ff_rad=args.basis_delta_steer_ff_rad,
            basis_delta_ax_bias_mps2=args.basis_delta_ax_bias_mps2,
            basis_delta_heading_relax_rad=args.basis_delta_heading_relax_rad,
            closed_loop=True,
        )
    elif args.policy_mode == "delta-basis":
        basis_config = ResidualBasisConfig(
            track_length_m=args.track_length_m,
            basis_count=args.basis_count,
            output_samples=args.output_samples,
            basis_width_m=args.basis_width_m,
            max_delta_v_ref_mps=args.max_delta_v_ref_mps,
            max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
            max_delta_ax_bias_mps2=args.max_delta_ax_bias_mps2,
            max_delta_heading_relax_rad=args.max_delta_heading_relax_rad,
            basis_delta_v_ref_mps=args.basis_delta_v_ref_mps,
            basis_delta_steer_ff_rad=args.basis_delta_steer_ff_rad,
            basis_delta_ax_bias_mps2=args.basis_delta_ax_bias_mps2,
            basis_delta_heading_relax_rad=args.basis_delta_heading_relax_rad,
            closed_loop=True,
        )
    base_profile = (
        json.loads(args.initial_profile.read_text(encoding="utf-8"))
        if args.initial_profile is not None
        else None
    )
    initial = load_initial_action(
        args.initial_profile,
        policy_mode=args.policy_mode,
        basis_config=basis_config,
        knot_count=args.knots,
        track_length_m=args.track_length_m,
        max_delta_v_ref_mps=args.max_delta_v_ref_mps,
        max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
    )
    state = CEMState(
        mean=initial.copy(),
        std=np.full_like(initial, args.initial_std),
        best_action=initial.copy(),
    )
    rng = np.random.default_rng(args.seed)
    log_path = out_dir / "training_log.jsonl"
    best_profile_path = out_dir / "best_residual_profile.json"
    initial_projection_error = None
    if (
        args.initial_profile is not None
        and basis_config is not None
        and args.policy_mode == "basis"
    ):
        initial_projection_error = basis_reconstruction_rmse(
            json.loads(args.initial_profile.read_text(encoding="utf-8")),
            basis_config,
        )

    total_episodes = 0
    with log_path.open("w", encoding="utf-8") as log_file:
        if args.initial_profile is not None and not args.skip_initial_profile_eval:
            result = env.evaluate_profile_file(args.initial_profile)
            total_episodes += 1
            initial_info = {
                "iteration": -1,
                "member": -1,
                "reward": result.reward,
                "lap_completed": result.lap_completed,
                "lap_time": result.lap_time,
                "distance_traveled": result.distance_traveled,
                "max_track_limit_excess": result.max_track_limit_excess,
                "mpc_fail_count": result.mpc_fail_count,
                "csv_path": str(result.csv_path),
                "profile_path": str(result.profile_path),
                "analysis": result.analysis,
                "source": "exact_initial_profile",
                "policy_mode": args.policy_mode,
            }
            if initial_projection_error is not None:
                initial_info["basis_projection_error"] = initial_projection_error
            initial_info = json_safe(initial_info)
            log_file.write(dumps_json(initial_info) + "\n")
            log_file.flush()
            state.best_reward = result.reward
            state.best_action = initial.copy()
            state.best_info = initial_info
            best_profile_path.write_text(
                args.initial_profile.read_text(encoding="utf-8"),
                encoding="utf-8",
            )

        for iteration in range(args.iterations):
            actions: list[np.ndarray] = []
            rewards: list[float] = []
            infos: list[dict[str, Any]] = []
            population = max(1, args.population)
            for member in range(population):
                if iteration == 0 and member == 0:
                    action = state.mean.copy()
                else:
                    action = rng.normal(state.mean, state.std)
                    action = np.clip(action, -1.0, 1.0)
                actions.append(action)

            def record_evaluation(member: int, action: np.ndarray, result) -> None:
                nonlocal total_episodes
                total_episodes += 1
                info = {
                    "iteration": iteration,
                    "member": member,
                    "reward": result.reward,
                    "lap_completed": result.lap_completed,
                    "lap_time": result.lap_time,
                    "distance_traveled": result.distance_traveled,
                    "max_track_limit_excess": result.max_track_limit_excess,
                    "mpc_fail_count": result.mpc_fail_count,
                    "csv_path": str(result.csv_path),
                    "profile_path": str(result.profile_path),
                    "analysis": result.analysis,
                    "source": "cem_sample",
                    "policy_mode": args.policy_mode,
                }
                info = json_safe(info)
                log_file.write(dumps_json(info) + "\n")
                log_file.flush()
                rewards.append(result.reward)
                infos.append(info)
                if result.reward > state.best_reward:
                    state.best_reward = result.reward
                    state.best_action = action.copy()
                    state.best_info = info
                    write_profile(
                        best_profile_path,
                        state.best_action,
                        policy_mode=args.policy_mode,
                        basis_config=basis_config,
                        base_profile=base_profile,
                        knot_count=args.knots,
                        track_length_m=args.track_length_m,
                        max_delta_v_ref_mps=args.max_delta_v_ref_mps,
                        max_delta_steer_ff_rad=args.max_delta_steer_ff_rad,
                        smoothing_passes=args.smoothing_passes,
                        description="Best residual profile from CEM attack residual training.",
                    )

            def evaluate_member(member_index: int, action: np.ndarray):
                result = evaluate_action(
                    env,
                    action,
                    args=args,
                    basis_config=basis_config,
                    base_profile=base_profile,
                )
                return member_index, action, result

            worker_count = max(1, min(int(args.workers), population))
            if worker_count == 1:
                for member, action in enumerate(actions):
                    member_index, evaluated_action, result = evaluate_member(member, action)
                    record_evaluation(member_index, evaluated_action, result)
            else:
                evaluated = []
                with ThreadPoolExecutor(max_workers=worker_count) as executor:
                    futures = [
                        executor.submit(evaluate_member, member, action)
                        for member, action in enumerate(actions)
                    ]
                    for future in as_completed(futures):
                        evaluated.append(future.result())
                evaluated.sort(key=lambda item: item[0])
                for member, action, result in evaluated:
                    record_evaluation(member, action, result)

            rewards_array = np.asarray(rewards, dtype=np.float64)
            actions_array = np.asarray(actions, dtype=np.float64)
            elite_count = max(1, int(math.ceil(population * args.elite_fraction)))
            elite_indices = np.argsort(rewards_array)[-elite_count:]
            elite_actions = actions_array[elite_indices]
            elite_mean = np.mean(elite_actions, axis=0)
            elite_std = np.std(elite_actions, axis=0)
            state.mean = (1.0 - args.update_rate) * state.mean + args.update_rate * elite_mean
            state.std = np.clip(
                (1.0 - args.update_rate) * state.std + args.update_rate * elite_std,
                args.min_std,
                args.max_std,
            )
            iteration_summary = {
                "iteration": iteration,
                "episode_count": total_episodes,
                "reward_mean": float(np.mean(rewards_array)),
                "reward_max": float(np.max(rewards_array)),
                "elite_count": elite_count,
                "best_reward": state.best_reward,
                "best_info": state.best_info,
                "policy_mode": args.policy_mode,
            }
            (out_dir / "latest_state.json").write_text(
                dumps_json(
                    {
                        **iteration_summary,
                        "policy_mode": args.policy_mode,
                        "initial_projection_error": initial_projection_error,
                        "mean": state.mean.tolist(),
                        "std": state.std.tolist(),
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            print(dumps_json(iteration_summary, indent=2))

    return {
        "out_dir": str(out_dir),
        "best_profile": str(best_profile_path),
        "training_log": str(log_path),
        "episodes": total_episodes,
        "best_reward": state.best_reward,
        "best_info": state.best_info,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, default=REPO_ROOT / "build/cpp/laptime_sim")
    parser.add_argument("--track", default="autocross")
    parser.add_argument(
        "--params-file",
        type=Path,
        default=REPO_ROOT / "matlab/parameter_presets/params_flexible_torque.json",
    )
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--initial-profile", type=Path)
    parser.add_argument("--time", type=float, default=120.0)
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--track-length-m", type=float, default=1000.0)
    parser.add_argument(
        "--policy-mode",
        choices=["basis", "delta-basis", "knots"],
        default="delta-basis",
        help="Residual policy parameterization used for CEM samples.",
    )
    parser.add_argument("--knots", type=int, default=64)
    parser.add_argument("--basis-count", type=int, default=24)
    parser.add_argument("--basis-width-m", type=float)
    parser.add_argument("--output-samples", type=int, default=128)
    parser.add_argument("--basis-delta-v-ref-mps", type=float, default=0.75)
    parser.add_argument("--basis-delta-steer-ff-rad", type=float, default=0.010)
    parser.add_argument("--basis-delta-ax-bias-mps2", type=float, default=0.25)
    parser.add_argument("--basis-delta-heading-relax-rad", type=float, default=0.015)
    parser.add_argument("--max-delta-v-ref-mps", type=float, default=2.0)
    parser.add_argument("--max-delta-steer-ff-rad", type=float, default=0.035)
    parser.add_argument("--max-delta-ax-bias-mps2", type=float, default=1.0)
    parser.add_argument("--max-delta-heading-relax-rad", type=float, default=0.08)
    parser.add_argument("--smoothing-passes", type=int, default=0)
    parser.add_argument("--lap-time-target", type=float, default=66.0)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--population", type=int, default=4)
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Parallel laptime evaluations per CEM iteration. Keep small for full 14DOF runs.",
    )
    parser.add_argument("--elite-fraction", type=float, default=0.25)
    parser.add_argument("--update-rate", type=float, default=0.65)
    parser.add_argument("--initial-std", type=float, default=0.20)
    parser.add_argument("--min-std", type=float, default=0.02)
    parser.add_argument("--max-std", type=float, default=0.50)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--skip-initial-profile-eval",
        action="store_true",
        help="Do not benchmark --initial-profile exactly before CEM sampling.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.executable.is_file():
        raise FileNotFoundError(f"laptime executable not found: {args.executable}")
    if args.params_file is not None and not args.params_file.is_file():
        raise FileNotFoundError(f"params file not found: {args.params_file}")
    summary = run_cem(args)
    print(dumps_json(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
