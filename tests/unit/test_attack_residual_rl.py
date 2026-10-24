from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from fsae_sim.rl import (  # noqa: E402
    AttackResidualEnvConfig,
    AttackResidualProfileEnv,
    ResidualBasisConfig,
    action_to_profile,
    basis_action_to_profile,
    basis_reconstruction_rmse,
    delta_basis_action_to_profile,
    build_attack_residual_dataset,
    profile_to_action,
    profile_to_basis_action,
    residual_basis_dim,
)
from fsae_sim.rl.attack_residual_env import _default_run  # noqa: E402


def test_action_profile_round_trip_uses_bounded_residuals() -> None:
    action = np.linspace(-1.5, 1.5, 8)
    profile = action_to_profile(
        action,
        knot_count=4,
        track_length_m=100.0,
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=0.035,
        smoothing_passes=0,
    )

    assert profile["schema"] == "attack_residual_profile_v1"
    assert len(profile["samples"]) == 4
    assert max(abs(sample["delta_v_ref_mps"]) for sample in profile["samples"]) <= 2.0
    assert max(abs(sample["delta_steer_ff_rad"]) for sample in profile["samples"]) <= 0.035

    recovered = profile_to_action(
        profile,
        knot_count=4,
        track_length_m=100.0,
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=0.035,
    )
    assert recovered.shape == (8,)
    assert np.all(recovered <= 1.0)
    assert np.all(recovered >= -1.0)


def test_basis_policy_generates_bounded_profile_and_projects_seed() -> None:
    config = ResidualBasisConfig(
        track_length_m=1000.0,
        basis_count=12,
        output_samples=48,
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=0.035,
    )
    action = np.linspace(-2.0, 2.0, residual_basis_dim(config))
    profile = basis_action_to_profile(action, config)

    assert profile["metadata"]["policy_mode"] == "basis"
    assert len(profile["samples"]) == 48
    assert max(abs(sample["delta_v_ref_mps"]) for sample in profile["samples"]) <= 2.0
    assert max(abs(sample["delta_steer_ff_rad"]) for sample in profile["samples"]) <= 0.035

    seed = {
        "schema": "attack_residual_profile_v1",
        "samples": [
            {"s": 0.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.0},
            {"s": 820.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.015},
            {"s": 900.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.015},
            {"s": 940.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.0},
        ],
    }
    projected = profile_to_basis_action(seed, config)
    assert projected.shape == (residual_basis_dim(config),)
    assert np.all(projected <= 1.0)
    assert np.all(projected >= -1.0)
    rmse = basis_reconstruction_rmse(seed, config)
    assert rmse["delta_steer_ff_rmse_rad"] < 0.010


def test_delta_basis_policy_perturbs_existing_profile_within_limits() -> None:
    config = ResidualBasisConfig(
        track_length_m=100.0,
        basis_count=4,
        output_samples=8,
        max_delta_v_ref_mps=2.0,
        max_delta_steer_ff_rad=0.035,
        basis_delta_v_ref_mps=0.5,
        basis_delta_steer_ff_rad=0.004,
    )
    base_profile = {
        "schema": "attack_residual_profile_v1",
        "samples": [
            {"s": 0.0, "delta_v_ref_mps": 0.25, "delta_steer_ff_rad": 0.010},
            {"s": 100.0, "delta_v_ref_mps": 0.25, "delta_steer_ff_rad": 0.010},
        ],
    }

    zero_profile = delta_basis_action_to_profile(
        np.zeros(residual_basis_dim(config)),
        base_profile,
        config,
    )
    assert all(sample["delta_v_ref_mps"] == 0.25 for sample in zero_profile["samples"])
    assert all(sample["delta_steer_ff_rad"] == 0.010 for sample in zero_profile["samples"])
    assert all(sample["delta_ax_bias_mps2"] == 0.0 for sample in zero_profile["samples"])
    assert all(sample["delta_heading_relax_rad"] == 0.0 for sample in zero_profile["samples"])

    perturbed = delta_basis_action_to_profile(
        np.ones(residual_basis_dim(config)) * 3.0,
        base_profile,
        config,
    )
    assert max(abs(sample["delta_v_ref_mps"]) for sample in perturbed["samples"]) <= 2.0
    assert max(abs(sample["delta_steer_ff_rad"]) for sample in perturbed["samples"]) <= 0.035
    assert max(abs(sample["delta_ax_bias_mps2"]) for sample in perturbed["samples"]) <= 1.0
    assert max(sample["delta_heading_relax_rad"] for sample in perturbed["samples"]) <= 0.08
    assert perturbed["metadata"]["policy_mode"] == "delta-basis"


def test_default_runner_converts_timeout_to_completed_process(tmp_path: Path) -> None:
    completed = _default_run(
        [sys.executable, "-c", "import time; time.sleep(1.0)"],
        tmp_path,
        0.01,
    )

    assert completed.returncode == 124
    assert "TIMEOUT" in completed.stderr


def test_dataset_builder_uses_teacher_profile_targets(tmp_path: Path) -> None:
    csv_path = tmp_path / "attack.csv"
    csv_path.write_text(
        "\n".join(
            [
                "time,s,speed,course_v_ref,course_target_gap,dynamic_cap_gap,"
                "course_lateral_error,d_heading,course_kappa_ref,"
                "attack_dynamic_speed_cap,attack_heading_speed_cap,attack_margin_speed_cap,"
                "mpc_predicted_min_margin,attack_selected_speed_scale,throttle,brake,"
                "healthy_underspeed",
                "0.0,0.0,10.0,12.0,2.0,0.0,0.1,0.02,0.04,12.0,20.0,20.0,1.5,0.95,0.5,0.0,true",
                "0.1,9.0,10.5,12.0,1.5,0.0,0.1,0.02,0.04,12.0,20.0,20.0,1.5,0.95,0.5,0.0,true",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    teacher_path = tmp_path / "teacher.json"
    teacher_path.write_text(
        json.dumps(
            {
                "schema": "attack_residual_profile_v1",
                "samples": [
                    {"s": 0.0, "delta_v_ref_mps": 0.5, "delta_steer_ff_rad": -0.01},
                    {"s": 20.0, "delta_v_ref_mps": 0.5, "delta_steer_ff_rad": -0.01},
                ],
            }
        )
        + "\n",
        encoding="utf-8",
    )

    dataset = build_attack_residual_dataset(
        [csv_path],
        bin_width_m=20.0,
        track_length_m=100.0,
        teacher_profile=teacher_path,
    )

    assert dataset.features.shape == (1, 16)
    assert dataset.actions.shape == (1, 2)
    np.testing.assert_allclose(dataset.actions[0], [0.5, -0.01], rtol=1e-6)


def test_attack_residual_env_fake_runner_writes_profile_and_rewards(tmp_path: Path) -> None:
    def fake_runner(cmd: list[str], cwd: Path, timeout: float | None):
        csv_path = Path(cmd[cmd.index("--output") + 1])
        csv_path.write_text(
            "time,s,speed,course_target_gap\n"
            "0.0,0.0,10.0,1.0\n"
            "1.0,20.0,12.0,0.5\n",
            encoding="utf-8",
        )
        stdout = (
            "%%JSON_BEGIN%%"
            '{"lap_completed":true,"lap_time":87.0,"distance_traveled":980.0,'
            '"max_track_limit_excess":0.0,"mpc_fail_count":0}'
            "%%JSON_END%%"
        )
        return subprocess.CompletedProcess(cmd, 0, stdout=stdout, stderr="")

    env = AttackResidualProfileEnv(
        AttackResidualEnvConfig(
            executable=tmp_path / "laptime_sim",
            repo_root=tmp_path,
            out_dir=tmp_path / "runs",
            params_file=None,
            knot_count=2,
            track_length_m=100.0,
            time_limit_s=120.0,
        ),
        run_command=fake_runner,
    )
    _, reward, terminated, truncated, info = env.step(np.zeros(env.action_dim))

    assert terminated is True
    assert truncated is False
    assert reward == 66.0 - 87.0
    assert info["lap_completed"] is True
    assert Path(info["profile_path"]).exists()


def test_attack_residual_env_noncompleted_json_is_not_runner_error(tmp_path: Path) -> None:
    def fake_runner(cmd: list[str], cwd: Path, timeout: float | None):
        csv_path = Path(cmd[cmd.index("--output") + 1])
        csv_path.write_text("time,s,speed,course_target_gap\n0.0,0.0,8.0,2.0\n", encoding="utf-8")
        stdout = (
            "%%JSON_BEGIN%%"
            '{"lap_completed":false,"lap_time":0.0,"distance_traveled":100.0,'
            '"max_track_limit_excess":0.0,"mpc_fail_count":0}'
            "%%JSON_END%%"
        )
        return subprocess.CompletedProcess(cmd, 1, stdout=stdout, stderr="lap_not_completed")

    env = AttackResidualProfileEnv(
        AttackResidualEnvConfig(
            executable=tmp_path / "laptime_sim",
            repo_root=tmp_path,
            out_dir=tmp_path / "runs",
            params_file=None,
            knot_count=2,
            track_length_m=100.0,
            time_limit_s=120.0,
        ),
        run_command=fake_runner,
    )

    _, reward, _, _, info = env.step(np.zeros(env.action_dim))

    assert info["lap_completed"] is False
    assert reward == -120.0 + 0.02 * 100.0


def test_attack_residual_env_evaluates_exact_profile_without_resampling(tmp_path: Path) -> None:
    input_profile = tmp_path / "initial.json"
    input_profile.write_text(
        json.dumps(
            {
                "schema": "attack_residual_profile_v1",
                "samples": [
                    {"s": 0.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.0},
                    {"s": 835.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.015},
                    {"s": 905.0, "delta_v_ref_mps": 0.0, "delta_steer_ff_rad": 0.015},
                ],
            },
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    def fake_runner(cmd: list[str], cwd: Path, timeout: float | None):
        profile_path = Path(cmd[cmd.index("--attack-residual-profile") + 1])
        assert profile_path.read_text(encoding="utf-8") == input_profile.read_text(
            encoding="utf-8"
        )
        csv_path = Path(cmd[cmd.index("--output") + 1])
        csv_path.write_text("time,s,speed,course_target_gap\n0.0,0.0,8.0,0.0\n", encoding="utf-8")
        stdout = (
            "%%JSON_BEGIN%%"
            '{"lap_completed":true,"lap_time":87.0,"distance_traveled":1000.0,'
            '"max_track_limit_excess":0.0,"mpc_fail_count":0}'
            "%%JSON_END%%"
        )
        return subprocess.CompletedProcess(cmd, 0, stdout=stdout, stderr="")

    env = AttackResidualProfileEnv(
        AttackResidualEnvConfig(
            executable=tmp_path / "laptime_sim",
            repo_root=tmp_path,
            out_dir=tmp_path / "runs",
            params_file=None,
        ),
        run_command=fake_runner,
    )

    result = env.evaluate_profile_file(input_profile)

    assert result.lap_completed is True
    assert result.lap_time == 87.0
