# Attack Residual RL

This workflow learns a small residual profile for the C++ attack driver.  The
real-time controller stays in C++; Python only proposes fixed-course
corrections consumed by `--attack-residual-profile`.

## Artifacts

- Dataset: `.npz` with station-binned features and residual targets.
- Policy: `attack_residual_profile_v1` JSON with bounded speed, steering,
  longitudinal acceleration, and heading-safety residual samples.
- Training log: JSONL, one row per laptime episode.

## Build Dataset

```bash
python3 python/scripts/build_attack_residual_dataset.py \
  --csv /path/to/attack_laptime.csv \
  --output /tmp/attack_residual_dataset.npz \
  --teacher-profile data/driver/attack_residual_autocross_torque_flex_v1.json \
  --bin-width-m 10 \
  --track-length-m 1000
```

## Train Residual Profile

The default trainer uses CEM black-box policy search and does not require
Gymnasium or Stable-Baselines3.

```bash
python3 python/scripts/train_attack_residual_rl.py \
  --iterations 8 \
  --population 6 \
  --time 120 \
  --out-dir /tmp/attack_residual_rl_run \
  --initial-profile data/driver/attack_residual_autocross_torque_flex_v1.json \
  --track autocross \
  --params-file matlab/parameter_presets/params_flexible_torque.json \
  --driver-torque-max 40 \
  --policy-mode delta-basis \
  --basis-count 24 \
  --output-samples 128 \
  --timeout 600
```

Then benchmark the best profile:

```bash
./build/cpp/laptime_sim \
  --track autocross \
  --driver attack \
  --time 120 \
  --params_file matlab/parameter_presets/params_flexible_torque.json \
  --driver-torque-max 40 \
  --attack-residual-profile /tmp/attack_residual_rl_run/best_residual_profile.json \
  --output /tmp/attack_residual_rl_best.csv \
  --output_json
```

## Design Contract

- Non-completed laps with valid JSON/CSV are valid training episodes.  They are
  rewarded by distance and safety metrics, not treated as runner crashes.
- Runner errors are cases where `laptime_sim` returns no JSON result.
- The residual action is bounded before C++ sees it:
  - `delta_v_ref_mps` in `[-2.0, 2.0]`
  - `delta_steer_ff_rad` in `[-0.035, 0.035]`
  - `delta_ax_bias_mps2` in `[-1.0, 1.0]`
  - `delta_heading_relax_rad` in `[0.0, 0.08]`
- Longitudinal residuals are safety-gated in C++: they only apply when margin is
  healthy, outward rate is low, and raw heading error remains below the hard
  safety threshold.  Hard boundary and hard heading braking are not bypassed.
- The default CEM policy uses low-dimensional RBF delta-basis coefficients on
  top of the exact initial profile.  Zero action preserves the seed profile.
  Use `--policy-mode knots` only for direct knot-by-knot ablations.
- Runner timeouts are logged as failed episodes instead of aborting the whole
  training job.  Use a timeout comfortably above one full attack run on the
  current machine.
- Existing PID/MPC/RL drivers are not modified by this workflow.
