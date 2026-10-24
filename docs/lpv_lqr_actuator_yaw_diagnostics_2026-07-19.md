# LPV-LQR actuator and yaw-model diagnostics

Date: 2026-07-19

## Decision

Do not modify PINN-derived cornering stiffness yet.

The actuator-aware LPV model passes its synchronized one-step prediction gate under nominal,
steering-inertia +50%, and mass/inertia +20% conditions. The active bicycle model also matches
the PINN/14DOF truth plant's measured yaw gain within 3% on the diagnostic courses. Replacing
`Cf` or `Cr` now would not be supported by the measured steady-state response and could damage
the already-correct yaw gain.

The remaining autocross mean path-error gap should therefore be investigated in preview,
feedforward, frozen speed-profile, and safety-envelope behavior before tire stiffness.

## Diagnostic contract

All logged steering signals use tire-equivalent radians and the repository driver convention
(right turn positive):

```text
required_delta_preview  preview equilibrium feedforward
raw_delta_target        LPV feedback result before analytic safety projection
delta_target            post-safety 40 Hz target
inner_target_delta_tire 1 kHz shaped target_sw / R_steer
actual_delta_tire       measured rack-equivalent delta_sw / R_steer
predicted_delta_tire    Tustin-model prediction at t + 25 ms
```

The prediction uses the same second-order steering model and Tustin discretization as the
six-state LQR gain table. A prediction generated at outer update `k` is compared online with
the measured angle at update `k+1`; it is never compared with the same-row measurement.
`lpv_outer_sequence`, prediction target time, horizon, and validity fields make the alignment
explicit in CSV traces.

The prediction residual sign is:

```text
prediction_error = predicted(t + 25 ms) - actual(t + 25 ms)
```

The offline robust normalization is:

```text
NRMSE = RMSE / max(P95(actual) - P05(actual), 0.05 rad)
```

The diagnostic analyzer reports `insufficient_excitation` instead of treating a nearly
constant steering signal as a passing NRMSE result. Diagnostic runs can request 1 kHz CSV with
`--output-rate 0.001`; logging is scheduled by the next output time instead of floating-point
`fmod` coincidence.

## Reproduction

Example nominal transient run:

```bash
./build/cpp/laptime_sim \
  --track straight_corner --radius 15 --track-width 4 \
  --driver lpv_lqr --lpv-model steering6 \
  --initial-speed 8 --max-speed 12 --time 12 \
  --output-rate 0.001 \
  --params_file matlab/parameter_presets/params_flexible_torque.json \
  --output /tmp/lpv_diag_straight_nominal.csv \
  --output_json

python3 scripts/analyze_driver_trace.py \
  --trace /tmp/lpv_diag_straight_nominal.csv \
  --output /tmp/lpv_diag_straight_nominal.md \
  --summary-json /tmp/lpv_diag_straight_nominal.json
```

The same transient was run with `I_steer=0.03 kg m^2` and with mass/inertia +20%. Nominal
`circle` and `circle_cw` runs used radius 30 m, width 4 m, and the same 8--12 m/s speed range.

## Results

### Synchronized steering prediction

| Case | Aligned samples | Bias | RMSE | Robust NRMSE | Max error | Target-to-actual lag |
|---|---:|---:|---:|---:|---:|---:|
| Straight corner, nominal | 480 | +0.000160 rad | 0.000380 rad | 0.343% | 0.001598 rad | 51 ms |
| Straight corner, steering inertia +50% | 480 | +0.000112 rad | 0.000272 rad | 0.246% | 0.001418 rad | 50 ms |
| Straight corner, mass/inertia +20% | 480 | +0.000168 rad | 0.000393 rad | 0.349% | 0.002172 rad | 51 ms |
| Circle CCW, nominal | 651 | +0.000029 rad | 0.000353 rad | 0.707% | 0.001188 rad | insufficient excitation |
| Circle CW, nominal | 650 | -0.000038 rad | 0.000356 rad | 0.712% | 0.001215 rad | insufficient excitation |

The approximately 50 ms target-to-actual lag is the physical/ZOH command response, not a
25 ms prediction phase error. The aligned model prediction remains below 0.0004 rad RMSE in
the transient tests, so the model reproduces that lag rather than ignoring it.
The two circle traces have target excitation below 0.01 rad standard deviation; their actuator
gate is therefore `insufficient_excitation` despite the small descriptive prediction errors.
The three straight-corner actuator gates pass.

### Yaw gain

| Case | Actual gain | Model gain | Relative mismatch | Yaw bias |
|---|---:|---:|---:|---:|
| Straight corner, nominal | 7.282 1/s | 7.166 1/s | 1.62% | -0.00275 rad/s |
| Straight corner, steering inertia +50% | 7.284 1/s | 7.168 1/s | 1.62% | -0.00287 rad/s |
| Straight corner, mass/inertia +20% | 7.215 1/s | 7.105 1/s | 1.55% | -0.00121 rad/s |
| Circle CCW, nominal | 7.037 1/s | 6.990 1/s | 0.68% | -0.00019 rad/s |
| Circle CW, nominal | 7.177 1/s | 6.989 1/s | 2.70% | +0.00034 rad/s |

The left/right measured gain-amplitude difference is approximately 2.0%, below the 5% review
threshold. Bias is also well below 0.02 rad/s. These results do not support a scalar `Cf`/`Cr`
correction.

## Regression

The instrumented nominal torque-mode autocross still completes in 79.849 s with zero steering
saturation and zero track-limit excess. Its 3,193 aligned predictions have 0.000536 rad RMSE,
0.000013 rad bias, and 0.001613 rad maximum error. The outer controller averages 0.0083 ms.

## Next experiment

Freeze one nominal station-based speed profile and replay it for both bicycle4 and steering6.
This removes the current feedback loop in which better yaw/steering tracking also releases
speed caps. Compare station-binned path RMS, required-to-raw feedback contribution, and
raw-to-safe projection. Only if a later fixed-profile circle test shows yaw-gain error above
10% should the active PINN artifact's local `dFy/dalpha(Fz, camber, speed)` be extracted and
scheduled into the LPV model.
