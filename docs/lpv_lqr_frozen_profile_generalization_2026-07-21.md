# Frozen-profile LPV-LQR generalization experiment

Date: 2026-07-21

## Decision

The six-state steering-actuator LPV-LQR is suitable as the default LPV model for the tested
torque-steering vehicles. It completed the ASE autocross with the same frozen station-speed
profile under nominal, steering-inertia +50%, and mass/inertia +20% conditions. All runs stayed
inside the track and remained finite. Relative to the four-state fallback, the six-state model
reduced lap time by 10.9--14.7% and mean steering-wheel tracking error by 73.8--79.4%.

The experiment also exposed unconstrained multi-radian LQR requests hidden by the safety
projection. The controller now limits its pre-safety request to the physical rack-angle
contract. The independent rate, stability, and track-margin safety projections remain active.

## Frozen-profile contract

The canonical artifact is
`data/driver/ase_autocross_nominal_torque_velocity_profile_v1.json`. It was generated once with:

- track: `autocross` (`data/ASE_autocross_sim.csv`)
- vehicle: `matlab/parameter_presets/params_flexible_torque.json`
- initial speed: 6 m/s
- station spacing: 1 m
- points: 1007
- semantic fingerprint: `fnv1a64:83acdae2d53787bd`
- file SHA-256: `2a67f4a5d38a3d97de2618ccb65184f78daea5b47d5379a9e78cbb007c7671df`

`--velocity-profile-out FILE` exports a generated profile and
`--velocity-profile-in FILE` bypasses profile generation. The loader rejects schema, finite-value,
station-order, endpoint, closed-loop seam, track-length, track-geometry, and fingerprint failures.
The JSON result reports the source, path, semantic fingerprint, and point count.

Only the planner base `v(s)` is frozen. Online curvature, steering, stability, and track-margin
caps remain enabled because they are part of the controller's response to the actual vehicle.

## Vehicle matrix

All six runs used the same artifact and controller weights. The two perturbed profiles are
committed under `data/driver/` and change no controller parameter.

| vehicle | LPV model | lap time s | max lateral m | mean lateral m | heading RMS deg | mean SW error rad | saturation | track excess m |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| nominal | bicycle4 | 89.788 | 0.5909 | 0.1456 | 4.4449 | 0.3316 | 0.459% | 0 |
| nominal | steering6 | 79.967 | 0.4995 | 0.1851 | 4.4847 | 0.0868 | 0% | 0 |
| steering inertia +50% | bicycle4 | 90.871 | 0.4671 | 0.1366 | 4.3616 | 0.3666 | 3.914% | 0 |
| steering inertia +50% | steering6 | 79.959 | 0.5027 | 0.1845 | 4.4579 | 0.0835 | 0% | 0 |
| mass/inertia +20% | bicycle4 | 93.570 | 0.4551 | 0.1302 | 4.3238 | 0.4190 | 1.674% | 0 |
| mass/inertia +20% | steering6 | 79.790 | 0.5456 | 0.1935 | 4.3245 | 0.0862 | 0% | 0 |

| vehicle | lap improvement | SW error reduction | six-state actuator RMSE rad | outer avg ms | outer max ms |
|---|---:|---:|---:|---:|---:|
| nominal | 10.9% | 73.8% | 0.000545 | 0.0107 | 0.0280 |
| steering inertia +50% | 12.0% | 77.2% | 0.000431 | 0.0107 | 0.0637 |
| mass/inertia +20% | 14.7% | 79.4% | 0.000540 | 0.0108 | 0.0555 |

The three six-state cases reproduced the same lap times in independent equivalent-weight runs
to the simulator's 1 ms reporting resolution.

## Station-weighted tracking and safety decomposition

`scripts/analyze_lpv_frozen_profile_matrix.py` unwraps the first lap and linearly resamples all
traces on a common 0.5 m grid. This avoids giving slower cases extra weight merely because they
spend longer in a segment.

| vehicle | model | lateral RMS m | lateral P95 m | max lateral m | feedback RMS rad | safety projection RMS rad | projection active |
|---|---|---:|---:|---:|---:|---:|---:|
| nominal | bicycle4 | 0.1797 | 0.3661 | 0.5453 | 0.1014 | 0.0517 | 56.6% |
| nominal | steering6 | 0.2253 | 0.4005 | 0.4868 | 0.0703 | 0.0352 | 2.9% |
| steering inertia +50% | bicycle4 | 0.1692 | 0.3451 | 0.4593 | 0.1048 | 0.0571 | 63.9% |
| steering inertia +50% | steering6 | 0.2253 | 0.4007 | 0.4942 | 0.0681 | 0.0326 | 3.1% |
| mass/inertia +20% | bicycle4 | 0.1619 | 0.3216 | 0.4448 | 0.1186 | 0.0673 | 74.2% |
| mass/inertia +20% | steering6 | 0.2388 | 0.4277 | 0.5336 | 0.0691 | 0.0334 | 3.1% |

The six-state model intentionally trades some station RMS lateral error for much lower steering
lag and higher speed. Its absolute station RMS remains below 0.24 m, the paired increase remains
below 0.08 m, and the worst ratio is 1.475. Crucially, feedback effort and safety-projection RMS
both decrease, so the lap gain is not produced by leaning harder on the safety filter.

Trace-interpolated `target_speed_base` values should not be used as a bit-exact equality test:
the 40 Hz target is held while CSV state stations are sampled at 100 Hz, and each controller
traverses stations at different times. Identity is instead enforced by the validated artifact
fingerprint reported by every run.

## Reproduction

Generate the artifact once:

```bash
./build/cpp/laptime_sim \
  --track autocross --driver lpv_lqr --lpv-model steering6 \
  --initial-speed 6 \
  --params_file matlab/parameter_presets/params_flexible_torque.json \
  --time 0.01 \
  --velocity-profile-out data/driver/ase_autocross_nominal_torque_velocity_profile_v1.json \
  --output_json
```

Replay it by replacing the vehicle file and model as required:

```bash
./build/cpp/laptime_sim \
  --track autocross --driver lpv_lqr --lpv-model steering6 \
  --initial-speed 6 --time 130 --output-rate 0.01 \
  --params_file data/driver/vehicle_profile_mass_inertia_plus20.json \
  --velocity-profile-in data/driver/ase_autocross_nominal_torque_velocity_profile_v1.json \
  --output /tmp/mass_inertia_plus20_steering6.csv \
  --output_json
```

Analyze the six named traces:

```bash
python3 scripts/analyze_lpv_frozen_profile_matrix.py \
  --matrix-dir output/runtime/lpv_frozen_profile_20260721 \
  --profile data/driver/ase_autocross_nominal_torque_velocity_profile_v1.json \
  --output output/runtime/lpv_frozen_profile_20260721/station_metrics.json
```

## Gate summary

- all six runs: success, completed lap, full PINN/14DOF mode, finite, zero track excess
- all six-state runs: max lateral below 0.65 m, time-weighted heading RMS below 5 degrees,
  mean SW error below 0.12 rad, zero steering saturation
- controller runtime: average below 0.011 ms and maximum below 0.064 ms in the final matrix
- paired: SW error reduction above 60%, lap regression absent, median lap improvement above 5%
- station: RMS below 0.30 m, paired increase at most 0.08 m, ratio below 1.60
- safety: projection RMS and active ratio lower in every six-state pair
- prior transient diagnostic prerequisite: actuator robust NRMSE below 2% and yaw mismatch below 5%

The four-state `bicycle4` model remains available as an explicit fallback through
`--lpv-model bicycle4`.
