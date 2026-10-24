# PINN tire multirate runtime evaluation (2026-07-19)

## Decision

Use a 500 Hz simulation-time zero-order hold (ZOH) for the PINN tire force
evaluation by default. Keep `pinn_eval_rate_hz = 0` as the full RK4/K&C-stage
reference mode. Do not use 20 Hz for the 14DOF model: it failed the PID
acceleration test by leaving the track.

The rate is configurable through the nested vehicle parameter:

```json
{"tire":{"pinn_eval_rate_hz":500}}
```

`fsae_14dof` and `prof_fsae` also accept `--pinn-eval-rate-hz HZ` for direct
comparisons. A non-finite or negative rate is rejected.

## Why 20 Hz is too low

The physics step is 1 ms. Before this change, the PINN was evaluated at all
four RK4 stages plus the post-step cache refresh for every wheel: 20
predictions per physics step. A 20 Hz ZOH reduces this aggressively, but holds
the force for 50 ms.

At representative loads the effective tire relaxation lengths are about
0.11--0.12 m longitudinally and 0.19 m laterally. At 20--30 m/s this corresponds
to roughly 4--10 ms relaxation time constants. The model also contains steering
and unsprung modes around 13 Hz and 20 Hz. A 50 ms force hold therefore aliases
or delays dynamics that the 14DOF solver is intended to resolve.

## Evaluation count

Measured over 1,000 physics steps with runtime diagnostics enabled:

| PINN rate | predictions / physics step |
|---:|---:|
| full-stage reference (`0`) | 20.000 |
| 1000 Hz | 4.004 |
| 500 Hz | 2.004 |
| 250 Hz | 1.004 |
| 100 Hz | 0.404 |
| 50 Hz | 0.204 |
| 20 Hz | 0.084 |

The small `0.004` term is the four-wheel initial sample.

## Step-steer validation

The test used 20 m/s initial speed, a 0.1 rad steering step at 1 s, 1 ms
physics steps, and a 5 s duration. Relative RMS differences are computed over
the post-step interval (1--5 s) against the full-stage reference.

| rate | compute time | yaw-rate RMS diff | FL slip-angle RMS diff | FL vertical-load RMS diff | final position diff | final yaw diff |
|---:|---:|---:|---:|---:|---:|---:|
| 1000 Hz | 0.871 s | 0.087% | 0.089% | 0.262% | 0.062 m | 0.124 deg |
| 500 Hz | 0.520 s | 0.183% | 0.185% | 0.543% | 0.127 m | 0.254 deg |
| 250 Hz | 0.229 s | 0.390% | 0.381% | 1.175% | 0.270 m | 0.540 deg |
| 100 Hz | 0.100 s | 1.176% | 1.051% | 3.565% | 0.794 m | 1.594 deg |
| 50 Hz | 0.058 s | 2.749% | 2.357% | 8.264% | 1.821 m | 3.697 deg |
| 20 Hz | 0.033 s | 40.289% | 30.817% | 83.629% | 9.489 m | 27.026 deg |

The full-stage reference took 4.582 s. The 500 Hz case was about 8.8 times
faster while keeping the listed response errors below 1%. Error grows
monotonically as the evaluation rate is reduced.

## PID acceleration and autocross validation

PID acceleration:

| rate | result | lap time | process real time |
|---:|---|---:|---:|
| full-stage reference (`0`) | completed | 8.222 s | 11.19 s |
| 1000 Hz | completed | 8.222 s | 1.71 s |
| 500 Hz | completed | 8.222 s | 0.90 s |
| 250 Hz | completed | 8.223 s | 0.54 s |
| 100 Hz | completed | 8.223 s | 0.34 s |
| 50 Hz | completed | 8.192 s | 0.28 s |
| 20 Hz | **off track at 3.624 s** | not completed | 0.09 s |

Full PID autocross:

| metric | full-stage reference (`0`) | 500 Hz | difference |
|---|---:|---:|---:|
| completion | success | success | same |
| lap time | 79.538 s | 79.470 s | -0.068 s (-0.085%) |
| max lateral error | 0.862459 m | 0.866293 m | +0.003834 m |
| average lateral error | 0.242793 m | 0.240813 m | -0.001980 m |
| RMS heading error | 4.39649 | 4.40528 | +0.200% |
| process real time | 126.73 s | 20.31 s | 6.24x faster |

## Implementation contract and limitations

- All four wheels use independent force caches but a common simulation-time
  sampling grid, so one wheel cannot consume another wheel's sample slot.
- Cached values are after TTC-to-vehicle lateral sign conversion and before
  the normal tire-to-body rotation.
- `reset()` and `set_initial_velocity()` invalidate the cache.
- Direct external edits through the legacy mutable `state()` accessor are
  detected at the next `step()` and invalidate the cache before force reuse.
- The ZOH is deliberately simple and causal. It does not claim RK4 fourth-order
  accuracy for the held tire-force coupling.
- The current validation covers longitudinal acceleration, lateral step input,
  sprung/unsprung load transfer in that maneuver, and a complete PID autocross
  lap. Dedicated curb/bump inputs should still be added before reducing the
  default below 500 Hz.
- Camber-aware PINN exports with active K&C iteration have not been validated
  for multirate reuse; use rate `0` as the reference until that case is tested.
- Rates of 250 Hz and below remain research/surrogate options, not validated
  production settings. The 20 Hz result is an explicit failure boundary.

## Next optimization

The next safe speed target is not a lower fixed sampling rate. Prefer one of:

1. batch the four wheel PINN calls at each 500 Hz update;
2. cache or simplify zero-reference/symmetry/SA-zero postprocessing;
3. use a speed/load-aware adaptive rate bounded to 500--1000 Hz during fast
   transients, with lower rates only in verified quasi-steady regions.

These preserve the resolved tire and wheel-hop bandwidth better than a fixed
20--100 Hz force hold.

## Reproduction commands

```bash
cmake --build build --target prof_fsae fsae_14dof laptime_sim test_nn_tire

for rate in 0 1000 500 250 100 50 20; do
  ./build/cpp/prof_fsae 1000 --diagnostics --pinn-eval-rate-hz "$rate"
  ./build/cpp/fsae_14dof step_steer --pinn-eval-rate-hz "$rate"
  ./build/cpp/laptime_sim acceleration --driver pid --output_json \
    --params-json "{\"tire\":{\"pinn_eval_rate_hz\":${rate}}}"
done

./build/cpp/laptime_sim --track autocross --driver pid --output_json \
  --params-json '{"tire":{"pinn_eval_rate_hz":0}}'
./build/cpp/laptime_sim --track autocross --driver pid --output_json \
  --params-json '{"tire":{"pinn_eval_rate_hz":500}}'
```
