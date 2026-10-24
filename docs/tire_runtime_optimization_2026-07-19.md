# PINN tire runtime optimization (2026-07-19)

## Scope

This change accelerates the deployed `pinn_runtime_v2_multi_head` evaluator without changing
the tire artifact, input/output contract, runtime postprocessing order, or the 14DOF integration
scheme. It is an exact-runtime optimization, not a surrogate tire model.

## Changes

- Reuse persistent inference buffers instead of constructing a temporary vector for every
  LayerNorm and copying every head result through a second evaluation buffer.
- Normalize the five physical inputs directly into the reusable input buffer.
- Evaluate flattened row-major linear-layer weights with Eigen's vectorized matrix-vector path.
- Retain the nested-vector fallback used by legacy runtime assets.
- Add `prof_nn_tire`, which measures the production `include_mx=false` path over the exported
  512-vector set and prints a numerical checksum.
- Allow `prof_fsae` to accept a physics-step count and report time per step and wall/sim ratio.

## Performance

Release build with the repository's native CPU flags. The first cold run was discarded where
multiple runs were available.

| Measurement | Before | After | Result |
|---|---:|---:|---:|
| PINN prediction, paired warm median | 44.180 us | 25.595 us | 42.1% lower latency, 1.73x throughput |
| PINN prediction, independent after-run median | - | 27.021 us | 5 warm runs, 24.978-28.851 us |
| 14DOF, 5,000 steps | wall/sim 2.577 | median 1.339 | 48.0% lower wall/sim, 1.93x faster |
| 14DOF, 20,000 steps | 55.4-59.9 s | 28.544 s | 48.5-52.4% lower wall time |
| PID acceleration | 14.66 s wall | 10.62 s wall | same 8.222 s simulated result |

Timing varies with CPU state, so later comparisons should use serial warm runs and report the
median rather than a single best value.

## Numerical validation

The scalar implementation at repository `HEAD` and the optimized implementation were compiled
with the same Release flags and compared over four deployable tire artifacts. Each artifact used
6,521 inputs: its 512 exported vectors plus an operating lattice, input-boundary cases, and
postprocess cutoff neighborhoods. Both `include_mx=true` and `include_mx=false` were evaluated.

Maximum absolute differences over 26,084 inputs were:

| Output | Maximum absolute difference |
|---|---:|
| Fx | 4.55e-12 N |
| Fy | 6.37e-12 N |
| Mx | 1.42e-14 Nm |
| Mz | 2.34e-13 Nm |

The differences come from the SIMD accumulation order and are far below the runtime acceptance
limits of `1e-7 N` for forces and `1e-9 Nm` for moments. Zero-slip, symmetry, friction-cap,
multi-head mapping, exported-vector, and `include_mx=false` tests pass. The step-steer and PID
acceleration outputs remain finite and preserve their pre-change result contracts.

## Remaining optimization opportunities

One 1 ms 14DOF step performs five derivative evaluations (four RK4 stages plus final-state cache
refresh), with four tire predictions per derivative evaluation. The production path therefore
performs 20 tire predictions per physics step.

The next performance work should be evaluated separately:

1. Measure postprocess gate and variant counts on actual lap traces.
2. Reuse exactly identical original/zero-reference/mirror/zero-Fy evaluations.
3. Batch the required postprocess variants within one tire prediction.
4. Batch all four wheels within each derivative evaluation.

Approximate caches, load/temperature quantization, and lookup interpolation are intentionally
excluded from this exact-runtime change. They belong to a later surrogate-model phase with
vehicle-level error and transfer-learning validation.
