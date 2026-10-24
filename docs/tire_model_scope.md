# Tire model scope and fidelity

This document states what the PINN tire model and the TMeasy fallback actually
cover. It is a scope statement, not a claim that either model reproduces a full
commercial TMeasy implementation.

## PINN inputs and fixed conditions

The deployed PINN uses five inputs:

| Input | Unit | Range in `pinn_config.yaml` |
|---|---|---|
| `SR` | ratio | -0.5 to 0.5 |
| `SA` | deg | -13 to 13 |
| `Fz` | N | 100 to 2000 |
| `IA` | deg | -5 to 5 |
| `TSTC` | degC | 25 to 90 |

Outputs are `Fx` [N], `Fy` [N], `Mx` [Nm], and `Mz` [Nm].

Not represented as inputs:

- inflation pressure
- tread wear and tire age
- tire-to-tire manufacturing variation
- rim width/offset beyond the fitted dataset
- track surface, water, and contamination
- brake/DriveBrake thermal history beyond `TSTC`

The model is only valid inside the measured input distribution. The `random`
split measures in-distribution interpolation; it is not a run-level
extrapolation result. Use the `group` split for held-out source runs.

## TMeasy fallback implementation

`cpp/include/fsae/physics/tire/tmeasy.hpp` is a reduced, educational TMeasy-like
model. It implements:

- first-order longitudinal/lateral deflection relaxation with `Lx`, `Ly`
- load-dependent friction `mu(Fz) = mu_max + dmu_dFz * (Fz - Fz_nom)`
- parabolic combined-slip saturation
- a linear pneumatic-trail approximation `Mz = -trail * Fy`
- a virtual velocity `v_N` to avoid low-speed singularities

It does not implement the full TMeasy parameter set: no pressure, wear, tire
individualization, full combined-slip similarity, turn-slip, or a measured
pneumatic-trail curve. The `v_eff` value is computed for the low-speed
formulation but the deflection damping currently uses `abs(vcx)`, so the two
must not be treated as interchangeable.

The fallback is intended for tests, initialization, and controlled comparisons.
Production vehicle runs remain PINN-only.

## Validation expectations

A deployable PINN must pass:

- Fx/Fy R2 gates on validation
- Mz R2 gate when the vehicle uses tire Mz for steering SAT
- zero-slip force and moment gates with separate N/Nm thresholds
- synthetic and measured-distribution friction-circle gates
- measured combined-slip holdout gate
- symmetry, monotonicity, and pneumatic-trail checks

The final report includes the split interpretation, normalization statistics,
validation summary, and a one-time held-out test summary.
