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

- inflation pressure (training is filtered to one running pressure,
  `data.pressure_filter_kpa` in `pinn_config.yaml`, default 82.7 kPa = 12 psi; #18)
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
pneumatic-trail curve. The deflection damping uses `abs(vcx)` (the unused
`v_eff` was removed in #22).

`Cx`/`Cy` are contact-patch (carcass) stiffnesses in N/m, not slip stiffnesses.
In steady state `xi = Lx*kappa`, `eta = Ly*tan(alpha)` and the parabola has
initial slope 2, so `C_alpha = 2*Cy*Ly` [N/rad] and `C_kappa = 2*Cx*Lx` [N].
The default `Cy = 44900 N/m` reproduces the measured 22436 N/rad with
`Ly = 0.25 m` (`test_tmeasy` `SteadyStateCorneringStiffnessMatchesMeasured`).
Vehicle classes that set `Cy` themselves (e.g. `vehicle_params.hpp`
`tire.Cy = 22000`, 3/6/8-DOF models) still carry the old value; see `WIRING.md`.

The fallback is intended for tests, initialization, and controlled comparisons.
Production vehicle runs remain PINN-only.

### Known fallback limitations kept as test-only scope (#22)

These are deliberately not fixed because production is PINN-only
(`initialize_selected_tire_model` rejects other models outside test builds):

- TMeasy: isotropic parabolic combined slip `mu_eff = s(2-s)` instead of Rill's
  direction-wise characteristics (dF0, sM, FM, sS, FS); linear `dmu_dFz`; fixed
  pneumatic trail `Mz = -trail*Fy`; no camber, temperature or pressure inputs.
  Upgrade path if a non-PINN model is ever needed in production: Rill's
  per-direction TMeasy characteristics.
- C++ Magic Formula (`magic_formula.hpp`): `Mz` uses the pure-slip `Fy0` and
  omits the combined-slip `s*Fx` term. None of the shipped MF parameter sets
  contain `ssz1..ssz4` (and the loader does not parse them), so the term would
  be identically zero; add it together with those coefficients. No camber-thrust
  terms (`pKy3`, `pVy3`, `pHy3`); `Mx = 0`.
- Python `tire_fitting/models/magic_formula.py` is an intentional simplification
  (no `pDx2`/`pDy2` load dependence of mu, no SH/SV, no combined slip, no Mz). It
  is a fitting/teaching tool, not a truth model; the C++ MF reads the separate
  MATLAB fit. Keep as is unless it becomes a truth model.
- The former `thermal.hpp` `mu_factor` is gone (#19); temperature reaches the
  force model only through the PINN `TSTC` input.

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
