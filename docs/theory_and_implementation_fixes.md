# Theory and Implementation Fixes

This note tracks high-priority corrections where implementation details directly affected
the physical interpretation of simulation or optimization results.

## Fixed Items

### 1. Magic Formula longitudinal sign at the vehicle interface

The raw Magic Formula implementation keeps the TTC fitting convention internally: vertical load
is negative and positive drive slip produces a negative raw longitudinal force. The 14DOF vehicle
model and TMeasy convention use positive wheel-frame `Fx` as driving force.

The interface now converts raw Magic Formula forces through
`magic_formula_to_vehicle_convention()`, negating only `Fx`. Lateral force and aligning moment
remain in the existing vehicle convention.

Regression test:

```text
MagicFormulaTest.VehicleConventionMapsPositiveDriveSlipToPositiveFx
```

### 2. Ackermann steering side assignment

The 14DOF path uses SAE J670e coordinates: `X` forward, `Y` right, `Z` down. Under this convention,
positive steering is treated as a right turn in the driver/model interface.

Ackermann steering now assigns the inner wheel as:

- positive steering: front right is inner
- negative steering: front left is inner

The geometry uses absolute turn radius for inner/outer radii and reapplies the steering sign,
avoiding the previous negative-radius side reversal.

Regression tests:

```text
Phase5Test.AckermannGeometry_RightTurn_InnerWheelGreater
Phase5Test.AckermannGeometry_LeftTurn_InnerWheelGreater
Phase5Test.AckermannGeometry_FullAckermann_vs_Parallel
```

### 3. Frame torsional rigidity from distributed couple loads

FSAE frame torsion load cases distribute vertical couple forces across multiple front suspension
nodes. The previous solver detected only the first opposite-sign pair and computed torque from one
node load, which undercounted applied torque when loads were distributed.

The FEM TCP, standalone, and MEX solvers now share `torsion_rigidity.hpp`, which:

- uses direct `Rx` moment loads when present
- otherwise groups all positive and negative `uz` loads
- computes weighted left/right load centroids and weighted vertical displacements
- computes torque from the total balanced couple force and the centroid arm
- computes twist from the weighted displacement difference over the same arm

Regression test:

```text
FrameTorsionMetricTest.DistributedCoupleUsesTotalAppliedForce
```

The MEX build path was also updated for MATLAB R2025b on macOS. The solver uses the classic
`mexFunction()` C MEX entry point from a C++ source file, so `build_mex.m` builds with the
classic `-R2017b` ABI and disables C++ MEX adapter exports with `LINKEXPORTCPP=`; otherwise the
linker requires unrelated adapter symbols.

### 4. Fail-fast parameter and laptime handling

`VehicleParams::from_json()` and `from_json_string()` now throw parse/open errors instead of
silently returning default vehicle parameters. Laptime simulation now marks the result as failed
with `lap_not_completed` when the loop reaches `max_time` without completing a lap.

The laptime planner also validates core vehicle parameters before generating the velocity profile.
Invalid mass, tire radius, or tire friction limits now fail with `invalid_vehicle_params` instead of
being replaced by hidden proxy values such as `mu = 1.5`.

### 5. Tire fitting DriveBrake slip-angle filter

TTC `SA` is loaded in degrees. The DriveBrake pure-longitudinal filter now compares `|SA|` against
`5.0` degrees directly instead of comparing degrees against a radian threshold.

### 6. RK4 derivative cache side effects

`Vehicle14DOF_New::compute_derivatives()` is now evaluated as a stage-local calculation during
RK4. Tire force, suspension, POU, and load caches are written only when the caller explicitly asks
for a committed cache update after reset or after the full integration step.

This removes RK4 stage-to-stage mutation of `tire_Mz_`, `tire_Fz_`, and related debug arrays.
Compliance steer still uses the last committed aligning torque as an explicit-lag model, so it
should be treated as an explicit approximation rather than an implicit tire/steering solve.

### 7. Tire fitting entrypoint and validation

`tire_fitting/main.py` is now a compatibility wrapper around `tire_fitting/main_hybrid.py`.
The CLI options `--model-long` and `--model-lat` therefore select actual model implementations
instead of reporting Hybrid while fitting TMeasy only.

Quality validation now fails the command by default when thresholds are not met. Use
`--allow-validation-fail` only for exploratory fitting where a non-deployable artifact is expected.

PINN training/evaluation also treats non-deployable output as a failed CLI result by default.
`--allow-nondeployable` is now the explicit exploratory escape hatch; runtime export remains skipped
unless the deployment gate passes. The configured deployability threshold is
`validation_targets.r2_min` and is documented consistently as `0.90` unless overridden by a tire
profile.

### 8. Driver parameter linkage

PID, RL, and the RL TCP server now select gears from vehicle artifact parameters:
gear ratios, primary/final reduction, engine RPM points, and tire radius. PID DRS request is no
longer unconditional; it requests DRS only under high-speed, low-curvature, low-steering conditions,
with `DRSSystem` still applying final safety gating. RL curvature speed limiting now derives
friction and braking deceleration from the tire parameters instead of fixed `mu = 1.8`.

Regression tests:

```text
DriverParameterLinkageTest.GearSelectionUsesVehicleRatiosAndTireRadius
RLControlSafetyTest.CurvatureLimitUsesTireMuForBrakeDemand
```

### 9. POU and frame fail-fast integration

POU hardpoint optimization now fails when required polynomial targets or artifact hardpoints are
missing. If a vehicle artifact is provided, tie-rod hardpoints must also come from the artifact;
the optimizer no longer fills missing steering points with hardcoded placeholders. Non-converged
optimizer output is accepted only when explicitly requested through `allow_partial_solution`.
Batch success also requires packaging validity.

`matlab/run_full_pipeline.m` now treats POU failure as fatal by default. Use
`allow_pou_failure=true` only for diagnostic runs where downstream results are intentionally
allowed to omit physical POU hardpoints.

Frame compliance material checks now call `checkMaterialRestriction()` with the canonical
group/pipe/material signature and infer material identity from element elastic modulus when needed.
The canonical frame objective now includes the full frame compliance penalty/report in the
objective results instead of leaving it as a separate diagnostic.

Suspension inverse targets now carry a `sources` struct and `model_scope` notes. Any source marked
`fallback_*` is a provisional design assumption, not measured K&C data.

### 10. 14DOF demo CSV wheel channels

14DOF demo CSV output now writes cached per-wheel tire normal loads and lagged slip angles instead
of zero-filled placeholders. The slip-angle columns are explicitly named
`slip_angle_deg_*`.

### 11. Docker distribution and test environment

The Docker image now includes C++ build dependencies, OSQP/EnTT/autodiff, PyTorch/PINN Python
dependencies, pytest coverage tooling, Hypothesis, Black, Ruff, and MyPy. The container entrypoint
supports:

```text
build
test
test-cpp
test-python
dev
```

`test`, `test-cpp`, and `test-python` configure/build before running tests, so a stale Docker build
volume cannot silently run old binaries.

## Remaining Model-Scope Limitations

These were not converted into new physics models in this pass because they require validation data
or broader architecture work:

- POU hardpoint optimization still uses a reduced upright/pushrod kinematic model.
- Suspension K&C metrics still include proxy anti/compliance metrics.
- Velocity profile and RL safety still use simplified friction assumptions, although RL safety now
  reads the tire friction limit from the active vehicle parameters.
- Beam elements are still Euler-Bernoulli-form stiffness matrices despite older text referring to
  Timoshenko beams.

Treat results from those areas as optimization surrogates until each model is replaced or calibrated.
