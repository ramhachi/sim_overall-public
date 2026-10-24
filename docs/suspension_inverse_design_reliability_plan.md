# Suspension inverse design reliability improvement plan

Date: 2026-05-14

This plan treats every criticism in the suspension inverse-design review as an
improvement target. The acceptance bar is not "the scripts run"; it is "the
inverse design output is trustworthy enough to feed steering, POU, frame, and
vehicle dynamics decisions without hiding provisional assumptions."

## Current finding

The current suspension inverse-design stack is useful as a first hardpoint
surrogate, but it is not yet a reliable K&C inverse-design tool.

Primary evidence in the current codebase:

- `suspension_kinematics_design/build_inverse_targets.m` creates many targets
  from fallback defaults or zeros and only records source strings; the optimizer
  does not use this source quality to reject or down-weight provisional targets.
- `suspension_kinematics_design/main_a_arm_optimizer.m` and
  `suspension_kinematics_design/main_rear_a_arm_optimizer.m` evaluate A-arm
  geometry with `UseTieRod=false`, then save toe/steer-related K&C values.
- The front and rear A-arm optimizers save zero steering/tie-rod hardpoints in
  `build_corner_kc()`.
- A-arm chassis fore/aft X coordinates are fixed at `+/-0.198 m`, so side-view
  anti geometry and caster/anti tradeoffs are under-parameterized.
- Objective costs use hand-tuned weights across mixed units instead of
  tolerance-normalized residuals.
- `evaluate_double_wishbone_3d.m` calls `fsolve` without checking `exitflag`,
  residual norm, or continuation from the previous travel point.
- Optimizers write back to the same artifact path by default through
  `save_suspension_geometry_artifact()`.
- POU has started to fail fast on missing or zero tie-rod hardpoints, but this
  protects only one downstream consumer; the inverse-design stage can still
  publish misleading proxy K&C data.

Physics sanity check:

- Double-wishbone suspension with rack-and-pinion steering is a coupled
  closed-chain kinematic problem. Bump steer and Ackermann cannot be trusted
  when the tie rod/rack is absent from the solved constraints.
- Roll center, jacking, and anti metrics are strongly geometry- and force-path
  dependent. Side-view IC proxy values are useful diagnostics but should not be
  treated as final anti-dive/anti-squat design values without longitudinal
  force distribution, braking/drive torque paths, and CG geometry.
- Numerical kinematic solvers should report constraint residuals and reject
  non-converged poses; otherwise local optimizers can reward nonphysical
  branches.

## Reliability gates

Every generated suspension artifact should carry a quality verdict:

- `high`: measured or explicit design targets, all hardpoints complete, tie rod
  included where relevant, constraints satisfied, solver residuals below
  threshold, packaging checks passed, closed-loop regression passed.
- `medium`: explicit design targets and complete hardpoints, but some proxy
  metrics remain labelled and excluded from final-design claims.
- `low`: any critical fallback target, missing tie rod for front toe/bump steer,
  proxy anti/compliance used as optimization target, or unverified packaging.
- `invalid`: zero dummy hardpoints, failed fsolve evaluations above threshold,
  infeasible packaging, direct overwrite attempted without explicit approval, or
  failed artifact round-trip.

Until the Phase 1 gates are implemented, A-arm inverse-design outputs should be
labelled "initial hardpoint surrogate, not final K&C design."

## Phase 1: Stop publishing misleading outputs

Priority: P0

Goal: prevent provisional or physically incomplete outputs from being mistaken
for trusted K&C.

TODO:

- Add target-source counting in `build_inverse_targets.m`: count measured,
  artifact, fallback, and proxy sources; expose `targets.quality`.
- Add `is_trusted_target(field)` or equivalent utility. Fallback/proxy fields
  must be excluded from hard pass/fail achievement scoring unless explicitly
  allowed for diagnostics.
- In `main_a_arm_optimizer.m`, do not save `toe_vs_travel`, `toe_vs_steer`, or
  `roll_steer_gain` as trusted K&C when `UseTieRod=false`.
- For rear, either omit steering-specific fields or store them under
  `proxy_kinematics` with `confidence='low'`.
- Remove zero dummy `steering_arm_outer`, `tie_rod_inner`, and `tie_rod_outer`
  from A-arm outputs. Missing steering hardpoints must be absent/null plus a
  quality violation, never `[0, 0, 0]`.
- Add artifact quality metadata:
  `target_source_counts`, `fallback_fields`, `proxy_fields`,
  `constraint_violations`, `solver_failure_rate`, `confidence`,
  `recommendation`, and `do_not_use_for_final_design` when applicable.
- Change default optimizer output to a new artifact file in `output/runtime/`
  or `output/suspension/`; require an explicit option for in-place overwrite.
- Add a fail-fast guard in `save_suspension_geometry_artifact.m` for zero dummy
  hardpoints and low-confidence trusted K&C fields.

Acceptance checks:

- A front A-arm run without steering hardpoints does not publish trusted bump
  steer or toe curve data.
- A fallback-heavy target set produces `confidence='low'`.
- Existing input artifact is not modified unless an explicit overwrite flag is
  passed.
- POU and frame resolvers can distinguish complete hardpoints from surrogate
  A-arm-only geometry.

## Phase 2: Make the objective physically interpretable

Priority: P0/P1

Goal: replace hand-weighted mixed-unit costs with tolerance-normalized residuals
and separate feasibility constraints from design objectives.

TODO:

- Define a K&C tolerance table, for example:
  roll center height `0.010 m`, KPI `0.5 deg`, caster `0.5 deg`, scrub
  `0.005 m`, mechanical trail `0.005 m`, camber gain `0.1 rad/m`, toe gradient
  project-specific after steering integration.
- Implement normalized residual cost:
  `cost += ((achieved - target) / tolerance)^2`.
- Scale or disable residuals based on target source:
  measured/design targets = full weight, artifact-derived = normal weight,
  fallback = diagnostic only or very low weight, proxy = diagnostic unless
  explicitly requested.
- Move hard feasibility checks from penalty-only logic into `nonlcon` where
  possible:
  kingpin Z separation, upright radius, arm clearance, rod-end angle, wheel
  envelope, brake envelope, frame hardpoint volume, and minimum arm length.
- Keep penalties only for soft preferences, not safety/packaging feasibility.
- Produce a target-achievement report with normalized residuals and worst
  offenders.

Acceptance checks:

- Cost terms are unitless and explainable.
- A target exactly at its tolerance contributes about `1` to the scalar cost.
- Infeasible geometry is rejected as infeasible, not merely discouraged by an
  arbitrary penalty coefficient.

## Phase 3: Fix kinematic solve robustness

Priority: P0/P1

Goal: stop the optimizer from rewarding non-converged or branch-jumped pose
solutions.

TODO:

- Capture `[x, fval, exitflag, output]` from every `fsolve` call in
  `evaluate_double_wishbone_3d.m`.
- Add residual norm thresholds for travel and tie-rod constraints.
- Return `metrics.solver_status` containing per-sample exit flags, residuals,
  iterations, and failure count.
- Use continuation: solve travel samples in ordered sequence and seed each
  sample from the previous accepted pose.
- Penalize or reject pose branch jumps using continuity limits on arm angles,
  kingpin motion, toe, and camber.
- Add deterministic seed and solver settings to artifact metadata.

Acceptance checks:

- Non-converged travel samples make the geometry infeasible.
- Travel sweeps are continuous across bump/rebound samples.
- Optimizer reports the fsolve failure rate in saved quality metadata.

## Phase 4: Integrate front steering into inverse design

Priority: P1

Goal: make front K&C include the actual five-link steering/suspension system.

TODO:

- Extend front design variables to include steering arm outer, tie-rod inner,
  tie-rod outer, or rack position parameterization.
- Evaluate front travel with `UseTieRod=true` when toe/bump steer is part of
  the objective.
- Add targets for bump steer over travel, Ackermann error over steer angle,
  steering arm moment arm, rack packaging, tie-rod length, and rod-end angular
  limits.
- Replace `main_steering_optimizer.m`'s simple bump proxy with the same
  `evaluate_double_wishbone_3d.m` closed-chain travel solve used by the A-arm
  stage.
- Save steering achievement and A-arm achievement as one consistent front
  design result, or explicitly version them as staged but re-evaluate final K&C
  after both stages.

Acceptance checks:

- Front `toe_vs_travel` is produced only from a tie-rod-constrained solve.
- Ackermann target and bump steer target are both reported across steer/travel
  samples.
- POU can consume the generated tie-rod hardpoints without fallback or manual
  patching.

## Phase 5: Increase hardpoint design freedom safely

Priority: P1/P2

Goal: allow anti/caster/side-view design to be optimized without exploding the
search space at once.

TODO:

- Stage 5A: keep current 10 variables but add separate fore/aft chassis pivot Z
  deltas and X offsets.
- Stage 5B: expose `U_F_x`, `U_R_x`, `L_F_x`, `L_R_x` and fore/aft Z
  independently.
- Stage 5C: support full 3D hardpoint optimization for chassis pivots and
  upright points.
- Add design-space regularizers for manufacturable A-arm length, pickup spacing,
  bracket plane, and arm crossing.
- Store design-mode metadata: `symmetric`, `asymmetric_allowed`, or
  `measured_left_right`.

Acceptance checks:

- Anti/caster objectives are not active unless corresponding side-view degrees
  of freedom are available.
- Symmetric mirror results pass sign convention tests; asymmetric mode can
  preserve left/right differences.

## Phase 6: Add packaging, manufacturing, and rule feasibility

Priority: P1/P2

Goal: reject geometry that is mathematically good but impossible to build.

TODO:

- Define reusable suspension envelopes:
  wheel/rim barrel, tire inner surface, brake rotor/caliper, hub/upright volume,
  frame hardpoint allowed volumes, driver/pedal exclusion, and tool access.
- Reuse the POU `validate_packaging.m` and frame `checkExclusionZones.m`
  concepts, but make them suspension-hardpoint aware.
- Add rod-end angular limit, minimum thread engagement/arm length, bolt access
  clearance, bracket plate thickness, and weld access checks.
- Add FSAE rule-linked checks where suspension and frame attachment constraints
  are relevant.
- Save feasibility margins, not just pass/fail.

Acceptance checks:

- Geometry with an upright point inside the wheel/brake exclusion zone fails.
- Geometry with rod-end angle beyond the configured limit fails.
- Frame optimizer receives hardpoints with allowed-volume margins.

## Phase 7: Improve K&C metric fidelity

Priority: P1/P2

Goal: evaluate the conditions that actually matter for FSAE handling.

TODO:

- Increase travel samples from three points to a configurable sweep, for
  example `[-40, -20, 0, 20, 40] mm`.
- Report camber and toe errors at samples, not only polynomial coefficients.
- Add roll + steer + bump combined cases, especially outside front tire camber
  in cornering.
- Separate static, bump, rebound, roll, and combined-state metrics.
- Label jacking, anti, and compliance as proxy until force-based and compliance
  models are upgraded.
- Replace compliance proxy with levels:
  Level 0 current A-arm axial proxy, Level 1 truss/beam pickup compliance,
  Level 2 frame/upright/FEM-coupled compliance.

Acceptance checks:

- Reports include sample-wise K&C curves and normalized errors.
- Proxy metrics are impossible to mistake for measured/final K&C values.

## Phase 8: Strengthen POU/frame/vehicle-dynamics integration

Priority: P2

Goal: make downstream consumers use the same physical assumptions as the inverse
design stage.

TODO:

- Add A-arm result to POU input completeness checks:
  hardpoints, tie rods, upright points, coordinate system, quality status.
- Add A-arm result to frame pickup feasibility checks:
  allowed volume, load path, bracket region, and suspension load-case mapping.
- Add motion-ratio consistency checks between A-arm/upright points and POU
  pushrod/rocker/damper points.
- Add post-steering re-evaluation of front K&C before POU and frame stages.
- Add closed-loop vehicle-dynamics regression:
  baseline vs optimized skidpad, acceleration, cornering, yaw response, roll
  angle, tire Fz distribution, tire utilization, and outside tire camber.

Acceptance checks:

- `matlab/run_full_pipeline.m` fails if a downstream stage receives a
  low-confidence or incomplete suspension artifact, unless diagnostic override
  is explicit.
- A suspension change that improves K&C but worsens vehicle dynamics is flagged.

## Phase 9: Build a real validation test suite

Priority: P0/P1/P2

Goal: convert "runs without error" tests into physical and operational
reliability checks.

P0 tests:

- Coordinate system and sign tests for SAE J670e X forward, Y right, Z down.
- Roll center sign tests where negative Z means above ground when ground is
  `Z=0`.
- KPI, caster, scrub, mechanical trail sign tests for mirrored left/right
  geometry.
- Fallback target source counting and low-confidence warning tests.
- Zero dummy hardpoint rejection tests.
- Artifact output path tests proving default runs do not overwrite input.
- fsolve exitflag/residual failure tests.

P1 tests:

- Tie-rod constrained bump steer benchmark.
- Ackermann sign and magnitude benchmark.
- Travel sweep continuity test.
- Constraint violation reporting test.
- Artifact round-trip test comparing saved and reloaded hardpoints/K&C.
- Packaging envelope rejection tests.

P2 tests:

- Known-geometry benchmark against CAD or external K&C tool outputs.
- POU/frame integration fixture with complete hardpoints.
- Full pipeline laptime/vehicle-dynamics regression.
- Multi-seed optimizer repeatability and best-candidate report test.

## Prioritized TODO board

P0:

- Add target-source quality accounting and confidence metadata.
- Stop saving trusted toe/steer K&C without tie rods.
- Remove zero dummy steering hardpoints.
- Stop default in-place artifact overwrite.
- Check fsolve exit flags and residuals.
- Add core sign/fallback/zero-hardpoint tests.

P1:

- Tolerance-normalize objective function.
- Split feasibility into nonlinear constraints.
- Re-evaluate final front K&C after steering hardpoints exist.
- Add tie-rod/rack variables and Ackermann/bump-steer objectives.
- Add packaging envelope checks and margins.
- Add sample-wise camber/toe/roll combined metrics.

P2:

- Expand hardpoint variables to full 3D in stages.
- Upgrade anti/compliance from proxy to force/compliance-aware models.
- Add asymmetric measured-left/right support.
- Add Pareto/global search workflow.
- Add CAD/K&C-tool benchmark and closed-loop vehicle-dynamics validation.

## Implementation order

1. Implement Phase 1 and P0 tests before changing optimizer design freedom.
2. Implement Phase 3 solver robustness because all later metrics depend on it.
3. Implement Phase 2 normalized objective and explicit constraints.
4. Integrate front steering in Phase 4.
5. Add packaging and integration checks before trusting POU/frame outputs.
6. Only then expand variables and global/Pareto optimization.

## Web/technical references checked

- SAE Technical Paper 2013-01-0967, target-cascading optimization of rack and
  pinion steering coupled to double-wishbone suspension.
- SAE Technical Paper 2008-01-2949, bump steer and steering influence on
  kinematic roll and pitch axes.
- SAE Technical Paper 2006-01-3617, force-based roll centers and improved
  kinematic roll center; notes the steering tie rod as the missing fifth link.
- Public summary of double-wishbone suspension behavior: camber, caster, toe
  pattern, roll center, scrub radius, and anti effects depend on bushing and
  ball-joint locations.
- Race Software suspension KPI notes for anti-dive/anti-lift/anti-squat, which
  report anti properties in terms tied to vertical force per longitudinal force,
  supporting the conclusion that current side-view anti values are proxy-only.

