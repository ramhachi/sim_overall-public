# WIRING: KC package (MATLAB) -> C++ loader/runtime

Producer: `suspension_kinematics_design/`, `matlab/steering_hardpoint_design/`,
`matlab/pou_hardpoint_design/`, `matlab/save_suspension_geometry_artifact.m`.
Consumer to change: `cpp/include/fsae/vehicle/vehicle_params.hpp` (parse) and
`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp` (use). MATLAB does not touch C++.

All tables below are plain JSON arrays. Units are SI, angles in rad, travel is
**compression-positive** wheel travel from static equilibrium (runtime
`susp_travel_from_eq` convention), knots strictly ascending. 2D tables are
row-major `values[i_row][j_col]` with rows = first knot vector listed.
Out-of-range queries: clamp to the end knots (no extrapolation).

Helper needed once (header-only, no deps), e.g. in `vehicle_params.hpp`:

```cpp
struct Table1D { std::vector<double> x, y; bool empty() const { return x.empty(); } double eval(double q) const; };      // clamp + linear
struct Table2D { std::vector<double> r, c; std::vector<double> v; /* r.size()*c.size(), row-major */ double eval(double qr, double qc) const; }; // clamp + bilinear
```

---

## #37 anti geometry (`params.suspension.kc.front|rear`)

New/changed keys (axle block):

```json
"anti_brake_pct": 0.42,                 // per-axle force fraction tan(theta)*L/h (NOT bias-weighted)
"anti_drive_pct": 0.0,
"anti_dive_pct": 0.23, "anti_lift_pct": 0.0, "anti_rise_pct": 0.0, "anti_squat_pct": 0.0, // classical, bias-weighted, report only
"anti_brake_vs_travel": [c0, c1, c2],   // fraction vs compression-positive travel [-, 1/m, 1/m^2]
"anti_drive_vs_travel": [c0, c1, c2],
"anti_model": "side_view_pivot_axes_ic"
```

Runtime: `compute_pitch_force_arm_for_wheel` already does
`h_cg*(1-clamp(anti,0,1))` per wheel with that wheel's own `Fx`; this is exactly
the per-axle fraction, so no formula change. Optional upgrade (cheap, no solver):
replace the scalar with `eval_quadratic(anti_*_vs_travel, axle_travel)` where
`axle_travel` is the mean of the two corner travels (same as RC height).
Values previously written were bias-weighted (`dive = geometry*bias`); old
artifacts therefore under-state anti by the bias factor.

Per-corner samples (if consumed as LUT): `kc.corners.X.sample_values.anti_brake`,
`.anti_drive` on knots `sample_values.travel_m` (see #40).

## #38 roll camber / roll steer double count

MATLAB now writes `kinematics.roll_camber_gain` / `roll_steer_gain` as the
**residual** after `camber_vs_travel(travel(phi))` / `toe_vs_travel(travel(phi))`
(both chassis-relative), so they are ~0 for a wishbone corner
(`kinematics.roll_gain_definition = "residual_after_travel_curve"`).

C++ changes:
1. `vehicle_14dof_new.hpp` `compute_camber_angle` / `compute_toe_angle`: keep
   `+ roll_*_gain * axle_roll` (now a residual, harmless), but if the runtime
   needs camber **to ground**, add `side * body_roll` explicitly
   (`side=+1` right, `-1` left for phi>0 compressing the right side); do not
   reintroduce the travel-induced term.
2. Pass `susp_travel_from_eq` (not `effective_susp_travel_from_eq`) to the K&C
   camber/toe evaluation (`vehicle_14dof_new.hpp:1252-1253`). Spring, damper,
   bump stop keep `effective_*`.
3. Old artifacts (no `roll_gain_definition` key) carry the double-counted total
   gain: when the key is absent, ignore `roll_camber_gain`/`roll_steer_gain`
   (set 0) and log once.

## #40 dense samples, fit gate, loaded contact patch

Per corner (`params.suspension.kc.corners.<FL|FR|RL|RR>`):

```json
"sample_values": {
  "travel_m":            [..],   // knots, compression-positive, ascending, +/-(bump_stop_gap+10mm) at <=2 mm
  "camber_rad":          [..],   // chassis-relative camber incl. static camber
  "toe_rad":             [..],   // only when a tie rod / toe link was evaluated
  "caster_rad": [..], "kpi_rad": [..],
  "track_change_m":      [..],   // contact-patch lateral motion, outboard positive (per corner = half-track change)
  "rc_height_m":         [..],   // axle RC height above ground at symmetric heave
  "jacking_force_per_fy":[..],   // same sign convention as kinematics.jacking_force_per_fy (proxy)
  "anti_brake": [..], "anti_drive": [..],          // per-axle force fractions (#37)
  "roll_rad": [..],                                  // knots, 14DOF roll convention, ascending, 0.5 deg
  "roll_camber_residual_rad": [..], "roll_toe_residual_rad": [..], "roll_center_height_m": [..],
  "steer_rad": [..], "steer_rack_m": [..], "steer_camber_rad": [..], "steer_toe_rad": [..]  // ~1 deg knots
},
"polyfit_quality": { "...max_error...": .., "tolerances": {"camber_rad":2e-3,"toe_rad":1e-3,"rc_m":5e-3}, "fit_ok": true }
```

Axle block adds `roll_center_height_per_travel_sq` [1/m] and
`roll_center_height_per_roll_sq` [m/rad^2]: RC fits are now quadratic (RC height
vs roll is even for a symmetric axle, so the old linear term was ~0 and the fit
error was tens of mm). Runtime upgrade in
`compute_roll_center_height_for_wheel`:
`h = h0 + c1*t + c2*t*t + r1*phi + r2*phi*phi` (`c2`,`r2` default 0 if absent).

Preferred runtime path (replaces the quadratics when present, linear interp,
no solver): `camber = Table1D{travel_m, camber_rad}.eval(susp_travel_from_eq)`,
same for `toe_rad`. Keep the quadratic as fallback when `sample_values` is
absent. Parse rule: arrays must be the same length as `travel_m`, else ignore
the table and log.

Export is refused in MATLAB when `fit_ok=false` unless
`run_options.allow_poor_fit=true`; such corners carry
`quality.verdict="provisional"` with reason `poor_polyfit` (see #39 for the
C++ verdict rule). `hardpoints.contact_patch` now uses the loaded radius
`R - Fz_static/k_tire` and the camber lateral shift; the wheel centre Z is at
`-R_loaded` (ground at Z=0).

## #41 compliance (Level-1 truss)

`kc.corners.X.compliance` keeps the existing 6-vectors (runtime unchanged in
shape) but every row is now populated:

```json
"compliance": {
  "toe_per_load": [6], "camber_per_load": [6], "caster_per_load": [6],
  "track_per_load": [6], "wheelbase_per_load": [6],   // columns Fx,Fy,Fz,Mx,My,Mz
  "model": "truss_level1" | "arm_axial_stiffness_proxy",
  "provisional": true, "scope": "model_default_stiffness" | "model_supplied_stiffness" | "proxy_not_validated",
  "basis": "forces at contact patch, moments as couples; SAE J670e",
  "stiffness": {...}, "assumptions": [...]
}
```

Sign conventions: toe = wheel yaw (nose right +, the same quantity the runtime
adds to `total_yaw`), camber = asin(spindle_z) (top outboard +), track = contact
patch outboard +, wheelbase = contact patch away from the other axle +.

C++ changes:
1. Loads vector passed to `compute_camber_angle/compute_toe_angle`
   (`vehicle_14dof_new.hpp:1246`) must be the wheel's contact-patch forces in
   the body (SAE) frame `{Fx, Fy, Fz, Mx, My, Mz}`; today only `Fz` is filled.
   Fill `Fx, Fy` (tire forces rotated by steer) and `Mz` (aligning moment); `Mx,
   My` may stay 0 (columns small).
2. Optionally consume `caster/track/wheelbase_per_load` (contact-patch shift
   changes scrub/trail and track width).
3. Production mode: `compliance.provisional==true` is expected until rig data
   exist; treat it as "apply but log once", not fail-fast (it is labelled per
   field, see #39, and is not part of `quality.verdict`).

## #42 rear toe link, front closed-link refresh

- Rear corners now carry `hardpoints.tie_rod_inner/outer` (= toe link),
  `kinematics.toe_vs_travel`, `kinematics.roll_steer_gain` and
  `sample_values.toe_rad`, with `evaluation_scope.tie_rod/toe/roll_steer =
  "evaluated"`, `steering = "not_applicable"`. The loader already parses these
  keys; the runtime toe path (`compute_toe_angle`) must not skip rear wheels.
- Front corners are rewritten as a whole by the steering stage
  (`source.script = matlab/steering_hardpoint_design/main_steering_optimizer.m`,
  `source.model = a_arm_plus_tie_rod_closed_link`): camber, roll gain, jacking,
  anti, compliance and samples are all tie-rod-constrained. A front corner with
  `source.model = a_arm_only_two_dof` is the pre-steering stage output and must
  not be used for toe (see #39 rule).
