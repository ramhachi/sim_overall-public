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
