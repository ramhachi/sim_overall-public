# WIRING.md — CORE-A (14DOF runtime) → other packages

Changes other packages must apply so their outputs match the new 14DOF runtime semantics.

## K&C MATLAB package (`suspension_kinematics_design/`)

1. **#38 roll gains are residuals.** The runtime now applies
   `kc.kinematics.roll_camber_gain` / `roll_steer_gain` **only when**
   `camber_vs_travel` / `toe_vs_travel` are all-zero. When a travel polynomial is
   exported, re-define the gain as the residual
   `roll_camber_gain = d/dphi [ camber(phi) - camber_vs_travel(travel(phi)) ]`
   (≈ 0 for a pure double wishbone; non-zero only for chassis-side effects such as
   roll-induced track change). Today `evaluate_double_wishbone_3d.m:181-183,282-283`
   and `build_inverse_targets.m:95-97` export `-c1*halfTrack` (the full travel effect);
   that value is now ignored at runtime whenever `camber_vs_travel` is present, so
   either keep exporting it (harmless) or switch to the residual.
   Also: kinematic polynomials are evaluated on the wheel-to-body stroke
   (`susp_travel_from_eq`), NOT on stroke + POU jack displacement.

2. **#9 link forces.** The runtime now uses the full `h_cg` arm for tyre Fx/Fy and adds the
   roll-centre / anti share as a vertical link force in `F_susp`. Consequences for the exporter:
   - `kc.corners.*.kinematics.jacking_force_per_fy` must be the **signed per-corner**
     vertical-link-force-per-lateral-force (upward on body, SAE Fy>0 = right):
     `-sign(y_corner) * tan(theta_IC)`, i.e. FL positive / FR negative for an RC above ground.
     If it is 0 and `kc.<axle>.roll_center_height != 0` the runtime derives
     `-sign(y) * h_rc / (t/2)` itself (preferred until the LUT exists — keeps RC and jacking consistent).
   - `anti_brake_pct` / `anti_drive_pct` are now interpreted per wheel: link force
     `-/+ Fx_wheel * anti * h_cg / L` (front/rear). Total load transfer no longer depends on them.
   - Planned upgrade: export `RCz(heave, roll)`, `anti(travel)`, `tan(theta_IC)(travel, roll)` LUTs
     from the same sweep (schema not added yet).

## Anyone writing vehicle JSON (MATLAB pipeline, GA, `optimized_*.json`)

3. **#10 element-side damper/bump values.** With `suspension.mr_applies_to_dampers = true`
   (default) `c0`, Bouc-Wen, LuGre and `bump_stop_*` are **damper/element-side** values
   (wheel force = MR·F(MR·x, MR·v)). If an exporter wrote wheel-rate values, either convert
   (`c0_elem = c0_wheel / MR^2`, `gap_elem = MR * gap_wheel`) or set the flag to `false`.
   New optional keys: `bump_stop_gap_rebound`, `bump_stop_k_rebound` (default = bump side),
   `motion_ratio_table: {travel: [...], FL: [...], FR: [...], RL: [...], RR: [...]}`
   (travel = wheel stroke from equilibrium, + compression).
   `k_arb_f/r` are now scaled by `kc.<axle>.arb_motion_ratio^2` (default 1 → unchanged); an
   exporter that sets `arb_motion_ratio != 1` must write `k_arb_*` as bar-side stiffness.
4. Other new optional keys (all default to legacy behaviour except `tire.C_rr`):
   `brake.pedal_table`, `brake.pressure_table`, `brake.pressure_lag_s`, `brake.omega_smooth`;
   `powertrain.I_engine`, `powertrain.I_gearbox`, `powertrain.shift_time_s`;
   `mass.Iw` as 4-array; `tire.C_rr` (default 0.015 — replace with coast-down fit).
   `brake.balance` is now the front/rear master-cylinder pressure ratio (0.5 neutral; #36).
