# WIRING (AERO-PT package → vehicle_14dof_new.hpp / vehicle_params.hpp / tests outside package)

Changes the AERO-PT package could not make itself (file-set rule). Apply verbatim.

## #34 engine (`cpp/include/fsae/physics/powertrain/engine.hpp`)

1. `vehicle_params.hpp` `PowertrainParams`: replace the 5-point default with the 13-point curve
   ```cpp
   std::vector<double> engine_rpm_points = physics::powertrain::kCbr600rrRpm;      // #include engine.hpp
   std::vector<double> engine_torque_curve = physics::powertrain::kCbr600rrTorque;
   std::vector<double> engine_torque_closed_curve;               // [Nm] <0, same size as rpm points; empty = documented estimate
   std::vector<double> engine_throttle_points;                   // [0-1] ascending; empty = 1D blend
   std::vector<std::vector<double>> engine_torque_table;         // rows = rpm points, cols = throttle points
   double I_engine = 0.0;                                         // [kg m^2] crank-side rotating inertia (see 4.)
   ```
   Parse from the `powertrain` JSON block: `torque_closed_curve`, `throttle_points`, `torque_table`, `I_engine`.
2. `Vehicle14DOF_New::setup_powertrain()` after `engine_map_.set_curve(...)`:
   ```cpp
   if (!pt.engine_torque_closed_curve.empty()) engine_map_.set_closed_curve(pt.engine_torque_closed_curve);
   if (!pt.engine_throttle_points.empty()) engine_map_.set_throttle_table(pt.engine_throttle_points, pt.engine_torque_table);
   ```
   (both throw `std::invalid_argument` on shape mismatch — let it propagate.)
3. Engine torque is now negative at closed throttle (engine braking) and the rev limiter returns only
   the motoring torque above `rpm_points.back()`. No vehicle code change is needed for that; `lsd_.distribute_torque`
   already receives the signed torque so `lsd_ramp_decel` becomes live.
4. Optional (`I_engine`): reflected inertia on the driven axle `I_engine * total_ratio^2 / 2` per rear wheel added to `Iw`
   in the wheel-spin equation (`dxdt(20+i)` for RL/RR). Not required for #34 acceptance.
5. **Currently failing test outside package** (only ctest failure on this branch):
   `cpp/tests/test_bicycle_model.cpp` `BicyclePreviewContractTest.AngleModeYawResponseMatches14DOFConvention`
   (r: preview −0.4129 vs plant −0.3926, tol 0.02; baseline passed). Cause: #35 LSD preload now acts at coast
   (throttle 0) and resists the rear-wheel speed difference → plant understeers vs the open-diff bicycle preview
   (neutral gear makes it worse, −0.343, so it is the diff, not engine braking). The test is a linear-bicycle contract,
   so isolate the diff there — verified locally to pass:
   ```cpp
   vehicle_params.aero.CD_A = 0.0;
   vehicle_params.powertrain.lsd_preload = 0.0;   // add: bicycle preview assumes an open differential
   ```

## #35 LSD (`cpp/include/fsae/physics/powertrain/lsd.hpp`)

1. New `SalisburyLSD::Params::omega_smooth` [rad/s] (default 0.1; RK4 @1 ms stability bound with Iw=0.5 is ≥0.094).
   Optional: add `lsd_omega_smooth` to `PowertrainParams` + JSON and copy it in `setup_powertrain()`.
   If `Iw` is lowered or `T_lock` > ~130 Nm, raise ω_s so that `T_lock·dt/(Iw·ω_s) < 2.78`.
2. `cpp/src/test_lsd_14dof.cpp` (CORE-A): bias is now `T_lock·tanh(Δω/ω_s)` without the `|T_in|/2` cap, preload acts at
   coast, and with #34 engine braking the decel ramp (`lsd_ramp_decel`) is selected under closed throttle. Expectations
   that assumed "no LSD effect at zero torque" or an accel-ramp-only result need updating; #34 acceptance wants a case
   showing `lsd_ramp_decel` changes the result (e.g. closed-throttle turn-in, compare 45° vs 60°).
3. No locking constraint was added (wheels still integrate independently); a real static lock would need
   reducing RL/RR spin to one DOF while |T_bias| < T_lock — not done (cost/complexity), marked `ponytail:` in lsd.hpp.

## #31 / #32 / #33 aero (`aero_map_4d.hpp`, `drs_system.hpp`)

Scope (user decision): the **target-aero analytic map is the production path**; a measured map JSON is optional and
overrides the targets. Do **not** throw or fail-fast when `aero_map_file` is empty or fails to load.

1. `vehicle_params.hpp` `AeroParams`: add the explicit design targets (all optional in JSON `aero` block, defaults =
   current behaviour) and parse them:
   ```cpp
   // synthetic-map sensitivities (design targets, not measured)
   double rh_sens_front = 0.015;    // CL rel. gain per mm front RH below nominal [1/mm]
   double rh_sens_rear  = 0.010;    // [1/mm]
   double roll_cl_loss  = 3.0;      // [1/rad]
   double yaw_cl_freq   = 2.0;      // CL ∝ cos(k·yaw)
   double yaw_cd_quad   = 2.0;      // CD ∝ 1 + k·yaw² [1/rad²]
   double yaw_cy_gain   = 0.5;      // CY = k·sin(2·yaw)
   double cop_sens_front = -0.003;  // CoP_x shift per mm front RH [m/mm]
   double cop_sens_rear  =  0.002;  // [m/mm]
   // DRS-on targets
   double drs_cl_ratio = 0.54;
   double drs_cd_ratio = 0.74;
   double drs_cop_shift = std::numeric_limits<double>::quiet_NaN(); // [m]; NaN → rear-wing rule CoP/ratio
   double drs_flap_tau = 0.0;       // [s], 0 = binary
   ```
   (`CL_A`, `CD_A`, `aero_balance`, `h_aero` already exist and stay the primary targets.)
2. `Vehicle14DOF_New::setup_aero_map()` — set geometry + targets **before** `set_coefficient_defaults(...)`
   (that call regenerates the map from `params()`):
   ```cpp
   auto& ap = aero_map_.params();
   ap.L_ref = params_.mass.a + params_.mass.b;   // wheelbase; CoP_x = aero_balance * L_ref
   ap.x_cg  = params_.mass.b;                     // CG from rear axle
   ap.h_cg  = params_.mass.h_cg;
   ap.h_aero = params_.aero.h_aero;               // drag/side force line → My += −Fx·(h_aero−h_cg), Mx += Fy·(h_aero−h_cg)
   ap.rh_f_nominal = params_.aero.nominal_front_ride_height_mm;
   ap.rh_r_nominal = params_.aero.nominal_rear_ride_height_mm;
   ap.dCL_drh_f = params_.aero.rh_sens_front;  ap.dCL_drh_r = params_.aero.rh_sens_rear;
   ap.roll_CL_loss = params_.aero.roll_cl_loss; ap.yaw_CL_freq = params_.aero.yaw_cl_freq;
   ap.yaw_CD_quad = params_.aero.yaw_cd_quad;   ap.yaw_CY_gain = params_.aero.yaw_cy_gain;
   ap.dCoP_drh_f = params_.aero.cop_sens_front; ap.dCoP_drh_r = params_.aero.cop_sens_rear;
   ap.drs_CL_ratio = params_.aero.drs_cl_ratio; ap.drs_CD_ratio = params_.aero.drs_cd_ratio;
   ap.drs_CoP_x_shift = params_.aero.drs_cop_shift;
   ```
   Do the same in `main_laptime_sim.cpp` for `perf.aero_map` (only Fx/Fz used there; balance/L_ref matter for Fz split).
   The moments returned by `compute_forces` are already **about the CG** (h_aero arm and, for direct-load maps with a
   `reference_point`, `M_cg = M_ref + (r_ref − r_cg)×F` are applied inside). Keep adding `Mx/My/Mz` as today; do **not**
   add a separate `−Fx·h_aero` term (it would double count). `AeroMap4D::transfer_moments(f, dx, dy, dz)` is public if
   another caller needs the transfer.
3. Load-failure handling (replace lines ~787-792):
   ```cpp
   if (!params_.aero.aero_map_file.empty() && !aero_map_.load_from_json(params_.aero.aero_map_file)) {
       // load_from_json already logged at info level with aero_map_.last_error(); target-aero map stays active
   }
   ```
   No throw (also when the file is empty). Same in `main_laptime_sim.cpp:1109` (drop the cerr warning or keep as info).
4. Out-of-domain diagnostics: `aero_result.out_of_domain` (bitmask bit0 RHf, bit1 RHr, bit2 roll, bit3 yaw) per call and
   `aero_map_.out_of_domain_count()` cumulative — expose in the runtime diagnostics/CSV next to ride heights.
5. DRS (`vehicle_14dof_new.hpp:165`): pass body lateral acceleration from the previous derivative evaluation instead of
   vx·r (old overload is `[[deprecated]]`, produces the warnings seen in the build):
   ```cpp
   // member: double last_ay_body_ = 0.0;   set in compute_derivatives (when update_cache is true) from
   //   ay_body = Fy_total / m  (+ g_body.y() if a lateral-g sensor convention is wanted; use Fy_total/m for tyre/aero load)
   drs_active_ = drs_system_.update(input.drs_button, state_.speed(), last_ay_body_, drs_steering_angle, dt);
   drs_flap_ = drs_system_.flap_position();   // new member, 0..1
   ```
   and at line ~1505 pass `drs_flap_` (double) instead of `drs_active_` to `compute_forces` (bool still compiles, = binary).
   `DRSConfig::flap_tau = params_.aero.drs_flap_tau` when constructing `drs_system_`.
   `DRSConfig::cl_ratio_active / cd_ratio_active` were deleted (were unused).
6. Ride height from inertial z (`vehicle_14dof_new.hpp:1484-1495`), includes tyre deflection that suspension travel misses:
   ```cpp
   // z down-positive, inertial; pitch θ nose-up positive; axle offsets a (front), b (rear) from CG
   const double z_front = state.z - params_.mass.a * std::sin(state.theta);
   const double z_rear  = state.z + params_.mass.b * std::sin(state.theta);
   const double z_eq_front = z_equilibrium_ - params_.mass.a * std::sin(theta_equilibrium_);  // or z_equilibrium_ if θ_eq=0
   const double z_eq_rear  = z_equilibrium_ + params_.mass.b * std::sin(theta_equilibrium_);
   double rh_front = params_.aero.nominal_front_ride_height_mm - 1000.0 * (z_front - z_eq_front);
   double rh_rear  = params_.aero.nominal_rear_ride_height_mm  - 1000.0 * (z_rear  - z_eq_rear);
   ```
   (check the sign of θ in the state: SAE pitch nose-up positive moves the front axle up = smaller z.)
7. Regression note: `CoP_x` is now `aero_balance·L_ref` (was `aero_balance` used directly as metres from the rear axle,
   i.e. 0.45 m ≈ 28 % front instead of 45 %), and `h_aero−h_cg` adds a drag/side-force pitch/roll moment
   (default 0.35−0.28 = 0.07 m until wired). Synthetic CoP-vs-RH sensitivity sign fixed (lower front → CoP forward).
   No existing ctest expectation changed.
