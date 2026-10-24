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
5. Test outside package: `cpp/tests/test_bicycle_model.cpp:277` `AngleModeYawResponseMatches14DOFConvention` runs
   throttle 0 in the default gear 3 and now sees engine braking (≈−27 Nm wheel torque at 15 m/s), r differs by
   0.0203 vs tol 0.02. That test is a lateral contract at constant speed: set `input.gear = 0;` (neutral) there.

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
