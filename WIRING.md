# WIRING — LAP package hooks for files outside the package

Changes the LAP package (Issues #23–#30) needs in files it may not edit. Each item says
what to parse, what to call and with which signature. All runtime additions are O(1) or
O(log n) per step and are evaluated once per `step()` (outside the RK4 stages).

## 1. Road height per wheel (Issue #28) — `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`

Track side (done): `TrackPoint` has `z` [m, up positive], `bank` [rad, positive helps a
left/κ>0 turn], `grade` [rad, uphill positive], `mu_scale` [-]; CSV columns are mapped by
header name (`x,y,width,z,bank,grade,mu_scale`); `ASE_autocross_sim.csv` now has `z`.
`CurvatureBasedTrack::get_point(s)` interpolates them (O(log n)) and
`CurvatureBasedTrack::road_height_at(x, y)` returns z at a map point (O(n), use it only for
one-off queries).

Plant side (to add):

```cpp
// Vehicle14DOF_New, public:
using RoadSample = std::function<std::array<double, 4>(const State14DOF&)>; // z_road per wheel [m, SAE +down]
void set_road_sampler(RoadSample f) { road_sampler_ = std::move(f); }
// private:
RoadSample road_sampler_;
std::array<double, 4> z_road_ = {0, 0, 0, 0};
```

- In `step()`: before the RK4 stages, `if (road_sampler_) z_road_ = road_sampler_(state_);`
  and hold `z_road_` constant over the four stages.
- In `compute_derivatives()`: tire vertical deflection becomes
  `defl_i = (z_u[i] - z_u_equilibrium_[i]) - z_road_[i]` wherever the unsprung-to-ground
  deflection is formed (tire spring `k_tire`, damper `c_tire`).
- Sign: the track stores z up-positive; the plant is SAE Z-down, so the sampler returns
  `-(z_track - z_track_at_start)`.

`laptime_sim` will install the sampler (one Frenet projection per step, already computed by
the driver loop):

```cpp
vehicle->set_road_sampler([&](const State14DOF& s) {
    const auto p = track->get_point(frenet_s);                  // O(log n)
    std::array<double, 4> z{};
    for (int i = 0; i < 4; ++i) {                               // wheel offsets in body frame
        const double xi = wheel_x[i], yi = wheel_y[i];          // +x fwd, +y right (SAE)
        // plane through the centreline point: grade along x (uphill: front higher),
        // bank across y (positive bank lowers the left/κ>0 side → right wheels higher)
        const double z_up = p.z - z0 + xi * std::tan(p.grade) + yi * std::tan(p.bank);
        z[i] = -z_up;
    }
    return z;
});
```

## 2. Wind (Issue #28) — `vehicle_14dof_new.hpp` (+ `vehicle_params.hpp`)

`laptime_sim --wind wx,wy` parses a map-frame wind vector into
`SimulationConfig::wind_x/wind_y` and currently only logs it.

- Add `aero.wind_x_mps`, `aero.wind_y_mps` (map frame; JSON keys of the same name) to
  `VehicleParams` (default 0).
- In the aero force evaluation of `compute_derivatives()`: map → body wind with the
  yaw of the state (laptime_sim map frame: `map_x = state.y`, `map_y = state.x`,
  `map_heading = π/2 − psi`):
  ```cpp
  const double wxb =  std::cos(psi) * wind_state_x + std::sin(psi) * wind_state_y;
  const double wyb = -std::sin(psi) * wind_state_x + std::cos(psi) * wind_state_y;
  const double vax = state.vx - wxb, vay = state.vy - wyb;    // air-relative velocity
  const double airspeed = std::hypot(vax, vay);
  const double aero_yaw = std::atan2(vay, std::max(vax, 0.1));
  aero_map_.compute_forces(airspeed, rh_f_mm, rh_r_mm, roll, aero_yaw, drs_active_);
  ```
  where `wind_state_x = aero.wind_y_mps`, `wind_state_y = aero.wind_x_mps` (state x/y are
  map y/x). Also use `airspeed` instead of `state_.speed()` as `v_wind` in the tire thermal
  update.
- `laptime_sim`: `vehicle_params.aero.wind_x_mps = config.wind_x; ... wind_y` before the
  vehicle is constructed, and delete the "[wind] NOT yet applied" log.

## 3. Single friction margin (Issue #26) — `cpp/include/fsae/control/mpc_problem.hpp`

`MPCConfig::attack_speed_lateral_safety = 0.70` and `attack_speed_braking_safety = 0.78`
are still stacked factors. Replace both defaults with `laptime::kDefaultFrictionMargin`
(`cpp/include/fsae/laptime/ggv_map.hpp`). The same for `AttackSpeedPlannerConfig`
(`attack_speed_planner.hpp`, LAP-owned, left at 0.78 only because
`cpp/tests/test_mpc.cpp:393 AttackSpeedPlannerTest.HighCurvatureTrackIsLimitedByLateralCapacity`
asserts `< 16.0 m/s`; with 0.90 it plans 16.02 m/s — relax that bound to `< 16.5` with the
note "single margin 0.90 replaces stacked 0.78"). `laptime_sim` already stopped stacking
`attack_vehicle_lateral_safety_scale()` (kept only for that test).

## 4. Steady-state tables in drivers (Issue #27)

The tables (`GGVMap::steer_angle/beta_limit/yaw_rate_limit`) are consumed when a
`std::shared_ptr<const laptime::GGVMap>` is provided:

- `cpp/include/fsae/driver/lpv_lqr_driver.hpp` `make_safety_filter_config()`:
  `safety.steady_state = ggv_;` and in the `AnalyticSafetyObservation{...}` add
  `.yaw_rate = state.r` (new optional field, NaN = unused). The driver needs a
  `set_steady_state_table(std::shared_ptr<const laptime::GGVMap>)` setter that
  `laptime_sim` calls with `perf.ggv`.
- `cpp/include/fsae/driver/attack_driver.hpp` (around line 418): 
  `attack_speed_planner_config_.steady_state = <same shared_ptr>;`
- `cpp/src/fsae_rl_server.cpp`: pass the GGV (load with `GGVMap::load(path)` from a
  `--ggv` option) as the new last argument of `apply_rl_curvature_speed_limit(...)`.

## 5. Standing start (Issue #24)

`laptime_sim` now starts the acceleration event at `initial_speed = 0`. The plant's
low-speed handling (`v_N`) is left to the v_N-unification issue; the fast 6DOF backend
completes the event from rest (7.55 s with PID).
