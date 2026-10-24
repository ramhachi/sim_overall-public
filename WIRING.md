# WIRING (TIRE package) — changes needed outside the tire file set

Apply these in `cpp/include/fsae/vehicle/vehicle_params.hpp` and
`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`. The tire-side APIs already exist.

## #19 Tire thermal: JSON params + cached stage slip

1. `vehicle_params.hpp`
   - `#include <fsae/physics/tire/tire_thermal_integrator.hpp>`
   - In `struct TireParams` add `physics::tire::TireThermalParams thermal;`
     (namespace-qualify as `fsae::physics::tire::TireThermalParams` if needed).
   - In the `tire` JSON parse block (next to `ambient_temperature_c`):
     ```cpp
     if (t.contains("thermal")) {
         p.tire.thermal = physics::tire::TireThermalParams::from_json(t["thermal"]);
     }
     ```
     Keys = member names of `TireThermalParams` (m_tread, c_p, h_conv_base, A_tread, v_ref,
     h_road, pressure_kpa, A_contact, T_road, T_initial, m_bulk, c_p_bulk, G_surface_bulk,
     surface_heat_fraction). Absent keys keep defaults; `m_bulk` absent/0 = 1-node path.
     If `tire.pressure_kpa` (see #18) is present and `tire.thermal.pressure_kpa` is not,
     copy it into `p.tire.thermal.pressure_kpa` so there is one source of truth.

2. `vehicle_14dof_new.hpp` constructor (`Vehicle14DOF_New(const Params&)`, ~line 105):
   ```cpp
   tire_thermal_.set_params(params_.tire.thermal);
   tire_thermal_.reset();
   ```
   (or construct `tire_thermal_(params.tire.thermal)` in the init list).

3. Stage slip cache instead of recomputing with driver steer only (`step()` ~lines 205-243):
   - Add members `mutable std::array<double, 4> tire_vsx_{}, tire_vsy_{};`
   - In `compute_derivatives`, inside `if (update_cache) { ... }` (~line 1424) store the
     stage values already computed with full steer+toe+camber rotation (~line 1272):
     `tire_vsx_[i] = vsx; tire_vsy_[i] = vsy;` (hoist `vsx/vsy` out of the inner scope
     or cache them in per-wheel locals so they are visible at the update_cache block).
   - In `step()` replace the whole contact-velocity / `delta_driver` recomputation with:
     ```cpp
     for (int i = 0; i < 4; ++i) {
         tire_forces[i] = {tire_Fx_[i], tire_Fy_[i], 0.0, tire_Mz_[i]};
         slip_info[i] = {tire_vsx_[i], tire_vsy_[i], tire_Fz_[i]};
     }
     ```
     Passing `Fz` activates `A_contact = Fz / P`; a lifted wheel (Fz=0) then has no road
     conduction. (`Mx` is not cached by the vehicle today; it is not used by the heat
     balance, so 0 is fine.)
   - `compute_derivatives(state_, input, true)` is already called right before, so the
     caches hold the post-step values.

## #22 TMeasy Cy units

- `vehicle_params.hpp:216` `double Cy = 22000.0; // 横剛性 [N/rad]` is copied into
  `TMeasyParams::Cy` (`vehicle_14dof_new.hpp:707`), which is a carcass stiffness [N/m].
  Change to `double Cy = 44900.0; // 接地面横剛性 [N/m] = Cα/(2·Ly), Cα 実測 22436 N/rad`.
  This only affects TMeasy test builds (`FSAE_TEST_TIRE_MODEL_TYPE="tmeasy"`); expect
  regression numbers in TMeasy-based vehicle tests to shift (stiffer front/rear axle).
  `bicycle_model.hpp:100` already uses `2·Cy·Ly` as Cα, so it becomes consistent too.
- JSON presets with `"Cy": 22000` (`correlation/params_correlated_template.json`,
  `scratch/pinn_benchmark_params.json`) should become 44900 for the same reason.
- `vehicle_3dof/6dof/8dof*.hpp` hard-code `Cy = 22000` / `60000`; legacy models, leave
  unless their owner wants the fix (same formula).
