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
