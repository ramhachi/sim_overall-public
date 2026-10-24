# Real vehicle correlation blueprint

## Objective

Populate `real_vehicle_correlation_config.json` until the simulator no longer depends on guessed vehicle-package constants. The goal is not just lap-time agreement; it is response agreement in load transfer, transient yaw, steering build-up, longitudinal acceleration, and brake stability.

## Mandatory static measurements

| Block | Why it matters | Minimum test |
| --- | --- | --- |
| Mass properties | Controls every load-transfer term | Corner scales with driver + fuel state, CG height, yaw inertia estimate |
| Springs / motion ratios | Sets wheel rates and ride frequency | Spring rig + installation ratio check |
| Dampers | Governs transient pitch/roll and wheel-hop | Damper dyno front/rear, rebound and compression |
| Bump stops | Large influence near peak aero / curb events | Engagement gap and stiffness curve |
| K&C / steering | Toe, camber, steering gradients must not be guessed | Static alignment + K&C rig or hardpoint/K&C export |
| Chassis torsion | Frame twist model is now active | Torsion rig for `K_chassis`, damping estimate, twist inertia estimate |
| Tire package | Current runtime uses stiffness and relaxation proxies | Radius, pressure, `Cx`, `Cy`, vertical stiffness/damping, `Fz_nom`, `Lx`, `Ly`, trail proxy |
| Powertrain / diff | Acceleration event will overfit without this | Torque curve, gearing, driveline efficiency, LSD hardware |
| Brakes | Balance directly affects stopping and entry stability | Pressure capability, pad mu, piston area, effective radius, balance |
| Aero proxy | Even with sim-side aero maps, the real car still needs baseline loads | Coast-down drag estimate, ride-height operating window, aero balance proxy |

## Mandatory dynamic correlation tests

1. **75 m acceleration**
   - Validate launch, torque delivery, driveline efficiency, wheel-speed behavior.
2. **Skidpad CW + CCW**
   - Validate lateral acceleration ceiling, understeer gradient, steering balance, roll support.
3. **Constant-radius sweep**
   - Validate speed-dependent understeer / aero onset.
4. **Step steer**
   - Validate steering inertia/friction, yaw-rate rise time, overshoot, damping.
5. **Brake in straight line**
   - Validate brake balance, tire longitudinal stiffness proxy, pitch support.
6. **Lift-off / trail-brake entry**
   - Validate rear stability and differential decel locking behavior.
7. **Autocross lap with telemetry**
   - Final end-to-end check after the lower-level tests are aligned.

## Required telemetry channels

| Channel | Minimum rate | Purpose |
| --- | --- | --- |
| time | 100 Hz | Sync |
| GPS or local position | 20 Hz+ | Path / velocity |
| speed | 100 Hz | Longitudinal validation |
| longitudinal accel | 100 Hz | Launch / brake / drag |
| lateral accel | 100 Hz | Skidpad / transient correlation |
| yaw rate | 100 Hz | Transient yaw response |
| steering wheel angle | 100 Hz | Driver input |
| steering torque (preferred) | 100 Hz | Steering model calibration |
| throttle command | 100 Hz | Driver demand |
| brake pressure or command | 100 Hz | Brake mapping |
| wheel speeds FL/FR/RL/RR | 100 Hz | Slip / diff / launch |
| engine rpm | 100 Hz | Powertrain map correlation |
| gear | 50 Hz | Shift / total ratio |
| damper pots FL/FR/RL/RR | 100 Hz | Ride / roll / pitch / bump-stop engagement |
| tire temps / pressure (preferred) | 10 Hz+ | Tire state sanity |
| ride heights front/rear (preferred) | 100 Hz | Aero operating point |

## Acceptance philosophy

- Do **not** accept a setup only because lap time matches.
- First align:
  1. static mass / geometry,
  2. wheel rates / damping / bump stops,
  3. low-speed skidpad balance,
  4. acceleration / coast-down,
  5. transient yaw and steering response.
- Use autocross last as an integration check, not as the first calibration target.

## Parameters that should stay measurement-config-driven

- K&C hardpoints and compliance matrices,
- raw tire load-sensitivity parameters,
- SAT trail calibration,
- LSD clutch hardware internals,
- aero-map source files and reference-state metadata,
- driver-controller tuning.

These should be measured or selected as packages, then locked before GA is trusted for vehicle design decisions.
