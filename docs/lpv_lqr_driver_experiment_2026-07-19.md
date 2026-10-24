# Preview LPV-LQR driver: implementation and actuator-aware experiment

Date: 2026-07-19

## Outcome

The repository now has a lightweight vehicle-parameter-conditioned driver selected with
`--driver lpv_lqr`. Torque steering defaults to an actuator-aware six-state LPV-LQR and the
original four-state bicycle controller remains selectable with `--lpv-model bicycle4` for
paired comparisons. The 14DOF/PINN model remains the truth plant; it is not rolled out inside
the controller.

The first nominal autocross run completed in 79.782 s with zero track-limit excess. The
40 Hz outer control computation averaged 0.0111 ms over the full lap. In an isolated 5 s
comparison, LPV-LQR averaged 0.00308 ms per update and the existing sampling MPC averaged
1.992 ms. The previously measured Attack controller averaged 219.896 ms per update under
the same 5 s autocross setup.

The actuator-aware torque-mode run completed in 79.849 s, compared with 89.788 s for the
four-state torque-mode baseline from the same binary and configuration. It preserved zero
track-limit excess and eliminated steering saturation. This is a viable fast baseline, not
yet the final universal driver. Nominal lap time remains slower than the current no-residual
Attack reference of 72.608 s.

## Runtime architecture

```text
Velocity profile + local track geometry
  -> 40 Hz Preview LPV-LQR
       angle mode:  [e_y, e_psi, v_y - v_y_ref, r - r_ref]
       torque mode: above + [delta_actual - delta_ref, delta_rate_actual]
       schedule = longitudinal speed and VehicleParams-derived bicycle model
       output = target tire angle
  -> constant-time analytic safety projection
       steer angle/rate, track margin, sideslip/yaw envelope, finite fallback
  -> 1 kHz shared steering actuator controller in torque mode
       target tire angle + measured steering state + SAT -> driver torque
  -> 14DOF/PINN truth plant
```

The LQR uses standard map/SAE signs internally: lateral error, lateral velocity, yaw rate,
curvature, and steering are left/CCW positive. Only the driver boundary converts steering to
the repository's right-positive command convention.

For each speed knot, the continuous dynamic-bicycle error model is discretized using the
bilinear transform. The discrete Riccati equation is solved once during construction. Runtime
work is gain interpolation, a 4x1 or 6x1 feedback product, local preview queries, and analytic
clamps. The six-state gain table embeds a second-order approximation of the shared steering
actuator. Measured steering-wheel angle and rate are converted by `-1 / R_steer` into the
left-positive tire-angle states used by the LPV model.

The outer safety layer retains the 2.8 rad/s safe target-angle slew envelope. The 1 kHz inner
target follower uses 12 rad/s so the same 2.8 rad/s ramp is not applied twice. Non-finite
steering angle, steering rate, SAT, or time step produces a deterministic zero-torque,
throttle-cut, braking fallback.

The steady-state `v_y`, yaw rate, heading offset, and feedforward tire angle are solved from
the scheduled bicycle model rather than using only `wheelbase * curvature`. Below 3 m/s the
controller uses pure pursuit to avoid the dynamic bicycle singularity.

## Safety-layer scope

`analytic_safety_filter.hpp` is independent of the LQR control law. It currently provides:

- finite-input fallback;
- tire-angle and tire-rate projection;
- centerline recovery near a boundary;
- track-margin speed cap;
- sideslip and yaw-rate-error speed cap.

It is an operational governor, not a formal CBF or friction-circle guarantee. A later phase
can replace or supplement this layer with a tiny CBF-QP after solver/deadline behavior is
validated.

## Reproduction

Build and focused tests:

```bash
cmake --build build --target laptime_sim test_lpv_lqr_driver -j4
./build/cpp/test_lpv_lqr_driver --gtest_color=no
ctest --test-dir build -R 'LPVLQRDriver|AnalyticSafetyFilter' --output-on-failure
```

Nominal full lap:

```bash
/usr/bin/time -p ./build/cpp/laptime_sim \
  --track autocross \
  --driver lpv_lqr \
  --time 120 \
  --output /tmp/lpv_auto_final.csv \
  --output_json
```

Torque-steering vehicle:

```bash
./build/cpp/laptime_sim \
  --track autocross \
  --driver lpv_lqr \
  --lpv-model steering6 \
  --time 130 \
  --params_file matlab/parameter_presets/params_flexible_torque.json \
  --output /tmp/lpv_torque_final.csv \
  --output_json
```

Heavy vehicle perturbation:

```bash
./build/cpp/laptime_sim \
  --track autocross \
  --driver lpv_lqr \
  --lpv-model steering6 \
  --time 130 \
  --params-json '{"mass":{"m_total":338.4,"m_sprung":276.0,"m_unsprung":[14.4,14.4,16.8,16.8],"Ixx":38.4,"Iyy":114.0,"Izz":102.0,"Ixz":-3.6,"Iw":0.6},"suspension":{"K_chassis":2000,"C_chassis":5,"I_twist":1.5},"steering":{"torque_mode":true,"I_steer":0.02,"R_steer":4.5,"T_fric":0.3,"C_steer_torque":0.3,"T_driver_max":25}}' \
  --output /tmp/lpv_heavy_final.csv \
  --output_json
```

For causal comparison, run the same command with `--lpv-model bicycle4`. Experimental weight
scales are exposed as `--lpv-lateral-weight-scale` and `--lpv-heading-weight-scale`; both
default to 1.0 and must be finite and positive.

## Results

| Torque-mode case | Model | Lap | Max / mean lateral error | Heading RMS | SW error mean | Saturation | Outer mean |
|---|---|---:|---:|---:|---:|---:|---:|
| Nominal | bicycle4 | 89.788 s | 0.591 / 0.146 m | 4.445 deg | 0.332 rad | 0.459% | 0.0092 ms |
| Nominal | steering6 | 79.849 s | 0.528 / 0.197 m | 4.563 deg | 0.082 rad | 0% | 0.0108 ms |
| Steering inertia +50% | bicycle4 | 91.071 s | 0.430 / 0.138 m | 4.357 deg | 0.339 rad | 3.840% | 0.0227 ms |
| Steering inertia +50% | steering6 | 80.557 s | 0.495 / 0.192 m | 4.481 deg | 0.079 rad | 0% | 0.0165 ms |
| Mass/inertia +20% | bicycle4 | 93.948 s | 0.599 / 0.132 m | 4.396 deg | 0.420 rad | 1.769% | 0.0112 ms |
| Mass/inertia +20% | steering6 | 79.883 s | 0.570 / 0.204 m | 4.441 deg | 0.083 rad | 0% | 0.0159 ms |

All six perturbation runs completed with zero track-limit excess and no controller failure.
On the nominal paired run, the six-state model reduced lap time by 9.939 s (11.1%), mean
steering-wheel error by 75.2%, and maximum lateral error by 10.7%. Mean lateral error remains
35.0% above the slower four-state baseline, so this result is a strong actuator-model success
but not a claim that every tracking metric is already optimal.

The final actuator cost is
`Q6 = diag(18, 24, 0.35, 1.8, 0.5, 0.05), R = 2.2`. A global 18 or 20 m/s maximum-speed cap
did not improve lateral error. A prototype lateral-error speed cap also failed its paired
gate (79.763 s, 0.214 m mean lateral error) and was removed rather than retained as an
unvalidated safety feature.

The clockwise and counter-clockwise 30 m circle smoke tests both completed. Their lap times
were 16.290 s and 16.303 s in the pre-final tuning pass, with maximum lateral errors 0.062 m
and 0.056 m respectively.

## Known limitations and next experiments

1. `BicycleParams::from_vehicle_params()` presently derives cornering stiffness from the
   TMeasy parameterization, even when the truth plant uses PINN. Generalization claims require
   extracting the active PINN artifact's zero-slip lateral-force Jacobian, or identifying an
   effective stiffness from 14DOF response data.
2. The second-order steering approximation uses configured bandwidth and damping rather than
   values identified from one-step plant response. Log `delta_required`, `delta_target`,
   `delta_predicted`, and `delta_actual`, then identify one-step angle NRMSE and phase error
   before changing tire stiffness. Otherwise actuator and tire-model corrections are
   confounded.
3. The speed profile is regenerated from each vehicle. A proper robustness study must report
   both a frozen nominal profile (controller robustness) and a vehicle-specific profile
   (achievable performance).
4. The analytic safety layer uses center-of-gravity track margin. The rejected absolute
   lateral-error cap showed that a new guard must be validated against a frozen speed profile
   and should use planned-line error plus body-aware usable corridor, not centerline error
   alone. Formal vehicle-envelope
   validation should include body width, asymmetric track bounds, command age, and a
   combined-slip-aware braking fallback.
5. Only after the physics/controller baseline passes grouped vehicle/tire/track holdouts should
   an Attack-derived learned residual be added. Its output should be bounded corrections to
   tire-angle target and longitudinal acceleration, with the analytic safety layer retaining
   final authority.

Research basis for the later optimization/safety phases: [MPCC](https://arxiv.org/abs/1711.07300),
[real-time iteration](https://cdn.syscop.de/publications/Diehl2005c.pdf), and
[CBF-QP](https://arxiv.org/abs/1609.06408).
