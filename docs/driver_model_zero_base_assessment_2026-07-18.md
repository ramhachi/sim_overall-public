# Driver Model Zero-base Assessment (2026-07-18)

## Executive summary

The repository does not contain one driver model. The operational `laptime_sim`
entry point exposes five driver modes (`pid`, `mpc`, `mpc_v2`, `attack`, `rl`),
while residual profiles and safety layers create additional evaluation contracts.
For fair comparison, the minimum useful contract matrix is:

1. PID baseline / teacher
2. legacy sampling MPC
3. MPC v2 configuration wrapper
4. generic attack driver without course residuals
5. attack driver with a fixed-course residual profile
6. direct RL actor with its hand-written safety/recovery layer

The strongest current full-PINN evidence is the generic attack driver's
`72.608 s` nominal ASE autocross lap. The completed CEM run found `72.232 s` on
the same course and plant, only `0.376 s` faster. Both remain more than six
seconds slower than the configured `66.0 s` target. The old residual evaluated
before that training was slower (`73.175 s`) than no residual.

Current generalization evidence is insufficient. The only current holdout result
is a single `+50%` steering-inertia case (`73.068 s`). There is no evidence yet
for unseen course geometry, combined plant shifts, tire-artifact shifts, initial
condition perturbations, or confidence intervals. A station-indexed attack
residual is a course setup artifact, not a general driver policy.

Runtime is also a first-order blocker. A saved full-PINN attack lap took
`704.4 s` wall time for `72.608 s` simulated time. Predictive-driver solve time
accounts for about `151 s` (`21.5%`), so optimizing the attack sampler alone
cannot solve the runtime problem; the full 14DOF/PINN integration and surrounding
lap loop dominate the remainder. The existing `fast` evaluator does not support
the target torque-steering contract.

## Scope and provenance

- Repository: `/Users/sota/projects/FomulaTMU/sim_overall`
- Branch: `codex/driver-model-foundation-fixes`
- HEAD: `f085c034d8fc0409bdb98cd463f1a993b65ea7b0`
- Latest driver-related commit date: 2026-06-20
- Assessment date: 2026-07-18
- Existing unrelated dirty files were not modified.
- Existing full-lap artifacts do not record a binary/build hash. They are treated
  as historical measurements at the stated HEAD family, not as cryptographic
  proof of the exact executable.
- The current CMake targets were checked/rebuilt before the short acceleration
  measurements below.

## Operational architecture

```text
laptime_sim CLI
  -> track + velocity profile + vehicle/tire parameters
  -> selected driver contract
  -> full evaluator: 1 ms 14DOF/PINN loop
       PID: control every 1 ms
       MPC / MPC v2 / attack: outer control every 25 ms,
                              steering-torque inner loop every 1 ms
       direct RL: actor every 20 ms, held between updates
  -> DriverInput (steering or torque, throttle, brake, gear, DRS)
  -> vehicle integration
  -> common CSV and JSON metrics
```

### Model roles and limitations

| Contract | Actual implementation | Intended role | Critical limitation |
|---|---|---|---|
| PID | Path/velocity PID plus many actuator- and stability-aware speed caps | fallback, teacher, diagnostic baseline | Current full-PINN nominal run leaves the track; heuristics can mask plant weakness by slowing down |
| legacy MPC | Deterministic 7 steering x 5 acceleration finite-horizon sampler using a simplified bicycle prediction | legacy model-based comparison | Prediction does not reproduce the full tire, flexible-body, and powertrain dynamics |
| MPC v2 | The legacy sampler with a different horizon, weights, actuator prediction and margin/progress settings | model-based candidate | It is not yet a distinct solver or model; current evidence is a configuration comparison |
| attack | 7 tracking primitives x 4 speed scales, course/phase plan, safety caps, line and steering-actuator prediction | current strongest generic controller | Large monolithic stack; course complexity raises solve cost sharply |
| attack residual | Four station-dependent corrections on top of attack | fixed-course performance correction | Absolute station and track length bind it to one course; not a general policy |
| direct RL | MATLAB SAC actor exported to C++, 45 observations, 2/3 actions | learned candidate | Evaluation includes a strong hand-written safety/recovery layer; actor ability is not isolated |

The Python attack-residual "RL" is a one-step black-box CEM search over a whole
course profile. It must not be confused with the online direct RL driver.

## Current performance evidence

### Lap time and robustness

| Case | Plant/tire | Result | Wall time | Interpretation |
|---|---|---:|---:|---|
| attack, no residual, nominal | flexible torque, full PINN | `72.608 s`, complete, no track excess, no MPC failures | `704.4 s` | strongest comparable nominal baseline |
| attack, old v5 residual, nominal | same | `73.175 s`, complete | `786.7 s` | `+0.567 s` worse than generic attack |
| attack, completed CEM best | same course and plant used for selection | `72.232 s`, complete | not separately timed | `-0.376 s` / `-0.52%`, but in-sample only |
| attack, no residual, steering inertia +50% | same except `I_steer=0.03` | `73.068 s`, complete | `802.9 s` | only current plant holdout; `+0.460 s` |
| attack, legacy corridor | nominal | `74.421 s`, complete | `812.8 s` | older longitudinal contract is slower |
| PID, nominal full PINN | nominal | off-track at `9.84 s`, 129.5 m | `113.9 s` | not a viable current full-PINN baseline |

The CEM run evaluated 72 candidates; all completed, and the best lap was
`72.232 s`. Selection and evaluation used the same ASE autocross and nominal
plant, so this is course optimization rather than validation.

Historical controller-comparison artifacts using the winged-flexible TMeasy
configuration show PID completing in `78.312 s`, legacy MPC leaving the track at
`11.908 s`, and MPC v2 leaving the track at `83.407 s`. Historical direct-RL
checkpoint records contain 44 evaluations, 15 completions, and a best completed
time of `143.926 s`; that lap also had nonzero track-limit excess. These results
are useful failure evidence but are not directly comparable to the later PINN
attack results because plant, tire, code date, and driver contract differ.

The `66.0 s` value is currently a configured target/reference. This assessment
did not find a manifest linking it to a hashed measured lap, track alignment,
vehicle configuration, and timing protocol. That calibration must be established
before treating the six-second gap as a validated sim-to-real error.

### Runtime

Fresh short full-PINN acceleration checks after ensuring the targets were built:

| Driver | Simulated result | Wall | Wall/sim | Predictive solve |
|---|---:|---:|---:|---:|
| PID | `9.899 s` complete | `14.66 s` | `1.48` | n/a |
| attack | `10.004 s` complete | `15.71 s` | `1.57` | 401 solves, mean `4.45 ms`, max `8.28 ms`, about `1.78 s` total |

On the full ASE autocross, the saved generic attack run reports 2,905 solves at
`52.05 ms` mean and `706.65 ms` maximum. Estimated solve time is `151.2 s`,
`21.5%` of total wall time. Thus about `78.5%` remains outside the predictive
solve, principally full 14DOF/PINN integration, track/metric work, and logging.
The per-solve cost also rises by more than an order of magnitude between the
simple acceleration event and the full course.

The existing performance plan was partially implemented as `FastLapVehicle`,
but fast evaluation deliberately rejects torque steering because it does not
model steering SAT feedback. Residual training and current generalization
sweeps therefore still invoke the expensive full evaluator.

## What is proven and what is not

Proven at the current evidence level:

- Common JSON/CSV health metrics exist.
- Generic attack can complete the nominal full-PINN ASE autocross.
- It can also complete one `+50%` steering-inertia holdout.
- Fixed-course CEM improved its own nominal selection case by `0.52%`.
- Unit-level PID, MPC, attack and RL-safety contracts are broadly covered.

Not proven:

- Generalization to unseen track geometry.
- Robustness to joint vehicle-parameter shifts.
- Robustness to tire compound, artifact, radius, thermal or wear differences.
- Robustness to lateral/heading/yaw/steering initial-state perturbations.
- Statistical completion confidence or tail performance.
- Competitive current MPC v2 or direct-RL performance under the same contract.
- That the `66 s` target is an apples-to-apples measured reference.
- Exact reproducibility of old results from git SHA alone.

Additional evaluation defects:

- `autocross` and `ASE_autocross` resolve to the same CSV and cannot be treated
  as train/test tracks.
- The Python driver-generalization preset defines 8 plant cases x 4 contracts,
  but stored current results cover only nominal subsets and one holdout.
- Its default `300 s` timeout is shorter than the observed `700-800 s` attack
  runtime and would time out valid full-PINN cases.
- The Python matrix omits legacy MPC, MPC v2 and direct RL; the MATLAB comparison
  and robustness runners use different matrices and output contracts.
- CTest currently runs the driver-capability test from `build/`, where the
  default relative PINN manifest path cannot be resolved. The same test passes
  from the repository root in `16.2 s`. Evaluation is therefore working-directory
  dependent.
- The quick capability test reloads the PINN model for each maneuver (13 loads
  were observed), adding avoidable startup cost.
- Current initial-condition control exposes initial speed but not lateral,
  heading, yaw-rate or steering-state perturbations.

## Recommended execution plan

### Phase 0: make measurements reproducible and comparable (P0)

1. Create one benchmark manifest and runner for all six driver contracts.
2. Record git SHA and dirty flag, compiler/build type, executable hash, command,
   host, track hash, full vehicle/tire hash, controller/profile hash, seed and
   timestamp for every case.
3. Add stage timers: initialization/tire load, velocity profile, plant step,
   track queries, driver outer solve, inner steering, metrics, CSV, total wall.
4. Add a `--no-trace`/summary-only measurement path and compare it with CSV
   output to quantify diagnostic overhead.
5. Fix asset resolution in CTest and make the default timeout fidelity-aware.
6. Rebuild Release and establish a locked nominal result for each runnable
   contract. Do not tune yet.

Exit gate: a repeated nominal run must reproduce dynamics metrics exactly (or
within declared numerical tolerances), and wall-time variation must be reported
separately from physics variation.

### Phase 1: build a fidelity ladder that supports torque steering (P0)

Use three explicit levels rather than calling one result "the simulator":

- L0 driver microbench: fixed/replayed states, no plant, for controller cost and
  safety-contract tests.
- L1 torque-aware fast evaluator: 3/6DOF or bicycle plant at 10-20 ms with
  steering-wheel angle/rate, torque/rate limits and an identified SAT proxy.
- L2 full 14DOF with TMeasy for screening correlation.
- L3 full 14DOF with deployed PINN for final validation.

Cache track geometry and PINN artifacts, reuse the vehicle/evaluator in-process,
and avoid one-process/one-model-load per case. Profile before changing numerical
methods. Any change to `dt`, integration scheme, tire evaluation or control rate
requires a convergence/correlation study against L3.

Initial performance gates:

- L1: below `1 s` wall per autocross lap.
- L2: at least `10x` faster than current L3.
- L3: first reach real-time or better while keeping lap/health deltas within
  predeclared tolerances.
- L1/L2 rankings must correlate with L3; raw speed without ranking fidelity is
  not an acceptable optimizer backend.

### Phase 2: locate and close the lap-time gap (P0/P1)

1. Validate the `66 s` reference: track coordinate/length, start-finish rule,
   vehicle/tire/aero configuration, driver and environmental conditions.
2. Compare generic attack, course plan and attainable velocity profile segment
   by segment. The nominal trace has 1,220 under-reference samples, and 56.8% of
   those have no throttle; per-row dominant costs are mainly tracking (`45.2%`)
   and speed (`41.0%`). Test longitudinal logic before another broad residual run.
3. Run controlled ablations for line plan, speed plan, boundary/heading caps,
   steering actuator prediction and fixed-course residual. Change one causal
   component per experiment.
4. Promote a change only when it improves clean completion first, then lap time,
   without worsening holdout tail metrics.

Do not spend more full-PINN CEM budget until Phase 1 supplies a correlated
screening evaluator and Phase 3 supplies fixed holdouts.

### Phase 3: evaluate real generalization (P1)

Freeze train/development/test manifests before tuning:

- Tracks: current ASE and selected synthetic geometries for development; at
  least three genuinely unseen autocross CSVs for final test. Mirror/reverse
  variants are useful stress tests but not sufficient independent tracks.
- Plant: move from one-at-a-time changes to physically constrained joint
  Sobol/LHS samples. Keep OAT cases only for diagnosis.
- Tires: hold out complete tire artifacts/compounds/radii, not only `mu` scalars.
- Initial conditions: speed `{0,3,6,9,12} m/s`, lateral offsets, heading errors,
  yaw-rate and steering-state perturbations.
- Controllers: run the exact same case manifest for all six contracts.

Use a lexicographic gate:

1. valid completion (`track excess=0`, solver failures=0)
2. completion rate and Wilson confidence lower bound
3. worst-case/CVaR lap ratio against the same-plant baseline
4. steering saturation/lag, twist, smoothness and safety intervention
5. runtime factor and p95/p99 solve time

Suggested first promotion criteria are OOD completion at least `95%` with a
`90%` or higher 95% confidence lower bound, nominal regression no worse than
`1%`, and for residuals at least `1%` median improvement with no degradation in
the worst 10% of cases.

### Phase 4: choose the driver architecture from evidence (P2)

- Keep PID as fallback/teacher/diagnostic, not the default optimization target.
- Treat current MPC v2 as a configuration baseline until a distinct prediction
  model/solver earns the v2 name through the shared gate.
- Keep station residuals explicitly labeled as course optimization.
- If cross-course learning is needed, replace absolute-`s` residuals with a
  state- and local-geometry-conditioned residual, trained with domain
  randomization and evaluated only on frozen holdouts.
- Report direct RL both before and after the safety layer so actor performance
  and recovery-controller performance are not conflated.

## First experiment batch

After Phase 0 instrumentation and the torque-aware screening path exist:

1. Compare zero residual and the current `72.232 s` CEM profile on the 8 existing
   plant cases (16 runs).
2. Add PID, MPC v2 and direct RL under the same exact manifest.
3. Run the controller set on synthetic circle, figure-eight, skidpad and
   straight-corner development tracks.
4. Only after those pass, run joint plant and initial-condition samples, then
   untouched course/tire holdouts.

Without a faster evaluator, the first 16 full-PINN attack runs alone are roughly
3.5 hours sequential. This is why reproducibility and the fidelity ladder precede
another large sweep.

## Verification performed during this assessment

- Release targets checked/rebuilt: `laptime_sim`, `driver_benchmark`, `fsae_tests`.
- 91 selected C++ driver tests: 90 passed; the capability test failed only under
  CTest's build-directory working directory due to the relative PINN path.
- The same capability test from repository root: passed in `16.2 s`.
- Python attack-sweep tests: 12 passed.
- Fresh full-PINN acceleration PID/attack runs: both completed with the metrics
  reported above.

## Primary evidence paths

- `cpp/src/main_laptime_sim.cpp`
- `cpp/include/fsae/driver/`
- `python/scripts/sweep_attack_driver.py`
- `python/scripts/train_attack_residual_rl.py`
- `python/fsae_sim/rl/attack_residual_env.py`
- `matlab/driver_generalization/`
- `matlab/rl_driver/`
- `docs/driver_control_generalization_plan.md`
- `docs/performance_lightweight_sim_plan_2026-06-15.md`
- `output/driver_generalization/nominal_residual_contract_20260619_155336/`
- `output/driver_generalization/holdout_steer_inertia_attack_no_residual_20260619_163930/`
- `output/runtime/driver_generalization/controller_comparison/20260610_132928/`
- `rl_results/attack_residual_cem_guard_zero_20260620_w2/`
- `output/runtime/rl_checkpoint_benchmark/`
