# Frame V2 Weight / Speed Follow-up Investigation

Date: 2026-05-10

This note records the follow-up investigation after the 15000-evaluation V2 MATLAB GA run, plus the first corrective implementation and experiment results from the same day.

## Inputs

- Artifact: `output/runtime/frame_v2_ga_15000/frame_v2_ga_15000_vehicle_artifact.json`
- Generated investigation report: `output/runtime/frame_v2_investigation/frame_v2_bottleneck_investigation.md`
- Investigation helper: `frame_optimize/examples/investigate_frame_v2_bottlenecks.m`

## Corrective Implementation Applied

Implemented after the first investigation:

- Added native `SolverBackend='mex'` path to the canonical optimizer and skipped FEM server/Docker startup when a MEX solver is available.
- Added FEM lifecycle modes for TCP usage: `managed_once`, `reuse_if_alive`, and `external`.
- Cached front/rear suspension hardpoint parsing inside the MATLAB loaders and silenced V2 per-evaluation loader output.
- Split heavy/ambiguous rear and support groups into `front_susp_mount`, `diff_mount`, `rear_box`, and `secondary_brace`.
- Reduced the mandatory rear box to a minimal trapezoid-to-rear-plane structure by default, and made dense rear optional edges opt-in.
- Seeded MATLAB `ga` with a feasible-neighborhood initial population instead of only one feasible initial row plus broad random samples.
- Tightened V2 schema ranges for Main Hoop Brace attachment/end geometry so random candidates are less likely to violate known hard constraints.
- For MATLAB `ga`, changed rejected cheap-check/exclusion candidates from `Inf` to a large finite penalty while still skipping FEM.
- Rebalanced V2 objective defaults: frame-only CG hard limit relaxed to 0.55 m, mass and stiffness penalties strengthened, sparsity penalty scaled down from an accidental hard-constraint regime.

## Current Experiment Snapshot

Best stabilized short run so far:

- Run: `output/runtime/frame_v2_ga_300_mex_final`
- Budget: 309 actual function evaluations, MATLAB `ga`, torsion-only inner loop, final full-case re-evaluation
- Runtime: 61.2 s, 0.1980 s/eval
- Final finite score rate: 100%; true valid individual rate: 62.5%
- Final frame: 65 nodes, 97 elements, compliance pass
- Mass: 45.06 kg
- Torsional stiffness: 4909.0 Nm/deg
- Total tube length: 40.62 m
- Native MEX torsion median on saved final frame: 0.001432 s/eval

This confirms the loop is no longer Docker-bound, and MATLAB `ga` no longer collapses into `Inf` population statistics. It also confirms the 25 kg target is not reachable inside the current 65-node / 97-element topology family without deeper topology reduction or a different material/regulation assumption.

## 2026-05-11 Sparse Topology Follow-up

The next lower-bound experiment added `TopologyMode='regulation_minimal'` to `frame_optimize/examples/experiment_frame_v2_mass_cap_stiffness.m`.

| Topology | Nodes | Elements | Lightest mass kg | K torsion Nm/deg | Interpretation |
| --- | ---: | ---: | ---: | ---: | --- |
| Saved fixed topology | 65 | 97 | 31.03 | 3479 | Pipe assignment alone cannot reach 28 kg |
| Regulation-minimum + diff mounts | 61 | 82 | 27.23 | 3357 | 28 kg feasible if non-required rear/free tubes are removed |
| Regulation-minimum + suspension only | 57 | 74 | 24.28 | 3357 | Useful lower bound, but no diff reaction case |

This reverses the strongest version of the earlier mass conclusion: the current 97-element topology is infeasible under 28 kg, but the current geometric package is not obviously infeasible once non-regulation tubes are removed. The limiting risk is now regulation fidelity, especially the V2 Upper Side Impact node model.

## Findings

### 1. Weight Is Dominated By Rear / Support Tubes

The final frame is 66.96 kg against a 25 kg target. Recomputed mass from the saved nodes, connectivity, group names, selected catalog pipes, and material densities matches the artifact mass.

Mass by group:

| Group | Pipe | Elements | Length m | Mass kg | Share |
| --- | --- | ---: | ---: | ---: | ---: |
| secondary_brace | STL-R-31.8x2.41 | 23 | 10.46 | 18.27 | 27.3% |
| fbh_support | STL-SQ-25.4x1.65 | 22 | 7.99 | 9.84 | 14.7% |
| component_protection | STL-R-31.8x2.41 | 24 | 5.62 | 9.82 | 14.7% |
| side_impact | STL-R-28.6x2.11 | 14 | 6.26 | 8.62 | 12.9% |
| mh_brace_support | STL-SQ-25.4x1.65 | 8 | 4.49 | 5.53 | 8.3% |

The design uses 44.95 m of total tube length, only 0.05 m below the current 45 m sparsity limit. This means the optimizer found a feasible high-stiffness frame by using almost the full allowed tube budget.

The heaviest issue is not only pipe size. The topology generator creates many mandatory and rescue edges around FBH support, component protection, and rear secondary bracing. Even if pipe sizes improve, the candidate graph currently encourages too many members.

### 2. The Scalar Objective Still Tolerates Heavy Feasible Frames

The final objective is 118.02:

- stiffness penalty: 0.00
- mass target penalty: 70.44
- mass term contribution: 46.32
- CG term contribution: 1.26
- compliance / geometry / sparsity penalties: 0.00

This objective sees the 66.96 kg design as acceptable because it is feasible, passes CG, passes regulation checks, avoids sparsity penalties, and exceeds the 7000 Nm/deg stiffness target. The 25 kg target is present but not strong enough to force topology simplification or smaller non-critical groups once stiffness is satisfied.

### 3. The FEM Kernel Itself Is Not The Main Speed Problem

The native MEX torsion solve on the saved final frame has:

- median: 0.002094 s/eval
- mean: 0.011442 s/eval, including first-call overhead
- path: `frame_optimize/mex/beam_fem_solver.mexmaca64`

Measured V2 run speeds:

- V2 MATLAB GA, torsion-only inner loop + final full re-evaluation: 0.0215 s/eval
- V2 full load-case objective loop: 0.2347 s/eval

The full V2 path is therefore slow mainly because each accepted individual can run five FEM cases through the TCP/Docker server, with JSON serialization, socket setup, verbose logging, geometry checks, compliance checks, and MATLAB overhead around each call.

### 4. Docker / Server Lifecycle Is Too Aggressive

The server manager always calls stale cleanup at start:

- `beam_fem_server_manager('start')`
- `start_server_only()`
- `cleanup_existing_server(ctx)`
- `send_shutdown_request(ctx.port)`
- `kill_processes(pids)`
- `cleanup_docker_processes(ctx)`

`cleanup_docker_processes(ctx)` runs `pkill -f beam_fem_server` inside the `fsae-sim` container and may stop containers publishing the FEM port. At the end of canonical optimization, `onCleanup` calls `beam_fem_solver_tcp_java('stop')`, which again stops or kills the server process.

This is robust for one-shot demos, but it is unpleasant for iterative optimization work. It also prevents a warm FEM server from being reused across repeated experiments.

### 5. Random GA Wastes Many Evaluations On Invalid Designs

The 15000-evaluation MATLAB GA log showed `Mean = Inf` for most generations. This indicates many population members are rejected by cheap checks, exclusion checks, or solver failures. That affects both quality and speed:

- invalid individuals consume decoding/topology/check overhead
- GA selection receives poor information for many candidates
- the search finds feasible heavy frames late, then spends little effort in a lightweight feasible basin

## Remaining Next Steps

### A. Weight / Topology

1. Continue group splitting beyond the first pass, especially separating side-impact mandatory members from optional braces and splitting MH brace support upper/lower paths.
2. Add an actual topology-pruning pass. The current best still has about 40.7 m total tube length, which is too high for a 25 kg target with steel-heavy regulation groups.
3. Make optional side/FBH/rear support members true topology variables rather than always-on generated members.
4. Penalize group mass directly, especially `side_impact`, `fbh_support`, `mh_brace_support`, and `rear_box`.
5. Revisit whether square tubes should be allowed in regulation-critical groups, because they can pass the current material/size checks but may be undesirable or ambiguous for FSAE primary structures.
6. Establish a lower-bound feasibility study: run the lightest compliant pipe assignment on the current mandatory topology to estimate the minimum possible mass before spending 15000+ evaluations.

### B. Speed

1. Add a profiler report for one objective evaluation, because MEX itself is about 1-2 ms but the full GA loop is still about 0.19 s/eval.
2. Keep full load cases for final validation, but use staged evaluation during optimization:
   - cheap geometry/regulation checks
   - torsion MEX
   - lateral / rollover only for feasible top candidates
   - full five-case validation for elites and final reports
3. Cache generated nodes/topology/load cases for identical or nearly identical geometry variables where only pipe indices change.
4. Silence or gate per-evaluation regulation-check output at the source, rather than relying only on `evalc` capture.
5. Track valid individual rate by generation and fail the run early if valid rate stays too low.

### C. Docker / Server Lifecycle

1. Keep `mex` as the default investigation backend when the native solver exists.
2. Use `reuse_if_alive` or `external` for TCP experiments that must use the FEM server.
3. Make `cleanup_existing_server(ctx)` optional and avoid `docker stop` unless the caller explicitly asks for a hard reset.
4. Add a separate `fem_server_reset()` helper for stale process cleanup.
5. Prefer local native MEX or local native server on macOS before Docker when binaries are available.

## Immediate Recommendation

Before another long GA run, do not only increase evaluation count. First run a feasibility and optimizer comparison with:

- current mandatory topology with lightest compliant pipes
- optional side/FBH/rear topology switches
- staged torsion-only MEX evaluation
- final full validation on the top 10-20 designs

This should tell us whether the 25 kg target is physically reachable in the current node/topology family. If all feasible designs stay above roughly 45-55 kg, the problem is not GA tuning; it is topology and package assumptions.
