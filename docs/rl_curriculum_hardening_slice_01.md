# RL curriculum hardening slice 01

## Why this slice was needed

The first later-phase continuation from `phase2o_circle_preview_always_on/Agent19.mat` proved that:

- `phase3_skidpad/agent.mat` preserved a deterministic circle pass and reached meaningful skidpad progress,
- `phase4_autocross/agent.mat` and `final_agent.mat` catastrophically forgot circle while still failing autocross/skidpad readiness,
- exporting the final agent directly to `matlab/rl_driver/rl_actor_export/` is unsafe because the last trained weights are not always the deployment candidate.

So the next slice hardens the training entrypoints before the next long run.

## Code changes

### 1. `trainWithCurriculum.m`

- added explicit `StopTrainingCriteria` / `StopTrainingValue` control
  - default is now `AverageReward = 1e9` so long runs do not stop early by accident
- added `PhaseTrackPools`
  - later phases can now mix review tracks instead of training on a single terminal track
- added `PhaseRewardOverrides`
  - later-phase reward shaping can be changed without forking the trainer
- changed default phase 3/4 pools to include review tracks
  - phase 3 keeps skidpad-heavy sampling but revisits circle variants
  - phase 4 is autocross-heavy but still revisits skidpad / figure-eight / circle variants
- changed final export behavior
  - end-of-run export now goes to a run-local `final_actor_export/`
  - the default runtime actor path is no longer overwritten automatically by a possibly regressed final agent

### 2. Explicit deployment publish

- added `publishRLDeploymentCandidate.m`
  - writes a canonical `rl_results/deployment/deployment_agent.mat`
  - exports the chosen actor into the stable default runtime path `matlab/rl_driver/rl_actor_export/`
  - stores deployment metadata JSON for traceability
- added `loadRLAgentFromFile.m`
  - checkpoint files with either `agent` or `saved_agent` can now be loaded consistently

### 3. Readiness gate / launcher integration

- `runRLReadinessGate.m`
  - now exports to a run-local staging actor dir for parity/readiness checks
  - can now `PublishIfPass`
  - auto-publish is only allowed when PID baseline lap times are provided for all gated tracks
  - can gate checkpoint files, not just `final_agent.mat`
- `EasyStart.m`
  - readiness gate now uses checked-in PID baseline lap times and can auto-publish on pass
  - `find_best_rl_agent()` now prefers the explicit deployment candidate over heuristic "latest/final" picks
  - the default runtime export path is no longer back-filled from ungated training artifacts
  - the launcher now adds both `matlab/` and `matlab/rl_driver/` before readiness/export flows so shared helpers like `merge_nested_structs.m` resolve reliably
  - RL actor export path validation now uses canonical path containment with a directory separator check, preventing sibling-prefix paths from being treated as inside the repo

### 5. PID baseline helper

- added `getDefaultRLPIDBaselineLapTimes.m`
  - provides checked-in default baselines for:
    - `circle = 15.684 s`
    - `skidpad = 13.014 s`
    - `autocross = 60.010 s`
  - these are sourced from existing PID report files already stored under `matlab/`

### 4. Long-run recovery entrypoint

- added `trainPhase34Recovery.m`
  - resumes from the stable `phase3_skidpad/agent.mat`
  - keeps always-on preview-steer shaping
  - launches a mixed review-heavy phase 3/4 continuation suitable for multi-hour / multi-day runs

## Operational impact

- the default runtime RL actor path is now meant to represent a **chosen deployment candidate**, not simply the newest final checkpoint
- later-phase continuation can be run for long budgets without the hidden early-stop behavior seen in the first `phase3_phase4_from_phase2o` attempt
- the next training run should start from `phase3_phase4_from_phase2o/phase3_skidpad/agent.mat`, not from the regressed phase-4 final weights

## Next action

Run `trainPhase34Recovery` and keep selecting deployment candidates by deterministic readiness, publishing only the candidate that actually passes.
