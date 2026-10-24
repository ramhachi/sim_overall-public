# Repository hygiene

Generated build trees, Python caches, simulator outputs, and vcpkg install
trees are not tracked. They are already covered by `.gitignore`; this document
records the policy after the one-time cleanup of 1,456 previously tracked
ignored files.

## Current policy

- Keep source, tests, configuration, and intentional small fixtures tracked.
- Keep rebuildable artifacts (`build/`, `build-*`, `vcpkg_installed/`,
  `__pycache__/`, compiled extensions) untracked.
- Keep simulator outputs (`output/`, `matlab/laptime_sim_*.csv`,
  timestamped reports) untracked unless a test explicitly needs a fixture.
- CI fails when `git ls-files -ci --exclude-standard` is non-empty.

## History

The cleanup removes files from the current tree. It does not rewrite existing
Git history. If the historical size or measured-data licensing requires a
clean public repository, perform a separate history migration with a tool such
as `git filter-repo`, then re-run the CI hygiene check on the rewritten tree.

Measurement data under `data/tiredata` is intentionally not removed by this
cleanup; its redistribution terms and any history split need a separate
decision.
