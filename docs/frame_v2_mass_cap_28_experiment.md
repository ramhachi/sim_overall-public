# Frame V2 28 kg Mass Cap Experiment

Date: 2026-05-11

## Purpose

The previous V2 runs optimized toward 7000 Nm/deg and then inspected the mass penalty. This experiment flips the question:

- Treat 28 kg as the effective mass cap.
- Check whether the current V2 topology family can reach that cap.
- If not, identify the nearest lower-bound stiffness and the remaining mass gap.

## Inputs

- Base artifact: `output/runtime/frame_v2_ga_300_mex_final/frame_v2_ga_15000_vehicle_artifact.json`
- Fixed-topology experiment:
  - Script: `frame_optimize/examples/experiment_frame_v2_mass_cap_stiffness.m`
  - Output: `output/runtime/frame_v2_mass_cap_28_fixed_topology/frame_v2_mass_cap_stiffness_experiment.md`
- Mass-focused GA experiment:
  - Output: `output/runtime/frame_v2_masscap28_ga_500_k3000/frame_v2_ga_15000_summary.json`

## Fixed-Topology Lower Bound

The saved V2 geometry/topology has 65 nodes, 97 elements, and 40.62 m total tube length.

For that fixed topology, every group was reassigned to the lightest pipe that passes the current regulation/material checker.

| Case | Mass kg | Gap to 28 kg | Torsion Nm/deg | Total length m | Elements |
| --- | ---: | ---: | ---: | ---: | ---: |
| Current artifact assignment | 45.06 | +17.06 | 4909 | 40.62 | 97 |
| Lightest compliant fixed topology | 31.03 | +3.03 | 3479 | 40.62 | 97 |

Result: with the current fixed topology, 28 kg is infeasible even with the lightest compliant pipe assignment. The lower-bound design is still 3.03 kg over the cap.

## Regulation-Minimum Topology Probe

After inspecting the fixed 97-element topology, a second experiment replaced the saved topology with an intentionally sparse topology on the same V2 node set:

- keep regulation-required approximations for Front Hoop, Main Hoop, Shoulder Bar, Side Impact, FBH, FBH supports, Main Hoop Braces, and Main Hoop Brace supports
- keep minimum front/rear suspension bracket links so hardpoints are not floating
- remove the current always-on rear box and other non-required rear/free-region links
- optionally keep differential mount links to the Main Hoop Brace trapezoid

Experiment entry:

```bash
bash scripts/run_matlab_batch.sh "addpath(fullfile(pwd,'frame_optimize','examples')); experiment_frame_v2_mass_cap_stiffness('ArtifactPath',fullfile(pwd,'output','runtime','frame_v2_ga_300_mex_final','frame_v2_ga_15000_vehicle_artifact.json'),'OutputDir',fullfile(pwd,'output','runtime','frame_v2_mass_cap_28_reg_min_diff'),'MassCapKg',28,'TopologyMode','regulation_minimal','ConnectDiffMounts',true);"
```

Results:

| Topology | Nodes | Elements | Total length m | Lightest mass kg | Gap to 28 kg | K torsion Nm/deg | Load cases |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Saved fixed topology | 65 | 97 | 40.62 | 31.03 | +3.03 | 3479 | torsion, lateral, longitudinal, rollover, diff |
| Regulation-minimum + diff mounts | 61 | 82 | 35.80 | 27.23 | -0.77 | 3357 | torsion, lateral, longitudinal, rollover, diff |
| Regulation-minimum + suspension only | 57 | 74 | 31.51 | 24.28 | -3.72 | 3357 | torsion, lateral, longitudinal, rollover |

This changes the interpretation: the 28 kg cap is not impossible inside the current geometric package, but it is impossible inside the previously saved 97-element topology. The non-regulation rear/free-region members dominate the mass lower bound.

Important caveat: the current V2 node set still approximates the Upper Side Impact member using hoop/topology nodes that do not fully model the F.6.4 geometry. Therefore these sparse-topology numbers are a topology lower-bound probe, not a final compliance claim.

## Load-Case Behavior At The Lower Bound

| Case | K torsion Nm/deg | Max translation m |
| --- | ---: | ---: |
| torsion | 3479 | 0.004894 |
| lateral_bending | 0 | 0.057433 |
| longitudinal_bending | 0 | 0.010220 |
| rollover | 0 | 0.318755 |
| diff_reaction | 3294 | 0.003509 |

The torsional number remains around 3.5 kNm/deg at the fixed-topology lower bound, but lateral and rollover displacement are poor. This suggests that a 28 kg frame cannot simply be obtained by swapping pipes; topology and load-path choices must change.

## Mass-Focused GA Check

An additional 500-evaluation MATLAB `ga` run used:

- `MassTargetKg=28`
- `TargetStiffnessNmPerDeg=3000`
- `MassPenaltyWeight=20000`
- `SparsityMaxTotalLengthM=34`
- `SparsityMaxElements=97`
- `SparsityPenaltyScale=2000`

Result:

- Mass: 42.86 kg
- Torsion: 5397 Nm/deg
- Nodes/elements: 65 / 97
- Compliance: pass
- Runtime: 187.3 s, 0.3575 s/eval
- True valid individual rate: 50%

This run did not find the fixed-topology lower-bound pipe assignment. It is useful as a stress test of the current optimizer, but not evidence that 42.86 kg is a physical lower bound.

## Interpretation

Current best answer to "what stiffness can be guaranteed under 28 kg?" is:

- In the saved 97-element V2 topology, no compliant 28 kg design has been found.
- In the regulation-minimum + diff-mount topology probe, a lightest compliant assignment is 27.23 kg and about 3357 Nm/deg torsional stiffness.
- In the suspension-only sparse probe, a lightest compliant assignment is 24.28 kg and about 3357 Nm/deg torsional stiffness, but diff reaction is intentionally not evaluated.
- Therefore, the next useful target is not "make the 97-element topology lighter", but "repair the regulation geometry model and then optimize from the sparse topology upward."

To make 28 kg feasible, the current lower-bound frame must remove at least 3.03 kg. Since many non-critical groups use about 0.688 kg/m tubing in the lower-bound assignment, this is roughly equivalent to removing 4.4 m of light tubing, or reducing the mandatory topology and dimensions enough to achieve a similar length reduction.

## Next Experiment

Before another long GA run:

1. Split the current Side Impact node model into explicit FH/MH upper-side-impact attachment points in the 290-345 mm band.
2. Promote the regulation-minimum topology into a generator option or initial population seed.
3. Add topology switches for optional support/rear members, then maximize stiffness under a hard 28 kg cap.
4. Re-check lateral, rollover, braking, and diff reaction displacement on the best 28 kg candidates.
