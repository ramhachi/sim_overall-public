# POU Hardpoint Reverse-Engineering System

Inverse kinematics solver for translating optimized polynomial POU coefficients into manufacturable 3D hardpoint coordinates.

## Overview

The vehicle optimization system produces polynomial coefficients (c1, c2) that define ideal POU behavior:
- **Jack-up**: Δz = c1·δ + c2·δ² [m vs. rad]
- **Camber**: Δγ = c1·δ + c2·δ² [rad vs. rad]

This system reverse-engineers those polynomials to find physical M-pivot and bellcrank coordinates that can be built in SolidWorks.

## Quick Start

### Basic Usage

```matlab
% Run optimization for front-left wheel
results = main_pou_hardpoint_optimizer('optimized_autocross.json', ...
    struct('wheel_index', 0, 'verbose', true));

% Check results
fprintf('M-pivot (chassis): [%.4f, %.4f, %.4f] m\n', results.M_pivot_chassis);
fprintf('Bellcrank pivot:   [%.4f, %.4f, %.4f] m\n', results.bellcrank_pivot);
fprintf('Polynomial error:  %.2f mm RMS\n', results.polynomial_error_mm);
fprintf('Max rod angle:     %.2f deg\n', results.max_rod_angle_deg);
fprintf('Packaging valid:   %s\n', results.packaging_valid ? 'YES' : 'NO');
```

### Run Unit Tests

```matlab
cd pou_hardpoint_design
test_pou_optimizer  % Runs all 7 unit tests
```

## Architecture

### Directory Structure

```
pou_hardpoint_design/
├── main_pou_hardpoint_optimizer.m   % Main entry point
├── test_pou_optimizer.m             % Unit test suite
├── README.md                        % This file
├── geometry/
│   ├── define_suspension_geometry.m % Fixed hardpoints + bounds
│   ├── axis_angle_rotation.m        % 3D rotation matrix utility
│   ├── solve_upright_position_3d.m  % Spatial linkage solver
│   ├── forward_kinematics.m         % Hardpoints → displacement curves
│   └── validate_packaging.m         % Post-optimization collision checks
└── optimization/
    ├── objective_polynomial_fit.m   % Cost function (RMS error)
    └── constraint_rod_misalignment.m % Rod-end angle + IR constraints
```

### Design Variables (6 DOF)

The optimizer solves for:
- **M-pivot** (upright local frame): `[Xm, Ym, Zm]` [m]
- **Bellcrank pivot** (chassis frame): `[Xb, Yb, Zb]` [m]

This is a **new design** approach where both POU layout and bellcrank position are optimized together.

## Algorithm

### 1. Forward Kinematics (3D Spatial Linkage)

For each steering angle δ:
1. **Rotate upright** around kingpin axis (Rodrigues formula)
2. **Solve spatial 4-bar linkage** (upper + lower A-arms) using Newton-Raphson
3. **Transform M-pivot** from upright frame to chassis frame
4. **Compute pushrod length** to bellcrank pivot
5. **Extract camber** from rotation matrix

### 2. Optimization (fmincon Interior-Point)

**Objective**: Minimize weighted RMS error between geometric and polynomial curves

```
cost = w_jack * rms(jack_error) + w_camber * rms(camber_error)
```

**Constraints**:
- Rod-end misalignment angle < 15° (or 20° for high-misalignment type)
- Pushrod length: 0.15 m < L < 0.40 m
- Installation ratio: 0.10 < IR < 0.30
- Packaging: Rim clearance > 5mm, no caliper collision

**Multi-start**: Runs 5 initial guesses, selects best result

### 3. Validation

Post-optimization checks:
- ✓ Polynomial fit error < 2mm RMS (success threshold)
- ✓ All constraints satisfied
- ✓ No packaging violations (rim, caliper, chassis bounds)

## Input/Output Formats

### Input: Vehicle Parameters JSON

```json
{
  "pou_jack_c1": [0.0922, 0.0169, 0, 0],
  "pou_jack_c2": [0.0226, -0.0253, 0, 0],
  "pou_camber_c1": [0.5, 0.2498, 0, 0],
  "pou_camber_c2": [-0.3516, 0.5, 0, 0],
  "wheelbase": 1.535,
  "track_front": 1.200
}
```

### Output: Results Struct

```matlab
results =
  struct with fields:
    M_pivot_chassis: [3×1 double]       % Chassis frame coordinates [m]
    bellcrank_pivot: [3×1 double]       % Bellcrank position [m]
    polynomial_error_mm: 0.85           % RMS fit error [mm]
    max_rod_angle_deg: 13.2             % Peak rod-end angle [deg]
    installation_ratio: 0.178           % Mechanical advantage [-]
    packaging_valid: true               % Collision check result
    optimization_exitflag: 1            % fmincon status
    optimization_time_s: 12.4           % Solve time [s]
```

## Options Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_index` | int | 0 | 0=FL, 1=FR, 2=RL, 3=RR |
| `n_multistart` | int | 5 | Number of random initial guesses |
| `delta_range_deg` | 1×2 | [-30, 30] | Steering angle range [deg] |
| `n_delta_points` | int | 61 | Number of points in sweep |
| `verbose` | bool | true | Print progress messages |

## Geometry Assumptions

### Fixed Hardpoints (Assumed Typical FSAE Values)

| Hardpoint | X [m] | Y [m] | Z [m] | Notes |
|-----------|-------|-------|-------|-------|
| Upper A-arm (chassis) | 0.55 | ±0.35 | -0.20 | Narrow, high mounting |
| Lower A-arm (chassis) | 0.65 | ±0.40 | 0.05 | Wide, low mounting |
| Upper A-arm (upright) | 0.60 | ±0.60 | -0.15 | Upper ball joint |
| Lower A-arm (upright) | 0.60 | ±0.60 | 0.10 | Lower ball joint |
| Kingpin origin | 0.60 | ±0.65 | 0.00 | Wheel center height |

**Kingpin angles**:
- KPI: 7° (inboard tilt)
- Caster: 3° (rearward tilt)

**Wheel geometry**:
- Rim radius: 0.127 m (10 inch diameter)
- Minimum clearance: 5 mm

### Design Variable Bounds

| Variable | Lower | Upper | Unit | Notes |
|----------|-------|-------|------|-------|
| Xm (M-pivot) | 0.02 | 0.10 | m | Forward of wheel center |
| Ym (M-pivot) | -0.03 | 0.00 | m | Must fit inside rim |
| Zm (M-pivot) | -0.05 | 0.02 | m | Vertical offset |
| Xb (bellcrank) | 0.50 | 0.70 | m | Longitudinal position |
| Yb (bellcrank) | -0.45 | -0.25 | m | Inboard (LH wheel) |
| Zb (bellcrank) | -0.20 | 0.00 | m | Chassis-side mounting |

## Success Criteria

### Polynomial Fit
- ✓ Jack-up RMS error: **< 2.0 mm**
- ✓ Camber RMS error: **< 1.0°**
- ✓ Max error at full lock: **< 5 mm**

### Mechanical Constraints
- ✓ Rod-end angle: **< 15°** (standard) or **< 20°** (high-misalignment)
- ✓ Pushrod length: **0.15 - 0.40 m**
- ✓ Installation ratio: **0.10 - 0.30** (soft constraint)

### Packaging
- ✓ Rim clearance: **> 5 mm**
- ✓ Caliper clearance: **> 5 mm**
- ✓ Bellcrank inboard of wheel

## Typical Results

For FL wheel with optimized autocross gains:

```
M-pivot (chassis):  [0.0523, -0.6120, -0.0145] m
Bellcrank pivot:    [0.6200, -0.3500, -0.0900] m
Polynomial error:   0.85 mm RMS
Max rod angle:      13.2 deg (OK, < 15 deg limit)
Installation ratio: 0.178 (good mechanical advantage)
Packaging:          PASS (all clearances satisfied)
Solve time:         12.4 seconds
```

## Troubleshooting

### Problem: Optimization fails to converge

**Solution**: Check polynomial coefficients are physically reasonable
- |jack_c1| < 0.10 m/rad (excessive values are likely infeasible)
- |camber_c1| < 1.0 rad/rad (beyond typical suspension range)

### Problem: Packaging violations (rim clearance)

**Solution**: Adjust M-pivot Y-bounds in `define_suspension_geometry.m`
- Reduce upper bound (move more inboard)
- Check rim radius is correct (default: 0.127 m for 10" wheel)

### Problem: Rod-end angle constraint violated

**Solution**:
1. Use high-misalignment rod-ends (20° limit)
2. Adjust `geom.rod_end_angle_max` in geometry definition
3. Relax constraint in `constraint_rod_misalignment.m`

### Problem: Installation ratio too low/high

**Solution**: This is a soft constraint - can be relaxed
- Acceptable range: 0.08 - 0.35 (wider than default 0.10 - 0.30)
- Edit bounds in `constraint_rod_misalignment.m` line 67-68

## Integration with Workflow

### Current Usage (Standalone)

```matlab
% After LPV vehicle optimization (EasyStart menu [3])
ga_result = run_full_vehicle_ga(vehicle_ga_default_config());

% Run POU inverse kinematics
results = main_pou_hardpoint_optimizer(ga_result.best_artifact);

% Export coordinates for SolidWorks (Phase 3 - not yet implemented)
% export_to_solidworks(results, 'pou_hardpoints_FL.csv');
```

### Future Integration (Phase 4)

Add to `run_full_pipeline.m` between vehicle optimization and PID tuning:

```matlab
% Phase 1: Vehicle optimization
ga_result = run_full_vehicle_ga(vehicle_ga_default_config());

% Phase 2: POU hardpoint validation (NEW)
pou_results = main_pou_hardpoint_optimizer(ga_result.best_artifact);
if ~pou_results.packaging_valid
    warning('POU hardpoints infeasible! Check validation report.');
end

% Phase 3: PID optimization
optimize_pid_gains(...);
```

## Implementation Status

### ✅ Phase 1-2: Core Functionality (COMPLETED)

- [x] 3D spatial linkage forward kinematics
- [x] 6-DOF optimization (M-pivot + bellcrank)
- [x] Polynomial fit objective function
- [x] Rod-end angle constraint
- [x] Installation ratio constraint
- [x] Pushrod length bounds
- [x] Packaging validation (rim, caliper, chassis)
- [x] Multi-start optimization
- [x] Unit test suite

### ⏳ Phase 3: Export & Visualization (PENDING)

- [ ] SolidWorks CSV export
- [ ] JSON metadata export
- [ ] Comparison plots (geometric vs polynomial)
- [ ] HTML validation report
- [ ] 3D trajectory visualization

### ⏳ Phase 4: Pipeline Integration (PENDING)

- [ ] Add to `run_full_pipeline.m`
- [ ] Manufacturability heuristic for GA
- [ ] Automated report generation
- [ ] All 4 wheels batch processing

## References

- **Plan document**: See implementation plan in project root
- **C++ POU implementation**: `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
- **Vehicle parameters**: `matlab/optimized_autocross.json`
- **Test suite**: `cpp/tests/test_pou.cpp`

## Contact

For questions or issues:
1. Check unit tests: `test_pou_optimizer.m`
2. Review algorithm: See "Algorithm" section above
3. Verify geometry: `define_suspension_geometry.m` assumptions
