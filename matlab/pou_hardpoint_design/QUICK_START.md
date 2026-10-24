# POU Hardpoint Optimizer - Quick Start Guide

**5-Minute Setup & First Run**

## Prerequisites

✓ MATLAB R2020a or newer (Optimization Toolbox required)
✓ Optimized vehicle parameters JSON file (e.g., `optimized_autocross.json`)

## Installation

```matlab
% 1. Navigate to project root
cd /path/to/sim_overall/matlab

% 2. Add paths (automatically)
addpath('pou_hardpoint_design');
addpath('pou_hardpoint_design/geometry');
addpath('pou_hardpoint_design/optimization');
addpath('pou_hardpoint_design/visualization');
```

## Basic Usage (3 lines)

```matlab
% Run optimization for front-left wheel
results = main_pou_hardpoint_optimizer('optimized_autocross.json', ...
    struct('wheel_index', 0, 'verbose', true));
```

## Check Results

```matlab
% Print summary
fprintf('M-pivot:  [%.3f, %.3f, %.3f] m\n', results.M_pivot_chassis);
fprintf('Error:    %.2f mm RMS\n', results.polynomial_error_mm);
fprintf('Packaging: %s\n', results.packaging_valid ? 'PASS' : 'FAIL');

% Visualize
plot_comparison(results);
```

## Expected Output

```
=== POU Hardpoint Inverse Kinematics Optimizer ===
Loading vehicle parameters from: optimized_autocross.json
Wheel: FL
Target Polynomial Coefficients:
  Jack-up:  c1=0.0922 m/rad, c2=0.0226 m/rad²
  Camber:   c1=0.5000 rad/rad, c2=-0.3516 rad/rad²

Running optimization with 5 initial guesses...
[fmincon output...]

========== OPTIMIZATION RESULTS ==========
Exit Status: 1 (converged)
Solve Time: 15.2 seconds

--- Hardpoint Coordinates ---
M-pivot (chassis): [0.0523, -0.6120, -0.0145] m
Bellcrank pivot:   [0.6200, -0.3500, -0.0900] m

--- Performance Metrics ---
Polynomial Fit Error (RMS): 0.85 mm
Max Rod-End Angle:          13.2 deg (limit: 15 deg)
Installation Ratio:         0.178 (target: 0.15-0.25)

--- Packaging Validation ---
Status: ✓ PASS (all constraints satisfied)
==========================================
```

## Options Reference

| Option | Default | Description |
|--------|---------|-------------|
| `wheel_index` | 0 | 0=FL, 1=FR, 2=RL, 3=RR |
| `n_multistart` | 5 | Number of random initial guesses |
| `verbose` | true | Print progress messages |

## Full Example

```matlab
% Run with custom options
options = struct();
options.wheel_index = 0;        % Front-left
options.n_multistart = 3;       % Faster (less robust)
options.verbose = false;        % Quiet mode

results = main_pou_hardpoint_optimizer('optimized_autocross.json', options);

% Access results
M_pivot = results.M_pivot_chassis;     % [3×1] M-pivot coordinates [m]
bellcrank = results.bellcrank_pivot;   % [3×1] Bellcrank coordinates [m]
error_mm = results.polynomial_error_mm; % RMS fit error [mm]
rod_angle = results.max_rod_angle_deg;  % Max rod-end angle [deg]
IR = results.installation_ratio;        % Installation ratio [-]
valid = results.packaging_valid;        % Packaging check result (bool)
```

## Run Tests

```matlab
% Unit test suite (7 tests)
cd pou_hardpoint_design
test_pou_optimizer

% Full workflow demonstration
example_usage
```

## Troubleshooting

### Problem: "Cannot open file: optimized_autocross.json"

**Solution**: Provide full path or copy file to current directory
```matlab
results = main_pou_hardpoint_optimizer('../optimized_autocross.json');
```

### Problem: "Optimization failed to find feasible solution"

**Likely cause**: Polynomial coefficients are physically infeasible

**Check**:
```matlab
% Load JSON
fid = fopen('optimized_autocross.json', 'r');
json_text = fread(fid, '*char')';
fclose(fid);
params = jsondecode(json_text);

% Check FL wheel coefficients
fprintf('Jack c1: %.4f (should be < 0.10)\n', params.pou_jack_c1(1));
fprintf('Jack c2: %.4f (should be < 0.05)\n', params.pou_jack_c2(1));
```

### Problem: "Packaging violations"

**Solution**: Check `results.violations` for details
```matlab
if ~results.packaging_valid
    for i = 1:length(results.violations)
        fprintf('%d. %s\n', i, results.violations{i});
    end
end
```

## Next Steps

1. ✅ Run basic example above
2. ✅ Review plots (`plot_comparison(results)`)
3. ⚠️ Update geometry in `define_suspension_geometry.m` with actual CAD hardpoints
4. ⚠️ Run for all 4 wheels (change `wheel_index`)
5. ⏳ Export for SolidWorks (Phase 3 - not yet implemented)

## Documentation

- **Quick start**: This file (5 minutes)
- **User guide**: `README.md` (30 minutes, comprehensive)
- **Technical spec**: `docs/POU_HARDPOINT_SYSTEM.md` (1 hour, full details)
- **Implementation**: `IMPLEMENTATION_SUMMARY.md` (design decisions)

## Help

```matlab
help main_pou_hardpoint_optimizer    % Function documentation
type test_pou_optimizer.m            % See example usage
type example_usage.m                 % Full workflow
```

---

**TL;DR**: Run `main_pou_hardpoint_optimizer('optimized_autocross.json')` and check `results.packaging_valid`.
