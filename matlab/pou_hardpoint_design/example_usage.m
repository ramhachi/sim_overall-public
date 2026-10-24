% EXAMPLE_USAGE - Demonstration of POU hardpoint optimization workflow
%
% This script shows how to use the POU hardpoint reverse-engineering system
% to translate optimized polynomial coefficients into physical coordinates.

clear; clc; close all;

fprintf('=== POU Hardpoint Optimization - Example Usage ===\n\n');

% Add all required paths
addpath('geometry');
addpath('optimization');
addpath('visualization');

%% Step 1: Load vehicle parameters
fprintf('Step 1: Loading optimized vehicle parameters...\n');

% Path to optimized parameters JSON (from vehicle optimization)
params_file = '../optimized_autocross.json';

if ~exist(params_file, 'file')
    error('Cannot find optimized parameters file: %s', params_file);
end

fprintf('  ✓ Found: %s\n\n', params_file);

%% Step 2: Configure optimization options
fprintf('Step 2: Configuring optimization...\n');

options = struct();
options.wheel_index = 0;           % 0=FL, 1=FR, 2=RL, 3=RR
options.n_multistart = 5;          % Number of random initial guesses
options.delta_range_deg = [-30, 30];  % Steering range [deg]
options.n_delta_points = 61;       % Points in sweep
options.verbose = true;            % Print progress

fprintf('  Wheel: %s\n', ['FL', 'FR', 'RL', 'RR'](options.wheel_index*2 + (1:2)));
fprintf('  Multi-start: %d initial guesses\n', options.n_multistart);
fprintf('  Steering range: [%.0f, %.0f] deg\n', options.delta_range_deg);
fprintf('\n');

%% Step 3: Run optimization
fprintf('Step 3: Running inverse kinematics optimization...\n');
fprintf('  (This may take 10-30 seconds depending on complexity)\n\n');

tic;
results = main_pou_hardpoint_optimizer(params_file, options);
total_time = toc;

%% Step 4: Analyze results
fprintf('\n');
fprintf('========================================\n');
fprintf('OPTIMIZATION COMPLETED\n');
fprintf('========================================\n');
fprintf('Total Time: %.1f seconds\n\n', total_time);

% Check success
if results.optimization_exitflag >= 1
    fprintf('Status: ✓ CONVERGED\n');
else
    fprintf('Status: ✗ FAILED (exitflag=%d)\n', results.optimization_exitflag);
end

% Hardpoint coordinates
fprintf('\n--- Hardpoint Coordinates ---\n');
fprintf('M-pivot (upright frame): [%.4f, %.4f, %.4f] m\n', results.M_pivot_upright);
fprintf('M-pivot (chassis frame): [%.4f, %.4f, %.4f] m\n', results.M_pivot_chassis);
fprintf('Bellcrank pivot:         [%.4f, %.4f, %.4f] m\n', results.bellcrank_pivot);

% Performance metrics
fprintf('\n--- Performance Metrics ---\n');
fprintf('Polynomial Fit Error (RMS): %.2f mm', results.polynomial_error_mm);
if results.polynomial_error_mm < 2.0
    fprintf(' ✓ PASS (< 2mm)\n');
else
    fprintf(' ✗ FAIL (> 2mm)\n');
end

fprintf('Max Jack Error:             %.2f mm\n', results.max_jack_error_mm);
fprintf('Max Camber Error:           %.2f deg\n', results.max_camber_error_deg);

fprintf('Max Rod-End Angle:          %.2f deg', results.max_rod_angle_deg);
if results.max_rod_angle_deg < 15.0
    fprintf(' ✓ PASS (< 15°)\n');
else
    fprintf(' ⚠ WARNING (> 15°, use high-misalignment rod-end)\n');
end

fprintf('Installation Ratio:         %.3f', results.installation_ratio);
if results.installation_ratio >= 0.10 && results.installation_ratio <= 0.30
    fprintf(' ✓ PASS (0.10-0.30)\n');
else
    fprintf(' ⚠ OUT OF RANGE (target: 0.10-0.30)\n');
end

fprintf('Pushrod Length (nominal):   %.3f m\n', results.pushrod_length_nominal_m);

% Packaging validation
fprintf('\n--- Packaging Validation ---\n');
if results.packaging_valid
    fprintf('Status: ✓ PASS (all constraints satisfied)\n');
else
    fprintf('Status: ✗ FAIL (%d violations)\n', length(results.violations));
    fprintf('Violations:\n');
    for i = 1:length(results.violations)
        fprintf('  %d. %s\n', i, results.violations{i});
    end
end

fprintf('========================================\n\n');

%% Step 5: Visualize results
fprintf('Step 5: Generating comparison plots...\n');

try
    fig = plot_comparison(results);
    fprintf('  ✓ Plot generated successfully\n');

    % Optionally save figure
    % saveas(fig, 'pou_hardpoint_optimization_results.png');
    % fprintf('  ✓ Plot saved to: pou_hardpoint_optimization_results.png\n');
catch ME
    fprintf('  ✗ Plot generation failed: %s\n', ME.message);
end

%% Step 6: Export for SolidWorks (Phase 3 - not yet implemented)
fprintf('\nStep 6: Export for SolidWorks...\n');
fprintf('  ⏳ NOT YET IMPLEMENTED (Phase 3)\n');
fprintf('  Future: export_to_solidworks(results, ''pou_hardpoints_FL.csv'')\n');

%% Summary
fprintf('\n=== Example Completed ===\n');
fprintf('The optimized hardpoint coordinates can now be used in CAD design.\n');
fprintf('Next steps:\n');
fprintf('  1. Review plots to verify polynomial fit quality\n');
fprintf('  2. Check packaging validation results\n');
fprintf('  3. Export coordinates for SolidWorks import (Phase 3)\n');
fprintf('  4. Repeat for other wheels (FR, RL, RR) if needed\n\n');
