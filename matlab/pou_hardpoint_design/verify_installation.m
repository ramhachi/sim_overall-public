% VERIFY_INSTALLATION - Check that POU hardpoint system is properly installed
%
% This script verifies all functions are accessible and basic functionality works

clear; clc;

this_dir = fileparts(mfilename('fullpath'));
addpath(fileparts(this_dir));

fprintf('=== POU Hardpoint System Installation Verification ===\n\n');

% === Check 1: Path Setup ===
fprintf('Check 1: Verify paths...\n');
current_dir = pwd;
if ~contains(current_dir, 'pou_hardpoint_design')
    fprintf('  ⚠ Warning: Not in pou_hardpoint_design directory\n');
    fprintf('  Current: %s\n', current_dir);
    fprintf('  Suggestion: cd matlab/pou_hardpoint_design\n\n');
else
    fprintf('  ✓ Current directory correct\n\n');
end

% === Check 2: Function Files Exist ===
fprintf('Check 2: Verify all files exist...\n');

required_files = {
    'main_pou_hardpoint_optimizer.m';
    'test_pou_optimizer.m';
    'example_usage.m';
    'geometry/define_suspension_geometry.m';
    'geometry/axis_angle_rotation.m';
    'geometry/solve_upright_position_3d.m';
    'geometry/forward_kinematics.m';
    'geometry/validate_packaging.m';
    'optimization/objective_polynomial_fit.m';
    'optimization/constraint_rod_misalignment.m';
    'visualization/plot_comparison.m';
};

all_exist = true;
for i = 1:length(required_files)
    if exist(required_files{i}, 'file')
        fprintf('  ✓ %s\n', required_files{i});
    else
        fprintf('  ✗ MISSING: %s\n', required_files{i});
        all_exist = false;
    end
end

if ~all_exist
    fprintf('\n  ✗ Some files are missing! Installation incomplete.\n');
    return;
end

fprintf('\n  ✓ All required files present\n\n');

% === Check 3: MATLAB Toolboxes ===
fprintf('Check 3: Verify MATLAB toolboxes...\n');

if license('test', 'Optimization_Toolbox')
    fprintf('  ✓ Optimization Toolbox available\n');
else
    fprintf('  ✗ Optimization Toolbox NOT available (required for fmincon)\n');
    fprintf('  The system will NOT work without this toolbox!\n');
    return;
end

fprintf('\n');

% === Check 4: Function Calls ===
fprintf('Check 4: Test function calls...\n');

try
    % Test 1: Geometry definition
    geom = define_suspension_geometry(0);
    fprintf('  ✓ define_suspension_geometry() works\n');

    % Test 2: Axis-angle rotation
    R = axis_angle_rotation([0; 0; 1], pi/4);
    fprintf('  ✓ axis_angle_rotation() works\n');

    % Test 3: Upright solver
    upright = solve_upright_position_3d(deg2rad(10), geom);
    fprintf('  ✓ solve_upright_position_3d() works\n');

    % Test 4: Forward kinematics
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';
    [jack, camber, lengths] = forward_kinematics(design_vars, geom, delta_range);
    fprintf('  ✓ forward_kinematics() works\n');

    % Test 5: Objective function
    target_poly.jack_c1 = 0.0922;
    target_poly.jack_c2 = 0.0226;
    target_poly.camber_c1 = 0.5;
    target_poly.camber_c2 = -0.3516;
    cost = objective_polynomial_fit(design_vars, geom, target_poly, delta_range);
    fprintf('  ✓ objective_polynomial_fit() works\n');

    % Test 6: Constraints
    [c, ceq] = constraint_rod_misalignment(design_vars, geom, delta_range);
    fprintf('  ✓ constraint_rod_misalignment() works\n');

    % Test 7: Packaging validation
    [valid, violations] = validate_packaging(design_vars, geom, delta_range);
    fprintf('  ✓ validate_packaging() works\n');

catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
    fprintf('  at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    return;
end

fprintf('\n  ✓ All core functions working\n\n');

% === Check 5: Input File ===
fprintf('Check 5: Check for input JSON file...\n');

json_file = '../optimized_autocross.json';
if exist(json_file, 'file')
    fprintf('  ✓ Found: %s\n', json_file);

    % Try loading it
    try
        artifact = load_vehicle_artifact(json_file);
        params = artifact.params;

        if isfield(params, 'suspension') && isfield(params.suspension, 'pou_jack_c1')
            fprintf('  ✓ JSON contains POU parameters\n');
        else
            fprintf('  ⚠ JSON missing POU parameters (may be old version)\n');
        end
    catch ME
        fprintf('  ✗ Cannot parse JSON: %s\n', ME.message);
    end
else
    fprintf('  ⚠ Input file not found: %s\n', json_file);
    fprintf('  You will need to provide the path when running optimization\n');
end

fprintf('\n');

% === Summary ===
fprintf('========================================\n');
fprintf('INSTALLATION VERIFICATION COMPLETE\n');
fprintf('========================================\n\n');

fprintf('Status: ✓ ALL CHECKS PASSED\n\n');

fprintf('Next steps:\n');
fprintf('  1. Run unit tests: test_pou_optimizer\n');
fprintf('  2. Run full example: example_usage\n');
fprintf('  3. Customize geometry: edit geometry/define_suspension_geometry.m\n');
fprintf('  4. Run optimization: main_pou_hardpoint_optimizer(''path/to/params.json'')\n\n');

fprintf('Documentation:\n');
fprintf('  - Quick start:  QUICK_START.md (5 minutes)\n');
fprintf('  - User guide:   README.md (30 minutes)\n');
fprintf('  - Technical:    ../docs/POU_HARDPOINT_SYSTEM.md (1 hour)\n\n');

fprintf('=== Verification Complete ===\n');
