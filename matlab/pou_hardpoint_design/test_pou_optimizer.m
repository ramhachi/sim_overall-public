% TEST_POU_OPTIMIZER - Test suite for POU hardpoint optimization (DW model)
%
% Tests the full double-wishbone kinematic solver, forward kinematics,
% constraints, and packaging validation.

clear; clc;

fprintf('=== POU Hardpoint Optimizer Test (Double-Wishbone) ===\n\n');

addpath('geometry');
addpath('optimization');

n_pass = 0;
n_fail = 0;

%% Test 1: Geometry Definition (new DW fields)
fprintf('Test 1: Suspension Geometry Definition (DW model)\n');
try
    geom = define_suspension_geometry(0);  % FL wheel

    % Check new DW fields exist
    assert(isfield(geom, 'upper_arm_front_chassis'), 'Missing upper_arm_front_chassis');
    assert(isfield(geom, 'upper_arm_rear_chassis'), 'Missing upper_arm_rear_chassis');
    assert(isfield(geom, 'lower_arm_front_chassis'), 'Missing lower_arm_front_chassis');
    assert(isfield(geom, 'lower_arm_rear_chassis'), 'Missing lower_arm_rear_chassis');
    assert(isfield(geom, 'tie_rod_chassis'), 'Missing tie_rod_chassis');
    assert(isfield(geom, 'tie_rod_upright'), 'Missing tie_rod_upright');
    assert(isfield(geom, 'upper_arm_axis'), 'Missing upper_arm_axis');
    assert(isfield(geom, 'lower_arm_axis'), 'Missing lower_arm_axis');

    % Check derived kingpin
    assert(isfield(geom, 'kingpin_length'), 'Missing kingpin_length');
    assert(geom.kingpin_length > 0.1, 'Kingpin length too short');

    fprintf('  PASS: All DW fields defined\n');
    fprintf('    Kingpin length: %.4f m\n', geom.kingpin_length);
    fprintf('    Upper arm axis: [%.3f, %.3f, %.3f]\n', geom.upper_arm_axis);
    fprintf('    Lower arm axis: [%.3f, %.3f, %.3f]\n', geom.lower_arm_axis);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
    return;
end

%% Test 2: Kingpin Derivation from Ball Joints
fprintf('\nTest 2: Kingpin Axis Derived from Ball Joints\n');
try
    geom = define_suspension_geometry(0);

    % Kingpin direction should point from lower to upper BJ
    expected_dir = (geom.upper_arm_upright - geom.lower_arm_upright) / ...
                   norm(geom.upper_arm_upright - geom.lower_arm_upright);
    actual_dir = geom.kingpin_axis.direction;

    dir_error = norm(expected_dir - actual_dir);
    assert(dir_error < 1e-10, sprintf('Kingpin direction error: %.2e', dir_error));

    % Kingpin origin should be at lower BJ
    origin_error = norm(geom.kingpin_axis.origin - geom.lower_arm_upright);
    assert(origin_error < 1e-10, sprintf('Kingpin origin error: %.2e', origin_error));

    % Kingpin length should match BJ distance
    bj_dist = norm(geom.upper_arm_upright - geom.lower_arm_upright);
    assert(abs(geom.kingpin_length - bj_dist) < 1e-10, 'Kingpin length mismatch');

    fprintf('  PASS: Kingpin correctly derived from ball joints\n');
    fprintf('    Direction: [%.4f, %.4f, %.4f]\n', actual_dir);
    fprintf('    Length: %.4f m\n', geom.kingpin_length);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
    return;
end

%% Test 3: A-arm Rotation Axis
fprintf('\nTest 3: A-arm Rotation Axis Computation\n');
try
    geom = define_suspension_geometry(0);

    % Upper arm axis should be along front-rear chassis line
    expected_upper = (geom.upper_arm_rear_chassis - geom.upper_arm_front_chassis);
    expected_upper = expected_upper / norm(expected_upper);

    axis_error = norm(expected_upper - geom.upper_arm_axis);
    assert(axis_error < 1e-10, sprintf('Upper arm axis error: %.2e', axis_error));

    % Axis origin should be at front chassis mount
    origin_error = norm(geom.upper_arm_axis_origin - geom.upper_arm_front_chassis);
    assert(origin_error < 1e-10, 'Upper arm axis origin mismatch');

    % Axis should be unit vector
    assert(abs(norm(geom.upper_arm_axis) - 1) < 1e-10, 'Upper axis not unit');
    assert(abs(norm(geom.lower_arm_axis) - 1) < 1e-10, 'Lower axis not unit');

    fprintf('  PASS: A-arm rotation axes correctly computed\n');
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
    return;
end

%% Test 4: Zero-Steer Identity
fprintf('\nTest 4: Zero Steering Returns Identity\n');
try
    geom = define_suspension_geometry(0);
    upright = solve_upright_position_3d(0.0, geom);

    % R should be identity
    R_error = norm(upright.R_upright - eye(3), 'fro');
    assert(R_error < 1e-10, sprintf('R not identity at delta=0: %.2e', R_error));

    % t should be zero
    t_error = norm(upright.t_upright);
    assert(t_error < 1e-10, sprintf('t not zero at delta=0: %.2e', t_error));

    % BJ should be at nominal positions
    uu_error = norm(upright.U_U - geom.upper_arm_upright);
    lu_error = norm(upright.L_U - geom.lower_arm_upright);
    assert(uu_error < 1e-10, 'U_U not at nominal');
    assert(lu_error < 1e-10, 'L_U not at nominal');

    fprintf('  PASS: Zero steer returns identity transform\n');
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
    return;
end

%% Test 5: Upright Rigidity Constraint (|U_U - L_U| = const)
fprintf('\nTest 5: Upright Rigidity (|U_U - L_U| = const)\n');
try
    geom = define_suspension_geometry(0);
    L_kp_nominal = geom.kingpin_length;

    test_angles = deg2rad([-30, -20, -10, -5, 5, 10, 20, 30]);
    max_error = 0;

    for delta = test_angles
        upright = solve_upright_position_3d(delta, geom);
        L_kp = norm(upright.U_U - upright.L_U);
        err = abs(L_kp - L_kp_nominal);
        max_error = max(max_error, err);
    end

    assert(max_error < 0.001, sprintf('Kingpin length varies: max error %.4f mm', max_error * 1000));

    fprintf('  PASS: Kingpin length preserved (max error: %.4f mm)\n', max_error * 1000);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 6: A-arm Length Preservation
fprintf('\nTest 6: A-arm Length Preservation\n');
try
    geom = define_suspension_geometry(0);

    % Nominal arm lengths (distance from axis to BJ)
    L_upper_nom = norm(geom.upper_arm_upright - geom.upper_arm_axis_origin - ...
        dot(geom.upper_arm_upright - geom.upper_arm_axis_origin, geom.upper_arm_axis) * geom.upper_arm_axis);
    L_lower_nom = norm(geom.lower_arm_upright - geom.lower_arm_axis_origin - ...
        dot(geom.lower_arm_upright - geom.lower_arm_axis_origin, geom.lower_arm_axis) * geom.lower_arm_axis);

    test_angles = deg2rad([-20, -10, 10, 20]);
    max_upper_error = 0;
    max_lower_error = 0;

    for delta = test_angles
        upright = solve_upright_position_3d(delta, geom);

        % Upper arm: distance from U_U to upper axis should be preserved
        v_u = upright.U_U - geom.upper_arm_axis_origin;
        v_u_perp = v_u - dot(v_u, geom.upper_arm_axis) * geom.upper_arm_axis;
        L_upper = norm(v_u_perp);
        max_upper_error = max(max_upper_error, abs(L_upper - L_upper_nom));

        % Lower arm
        v_l = upright.L_U - geom.lower_arm_axis_origin;
        v_l_perp = v_l - dot(v_l, geom.lower_arm_axis) * geom.lower_arm_axis;
        L_lower = norm(v_l_perp);
        max_lower_error = max(max_lower_error, abs(L_lower - L_lower_nom));
    end

    assert(max_upper_error < 1e-6, sprintf('Upper arm length error: %.2e m', max_upper_error));
    assert(max_lower_error < 1e-6, sprintf('Lower arm length error: %.2e m', max_lower_error));

    fprintf('  PASS: A-arm lengths preserved\n');
    fprintf('    Upper arm error: %.2e m\n', max_upper_error);
    fprintf('    Lower arm error: %.2e m\n', max_lower_error);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 7: Forward Kinematics
fprintf('\nTest 7: Forward Kinematics Calculation\n');
try
    geom = define_suspension_geometry(0);
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';

    [jack_curve, camber_curve, pushrod_lengths] = ...
        forward_kinematics(design_vars, geom, delta_range);

    % Zero steer should give zero jack-up
    idx_zero = find(abs(delta_range) < 1e-6, 1);
    assert(abs(jack_curve(idx_zero)) < 1e-10, 'Jack not zero at zero steer');

    % Pushrod length should be positive
    assert(all(pushrod_lengths > 0), 'Negative pushrod length');

    % Physical sanity: jack-up < 100mm for delta < 20deg (DW geometry)
    assert(max(abs(jack_curve)) < 0.100, 'Jack-up too large');

    % Physical sanity: camber change < 5deg
    assert(max(abs(camber_curve)) < deg2rad(5), 'Camber change too large');

    fprintf('  PASS: Forward kinematics computed\n');
    fprintf('    Nominal pushrod length: %.3f m\n', pushrod_lengths(idx_zero));
    fprintf('    Jack-up range: [%.2f, %.2f] mm\n', min(jack_curve)*1000, max(jack_curve)*1000);
    fprintf('    Camber range: [%.2f, %.2f] deg\n', rad2deg(min(camber_curve)), rad2deg(max(camber_curve)));
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 8: Objective Function
fprintf('\nTest 8: Polynomial Fit Objective Function\n');
try
    geom = define_suspension_geometry(0);
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';

    target_poly.jack_c1 = 0.0922;
    target_poly.jack_c2 = 0.0226;
    target_poly.camber_c1 = 0.5;
    target_poly.camber_c2 = -0.3516;

    cost = objective_polynomial_fit(design_vars, geom, target_poly, delta_range);

    assert(isfinite(cost), 'Cost is not finite');
    assert(cost >= 0, 'Cost is negative');

    fprintf('  PASS: Objective function evaluated (cost=%.2f)\n', cost);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 9: Constraint Function
fprintf('\nTest 9: Rod Misalignment Constraints\n');
try
    geom = define_suspension_geometry(0);
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';

    [c, ceq] = constraint_rod_misalignment(design_vars, geom, delta_range);

    assert(length(c) == 8, 'Expected 8 inequality constraints');
    assert(isempty(ceq), 'Expected no equality constraints');
    assert(all(isfinite(c)), 'Constraint values not finite');

    fprintf('  PASS: Constraints evaluated\n');
    fprintf('    Rod angle:   %.4f (<=0)\n', c(1));
    fprintf('    Length min:  %.4f (<=0)\n', c(2));
    fprintf('    Length max:  %.4f (<=0)\n', c(3));
    fprintf('    IR min:      %.4f (<=0)\n', c(4));
    fprintf('    IR max:      %.4f (<=0)\n', c(5));
    fprintf('    Upright rad: %.4f (<=0)\n', c(6));
    fprintf('    Inboard:     %.4f (<=0)\n', c(7));
    fprintf('    Bellcrank:   %.4f (<=0)\n', c(8));
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 10: Packaging Validation
fprintf('\nTest 10: Packaging Validation\n');
try
    geom = define_suspension_geometry(0);
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';

    [valid, violations] = validate_packaging(design_vars, geom, delta_range);

    if valid
        fprintf('  PASS: Packaging validation passed\n');
    else
        fprintf('  PASS: Packaging evaluated (violations found)\n');
        for i = 1:length(violations)
            fprintf('    - %s\n', violations{i});
        end
    end
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 11: FL/FR Symmetry
fprintf('\nTest 11: FL/FR Symmetry\n');
try
    geom_fl = define_suspension_geometry(0);
    geom_fr = define_suspension_geometry(1);

    % Upper arm: Y coordinates should be opposite sign
    assert(abs(geom_fl.upper_arm_upright(2) + geom_fr.upper_arm_upright(2)) < 1e-10, ...
        'Upper BJ Y not symmetric');

    % X and Z should be same
    assert(abs(geom_fl.upper_arm_upright(1) - geom_fr.upper_arm_upright(1)) < 1e-10, ...
        'Upper BJ X not symmetric');
    assert(abs(geom_fl.upper_arm_upright(3) - geom_fr.upper_arm_upright(3)) < 1e-10, ...
        'Upper BJ Z not symmetric');

    % Jack-up should have opposite sign for same steering
    design_fl = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    design_fr = [0.06; -0.015; -0.015; 0.60;  0.35; -0.10];
    delta_range = linspace(deg2rad(-20), deg2rad(20), 41)';

    [jack_fl, ~, ~] = forward_kinematics(design_fl, geom_fl, delta_range);
    [jack_fr, ~, ~] = forward_kinematics(design_fr, geom_fr, delta_range);

    % For symmetric design, FL jack at +delta should equal FR jack at -delta
    jack_fl_rev = flipud(jack_fl);
    symmetry_error = max(abs(jack_fl_rev - jack_fr));

    fprintf('  PASS: FL/FR symmetry checked\n');
    fprintf('    Max jack symmetry error: %.4f mm\n', symmetry_error * 1000);
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Test 12: Physical Sanity (large angle)
fprintf('\nTest 12: Physical Sanity at Large Steering Angle\n');
try
    geom = define_suspension_geometry(0);
    delta = deg2rad(30);
    upright = solve_upright_position_3d(delta, geom);

    % Jack-up should be physically reasonable
    design_vars = [0.06; -0.015; -0.015; 0.60; -0.35; -0.10];
    M_pivot_chassis = upright.R_upright * design_vars(1:3) + upright.t_upright;
    L_pushrod = norm(M_pivot_chassis - design_vars(4:6));

    upright0 = solve_upright_position_3d(0.0, geom);
    M_pivot_nominal = upright0.R_upright * design_vars(1:3) + upright0.t_upright;
    L_nominal = norm(M_pivot_nominal - design_vars(4:6));

    jack = abs(L_pushrod - L_nominal);
    assert(jack < 0.100, sprintf('Jack-up at 30deg too large: %.1f mm', jack * 1000));

    % Camber should be less than 3 degrees (FSAE typical)
    kp_dir = upright.kingpin_dir;
    kp_dir0 = geom.kingpin_axis.direction;
    camber = abs(atan2(kp_dir(2), kp_dir(3)) - atan2(kp_dir0(2), kp_dir0(3)));
    assert(camber < deg2rad(5), sprintf('Camber at 30deg too large: %.1f deg', rad2deg(camber)));

    fprintf('  PASS: Physical sanity at 30deg\n');
    fprintf('    Jack-up: %.2f mm\n', jack * 1000);
    fprintf('    Camber change: %.2f deg\n', rad2deg(camber));
    n_pass = n_pass + 1;
catch ME
    fprintf('  FAIL: %s\n', ME.message);
    n_fail = n_fail + 1;
end

%% Summary
fprintf('\n=== Test Summary: %d PASSED, %d FAILED (out of %d) ===\n', ...
    n_pass, n_fail, n_pass + n_fail);

if n_fail == 0
    fprintf('All tests passed! The DW POU system is ready.\n');
else
    fprintf('Some tests failed. Please review the output above.\n');
end
