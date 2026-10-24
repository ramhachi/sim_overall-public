function test_a_arm_kinematics
% TEST_A_ARM_KINEMATICS - Unit tests for geometrical suspension analytical calculations
% Geometry follows the SAE Z-down convention: ground at Z = 0 and the wheel
% center at Z = -R_tire, so "above ground" means negative Z.

    disp('Running A-Arm Kinematics Tests...');
    R_tire = 0.25;
    ground_z = 0.0;

    % Test Case 1: Parallel horizontal arms (RC at ground, IC at infinity)
    hp_parallel.is_left = true;
    hp_parallel.R_tire  = R_tire;
    hp_parallel.W_C     = [0; 0.6; -R_tire];

    hp_parallel.U_F = [-0.2; 0.2; -0.35];
    hp_parallel.U_R = [ 0.2; 0.2; -0.35];
    hp_parallel.U_U = [ 0.0; 0.5; -0.35]; % Horizontal

    hp_parallel.L_F = [-0.2; 0.2; -0.15];
    hp_parallel.L_R = [ 0.2; 0.2; -0.15];
    hp_parallel.L_U = [ 0.0; 0.5; -0.15]; % Horizontal

    res1 = a_arm_kinematics_solver(hp_parallel);

    disp('Test 1: Parallel Horizontal Arms');
    assert(abs(res1.roll_center_z - ground_z) < 1e-4, 'RC should sit at ground level');
    assert(abs(res1.IC_y) > 1e4, 'IC should be at infinity');
    assert(abs(res1.kpi_deg - 0) < 1e-4, 'KPI should be 0');
    assert(abs(res1.caster_deg - 0) < 1e-4, 'Caster should be 0');
    disp('  -> PASS');

    % Test Case 2: Angled non-parallel arms (finite IC, RC above ground)
    hp_angle = hp_parallel;
    hp_angle.U_F(3) = -0.25; % Upper chassis pivots below the ball joint
    hp_angle.U_R(3) = -0.25;
    hp_angle.L_F(2) = 0.25;  % Lower chassis pivots outboard
    hp_angle.L_R(2) = 0.25;
    hp_angle.L_F(3) = -0.25; % Lower chassis pivots below the ball joint
    hp_angle.L_R(3) = -0.25;

    res2 = a_arm_kinematics_solver(hp_angle);
    disp('Test 2: Angled Converging Arms');
    assert(abs(res2.IC_y - 0.2273) < 1e-3, 'IC should be inboard of the contact patch');
    assert(abs(res2.roll_center_z + 0.4171) < 1e-3, 'RC should be above ground');
    assert(res2.roll_center_z < ground_z, 'RC must be above ground (negative Z)');
    disp(['  -> RC is at Z = ', num2str(res2.roll_center_z)]);
    disp('  -> PASS');

    disp('All tests passed successfully!');
end
