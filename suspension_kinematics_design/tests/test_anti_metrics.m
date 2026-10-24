classdef test_anti_metrics < matlab.unittest.TestCase
    methods (Test)
        function frontAndRearUseIndependentAntiChannels(tc)
            geom = build_test_geometry();
            front = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, 'BrakeBiasFront', 0.60, 'DriveBiasRear', 0.90);
            geom.is_front = false;
            rear = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, 'BrakeBiasFront', 0.60, 'DriveBiasRear', 0.90);

            tc.verifyEqual(front.anti_brake_pct, front.anti_dive_pct, 'AbsTol', 1e-12);
            tc.verifyEqual(front.anti_drive_pct, front.anti_lift_pct, 'AbsTol', 1e-12);
            tc.verifyEqual(rear.anti_brake_pct, rear.anti_rise_pct, 'AbsTol', 1e-12);
            tc.verifyEqual(rear.anti_drive_pct, rear.anti_squat_pct, 'AbsTol', 1e-12);
            tc.verifyEqual(front.anti_model, 'side_view_ic_with_assumed_bias');
            tc.verifyFalse(front.anti_validated);
        end
    end
end

function geom = build_test_geometry()
geom = struct();
geom.W_C = [0; -0.65; -0.228];
geom.R_tire = 0.228;
geom.is_left = true;
geom.is_front = true;
geom.U_U = [0.02; -0.60; 0.12];
geom.L_U = [-0.01; -0.63; -0.10];
geom.U_F = [0.12; -0.30; 0.12];
geom.U_R = [-0.14; -0.30; 0.12];
geom.L_F = [0.14; -0.28; -0.08];
geom.L_R = [-0.16; -0.28; -0.08];
geom.tie_rod_chassis = [0.12; -0.34; -0.02];
geom.tie_rod_upright = [0.12; -0.62; -0.02];
geom.upper_arm_axis = (geom.U_R - geom.U_F) / norm(geom.U_R - geom.U_F);
geom.lower_arm_axis = (geom.L_R - geom.L_F) / norm(geom.L_R - geom.L_F);
geom.upper_arm_axis_origin = geom.U_F;
geom.lower_arm_axis_origin = geom.L_F;
geom.kingpin_axis = struct('origin', geom.L_U, ...
    'direction', (geom.U_U - geom.L_U) / norm(geom.U_U - geom.L_U));
geom.Spindle_Dir = [0; -1; 0];
end
