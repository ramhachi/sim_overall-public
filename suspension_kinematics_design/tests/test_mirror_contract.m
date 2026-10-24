classdef test_mirror_contract < matlab.unittest.TestCase
    methods (Test)
        function mirrorFlipsYAndKeepsUnitAxes(tc)
            left = build_test_geometry();
            right = mirror_wishbone_geometry(left);

            tc.verifyFalse(right.is_left);
            tc.verifyEqual(right.W_C(2), -left.W_C(2), 'AbsTol', 1e-12);
            tc.verifyEqual(right.tie_rod_chassis(2), -left.tie_rod_chassis(2), 'AbsTol', 1e-12);
            tc.verifyEqual(right.tie_rod_upright(2), -left.tie_rod_upright(2), 'AbsTol', 1e-12);
            tc.verifyEqual(right.Spindle_Dir(2), -left.Spindle_Dir(2), 'AbsTol', 1e-12);
            tc.verifyEqual(norm(right.upper_arm_axis), 1.0, 'AbsTol', 1e-12);
            tc.verifyEqual(norm(right.lower_arm_axis), 1.0, 'AbsTol', 1e-12);
            tc.verifyEqual(right.kingpin_axis.origin, right.L_U, 'AbsTol', 1e-12);
        end

        function asymmetricStaticAlignmentIsAppliedPerCorner(tc)
            geom = build_test_geometry();
            metrics = evaluate_double_wishbone_3d(geom, ...
                'StaticCamber', deg2rad(-2.0), ...
                'StaticToe', 0.01, ...
                'UseTieRod', true);

            tc.verifyEqual(metrics.camber_deg, -2.0, 'AbsTol', 1e-6);
            tc.verifyEqual(metrics.toe_vs_travel(1), 0.01, 'AbsTol', 1e-4);
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
