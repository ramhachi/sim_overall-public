classdef test_compliance_sign < matlab.unittest.TestCase
    methods (Test)
        function mirrorFlipsSignedCamberCompliance(tc)
            left = build_test_geometry();
            right = mirror_wishbone_geometry(left);
            left_metrics = evaluate_double_wishbone_3d(left, 'UseTieRod', true);
            right_metrics = evaluate_double_wishbone_3d(right, 'UseTieRod', true);

            tc.verifyFalse(left_metrics.compliance.validated);
            tc.verifyEqual(left_metrics.compliance.sign_convention, ...
                'positive Fx/Fy in SAE axes; upper-minus-lower axial compliance');
            tc.verifyEqual(right_metrics.compliance.camber_per_load(2), ...
                -left_metrics.compliance.camber_per_load(2), 'AbsTol', 1e-12);
            tc.verifyEqual(right_metrics.compliance.toe_per_load(1), ...
                left_metrics.compliance.toe_per_load(1), 'AbsTol', 1e-12);
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
