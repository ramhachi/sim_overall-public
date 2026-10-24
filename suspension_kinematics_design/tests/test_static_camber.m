classdef test_static_camber < matlab.unittest.TestCase
    methods (Test)
        function staticCamberShiftsCamberCurves(testCase)
            addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
            geom = build_test_geometry();
            base = evaluate_double_wishbone_3d(geom, 'StaticCamber', 0.0);
            tilted = evaluate_double_wishbone_3d(geom, 'StaticCamber', deg2rad(1.0));

            testCase.verifyTrue(base.converged);
            testCase.verifyTrue(all(isfinite(base.travel_camber_curve)));
            testCase.verifyTrue(all(isfinite(base.steer_camber_curve)));
            testCase.verifyEqual(tilted.camber_deg - base.camber_deg, 1.0, 'AbsTol', 1e-9);
            testCase.verifyEqual(tilted.travel_camber_curve - base.travel_camber_curve, ...
                deg2rad(1.0) * ones(size(base.travel_camber_curve)), 'AbsTol', 1e-9);
            testCase.verifyEqual(tilted.steer_camber_curve, base.steer_camber_curve, ...
                'AbsTol', 1e-9);
            testCase.verifyEqual(tilted.camber_vs_travel(1), deg2rad(1.0), 'AbsTol', 1e-4);
            testCase.verifyEqual(tilted.camber_vs_steer(1), 0.0, 'AbsTol', 1e-4);
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
geom.Spindle_Dir = [0; 1; 0];
end
