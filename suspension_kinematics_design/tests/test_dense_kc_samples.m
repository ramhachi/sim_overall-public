classdef test_dense_kc_samples < matlab.unittest.TestCase
    methods (Test)
        function denseSweepStoresSamplesAndChecks(tc)
            geom = build_test_geometry();
            dense = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, ...
                'TravelSamples', linspace(-0.05, 0.05, 11), ...
                'RollSamples', deg2rad(linspace(-5.0, 5.0, 7)), ...
                'RackSamples', linspace(-0.06, 0.06, 7));

            tc.verifyTrue(dense.converged);
            tc.verifyEqual(numel(dense.sample_values.travel), 11);
            tc.verifyEqual(numel(dense.sample_values.roll), 7);
            tc.verifyEqual(numel(dense.sample_values.steer), 7);
            tc.verifyEqual(dense.kc_checks.travel_sample_count, 11);
            tc.verifyEqual(dense.kc_checks.roll_sample_count, 7);
            tc.verifyEqual(dense.kc_checks.steer_sample_count, 7);
            tc.verifyTrue(dense.kc_checks.travel_residual_ok);
            tc.verifyTrue(dense.kc_checks.roll_residual_ok);
            tc.verifyTrue(dense.kc_checks.steer_residual_ok);
            tc.verifyTrue(isfield(dense.polyfit_quality, 'fit_ok'));
            tc.verifyTrue(isfinite(dense.polyfit_quality.travel_toe_max_error_rad));
            tc.verifyGreaterThanOrEqual(dense.kc_checks.travel_toe_extrema_count, 0);
            tc.verifyGreaterThanOrEqual(dense.kc_checks.travel_camber_extrema_count, 0);
            tc.verifyEqual(dense.fsolve_calls, 11 + 2 * 11 + 2 * 7 + 7);
            tc.verifyGreaterThanOrEqual(dense.elapsed_s, 0.0);
        end

        function coarseAndDensePolynomialsCanBeCompared(tc)
            geom = build_test_geometry();
            coarse = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, ...
                'TravelSamples', [-0.03, 0.0, 0.03], ...
                'RollSamples', deg2rad([-3.0, 0.0, 3.0]), ...
                'RackSamples', [-0.05, 0.0, 0.05]);
            dense = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, ...
                'TravelSamples', linspace(-0.03, 0.03, 9), ...
                'RollSamples', deg2rad(linspace(-3.0, 3.0, 5)), ...
                'RackSamples', linspace(-0.05, 0.05, 5));

            x = -dense.travel_samples;
            coarse_toe = polyval(fliplr(coarse.toe_vs_travel), x);
            dense_toe = polyval(fliplr(dense.toe_vs_travel), x);
            diff = max(abs(coarse_toe - dense_toe));

            tc.verifyTrue(isfinite(diff));
            tc.verifyEqual(numel(dense.travel_samples), 9);
            tc.verifyEqual(numel(coarse.travel_samples), 3);
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
