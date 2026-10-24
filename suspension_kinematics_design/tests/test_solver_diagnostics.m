classdef test_solver_diagnostics < matlab.unittest.TestCase
    methods (Test)
        function reportsResidualIterationsAndExitflags(tc)
            geom = build_test_geometry();
            metrics = evaluate_double_wishbone_3d(geom, 'UseTieRod', true);

            tc.verifyTrue(metrics.converged);
            tc.verifyLessThanOrEqual(metrics.max_residual_norm, metrics.solver_status.residual_tolerance);
            tc.verifyTrue(isfield(metrics.solver_status, 'travel'));
            tc.verifyTrue(isfield(metrics.solver_status.travel, 'residual_norms'));
            tc.verifyTrue(isfield(metrics.solver_status.travel, 'iterations'));
            tc.verifyTrue(isfield(metrics.solver_status.travel, 'exitflags'));
            tc.verifyEqual(numel(metrics.solver_status.travel.residual_norms), ...
                numel(metrics.travel_samples));
            tc.verifyTrue(metrics.solver_status.branch_continuous);
            tc.verifyEqual(metrics.anti_model, 'side_view_pivot_axes_ic');
            tc.verifyEqual(metrics.jacking_model, 'roll_center_derived_proxy');
            tc.verifyEqual(metrics.compliance_model, 'truss_level1');
            tc.verifyFalse(metrics.anti_validated);
            tc.verifyGreaterThanOrEqual(metrics.anti_dive_pct, 0.0);
            tc.verifyLessThanOrEqual(metrics.anti_dive_pct, 1.0);
            tc.verifyGreaterThanOrEqual(metrics.anti_squat_pct, 0.0);
            tc.verifyFalse(metrics.jacking_validated);
            tc.verifyFalse(metrics.compliance_validated);
        end

        function diagnosticsAreConsistentAtExtremeTravel(tc)
            geom = build_test_geometry();
            metrics = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, ...
                'TravelSamples', [-0.080, 0.0, 0.080], ...
                'RollSamples', deg2rad([-8.0, 0.0, 8.0]), ...
                'ResidualTolerance', 5e-4);

            travel = metrics.solver_status.travel;
            tc.verifyEqual(travel.converged, isempty(travel.failed_samples));
            tc.verifyEqual(travel.converged, ...
                all(travel.residual_norms <= 5e-4));
            tc.verifyEqual(metrics.converged, ...
                travel.converged && metrics.solver_status.roll.converged && ...
                metrics.solver_status.rc_heave.converged && ...
                metrics.solver_status.branch_continuous);
        end

        function detectsBranchDiscontinuity(tc)
            geom = build_test_geometry();
            metrics = evaluate_double_wishbone_3d(geom, ...
                'UseTieRod', true, ...
                'BranchToeTolerance', 1e-12);

            if numel(unique(round(metrics.travel_toe_curve, 12))) > 1
                tc.verifyFalse(metrics.solver_status.branch_continuous);
                tc.verifyFalse(metrics.converged);
            else
                tc.verifyTrue(metrics.solver_status.branch_continuous);
            tc.verifyEqual(metrics.anti_model, 'side_view_pivot_axes_ic');
            tc.verifyEqual(metrics.jacking_model, 'roll_center_derived_proxy');
            tc.verifyEqual(metrics.compliance_model, 'truss_level1');
            tc.verifyFalse(metrics.anti_validated);
            tc.verifyGreaterThanOrEqual(metrics.anti_dive_pct, 0.0);
            tc.verifyLessThanOrEqual(metrics.anti_dive_pct, 1.0);
            tc.verifyGreaterThanOrEqual(metrics.anti_squat_pct, 0.0);
            tc.verifyFalse(metrics.jacking_validated);
            tc.verifyFalse(metrics.compliance_validated);
            end
        end

        function closedLinkSteerSweepUsesRackDisplacement(tc)
            geom = build_test_geometry();
            metrics = evaluate_double_wishbone_3d(geom, 'UseTieRod', true);

            tc.verifyEqual(metrics.solver_status.steer.mode, 'closed_link_rack');
            tc.verifyEqual(numel(metrics.rack_samples), 5);
            tc.verifyEqual(numel(metrics.steer_samples), 5);
            tc.verifyGreaterThan(max(abs(metrics.steer_samples)), 0.1);
            tc.verifyLessThan(max(abs(metrics.toe_vs_steer(2:3))), 0.1);
            tc.verifyTrue(metrics.solver_status.steer_branch_continuous);
        end

        function tieRodChangesTravelToeCurve(tc)
            geom = build_test_geometry();
            without_tie = evaluate_double_wishbone_3d(geom, 'UseTieRod', false);
            with_tie = evaluate_double_wishbone_3d(geom, 'UseTieRod', true);

            tc.verifyGreaterThan(max(abs(with_tie.toe_vs_travel - ...
                without_tie.toe_vs_travel)), 1e-6);
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
