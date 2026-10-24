classdef test_roll_residual_gains < matlab.unittest.TestCase
    % Pure roll (heave 0) must be reproduced by camber/toe_vs_travel at the
    % roll-induced travel; the exported roll gains are only the residual (#38).
    methods (Test)
        function pureRollIsCoveredByTravelCurves(tc)
            geom = build_test_geometry();
            track = 1.2;
            m = evaluate_double_wishbone_3d(geom, 'UseTieRod', true, 'TrackWidth', track, ...
                'TravelSamples', linspace(-0.04, 0.04, 21), ...
                'RollSamples', deg2rad(linspace(-3, 3, 13)));
            tc.verifyTrue(m.converged);
            tc.verifyEqual(m.roll_gain_definition, 'residual_after_travel_curve');

            % Runtime model: camber = camber_vs_travel(travel) + roll_camber_gain*phi
            phi = m.roll_samples;
            travel = -0.5 * track * phi;              % compression-positive
            runtimeCamber = polyval(fliplr(m.camber_vs_travel), travel) + m.roll_camber_gain * phi;
            runtimeToe = polyval(fliplr(m.toe_vs_travel), travel) + m.roll_steer_gain * phi;
            solvedCamber = [m.sample_values.roll.left_camber_rad];
            solvedToe = [m.sample_values.roll.left_toe_rad];
            tc.verifyEqual(runtimeCamber, solvedCamber, 'AbsTol', 2e-4);
            tc.verifyEqual(runtimeToe, solvedToe, 'AbsTol', 2e-4);

            % Residual is tiny; the old total gain is the double-counted term.
            tc.verifyLessThan(abs(m.roll_camber_gain), 0.01 * abs(m.roll_camber_gain_total));
            tc.verifyEqual(m.roll_camber_gain_total, -m.camber_vs_travel(2) * 0.5 * track, ...
                'RelTol', 0.05);
        end

        function derivedRollCamberTargetIsZeroResidual(tc)
            targets = build_inverse_targets(create_vehicle_baseline_template(), 'front');
            tc.verifyEqual(targets.roll_camber_gain, 0.0);
            tc.verifyEqual(targets.sources.roll_camber_gain, 'derived_residual_after_camber_vs_travel');
        end
    end
end

function geom = build_test_geometry()
geom = struct();
geom.W_C = [0.75; -0.60; -0.228];
geom.R_tire = 0.228;
geom.is_left = true;
geom.is_front = true;
geom.U_U = [0.755; -0.55; -0.33];
geom.L_U = [0.75; -0.58; -0.11];
geom.U_F = [0.88; -0.30; -0.29];
geom.U_R = [0.62; -0.30; -0.29];
geom.L_F = [0.88; -0.25; -0.10];
geom.L_R = [0.62; -0.25; -0.10];
geom.tie_rod_chassis = [0.85; -0.27; -0.15];
geom.tie_rod_upright = [0.85; -0.57; -0.15];
geom.Spindle_Dir = [0; -1; 0];
geom.upper_arm_axis = (geom.U_R - geom.U_F) / norm(geom.U_R - geom.U_F);
geom.lower_arm_axis = (geom.L_R - geom.L_F) / norm(geom.L_R - geom.L_F);
geom.upper_arm_axis_origin = geom.U_F;
geom.lower_arm_axis_origin = geom.L_F;
geom.kingpin_axis = struct('origin', geom.L_U, ...
    'direction', (geom.U_U - geom.L_U) / norm(geom.U_U - geom.L_U));
end
