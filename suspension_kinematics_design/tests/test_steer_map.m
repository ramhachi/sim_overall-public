classdef test_steer_map < matlab.unittest.TestCase
    % Rack x wheel-travel steer map from the closed-link solve (#43).
    methods (Test)
        function mapMatchesRackSweepAndMirror(tc)
            left = build_test_geometry();
            rack = linspace(-0.03, 0.03, 13);
            dz = linspace(-0.02, 0.02, 5);
            args = {'UseTieRod', true, 'RackSamples', rack, 'SteerMapTravelSamples', dz, ...
                'TravelSamples', linspace(-0.02, 0.02, 5), 'RollSamples', 0.0};
            ml = evaluate_double_wishbone_3d(left, args{:});
            mr = evaluate_double_wishbone_3d(mirror_wishbone_geometry(left), args{:});
            L = ml.steer_map;
            tc.verifyTrue(L.converged);
            tc.verifySize(L.steer_rad, [5, 13]);
            tc.verifyTrue(all(diff(L.wheel_travel_m) > 0));
            % Bump steer removed: the rack-centre column is zero.
            tc.verifyEqual(L.steer_rad(:, 7), zeros(5, 1), 'AbsTol', 1e-12);
            % Static-travel row equals the 1D rack sweep.
            k0 = find(abs(L.wheel_travel_m) < 1e-12, 1);
            tc.verifyEqual(L.steer_rad(k0, :), ml.steer_samples, 'AbsTol', 1e-4);  % fsolve tol 5e-4
            % Monotonic in rack at every travel.
            tc.verifyTrue(all(all(diff(L.steer_rad, 1, 2) > 0)));
            % Mirror: delta_R(rack) = -delta_L(-rack) (same rack moves both sides +Y).
            tc.verifyEqual(mr.steer_map.steer_rad, -fliplr(L.steer_rad), 'AbsTol', 1e-4);
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
