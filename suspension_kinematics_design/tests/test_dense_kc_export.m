classdef test_dense_kc_export < matlab.unittest.TestCase
    % Dense export sampling, persisted sample arrays and loaded contact patch (#40).
    methods (Test)
        function denseKnotsCoverBumpStopRange(tc)
            p = create_vehicle_baseline_template();
            s = kc_dense_samples(p, suspension_design_space(p, 'front'));
            half = p.suspension.bump_stop_gap + 0.010;
            tc.verifyEqual([min(s.TravelSamples), max(s.TravelSamples)], [-half, half], 'AbsTol', 1e-12);
            tc.verifyLessThanOrEqual(max(diff(s.TravelSamples)), 0.002 + 1e-12);
            tc.verifyLessThanOrEqual(max(diff(s.RollSamples)), deg2rad(0.5) + 1e-12);
            tc.verifyLessThanOrEqual(max(diff(s.SteerSamples)), deg2rad(1.0) + 1e-12);
            tc.verifyTrue(any(abs(s.TravelSamples) < 1e-12));
        end

        function cornerBlockPersistsAscendingSampleArrays(tc)
            geom = build_test_geometry();
            m = evaluate_double_wishbone_3d(geom, 'UseTieRod', true, ...
                'TravelSamples', linspace(-0.04, 0.04, 41), 'RollSamples', deg2rad(-4:0.5:4));
            corner = build_corner_kc(geom, m, 0.0, 0.0, 'test');
            sv = corner.sample_values;
            tc.verifyEqual(numel(sv.travel_m), 41);
            tc.verifyTrue(all(diff(sv.travel_m) > 0));
            for f = {'camber_rad', 'toe_rad', 'caster_rad', 'kpi_rad', 'track_change_m', ...
                    'rc_height_m', 'jacking_force_per_fy', 'anti_brake', 'anti_drive'}
                tc.verifySize(sv.(f{1}), [1, 41], f{1});
                tc.verifyTrue(all(isfinite(sv.(f{1}))), f{1});
            end
            tc.verifyTrue(all(diff(sv.roll_rad) > 0));
            % Static knot reproduces the static values (fsolve residual ~1e-7).
            k0 = find(abs(sv.travel_m) < 1e-12, 1);
            tc.verifyEqual(sv.caster_rad(k0), corner.caster, 'AbsTol', 1e-6);
            tc.verifyEqual(sv.kpi_rad(k0), corner.kpi, 'AbsTol', 1e-6);
            tc.verifyEqual(sv.track_change_m(k0), 0.0, 'AbsTol', 1e-6);
            % JSON round-trip keeps plain arrays.
            back = jsondecode(jsonencode(sv));
            tc.verifyEqual(numel(back.travel_m), 41);
            tc.verifyEqual(corner.quality.verdict, 'valid');
        end

        function contactPatchUsesLoadedRadiusAndCamberShift(tc)
            geom = build_test_geometry();
            gamma = deg2rad(-2.0);   % negative camber: wheel top inboard
            geom.Spindle_Dir = [0; -cos(gamma); sin(gamma)];
            geom.R_loaded = geom.R_tire - 0.004;
            m = evaluate_double_wishbone_3d(geom, 'UseTieRod', false, ...
                'TravelSamples', 0.0, 'RollSamples', 0.0, 'SteerSamples', 0.0);
            cp = m.contact_patch;
            tc.verifyEqual(norm(cp - geom.W_C), geom.R_loaded, 'AbsTol', 1e-12);
            % Lateral shift R*sin(|gamma|); for a left wheel with negative camber the
            % contact patch moves outboard (more negative Y).
            tc.verifyEqual(cp(2) - geom.W_C(2), -geom.R_loaded * sin(abs(gamma)), 'AbsTol', 1e-9);
            tc.verifyEqual(cp(3) - geom.W_C(3), geom.R_loaded * cos(gamma), 'AbsTol', 1e-9);
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
