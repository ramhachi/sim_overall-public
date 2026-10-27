classdef test_compute_vehicle_kc < matlab.unittest.TestCase
    % K&C is recomputed from the vehicle profile with one shared routine.
    methods (Test)
        function baselineReproducesStoredKcAndMirrorsRightCorners(tc)
            p = production_profile();
            q = compute_vehicle_kc(p);
            for n = {'FL', 'RL', 'RR'}
                a = p.suspension.kc.corners.(n{1}).sample_values;
                b = q.suspension.kc.corners.(n{1}).sample_values;
                tc.verifyEqual(b.camber_rad(:), a.camber_rad(:), 'AbsTol', 1e-9);
                tc.verifyEqual(b.roll_center_height_m(:), a.roll_center_height_m(:), 'AbsTol', 1e-9);
            end
            % Right corners are exact Y-mirrors: same physical camber curve.
            c = q.suspension.kc.corners;
            tc.verifyEqual(c.FR.sample_values.camber_rad, c.FL.sample_values.camber_rad, 'AbsTol', 1e-8);
            tc.verifyEqual(c.RR.sample_values.camber_rad, c.RL.sample_values.camber_rad, 'AbsTol', 1e-8);
        end

        function wideTrackMovesUprightsAndKeepsChassisPickups(tc)
            p = production_profile();
            p.mass.track_f = 1.90;
            p.mass.wheelbase = p.mass.wheelbase + 0.30;
            p.mass.a = p.mass.a + 0.30;
            q = compute_vehicle_kc(p);
            hp0 = p.suspension.kc.corners.FL.hardpoints;
            hp = q.suspension.kc.corners.FL.hardpoints;
            tc.verifyEqual(hp.wheel_center(2), -0.95, 'AbsTol', 1e-12);
            tc.verifyEqual(hp.wheel_center(1), p.mass.a, 'AbsTol', 1e-12);
            tc.verifyEqual(hp.upper_arm_chassis_fore(2), hp0.upper_arm_chassis_fore(2), 'AbsTol', 1e-12);
            tc.verifyEqual(hp.upper_arm_chassis_fore(1), hp0.upper_arm_chassis_fore(1) + 0.30, 'AbsTol', 1e-12);
            % Longer arms: less camber gain than the 1.2 m baseline.
            tc.verifyLessThan(abs(q.suspension.kc.corners.FL.kinematics.camber_vs_travel(2)), ...
                abs(p.suspension.kc.corners.FL.kinematics.camber_vs_travel(2)));
        end

        function hardpointOffsetsAreMirroredAndIdempotent(tc)
            p = production_profile();
            p.suspension.hardpoint_offsets.front = struct('lower_inner_up', 0.05, 'upper_inner_out', -0.03);
            q = compute_vehicle_kc(p);
            q2 = compute_vehicle_kc(q);
            hp0 = p.suspension.kc.corners;
            hp = q.suspension.kc.corners;
            % Z down: +50 mm up is -0.05 in Z; outboard on FR is +Y, on FL -Y.
            tc.verifyEqual(hp.FL.hardpoints.lower_arm_chassis_aft(3), ...
                hp0.FL.hardpoints.lower_arm_chassis_aft(3) - 0.05, 'AbsTol', 1e-12);
            tc.verifyEqual(hp.FR.hardpoints.upper_arm_chassis_fore(2), ...
                hp0.FR.hardpoints.upper_arm_chassis_fore(2) - 0.03, 'AbsTol', 1e-12);
            tc.verifyEqual(hp.FL.hardpoints.upper_arm_chassis_fore(2), ...
                hp0.FL.hardpoints.upper_arm_chassis_fore(2) + 0.03, 'AbsTol', 1e-12);
            tc.verifyEqual(hp.RL.hardpoints.lower_arm_chassis_aft(:), ...
                hp0.RL.hardpoints.lower_arm_chassis_aft(:), 'AbsTol', 1e-12);
            tc.verifyEqual(q2.suspension.kc.corners.FL.hardpoints, hp.FL.hardpoints);
            tc.verifyGreaterThan(q.suspension.kc.front.roll_center_height, ...
                p.suspension.kc.front.roll_center_height + 0.05);
        end

        function kcDeltasShiftCurvesConsistently(tc)
            p = production_profile();
            base = compute_vehicle_kc(p);
            p.suspension.kc_deltas.front = struct('rc_height', 0.05, 'camber_gain', -0.3, ...
                'bump_steer', 0.2, 'anti_drive', 0.1);
            q = compute_vehicle_kc(p);
            q2 = compute_vehicle_kc(q);
            ht = 0.5 * p.mass.track_f;
            a = base.suspension.kc.corners;
            b = q.suspension.kc.corners;
            t = b.FL.sample_values.travel_m(:);
            tc.verifyEqual(q.suspension.kc.front.roll_center_height, ...
                base.suspension.kc.front.roll_center_height + 0.05, 'AbsTol', 1e-12);
            tc.verifyEqual(b.FR.sample_values.jacking_force_per_fy(:), ...
                a.FR.sample_values.jacking_force_per_fy(:) - 0.05 / ht, 'AbsTol', 1e-12);
            tc.verifyEqual(b.FL.sample_values.track_change_m(:), ...
                a.FL.sample_values.track_change_m(:) + 0.05 / ht * t, 'AbsTol', 1e-12);
            tc.verifyEqual(b.FR.sample_values.camber_rad(:), ...
                a.FR.sample_values.camber_rad(:) - 0.3 * t, 'AbsTol', 1e-12);
            % Toe is stored left-positive: bump steer flips sign on the right.
            tc.verifyEqual(b.FR.sample_values.toe_rad(:), ...
                a.FR.sample_values.toe_rad(:) - 0.2 * t, 'AbsTol', 1e-12);
            tc.verifyEqual(b.RL.sample_values.camber_rad(:), a.RL.sample_values.camber_rad(:), 'AbsTol', 1e-15);
            tc.verifyEqual(q2.suspension.kc.corners.FL.sample_values.camber_rad(:), ...
                b.FL.sample_values.camber_rad(:), 'AbsTol', 1e-15);
        end
    end
end

function p = production_profile()
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'suspension_kinematics_design'));
artifact = load_vehicle_artifact(fullfile(root, 'matlab', 'parameter_presets', 'params_flexible_torque.json'));
p = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
end
