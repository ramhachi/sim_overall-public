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
    end
end

function p = production_profile()
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'suspension_kinematics_design'));
artifact = load_vehicle_artifact(fullfile(root, 'matlab', 'parameter_presets', 'params_flexible_torque.json'));
p = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
end
