classdef test_axle_geometry < matlab.unittest.TestCase
    methods (Test)
        function derivesWheelCenterFromMassAndTire(tc)
            params = create_vehicle_baseline_template();
            axle = resolve_axle_geometry(params, struct(), 'front');

            % Wheel centre sits at the loaded radius: R - Fz_static/k_tire (#40).
            fz = 0.5 * params.mass.m_total * 9.81 * params.mass.b / params.mass.wheelbase;
            rLoaded = params.tire.radius - fz / params.tire.k_tire;
            tc.verifyEqual(axle.W_C, [params.mass.a; -params.mass.track_f / 2; -rLoaded], ...
                'AbsTol', 1e-12);
            tc.verifyEqual(axle.R_loaded, rLoaded, 'AbsTol', 1e-12);
            tc.verifyEqual(axle.R_tire, params.tire.radius);
            tc.verifyEqual(axle.arm_x_offset, params.suspension.arm_chassis_x_offset);
        end

        function usesRearAxleFromWheelbase(tc)
            params = create_vehicle_baseline_template();
            axle = resolve_axle_geometry(params, struct(), 'rear');

            tc.verifyEqual(axle.W_C(1), -params.mass.b, 'AbsTol', 1e-12);
        end

        function explicitInputsOverrideDerivedGeometry(tc)
            params = create_vehicle_baseline_template();
            options = struct('wheel_center', [0.1; -0.5; -0.2], ...
                'tire_radius', 0.25, 'arm_x_offset', 0.16);
            axle = resolve_axle_geometry(params, options, 'front');

            tc.verifyEqual(axle.W_C, options.wheel_center, 'AbsTol', 1e-12);
            tc.verifyEqual(axle.R_tire, 0.25);
            tc.verifyEqual(axle.arm_x_offset, 0.16);
            tc.verifyEqual(axle.source.wheel_center, 'run_options');
        end

        function wheelbaseAndTireRadiusChangeResolvedGeometry(tc)
            params = create_vehicle_baseline_template();
            base = resolve_axle_geometry(params, struct(), 'front');

            params.mass.a = 0.80;
            params.tire.radius = 0.25;
            changed = resolve_axle_geometry(params, struct(), 'front');

            tc.verifyNotEqual(changed.W_C(1), base.W_C(1));
            tc.verifyNotEqual(changed.W_C(3), base.W_C(3));
        end

        function artifactHardpointsOverrideDefaults(tc)
            params = create_vehicle_baseline_template();
            hp = struct();
            hp.wheel_center = [0.0, -0.8, -0.22];
            hp.upper_arm_chassis_fore = [0.2, -0.3, -0.2];
            hp.upper_arm_chassis_aft = [-0.2, -0.3, -0.2];
            params.suspension.kc.corners.FL.hardpoints = hp;

            axle = resolve_axle_geometry(params, struct(), 'front');

            tc.verifyEqual(axle.W_C, [0; -0.8; -0.22], 'AbsTol', 1e-12);
            tc.verifyEqual(axle.arm_x_offset, 0.2, 'AbsTol', 1e-12);
            tc.verifyEqual(axle.source.wheel_center, 'artifact_hardpoints');
        end
    end
end
