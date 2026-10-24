classdef test_compliance_sign < matlab.unittest.TestCase
    % Level-1 truss compliance (#41) and the legacy proxy flag.
    methods (Test)
        function mirrorFlipsSignedCamberCompliance(tc)
            left = build_test_geometry();
            right = mirror_wishbone_geometry(left);
            l = evaluate_double_wishbone_3d(left, 'UseTieRod', true).compliance;
            r = evaluate_double_wishbone_3d(right, 'UseTieRod', true).compliance;

            tc.verifyEqual(l.model, 'truss_level1');
            tc.verifyFalse(l.validated);
            tc.verifyTrue(l.provisional);
            tc.verifyEqual(l.scope, 'model_default_stiffness');
            tc.verifyEqual(r.camber_per_load(2), -l.camber_per_load(2), 'RelTol', 1e-6);
            tc.verifyEqual(r.track_per_load(2), -l.track_per_load(2), 'RelTol', 1e-6);
            % toe is wheel yaw (nose right +, as runtime total_yaw): Fx is mirror-
            % invariant so yaw flips; Mz flips with the mirror so yaw does not.
            tc.verifyEqual(r.toe_per_load(1), -l.toe_per_load(1), 'RelTol', 1e-6);
            tc.verifyEqual(r.toe_per_load(6), l.toe_per_load(6), 'RelTol', 1e-6);
        end

        function fullMatrixWithPhysicalSigns(tc)
            c = evaluate_double_wishbone_3d(build_test_geometry(), 'UseTieRod', true).compliance;
            % Inboard ground force on a left wheel (+Fy) tips the top outboard.
            tc.verifyGreaterThan(c.camber_per_load(2), 0.0);
            % ... and moves the contact patch inboard.
            tc.verifyLessThan(c.track_per_load(2), 0.0);
            % Every row is populated (not just Fy->camber / Fx->toe / Mz->toe).
            for f = {'toe_per_load', 'camber_per_load', 'caster_per_load', 'track_per_load', 'wheelbase_per_load'}
                tc.verifySize(c.(f{1}), [1, 6]);
                tc.verifyGreaterThan(nnz(abs(c.(f{1})) > 1e-12), 2, f{1});
            end
        end

        function complianceScalesInverselyWithStiffness(tc)
            geom = build_test_geometry();
            base = struct('youngs_modulus', 2e11, 'rod_end_stiffness_n_per_m', 3e7, ...
                'ball_joint_stiffness_n_per_m', 1e8, 'ride_restraint_n_per_m', 1e9);
            stiff = structfun(@(v) 10 * v, base, 'UniformOutput', false);
            stiff.stiffness_source = 'test_supplied';
            a = evaluate_double_wishbone_3d(geom, 'UseTieRod', true, 'ComplianceModel', base, ...
                'SteeringStiffness', 5000).compliance;
            b = evaluate_double_wishbone_3d(geom, 'UseTieRod', true, 'ComplianceModel', stiff, ...
                'SteeringStiffness', 50000).compliance;
            % AbsTol: the Fz column is ~0 (load line passes through the ride restraint)
            % and only carries round-off from the stiff 1e9..1e10 N/m system.
            tc.verifyEqual(b.camber_per_load, a.camber_per_load / 10, 'RelTol', 1e-4, ...
                'AbsTol', 1e-2 * max(abs(a.camber_per_load)));
            tc.verifyEqual(b.toe_per_load, a.toe_per_load / 10, 'RelTol', 1e-4, ...
                'AbsTol', 1e-2 * max(abs(a.toe_per_load)));
            tc.verifyFalse(b.provisional);
            tc.verifyEqual(b.scope, 'model_supplied_stiffness');
        end

        function legacyProxyIsFlaggedProvisional(tc)
            m = evaluate_double_wishbone_3d(build_test_geometry(), 'UseTieRod', true, ...
                'ComplianceModel', struct('model', 'proxy_axial'));
            tc.verifyEqual(m.compliance_model, 'arm_axial_stiffness_proxy');
            tc.verifyTrue(m.compliance.provisional);
            tc.verifyEqual(m.compliance.scope, 'proxy_not_validated');
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
