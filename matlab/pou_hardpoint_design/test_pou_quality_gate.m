classdef test_pou_quality_gate < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addGeometryPath(tc)
            here = fileparts(mfilename('fullpath'));
            addpath(fullfile(here, 'geometry'));
        end
    end

    methods (Test)
        function acceptsEvaluatedCorner(tc)
            params = make_params(scope('evaluated', 'evaluated'), quality('valid'));
            geom = define_suspension_geometry(0, params, false);

            tc.verifyEqual(geom.tie_rod_chassis, [0.10; -0.34; -0.02], 'AbsTol', 1e-12);
            tc.verifyEqual(geom.evaluation_scope.tie_rod, 'evaluated');
        end

        function rejectsTieRodNotEvaluated(tc)
            params = make_params(scope('not_evaluated', 'not_evaluated'), quality('provisional'));

            tc.verifyError(@() define_suspension_geometry(0, params, false), ...
                'POU:TieRodNotEvaluated');
        end

        function rejectsToeNotEvaluated(tc)
            params = make_params(scope('evaluated', 'not_evaluated'), quality('valid'));

            tc.verifyError(@() define_suspension_geometry(0, params, false), ...
                'POU:ToeNotEvaluated');
        end

        function rejectsInvalidQuality(tc)
            params = make_params(scope('evaluated', 'evaluated'), quality('invalid'));

            tc.verifyError(@() define_suspension_geometry(0, params, false), ...
                'POU:InvalidHardpointQuality');
        end

        function rejectsZeroDummyTieRod(tc)
            params = make_params(scope('evaluated', 'evaluated'), quality('valid'));
            params.params.suspension.kc.corners.FL.hardpoints.tie_rod_inner = [0; 0; 0];
            params.params.suspension.kc.corners.FL.hardpoints.tie_rod_outer = [0; 0; 0];

            tc.verifyError(@() define_suspension_geometry(0, params, false), ...
                'POU:MissingTieRodHardpoints');
        end

        function explicitOverrideAllowsMissingTieRod(tc)
            params = make_params(scope('not_evaluated', 'not_evaluated'), quality('provisional'));
            params.params.suspension.kc.corners.FL.hardpoints.tie_rod_inner = [0; 0; 0];
            params.params.suspension.kc.corners.FL.hardpoints.tie_rod_outer = [0; 0; 0];

            tc.verifyWarning(@() define_suspension_geometry(0, params, true), ...
                'POU:MissingTieRodHardpoints');
        end
    end
end

function s = scope(tie_rod, toe)
s = struct('steering', tie_rod, 'tie_rod', tie_rod, 'toe', toe, ...
    'roll_steer', tie_rod, 'travel', 'evaluated', 'roll', 'evaluated');
end

function q = quality(verdict)
q = struct('verdict', verdict, 'confidence', 'high', 'reasons', {{}});
end

function params = make_params(scope, quality)
hp = struct();
hp.upper_arm_chassis_fore = [0.12, -0.30, 0.12];
hp.upper_arm_chassis_aft = [-0.14, -0.30, 0.12];
hp.lower_arm_chassis_fore = [0.14, -0.28, -0.08];
hp.lower_arm_chassis_aft = [-0.16, -0.28, -0.08];
hp.upper_ball_joint = [0.02, -0.60, 0.12];
hp.lower_ball_joint = [-0.01, -0.63, -0.10];
hp.wheel_center = [0, -0.65, -0.228];
hp.contact_patch = [0, -0.65, 0];
hp.tie_rod_inner = [0.10, -0.34, -0.02];
hp.tie_rod_outer = [0.10, -0.62, -0.02];

corner = struct();
corner.hardpoints = hp;
corner.evaluation_scope = scope;
corner.quality = quality;
corner.source = struct('model', 'a_arm_plus_tie_rod', ...
    'tie_rod_model', 'closed_link');

params = struct();
params.params.suspension.kc.corners.FL = corner;
params.params.mass.track_f = 1.20;
params.params.mass.track_r = 1.18;
params.params.mass.wheelbase = 1.55;
end
