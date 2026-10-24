classdef test_validate_vehicle_parameter_artifact < matlab.unittest.TestCase
    properties
        MatlabRoot
    end

    methods (TestClassSetup)
        function addMatlabRoot(tc)
            tests_root = fileparts(mfilename('fullpath'));
            tc.MatlabRoot = fileparts(tests_root);
            addpath(tc.MatlabRoot);
        end
    end

    methods (Test)
        function testCanonicalArtifactPasses(tc)
            artifact = tc.makeCanonicalArtifact();

            report = validate_vehicle_parameter_artifact(artifact);

            tc.verifyTrue(report.is_valid);
            tc.verifyEmpty(report.issues);
        end

        function testSameSignToeFails(tc)
            artifact = tc.makeCanonicalArtifact();
            artifact.params.suspension.static_toe(1:2) = [-0.02, -0.02];

            report = validate_vehicle_parameter_artifact(artifact);

            tc.verifyFalse(report.is_valid);
            tc.verifyTrue(tc.hasIssue(report, 'static_toe_not_mirrored'));
        end

        function testAsymmetricPouFails(tc)
            artifact = tc.makeCanonicalArtifact();
            artifact.params.suspension.pou_jack_c1(1:2) = [0.02, 0.01];

            report = validate_vehicle_parameter_artifact(artifact);

            tc.verifyFalse(report.is_valid);
            tc.verifyTrue(tc.hasIssue(report, 'pou_not_front_symmetric'));
        end

        function testLegacyDesignVectorFails(tc)
            artifact = tc.makeCanonicalArtifact();
            artifact.design_vector = artifact.design_vector(1:30);
            artifact.variable_names = artifact.variable_names(1:30);

            report = validate_vehicle_parameter_artifact(artifact);

            tc.verifyFalse(report.is_valid);
            tc.verifyTrue(tc.hasIssue(report, 'design_vector_length_mismatch'));
        end

        function testTrackedWingedPresetPasses(tc)
            preset_path = fullfile(tc.MatlabRoot, 'parameter_presets', 'params_winged_flexible.json');

            report = validate_vehicle_parameter_artifact(preset_path);

            tc.verifyTrue(report.is_valid);
            tc.verifyEmpty(report.issues);
        end
    end

    methods (Access = private)
        function artifact = makeCanonicalArtifact(~)
            schema = build_full_vehicle_design_schema();
            design_vector = mean(schema.variable_bounds, 2)';
            params = apply_full_design(create_vehicle_baseline_template(), design_vector, ...
                struct('extra_tunable_specs', schema.extra_tunable_specs));
            artifact = struct( ...
                'params', params, ...
                'design_vector', design_vector, ...
                'variable_names', {schema.variable_names});
        end

        function tf = hasIssue(~, report, code)
            tf = any(strcmp({report.issues.code}, code));
        end
    end
end
