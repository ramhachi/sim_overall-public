classdef test_design_sensitivity_offline < matlab.unittest.TestCase
    methods (Test)
        function testDesignVectorRoundTrip(tc)
            artifact = load_vehicle_artifact(fullfile(projectRoot(), ...
                'matlab', 'parameter_presets', 'params_flexible_torque.json'));
            params = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
            [x, schema, context] = build_design_vector_from_params(params);
            rebuilt = apply_full_design(params, x, context);

            tc.verifyEqual(numel(x), 46);
            tc.verifyEqual(numel(schema.variable_names), 46);
            tc.verifyEqual(rebuilt.mass.m_total, params.mass.m_total, 'RelTol', 1e-9);
            tc.verifyEqual(rebuilt.mass.track_f, params.mass.track_f, 'RelTol', 1e-12);
            tc.verifyEqual(rebuilt.mass.track_r, params.mass.track_r, 'RelTol', 1e-12);
            tc.verifyEqual(rebuilt.suspension.k_spring(1), params.suspension.k_spring(1), 'RelTol', 1e-12);
            tc.verifyEqual(rebuilt.suspension.K_chassis, params.suspension.K_chassis, 'RelTol', 1e-12);
            tc.verifyEqual(rebuilt.aero.CL_A, params.aero.CL_A, 'RelTol', 1e-12);
        end

        function testUnknownVariableAbortsBeforeSweep(tc)
            tc.verifyError(@() run_design_sensitivity(struct( ...
                'variable_names', {{'NotAVariable'}})), ...
                'run_design_sensitivity:unknownVariable');
        end
    end
end

function root = projectRoot()
tests_dir = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(tests_dir));
addpath(root);
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'matlab', 'design_sensitivity'));
end
