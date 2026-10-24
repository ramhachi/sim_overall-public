classdef test_target_provisional_weights < matlab.unittest.TestCase
    methods (Test)
        function fallbackTargetsGetZeroWeight(testCase)
            addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
            params = create_vehicle_baseline_template();
            targets = build_inverse_targets(params, 'front');

            testCase.verifyTrue(targets.provisional);
            testCase.verifyEqual(targets.model_version, 'suspension_inverse_targets/v1');
            testCase.verifyEqual(targets.weight_policy, 'tolerance_normalized');
            testCase.verifyEqual(targets.tolerances.roll_center_z, 0.010);
            testCase.verifyEqual(targets.quality.confidence, 'low');
            testCase.verifyTrue(targets.quality.do_not_use_for_final_design);
            testCase.verifyGreaterThan(targets.quality.source_counts.fallback, 0);
            testCase.verifyEqual(targets.source_quality.roll_center_z.kind, 'fallback');
            testCase.verifyEqual(targets.source_quality.roll_center_z.confidence, 0.0);
            testCase.verifyTrue(targets.source_quality.roll_center_z.provisional);
            testCase.verifyFalse(targets.source_quality.roll_center_z.required);
            testCase.verifyEqual(targets.source_quality.camber_vs_travel.kind, 'design_target');
            testCase.verifyTrue(targets.source_quality.camber_vs_travel.required);
            testCase.verifyEqual(targets.weights.roll_center_z, 0.0);
            testCase.verifyEqual(targets.weights.kpi, 0.0);
            testCase.verifyEqual(targets.weights.caster, 0.0);
            testCase.verifyEqual(targets.weights.scrub, 0.0);
            testCase.verifyEqual(targets.weights.mechanical_trail, 0.0);
            testCase.verifyEqual(targets.weights.compliance, 0.0);
            testCase.verifyEqual(targets.weights.jacking, 0.0);
            testCase.verifyGreaterThan(targets.weights.camber_gain, 0.0);
        end

        function explicitDesignTargetRestoresWeight(testCase)
            addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
            params = create_vehicle_baseline_template();
            params.suspension.design_targets.front.roll_center_z = -0.03;
            targets = build_inverse_targets(params, 'front');

            testCase.verifyEqual(targets.weights.roll_center_z, 1.0);
            testCase.verifyEqual(targets.source_quality.roll_center_z.kind, 'design_target');
            testCase.verifyEqual(targets.source_quality.roll_center_z.confidence, 0.8);
            testCase.verifyTrue(targets.source_quality.roll_center_z.required);
            testCase.verifyTrue(any(strcmp(targets.fallback_fields, 'kpi')));
        end
    end
end
