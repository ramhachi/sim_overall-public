classdef test_driver_attack_sweep_offline < matlab.unittest.TestCase
    properties
        ProjectRoot
        MatlabRoot
        DriverRoot
        WorkspaceRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            tests_root = fileparts(mfilename('fullpath'));
            testCase.MatlabRoot = fileparts(tests_root);
            testCase.ProjectRoot = fileparts(testCase.MatlabRoot);
            testCase.DriverRoot = fullfile(testCase.MatlabRoot, 'driver_generalization');
            testCase.WorkspaceRoot = fullfile(testCase.ProjectRoot, 'output', ...
                'test_artifacts', 'driver_attack_sweep_offline');

            addpath(testCase.MatlabRoot);
            addpath(testCase.DriverRoot);
            reset_dir(testCase.WorkspaceRoot);
        end
    end

    methods (TestClassTeardown)
        function cleanupWorkspace(testCase)
            if exist(testCase.WorkspaceRoot, 'dir')
                rmdir(testCase.WorkspaceRoot, 's');
            end
        end
    end

    methods (Test)
        function testAttackSweepProducesTorqueAndScaleGrid(testCase)
            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'offline_driver_attack';
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'runtime');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.trace_output_dir = fullfile(config.runtime_output_dir, 'traces');
            config.base_params = create_vehicle_baseline_template();
            config.driver_torque_max_values = [25.0, 40.0, 50.0];
            config.torque_speed_scales = [0.78, 0.80];
            config.events = struct( ...
                'name', 'autocross', ...
                'track_name', 'autocross', ...
                'sim_time', 180.0, ...
                'initial_speed', -1.0, ...
                'timeout_sec', 240.0);
            config.execute_case_fn = @mock_attack_case;

            analysis = run_driver_attack_sweep(config);

            testCase.verifyTrue(isfile(analysis.manifest_path));
            testCase.verifyTrue(isfile(analysis.samples_csv_path));
            testCase.verifyTrue(isfile(analysis.summary_csv_path));
            testCase.verifyTrue(isfile(analysis.report_path));

            samples = readtable(analysis.samples_csv_path, 'TextType', 'string');
            testCase.verifyEqual(height(samples), 6);
            testCase.verifyTrue(all(ismember([25.0; 40.0; 50.0], unique(samples.driver_torque_max))));
            testCase.verifyTrue(all(ismember([0.78; 0.80], unique(samples.torque_speed_scale))));
            testCase.verifyTrue(all(ismember("trace_csv_path", samples.Properties.VariableNames)));
            testCase.verifyTrue(all(ismember("accepted", samples.Properties.VariableNames)));
            testCase.verifyTrue(all(ismember("acceptance_reasons", samples.Properties.VariableNames)));
            testCase.verifyTrue(any(samples.driver_torque_max == 40.0 & samples.torque_speed_scale == 0.80));
            testCase.verifyTrue(any(samples.failure_code == "mock_offtrack"));
            testCase.verifyTrue(any(samples.acceptance_reasons == "steering_saturation"));
            testCase.verifyTrue(any(samples.acceptance_reasons == "not_completed;track_limit_excess"));

            summary = readtable(analysis.summary_csv_path, 'TextType', 'string');
            testCase.verifyEqual(height(summary), 3);
            row40 = summary(abs(summary.driver_torque_max - 40.0) < 1e-9, :);
            testCase.verifyEqual(row40.best_torque_speed_scale, 0.80, 'AbsTol', 1e-12);
            testCase.verifyEqual(row40.success_rate, 1.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(row40.accepted_rate, 1.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(row40.best_accepted_torque_speed_scale, 0.80, 'AbsTol', 1e-12);

            row25 = summary(abs(summary.driver_torque_max - 25.0) < 1e-9, :);
            testCase.verifyEqual(row25.accepted_rate, 0.0, 'AbsTol', 1e-12);

            report_text = fileread(analysis.report_path);
            testCase.verifyTrue(contains(report_text, 'Driver Attack Sweep'));
            testCase.verifyTrue(contains(report_text, 'Acceptance Criteria'));
            testCase.verifyTrue(contains(report_text, 'Best Accepted Overall'));
            testCase.verifyTrue(contains(report_text, 'driver_torque_max'));
            testCase.verifyTrue(contains(report_text, '40.000'));
        end
    end
end

function [result, success] = mock_attack_case(case_spec, ~, ~)
result = struct();
result.status = 'success';
result.failure_code = '';
result.failure_message = '';
result.lap_completed = true;
result.lap_time = 100.0 - 0.10 * case_spec.driver_torque_max ...
    - 20.0 * (case_spec.torque_speed_scale - 0.78);
result.distance_traveled = 990.0;
result.max_lateral_error = 1.0;
result.avg_lateral_error = 0.20;
result.max_track_limit_excess = 0.0;
result.max_speed = 22.0;
result.avg_speed = 10.0;
result.rms_heading_error = 5.0;
result.steering_saturation_ratio = max(0.0, 0.30 - 0.005 * case_spec.driver_torque_max);
result.mean_abs_sw_error = 0.20;
result.max_abs_sw_error = 0.80;
result.mean_steering_lag_ratio = 0.12;
result.max_steering_lag_ratio = 0.40;
result.avg_torque_speed_multiplier = case_spec.torque_speed_scale * 0.90;
result.min_steering_actuator_speed_cap = 8.0;
result.min_track_margin_speed_cap = 8.0;
result.min_velocity_profile_steering_cap = 39.0;
result.velocity_profile_steering_cap_ratio = 0.02;
result.max_abs_phi_twist = 0.002;
result.max_abs_dphi_twist = 0.03;

if case_spec.driver_torque_max >= 50.0 && case_spec.torque_speed_scale >= 0.80
    result.status = 'failed';
    result.failure_code = 'mock_offtrack';
    result.failure_message = 'mock high-speed off-track';
    result.lap_completed = false;
    result.lap_time = NaN;
    result.max_lateral_error = 3.0;
    result.max_track_limit_excess = 0.75;
end

success = logical(result.lap_completed);
end

function reset_dir(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
mkdir(path);
end
