classdef test_hybrid_driver_comparison_offline < matlab.unittest.TestCase
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
                'test_artifacts', 'hybrid_driver_comparison');

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
        function testComparisonAggregatesDriverVariants(testCase)
            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'offline_hybrid_comparison';
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'runtime');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.sweep = struct();
            config.sweep.base_params = create_vehicle_baseline_template();
            config.sweep.parameter_names = {'DriverTorqueMax'};
            config.sweep.events = struct( ...
                'name', 'autocross', ...
                'track_name', 'autocross', ...
                'sim_time', 180.0, ...
                'initial_speed', -1.0, ...
                'timeout_sec', 240.0);
            config.sweep.execute_event_fn = @mock_hybrid_driver_event;

            analysis = run_hybrid_driver_comparison(config);

            testCase.verifyTrue(isfile(analysis.manifest_path));
            testCase.verifyTrue(isfile(analysis.comparison_csv_path));
            testCase.verifyTrue(isfile(analysis.report_path));
            testCase.verifyEqual(height(analysis.summary), 3);
            variants = string(analysis.summary.variant);
            testCase.verifyTrue(any(variants == "pid_centerline"));
            testCase.verifyTrue(any(variants == "pid_line_planner"));
            testCase.verifyTrue(any(variants == "mpc_preview"));
            testCase.verifyLessThan( ...
                analysis.summary.baseline_lap_time(variants == "pid_line_planner"), ...
                analysis.summary.baseline_lap_time(variants == "pid_centerline"));
            testCase.verifyLessThan( ...
                analysis.summary.baseline_lap_time(variants == "mpc_preview"), ...
                analysis.summary.baseline_lap_time(variants == "pid_centerline"));

            report_text = fileread(analysis.report_path);
            testCase.verifyTrue(contains(report_text, 'Hybrid Driver Comparison'));
        end
    end
end

function [result, success] = mock_hybrid_driver_event(~, ~, sample, config)
result = struct();
result.status = 'success';
result.failure_code = '';
result.failure_message = '';
result.lap_completed = true;
result.distance_traveled = 1010.0;
result.max_lateral_error = 0.70;
result.avg_lateral_error = 0.08;
result.max_track_limit_excess = 0.0;
result.max_speed = 19.0;
result.avg_speed = 9.0;
result.rms_heading_error = 4.0;
result.steering_saturation_ratio = 0.05;
result.mean_abs_sw_error = 0.30;
result.max_abs_sw_error = 2.0;
result.mean_steering_lag_ratio = 0.20;
result.max_steering_lag_ratio = 0.80;
result.avg_torque_speed_multiplier = 0.60;
result.min_steering_actuator_speed_cap = 5.0;
result.min_track_margin_speed_cap = 0.0;
result.max_abs_phi_twist = 0.002;
result.max_abs_dphi_twist = 0.03;

if strcmp(config.driver_variant_name, 'mpc_preview')
    result.lap_time = 108.0;
    result.mean_steering_lag_ratio = 0.12;
elseif strcmp(config.driver_variant_name, 'pid_line_planner')
    result.lap_time = 110.0;
else
    result.lap_time = 114.0;
end
if strcmp(sample.direction, 'low')
    result.lap_time = result.lap_time + 1.0;
end

success = true;
end

function reset_dir(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
mkdir(path);
end
