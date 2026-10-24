classdef test_driver_generalization_offline < matlab.unittest.TestCase
    properties
        ProjectRoot
        MatlabRoot
        DriverRoot
        WorkspaceRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(tc)
            tests_root = fileparts(mfilename('fullpath'));
            tc.MatlabRoot = fileparts(tests_root);
            tc.ProjectRoot = fileparts(tc.MatlabRoot);
            tc.DriverRoot = fullfile(tc.MatlabRoot, 'driver_generalization');
            tc.WorkspaceRoot = fullfile(tc.ProjectRoot, 'output', ...
                'test_artifacts', 'driver_generalization_offline');

            addpath(tc.MatlabRoot);
            addpath(tc.DriverRoot);
            reset_dir(tc.WorkspaceRoot);
        end
    end

    methods (TestClassTeardown)
        function cleanupWorkspace(tc)
            if exist(tc.WorkspaceRoot, 'dir')
                rmdir(tc.WorkspaceRoot, 's');
            end
        end
    end

    methods (Test)
        function testMockSweepProducesDriverHealthOutputs(tc)
            config = struct();
            config.project_root = tc.ProjectRoot;
            config.run_id = 'offline_driver_generalization';
            config.runtime_output_dir = fullfile(tc.WorkspaceRoot, 'runtime');
            config.report_output_dir = fullfile(tc.WorkspaceRoot, 'reports');
            config.params_output_dir = fullfile(config.runtime_output_dir, 'params');
            config.base_params = create_vehicle_baseline_template();
            config.parameter_names = {'ChassisStiffness', 'DriverTorqueMax'};
            config.events = struct( ...
                'name', 'autocross', ...
                'track_name', 'autocross', ...
                'sim_time', 180.0, ...
                'initial_speed', -1.0, ...
                'timeout_sec', 240.0);
            config.execute_event_fn = @mock_driver_generalization_event;

            analysis = run_driver_generalization_sweep(config);

            tc.verifyTrue(isfile(analysis.manifest_path));
            tc.verifyTrue(isfile(analysis.samples_csv_path));
            tc.verifyTrue(isfile(analysis.summary_csv_path));
            tc.verifyTrue(isfile(analysis.report_path));

            samples = readtable(analysis.samples_csv_path, 'TextType', 'string');
            tc.verifyEqual(height(samples), 5);
            tc.verifyEqual(sum(samples.sample_role == "baseline"), 1);
            tc.verifyEqual(sum(samples.parameter_name == "DriverTorqueMax"), 2);
            tc.verifyTrue(any(samples.failure_code == "mock_low_torque"));
            tc.verifyTrue(any(samples.steering_saturation_ratio > 0.10));
            tc.verifyTrue(all(ismember("max_track_limit_excess", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("failure_time", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("brake_smoothness", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("min_steering_actuator_speed_cap", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("min_track_margin_speed_cap", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("driver_intervention_cost", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("control_intervention_cost", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("vehicle_health_penalty", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("driver_masking_risk", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("mean_torque_speed_loss", samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember("max_mpc_rollout_progress_deficit", samples.Properties.VariableNames)));
            tc.verifyTrue(all(samples.driver_intervention_cost >= samples.control_intervention_cost));

            report_text = fileread(analysis.report_path);
            tc.verifyTrue(contains(report_text, 'Driver Generalization Sweep'));
            tc.verifyTrue(contains(report_text, 'DriverTorqueMax'));
            tc.verifyTrue(contains(report_text, 'control_intervention_cost'));
            tc.verifyTrue(contains(report_text, 'vehicle_health_penalty'));
        end

        function testMockControllerComparisonProducesComparableRows(tc)
            config = struct();
            config.project_root = tc.ProjectRoot;
            config.run_id = 'offline_controller_comparison';
            config.runtime_output_dir = fullfile(tc.WorkspaceRoot, 'controller_runtime');
            config.report_output_dir = fullfile(tc.WorkspaceRoot, 'controller_reports');
            config.trace_output_dir = fullfile(config.runtime_output_dir, 'traces');
            config.base_params = create_vehicle_baseline_template();
            config.driver_torque_max = 40.0;
            config.driver_torque_rate_max = 0.0;
            config.reference_lap_time_sec = 66.0;
            config.events = struct( ...
                'name', 'autocross', ...
                'track_name', 'autocross', ...
                'sim_time', 180.0, ...
                'initial_speed', -1.0);
            config.variants = struct( ...
                'name', {'pid_baseline', 'mpc_preview', 'mpc_v2_preview'}, ...
                'driver_mode', {'pid', 'mpc', 'mpc_v2'}, ...
                'extra_cli_args', {'', '', ''}, ...
                'rl_actor_dir', {'', '', ''}, ...
                'enabled', {true, true, true});
            config.execute_variant_fn = @mock_controller_comparison_variant;

            analysis = run_controller_comparison_benchmark(config);

            tc.verifyTrue(isfile(analysis.manifest_path));
            tc.verifyTrue(isfile(analysis.samples_csv_path));
            tc.verifyTrue(isfile(analysis.summary_csv_path));
            tc.verifyTrue(isfile(analysis.report_path));
            tc.verifyTrue(isfile(analysis.effective_params_file));

            samples = readtable(analysis.samples_csv_path, 'TextType', 'string');
            tc.verifyEqual(height(samples), 3);
            tc.verifyTrue(all(ismember(["variant", "driver_mode", "lap_time_delta_to_reference"], ...
                samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember(["driver_role", "control_intervention_cost", ...
                "vehicle_health_penalty", "driver_masking_risk"], samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember(["failure_time", "steering_smoothness", ...
                "throttle_smoothness", "brake_smoothness"], samples.Properties.VariableNames)));
            tc.verifyTrue(all(ismember(["mean_mpc_rollout_progress_deficit", ...
                "max_mpc_rollout_progress_deficit"], samples.Properties.VariableNames)));
            tc.verifyEqual(samples.reference_lap_time_sec(1), 66.0);
            tc.verifyTrue(any(samples.driver_role == "fallback_teacher"));
            tc.verifyTrue(any(samples.driver_role == "legacy_model_based"));
            tc.verifyTrue(any(samples.driver_role == "model_based_candidate"));
            tc.verifyTrue(any(samples.driver_mode == "mpc"));
            tc.verifyTrue(any(samples.driver_mode == "mpc_v2"));
            tc.verifyTrue(any(samples.mpc_avg_solve_time_ms > 0));
            tc.verifyTrue(any(samples.dominant_driver_loss == "steering_lag"));

            report_text = fileread(analysis.report_path);
            tc.verifyTrue(contains(report_text, 'Controller Comparison Benchmark'));
            tc.verifyTrue(contains(report_text, 'PID is treated as the stable fallback'));
        end
    end
end

function [result, success] = mock_driver_generalization_event(~, ~, sample, ~)
result = struct();
result.status = 'success';
result.failure_code = '';
result.failure_message = '';
result.failure_time = NaN;
result.lap_completed = true;
result.lap_time = 113.0;
result.distance_traveled = 1010.0;
result.max_lateral_error = 0.50;
result.avg_lateral_error = 0.05;
result.max_track_limit_excess = 0.0;
result.max_speed = 19.0;
result.avg_speed = 8.9;
result.rms_heading_error = 4.0;
result.steering_smoothness = 0.05;
result.throttle_smoothness = 0.02;
result.brake_smoothness = 0.01;
result.steering_saturation_ratio = 0.04;
result.mean_abs_sw_error = 0.30;
result.max_abs_sw_error = 2.0;
result.mean_steering_lag_ratio = 0.20;
result.max_steering_lag_ratio = 0.80;
result.avg_torque_speed_multiplier = 0.60;
result.mean_torque_speed_loss = 0.40;
result.mean_steering_cap_speed_loss = 0.10;
result.mean_track_margin_speed_loss = 0.05;
result.mean_corner_throttle_loss = 0.08;
result.mean_chassis_twist_risk = 0.02;
result.chassis_twist_risk_integral = 2.0;
result.driver_intervention_cost = 0.69;
result.control_intervention_cost = 0.67;
result.vehicle_health_penalty = 0.02;
result.driver_masking_risk = 0.0106;
result.mean_mpc_rollout_progress_deficit = NaN;
result.max_mpc_rollout_progress_deficit = NaN;
result.min_steering_actuator_speed_cap = 12.0;
result.min_track_margin_speed_cap = 14.0;
result.max_abs_phi_twist = 0.002;
result.max_abs_dphi_twist = 0.03;

if strcmp(sample.parameter_name, 'DriverTorqueMax') && strcmp(sample.direction, 'low')
    result.status = 'failed';
    result.failure_code = 'mock_low_torque';
    result.failure_message = 'mock torque limit failure';
    result.failure_time = 42.0;
    result.lap_completed = false;
    result.lap_time = NaN;
    result.max_lateral_error = 2.6;
    result.max_track_limit_excess = 0.4;
    result.steering_saturation_ratio = 0.35;
    result.min_steering_actuator_speed_cap = 5.0;
    result.min_track_margin_speed_cap = 6.0;
end

success = logical(result.lap_completed);
end

function [result, success] = mock_controller_comparison_variant(variant, ~, ~, ~, ~)
result = struct();
result.status = 'success';
result.failure_code = '';
result.failure_message = '';
result.failure_time = NaN;
result.lap_completed = true;
result.lap_time = 92.0;
result.distance_traveled = 1010.0;
result.max_lateral_error = 0.42;
result.avg_lateral_error = 0.06;
result.max_track_limit_excess = 0.0;
result.max_speed = 25.0;
result.avg_speed = 10.9;
result.rms_heading_error = 0.08;
result.steering_smoothness = 0.12;
result.throttle_smoothness = 0.04;
result.brake_smoothness = 0.02;
result.steering_saturation_ratio = 0.08;
result.mean_abs_sw_error = 0.18;
result.max_abs_sw_error = 1.2;
result.mean_steering_lag_ratio = 0.12;
result.max_steering_lag_ratio = 0.44;
result.avg_torque_speed_multiplier = 0.86;
result.mean_torque_speed_loss = 0.12;
result.mean_steering_cap_speed_loss = 0.06;
result.mean_steering_reach_speed_loss = 0.03;
result.mean_track_margin_speed_loss = 0.02;
result.mean_corner_throttle_loss = 0.04;
result.mean_chassis_twist_risk = 0.02;
result.chassis_twist_risk_integral = 1.0;
result.driver_intervention_cost = 0.20;
result.control_intervention_cost = 0.18;
result.vehicle_health_penalty = 0.02;
result.driver_masking_risk = 0.0036;
result.mean_mpc_rollout_progress_deficit = NaN;
result.max_mpc_rollout_progress_deficit = NaN;
result.min_steering_actuator_speed_cap = 12.0;
result.min_steering_reach_speed_cap = 14.0;
result.min_track_margin_speed_cap = 15.0;
result.min_velocity_profile_steering_cap = 13.0;
result.velocity_profile_steering_cap_ratio = 0.10;
result.max_abs_phi_twist = 0.004;
result.max_abs_dphi_twist = 0.08;
result.dominant_driver_loss = 'none';
result.mpc_solve_count = 0;
result.mpc_avg_solve_time_ms = NaN;
result.mpc_max_solve_time_ms = NaN;
result.mpc_fail_count = 0;

if strcmp(variant.driver_mode, 'mpc')
    result.lap_time = 88.0;
    result.dominant_driver_loss = 'steering_lag';
    result.mpc_solve_count = 1500;
    result.mpc_avg_solve_time_ms = 0.8;
    result.mpc_max_solve_time_ms = 2.5;
    result.mean_mpc_rollout_progress_deficit = 0.45;
    result.max_mpc_rollout_progress_deficit = 1.4;
elseif strcmp(variant.driver_mode, 'mpc_v2')
    result.lap_time = 86.5;
    result.dominant_driver_loss = 'none';
    result.control_intervention_cost = 0.12;
    result.driver_intervention_cost = 0.14;
    result.mpc_solve_count = 1600;
    result.mpc_avg_solve_time_ms = 1.1;
    result.mpc_max_solve_time_ms = 3.0;
    result.mean_mpc_rollout_progress_deficit = 0.22;
    result.max_mpc_rollout_progress_deficit = 0.8;
end

success = logical(result.lap_completed);
end

function reset_dir(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
mkdir(path);
end
