classdef test_lap_speed_loss_analysis < matlab.unittest.TestCase
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
                'test_artifacts', 'lap_speed_loss_analysis');

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
        function testAnalyzerProducesSummarySegmentsAndReport(testCase)
            trace_path = fullfile(testCase.WorkspaceRoot, 'synthetic_laptime_trace.csv');
            write_synthetic_trace(trace_path);

            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'synthetic_loss';
            config.trace_csv_path = trace_path;
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'runtime');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.segment_length_m = 25.0;
            config.top_segment_count = 4;

            analysis = analyze_lap_speed_loss(config);

            testCase.verifyTrue(isfile(analysis.manifest_path));
            testCase.verifyTrue(isfile(analysis.summary_csv_path));
            testCase.verifyTrue(isfile(analysis.segments_csv_path));
            testCase.verifyTrue(isfile(analysis.report_path));
            testCase.verifyEqual(height(analysis.summary), 1);
            testCase.verifyEqual(height(analysis.segments), 4);
            testCase.verifyTrue(all(ismember( ...
                ["mean_controller_loss_mps", "max_time_to_reach_delta_s", ...
                 "min_torque_headroom", "min_corner_throttle_scale", ...
                 "mean_effective_max_tire_steer_rate", ...
                 "min_track_margin_effective_margin_m", "max_track_margin_outward_rate_mps", ...
                 "dominant_loss_source", "dominant_controller_risk"], ...
                string(analysis.segments.Properties.VariableNames))));

            worst = analysis.top_segments(1, :);
            testCase.verifyGreaterThanOrEqual(worst.s_start_m, 50.0);
            testCase.verifyLessThan(worst.s_start_m, 75.0);
            testCase.verifyEqual(string(worst.dominant_loss_source), "controller");
            testCase.verifyEqual(string(worst.dominant_controller_risk), "saturation_lag");
            testCase.verifyEqual(worst.min_torque_headroom, 0.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(worst.min_corner_throttle_scale, 0.30, 'AbsTol', 1e-12);
            testCase.verifyEqual(worst.mean_effective_max_tire_steer_rate, 3.8, 'AbsTol', 1e-12);
            testCase.verifyEqual(worst.min_track_margin_effective_margin_m, 0.20, 'AbsTol', 1e-12);
            testCase.verifyEqual(worst.max_track_margin_outward_rate_mps, 1.40, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(worst.max_time_to_reach_delta_s, 0.10);

            report_text = fileread(analysis.report_path);
            testCase.verifyTrue(contains(report_text, 'Lap Speed Loss Diagnostic'));
            testCase.verifyTrue(contains(report_text, 'Worst Segments'));
            testCase.verifyTrue(contains(report_text, 'min_headroom'));
            testCase.verifyTrue(contains(report_text, 'steer_rate'));
            testCase.verifyTrue(contains(report_text, 'margin_m'));
            testCase.verifyTrue(contains(report_text, 'out_rate'));
            testCase.verifyTrue(contains(report_text, 'corner_scale'));
        end

        function testMissingTraceColumnFailsClearly(testCase)
            trace_path = fullfile(testCase.WorkspaceRoot, 'bad_trace.csv');
            tbl = table((0:3)', (0:3)', 'VariableNames', {'time', 's'});
            writetable(tbl, trace_path);

            config = struct();
            config.trace_csv_path = trace_path;
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'bad_runtime');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');

            testCase.verifyError(@() analyze_lap_speed_loss(config), ?MException);
        end
    end
end

function write_synthetic_trace(path)
n = 101;
time = linspace(0, 10, n)';
s = linspace(0, 100, n)';
speed = 18.0 * ones(n, 1);
target_speed_base = 24.0 * ones(n, 1);
target_speed_preview = target_speed_base;
target_speed_final = target_speed_preview;
torque_speed_multiplier = ones(n, 1);
steering_lag_ratio = 0.10 * ones(n, 1);
steering_saturation = zeros(n, 1);
sw_error = 0.03 * ones(n, 1);
effective_max_tire_steer_rate = 3.0 * ones(n, 1);
steering_actuator_speed_cap = inf(n, 1);
track_margin_speed_cap = inf(n, 1);
track_margin_effective_margin = inf(n, 1);
track_margin_outward_rate = zeros(n, 1);
lateral_error = 0.4 * ones(n, 1);
heading_error_deg = 1.0 * ones(n, 1);
kappa = 0.02 * ones(n, 1);
required_delta_preview = -0.04 * ones(n, 1);
actual_delta_tire = -0.02 * ones(n, 1);
predicted_delta_tire = -0.025 * ones(n, 1);
preview_delta_gap = required_delta_preview - actual_delta_tire;
torque_headroom = 0.60 * ones(n, 1);
time_to_reach_delta = 0.02 * ones(n, 1);
corner_throttle_steer_ratio = 0.20 * ones(n, 1);
corner_throttle_scale = 0.86 * ones(n, 1);

preview_mask = s >= 25 & s < 50;
target_speed_preview(preview_mask) = 19.0;
target_speed_final(preview_mask) = 19.0;

controller_mask = s >= 50 & s < 75;
target_speed_preview(controller_mask) = 23.0;
target_speed_final(controller_mask) = 14.0;
torque_speed_multiplier(controller_mask) = 0.60;
steering_lag_ratio(controller_mask) = 0.85;
steering_saturation(controller_mask) = 1.0;
sw_error(controller_mask) = 0.40;
effective_max_tire_steer_rate(controller_mask) = 3.8;
steering_actuator_speed_cap(controller_mask) = 12.0;
track_margin_effective_margin(controller_mask) = 0.20;
track_margin_outward_rate(controller_mask) = 1.40;
required_delta_preview(controller_mask) = -0.55;
actual_delta_tire(controller_mask) = -0.10;
predicted_delta_tire(controller_mask) = -0.15;
preview_delta_gap(controller_mask) = -0.45;
torque_headroom(controller_mask) = 0.0;
time_to_reach_delta(controller_mask) = 0.18;
corner_throttle_steer_ratio(controller_mask) = 1.0;
corner_throttle_scale(controller_mask) = 0.30;

tbl = table(time, s, speed, target_speed_base, target_speed_preview, ...
    target_speed_final, torque_speed_multiplier, steering_lag_ratio, ...
    steering_saturation, sw_error, effective_max_tire_steer_rate, steering_actuator_speed_cap, ...
    track_margin_speed_cap, track_margin_effective_margin, track_margin_outward_rate, ...
    lateral_error, heading_error_deg, kappa, ...
    required_delta_preview, actual_delta_tire, predicted_delta_tire, ...
    preview_delta_gap, torque_headroom, time_to_reach_delta, ...
    corner_throttle_steer_ratio, corner_throttle_scale);
writetable(tbl, path);
end

function reset_dir(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
mkdir(path);
end
