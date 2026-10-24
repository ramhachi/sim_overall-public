classdef test_steering_inner_loop_benchmark < matlab.unittest.TestCase
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
                'test_artifacts', 'steering_inner_loop_benchmark');

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
        function testBenchmarkProducesSummaryAndReport(testCase)
            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'offline_steering_inner_loop';
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'runtime');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.duration = 1.2;

            analysis = run_steering_inner_loop_benchmark(config);

            testCase.verifyTrue(isfile(analysis.manifest_path));
            testCase.verifyTrue(isfile(analysis.summary_csv_path));
            testCase.verifyTrue(isfile(analysis.report_path));
            % 基本 3 + I_steer sweep 4点 = 7 シナリオ
            testCase.verifyEqual(height(analysis.summary), 7);
            testCase.verifyTrue(all(ismember( ...
                ["saturation_ratio", "safe_tire_rate_radps", "rms_error_sw_rad", "Ki_inner_used"], ...
                string(analysis.summary.Properties.VariableNames))));
            testCase.verifyGreaterThan(analysis.summary.safe_tire_rate_radps(1), 0.0);
            testCase.verifyLessThan(analysis.summary.saturation_ratio(1), 0.25);

            report_text = fileread(analysis.report_path);
            testCase.verifyTrue(contains(report_text, 'Steering Inner Loop Benchmark'));
        end

        function testISteerSweepPassesCriteria(testCase)
            % A-3: I_steer sweep 全点が rise_time < 120ms、saturation < 0.05 を満たすことを確認
            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'offline_isteer_sweep';
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'isteer_sweep');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.duration = 1.2;
            config.inner_loop_auto_tune = true;
            config.inner_loop_omega_n = 30.0;
            config.inner_loop_zeta = 0.70;

            analysis = run_steering_inner_loop_benchmark(config);
            tbl = analysis.summary;

            % I_steer sweep 行だけ抽出
            is_sweep = startsWith(string(tbl.scenario), 'step_I');
            sweep_rows = tbl(is_sweep, :);
            testCase.verifyEqual(height(sweep_rows), 4, ...
                'I_steer sweep should have 4 scenarios');

            % 全点 pass
            testCase.verifyTrue(all(sweep_rows.pass_rise_time), ...
                'All I_steer sweep scenarios must have rise_time < 120ms');
            testCase.verifyTrue(all(sweep_rows.pass_saturation), ...
                'All I_steer sweep scenarios must have saturation_ratio < 0.05');
        end

        function testKiOverrideKeepsAutoTune(testCase)
            config = struct();
            config.project_root = testCase.ProjectRoot;
            config.run_id = 'steering_inner_loop_ki_override';
            config.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'ki_override');
            config.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            config.duration = 1.0;
            config.inner_loop_auto_tune = true;
            config.inner_loop_omega_n = 30.0;
            config.inner_loop_zeta = 0.70;
            config.inner_loop_ki_override = 2.5;
            config.scenarios = struct( ...
                'name', 'step_0p10', ...
                'kind', 'step', ...
                'amplitude_tire_rad', 0.10, ...
                'frequency_hz', 0.0, ...
                'start_time', 0.10, ...
                'override_I_steer', NaN);

            analysis = run_steering_inner_loop_benchmark(config);

            testCase.verifyEqual(analysis.summary.Ki_inner_used(1), 2.5, 'AbsTol', 1e-12);
            testCase.verifyEqual(analysis.summary.Kp_inner_used(1), ...
                config.inner_loop_omega_n^2 * 0.02, 'AbsTol', 1e-12);
        end

        function testLowerTorqueLimitIncreasesSaturation(testCase)
            baseline = struct();
            baseline.project_root = testCase.ProjectRoot;
            baseline.run_id = 'steering_inner_loop_baseline';
            baseline.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'baseline');
            baseline.report_output_dir = fullfile(testCase.WorkspaceRoot, 'reports');
            baseline.duration = 1.0;
            baseline.max_tire_steer_rate = 100.0;
            baseline.scenarios = struct( ...
                'name', 'step_0p10', ...
                'kind', 'step', ...
                'amplitude_tire_rad', 0.10, ...
                'frequency_hz', 0.0, ...
                'start_time', 0.10, ...
                'override_I_steer', NaN);

            limited = baseline;
            limited.run_id = 'steering_inner_loop_limited';
            limited.runtime_output_dir = fullfile(testCase.WorkspaceRoot, 'limited');
            limited.T_driver_max = 8.0;

            baseline_result = run_steering_inner_loop_benchmark(baseline);
            limited_result = run_steering_inner_loop_benchmark(limited);

            testCase.verifyGreaterThan( ...
                limited_result.summary.saturation_ratio(1), ...
                baseline_result.summary.saturation_ratio(1));
            testCase.verifyGreaterThan( ...
                limited_result.summary.rms_error_sw_rad(1), ...
                baseline_result.summary.rms_error_sw_rad(1));
        end
    end
end

function reset_dir(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
mkdir(path);
end
