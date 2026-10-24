classdef test_rl_checkpoint_benchmark_offline < matlab.unittest.TestCase
    properties
        ProjectRoot
        MatlabRoot
        RLRoot
        WorkspaceRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(tc)
            testsRoot = fileparts(mfilename('fullpath'));
            tc.MatlabRoot = fileparts(testsRoot);
            tc.ProjectRoot = fileparts(tc.MatlabRoot);
            tc.RLRoot = fullfile(tc.MatlabRoot, 'rl_driver');
            tc.WorkspaceRoot = fullfile(tc.ProjectRoot, 'output', ...
                'test_artifacts', 'rl_checkpoint_benchmark_offline');

            addpath(tc.MatlabRoot);
            addpath(tc.RLRoot);
            resetDir(tc.WorkspaceRoot);
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
        function testMockCheckpointBenchmarkWritesBestAndSamples(tc)
            checkpointDir = fullfile(tc.WorkspaceRoot, 'checkpoints');
            mkdir(checkpointDir);
            createDummyFile(fullfile(checkpointDir, 'Agent10.mat'));
            createDummyFile(fullfile(checkpointDir, 'Agent20.mat'));

            config = struct();
            config.project_root = tc.ProjectRoot;
            config.run_id = 'offline_rl_checkpoint_benchmark';
            config.checkpoint_dir = checkpointDir;
            config.output_dir = fullfile(tc.WorkspaceRoot, 'runtime');
            config.params_file = fullfile(tc.ProjectRoot, 'matlab', ...
                'parameter_presets', 'params_winged_flexible.json');
            config.episodes = [10 20];
            config.execute_checkpoint_fn = @mockCheckpointExecution;

            analysis = benchmarkRLCheckpoints(config);

            tc.verifyTrue(isfile(analysis.manifest_path));
            tc.verifyTrue(isfile(analysis.samples_csv_path));
            tc.verifyTrue(isfile(analysis.best_json_path));

            samples = readtable(analysis.samples_csv_path, 'TextType', 'string');
            tc.verifyEqual(height(samples), 2);
            tc.verifyTrue(all(ismember(["episode", "selection_score", ...
                "brake_smoothness"], samples.Properties.VariableNames)));

            best = jsondecode(fileread(analysis.best_json_path));
            tc.verifyEqual(best.episode, 20);
            tc.verifyTrue(best.lap_completed);
        end

        function testSingleCheckpointBenchmarkWritesOneRow(tc)
            checkpointDir = fullfile(tc.WorkspaceRoot, 'single_checkpoint');
            mkdir(checkpointDir);
            createDummyFile(fullfile(checkpointDir, 'Agent20.mat'));

            config = struct();
            config.project_root = tc.ProjectRoot;
            config.run_id = 'offline_single_rl_checkpoint_benchmark';
            config.checkpoint_dir = checkpointDir;
            config.output_dir = fullfile(tc.WorkspaceRoot, 'single_runtime');
            config.params_file = fullfile(tc.ProjectRoot, 'matlab', ...
                'parameter_presets', 'params_winged_flexible.json');
            config.episodes = 20;
            config.execute_checkpoint_fn = @mockCheckpointExecution;

            analysis = benchmarkRLCheckpoints(config);
            samples = readtable(analysis.samples_csv_path, 'TextType', 'string');
            best = jsondecode(fileread(analysis.best_json_path));

            tc.verifyEqual(height(samples), 1);
            tc.verifyEqual(samples.episode(1), 20);
            tc.verifyEqual(best.episode, 20);
        end
    end
end

function [result, success, commandText, commandStatus] = mockCheckpointExecution( ...
    checkpoint, ~, ~, ~)
result = struct();
result.status = 'failed';
result.failure_code = 'mock_incomplete';
result.failure_message = 'mock incomplete lap';
result.failure_time = 60.0;
result.lap_completed = false;
result.lap_time = NaN;
result.distance_traveled = 600.0;
result.max_lateral_error = 2.0;
result.max_track_limit_excess = 0.1;
result.max_speed = 15.0;
result.avg_speed = 8.0;
result.rms_heading_error = 0.2;
result.steering_smoothness = 4.0;
result.throttle_smoothness = 0.1;
result.brake_smoothness = 0.2;
result.steering_saturation_ratio = 0.3;
result.mean_steering_lag_ratio = 0.4;
result.max_steering_lag_ratio = 0.9;
result.dominant_loss = 'steering_lag';

if checkpoint.episode == 20
    result.status = 'success';
    result.failure_code = '';
    result.failure_message = '';
    result.failure_time = NaN;
    result.lap_completed = true;
    result.lap_time = 95.0;
    result.distance_traveled = 1010.0;
    result.max_track_limit_excess = 0.0;
    result.steering_smoothness = 2.0;
end

success = logical(result.lap_completed);
commandText = 'mock';
commandStatus = double(~success);
end

function createDummyFile(pathIn)
fid = fopen(pathIn, 'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'dummy');
end

function resetDir(pathIn)
if exist(pathIn, 'dir')
    rmdir(pathIn, 's');
end
mkdir(pathIn);
end
