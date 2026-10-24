classdef test_fine_tune_autocross_offline < matlab.unittest.TestCase
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
                'test_artifacts', 'fine_tune_autocross_offline');

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
        function testMissingAgentWritesCrashManifestWithoutRethrow(tc)
            config = struct();
            config.project_root = tc.ProjectRoot;
            config.run_id = 'offline_fine_tune_crash_manifest';
            config.output_dir = fullfile(tc.WorkspaceRoot, 'v7');
            config.resume_agent_file = fullfile(tc.WorkspaceRoot, 'missing_agent.mat');
            config.start_server = false;
            config.rethrow_on_error = false;
            config.export_final = false;
            config.benchmark_after_training = false;
            config.max_episodes = 1;

            result = fine_tune_autocross(config);

            runManifestPath = fullfile(config.output_dir, 'run_manifest.json');
            crashManifestPath = fullfile(config.output_dir, 'crash_manifest.json');
            crashManifest = jsondecode(fileread(crashManifestPath));

            tc.verifyEqual(result.status, 'failed');
            tc.verifyTrue(isfile(runManifestPath));
            tc.verifyTrue(isfile(crashManifestPath));
            tc.verifyEqual(crashManifest.status, 'failed');
            tc.verifyTrue(contains(string(crashManifest.error_message), 'Agent file not found'));
        end
    end
end

function resetDir(pathIn)
if exist(pathIn, 'dir')
    rmdir(pathIn, 's');
end
mkdir(pathIn);
end
