classdef test_save_suspension_geometry_artifact < matlab.unittest.TestCase
    properties
        MatlabRoot
        WorkDir
        BasePath
    end

    methods (TestClassSetup)
        function addMatlabRoot(tc)
            tests_root = fileparts(mfilename('fullpath'));
            tc.MatlabRoot = fileparts(tests_root);
            addpath(tc.MatlabRoot);
        end
    end

    methods (TestMethodSetup)
        function makeArtifact(tc)
            tc.WorkDir = tempname;
            mkdir(tc.WorkDir);
            tc.BasePath = fullfile(tc.WorkDir, 'vehicle.json');

            artifact = build_vehicle_artifact(create_vehicle_baseline_template());
            save_vehicle_artifact(tc.BasePath, artifact);
        end
    end

    methods (TestMethodTeardown)
        function removeWorkDir(tc)
            if exist(tc.WorkDir, 'dir')
                rmdir(tc.WorkDir, 's');
            end
        end
    end

    methods (Test)
        function defaultOutputDoesNotTouchBaseArtifact(tc)
            before = fileread(tc.BasePath);

            result = save_suspension_geometry_artifact( ...
                'BaseArtifactPath', tc.BasePath, ...
                'SuspensionGeometry', struct('stage_marker', 1));

            expected = fullfile(tc.WorkDir, 'vehicle.optimized.json');
            tc.verifyEqual(result.output_path, expected);
            tc.verifyFalse(result.in_place);
            tc.verifyTrue(isfile(expected));
            tc.verifyEqual(fileread(tc.BasePath), before);
        end

        function implicitInPlaceOverwriteIsRejected(tc)
            before = fileread(tc.BasePath);

            tc.verifyError(@() save_suspension_geometry_artifact( ...
                'BaseArtifactPath', tc.BasePath, ...
                'OutputPath', tc.BasePath), ...
                'save_suspension_geometry_artifact:implicitOverwrite');

            tc.verifyEqual(fileread(tc.BasePath), before);
        end

        function explicitInPlaceCreatesBackup(tc)
            before = fileread(tc.BasePath);

            result = save_suspension_geometry_artifact( ...
                'BaseArtifactPath', tc.BasePath, ...
                'InPlace', true, ...
                'SuspensionGeometry', struct('stage_marker', 2));

            tc.verifyTrue(result.in_place);
            tc.verifyTrue(isfile([tc.BasePath, '.bak']));
            tc.verifyEqual(fileread([tc.BasePath, '.bak']), before);
            tc.verifyNotEqual(fileread(tc.BasePath), before);
        end

        function failedInPlaceSaveKeepsBaseArtifact(tc)
            before = fileread(tc.BasePath);
            threw = false;

            try
                save_suspension_geometry_artifact( ...
                    'BaseArtifactPath', tc.BasePath, ...
                    'InPlace', true, ...
                    'StageResult', struct('not_serializable', @() 1));
            catch
                threw = true;
            end

            tc.verifyTrue(threw);
            tc.verifyEqual(fileread(tc.BasePath), before);
        end
    end
end
