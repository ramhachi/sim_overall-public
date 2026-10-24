classdef test_roll_center_frame < matlab.unittest.TestCase
    methods (Test)
        function roundTripsCoordinateAndHeight(testCase)
            addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
            f = roll_center_frame(-0.035);
            testCase.verifyEqual(f.roll_center_z_down_m, -0.035);
            testCase.verifyEqual(f.roll_center_height_m, 0.035);
            testCase.verifyEqual(f.roll_center_z_m, 0.035);
        end
    end
end
