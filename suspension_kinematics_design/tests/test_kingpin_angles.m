classdef test_kingpin_angles < matlab.unittest.TestCase
    methods (Test)
        function separatesKpiAndCaster(testCase)
            addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
            theta_deg = 4.0;
            phi_deg = 6.0;
            dir = [sind(phi_deg); sind(theta_deg); -cosd(theta_deg) * cosd(phi_deg)];
            [kpi, caster] = kingpin_angles(dir);
            testCase.verifyEqual(kpi, 4.0, 'AbsTol', 1e-9);
            testCase.verifyEqual(caster, 6.0, 'AbsTol', 1e-9);
        end
    end
end
