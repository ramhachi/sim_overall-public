function results = run_design_sensitivity_offline_tests()
tests_root = fileparts(mfilename('fullpath'));
matlab_root = fileparts(tests_root);
project_root = fileparts(matlab_root);

addpath(matlab_root);
addpath(fullfile(matlab_root, 'design_sensitivity'));

results = runtests(fullfile(tests_root, 'test_design_sensitivity_offline.m'));
disp(results);
if ~all([results.Passed])
    error('run_design_sensitivity_offline_tests:failed', ...
        'Design sensitivity offline tests failed for %s.', project_root);
end
end
