script_dir = fileparts(mfilename('fullpath'));
module_dir = fullfile(script_dir, '..');
addpath(module_dir);
addpath(fullfile(module_dir, 'geometry'));
addpath(fullfile(module_dir, 'optimization'));
addpath(fullfile(module_dir, 'visualization'));
addpath(fullfile(module_dir, 'export'));

% Setup generic options for testing
opts.wheels = [0]; % Test Front Left only
opts.n_multistart = 1;
opts.output_dir = fullfile(module_dir, 'test_output');
opts.export_csv = true;
opts.export_json = false;
opts.export_report = false;
opts.update_params = false;
opts.verbose = true;

fprintf('\nRunning Layer 2 (POU)\n');
run_all_wheels(fullfile(module_dir, '..', 'optimized_autocross.json'), opts);

fprintf('\nLayer 2 Test Complete\n');
