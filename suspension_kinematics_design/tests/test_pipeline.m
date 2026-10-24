script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..'));
addpath(fullfile(script_dir, '..', '..', 'matlab', 'steering_hardpoint_design'));
params_file = resolve_default_vehicle_json(fullfile(script_dir, '..', '..'), 'RequireExportReady', true);
if isempty(params_file)
    error('No vehicle artifact candidate found for suspension pipeline test.');
end

fprintf('Running Layer 1 (A-Arm)\n');
main_a_arm_optimizer(params_file);

fprintf('\nRunning Layer 1.5 (Steering)\n');
main_steering_optimizer(params_file);

fprintf('\nPipeline Test Complete\n');
