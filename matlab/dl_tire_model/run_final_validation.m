% Run Final System Validation
% This script executes the complete final validation checkpoint for the
% MATLAB Deep Learning Tire Model system

% Clear workspace and command window
clear;
clc;

% Add current directory to path
addpath(pwd);

% Display header
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║  MATLAB Deep Learning Tire Model - Final System Validation   ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

% Check for required toolboxes
fprintf('Checking for required toolboxes...\n');
required_toolboxes = {
    'Deep Learning Toolbox', 'neural_network_toolbox'
    'Statistics and Machine Learning Toolbox', 'statistics_toolbox'
};

missing_toolboxes = {};
for i = 1:size(required_toolboxes, 1)
    toolbox_name = required_toolboxes{i, 1};
    toolbox_id = required_toolboxes{i, 2};
    
    if ~license('test', toolbox_id)
        missing_toolboxes{end+1} = toolbox_name;
    end
end

if ~isempty(missing_toolboxes)
    fprintf('⚠ WARNING: Missing required toolboxes:\n');
    for i = 1:length(missing_toolboxes)
        fprintf('  - %s\n', missing_toolboxes{i});
    end
    fprintf('\nSome tests may fail without these toolboxes.\n\n');
else
    fprintf('✓ All required toolboxes available\n\n');
end

% Check GPU availability
if gpuDeviceCount > 0
    gpu = gpuDevice();
    fprintf('✓ GPU detected: %s\n', gpu.Name);
    fprintf('  Memory: %.2f GB\n', gpu.AvailableMemory / 1e9);
else
    fprintf('⚠ No GPU detected - training will use CPU\n');
end
fprintf('\n');

% Run validation
try
    results = final_system_validation();
    
    % Save results
    save('output/final_validation_results.mat', 'results');
    fprintf('\nResults saved to: output/final_validation_results.mat\n');
    
catch ME
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  VALIDATION FAILED                                            ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n');
    fprintf('\nError: %s\n', ME.message);
    fprintf('Stack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    
    rethrow(ME);
end

fprintf('\nValidation complete!\n\n');
