% run_export_for_cpp.m
% Export trained model for C++ integration

clear; clc;

fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║  Export DL Tire Model for C++ Integration                    ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');

% Model path
model_path = 'output/ultra_deep_validation_model.mat';

% Output directory
output_dir = '../../cpp_export';

% Check if model exists
if ~isfile(model_path)
    error('Model file not found: %s\nPlease run training first.', model_path);
end

% Export model
try
    exportModelForCpp(model_path, output_dir);
    
    fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  ✓ Export Successful!                                        ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');
    
    fprintf('Files are ready in: %s\n\n', output_dir);
    fprintf('Next steps:\n');
    fprintf('  1. Review the generated files\n');
    fprintf('  2. Integrate into your C++ project\n');
    fprintf('  3. Run validation tests\n\n');
    
catch ME
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  ✗ Export Failed                                             ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');
    fprintf('Error: %s\n\n', ME.message);
    rethrow(ME);
end
