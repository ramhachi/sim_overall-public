% run_cpp_export.m
% Export trained DL tire model for C++ integration

clear;
clc;

fprintf('\n');
fprintf('=================================================================\n');
fprintf('DL Tire Model - C++ Export Script\n');
fprintf('=================================================================\n\n');

% Model path
model_path = 'output/ultra_deep_validation_model.mat';

% Output directory
output_dir = 'cpp_export';

% Check if model exists
if ~isfile(model_path)
    error('Model file not found: %s\nPlease run training first.', model_path);
end

% Grid configuration (optimized for C++ performance)
grid_config = struct();
grid_config.SR_range = [-0.2, 0.2, 40];      % 40 points
grid_config.SA_range = [-6, 6, 50];          % 50 points
grid_config.Fz_range = [200, 1300, 30];      % 30 points
grid_config.IA_range = [0, 4, 15];           % 15 points
grid_config.TSTC_range = [55, 80, 15];       % 15 points

% Total grid points
total_points = grid_config.SR_range(3) * grid_config.SA_range(3) * ...
               grid_config.Fz_range(3) * grid_config.IA_range(3) * ...
               grid_config.TSTC_range(3);

fprintf('Grid configuration:\n');
fprintf('  Total points: %d (%.1f million)\n', total_points, total_points/1e6);
fprintf('  Estimated memory: %.1f MB\n\n', total_points * 4 * 8 / 1e6);

% Export for C++
exportForCpp(model_path, output_dir, grid_config);

fprintf('Export complete! Files are in: %s\n', output_dir);
