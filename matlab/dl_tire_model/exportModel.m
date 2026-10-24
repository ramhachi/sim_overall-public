function exportModel(trained_net, norm_params, output_path)
% EXPORTMODEL Export trained tire model to .mat file
%
% This function saves the trained neural network and normalization parameters
% to a .mat file for later use in simulations or predictions.
%
% Inputs:
%   trained_net  - Trained neural network (from trainTireModel)
%   norm_params  - Structure with normalization parameters:
%                  .input_mean, .input_std, .output_mean, .output_std
%   output_path  - Path where the model should be saved (e.g., 'model.mat')
%
% Outputs:
%   None (saves file to disk)
%
% Example:
%   exportModel(trained_net, norm_params, 'output/tire_model.mat');
%
% Requirements: 6.1, 6.2

    fprintf('=== Exporting Model ===\n');
    
    % Validate inputs
    if nargin < 3
        error('exportModel requires 3 arguments: trained_net, norm_params, output_path');
    end
    
    % Check that norm_params has required fields
    required_fields = {'input_mean', 'input_std', 'output_mean', 'output_std'};
    for i = 1:length(required_fields)
        if ~isfield(norm_params, required_fields{i})
            error('norm_params missing required field: %s', required_fields{i});
        end
    end
    
    % Create output directory if it doesn't exist
    [output_dir, ~, ~] = fileparts(output_path);
    if ~isempty(output_dir) && ~exist(output_dir, 'dir')
        mkdir(output_dir);
        fprintf('Created output directory: %s\n', output_dir);
    end
    
    % Save trained network and normalization parameters
    fprintf('Saving model to: %s\n', output_path);
    save(output_path, 'trained_net', 'norm_params', '-v7.3');
    
    % Verify file was created
    if ~isfile(output_path)
        error('Failed to create output file: %s', output_path);
    end
    
    % Report file size
    file_info = dir(output_path);
    file_size_mb = file_info.bytes / (1024 * 1024);
    fprintf('Model saved successfully (%.2f MB)\n', file_size_mb);
    fprintf('======================\n\n');
end
