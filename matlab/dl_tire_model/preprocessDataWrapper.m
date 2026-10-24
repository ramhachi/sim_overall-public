function [X_train, Y_train, X_val, Y_val, norm_params] = preprocessDataWrapper(data_input, config)
% PREPROCESSDATAWRAPPER Wrapper for preprocessData that handles both tables and file paths
%
% This wrapper allows preprocessData to accept either:
% 1. A data table (from loadTireData)
% 2. Cell array of file paths
%
% Inputs:
%   data_input - Either a table or cell array of file paths
%   config     - Structure with configuration (optional)
%
% Outputs:
%   Same as preprocessData

    if nargin < 2
        config = struct();
    end
    
    if ~isfield(config, 'split_ratio')
        config.split_ratio = 0.8;
    end
    
    % Check if input is a table or file paths
    if istable(data_input)
        % Input is already a table, use it directly
        data = data_input;
    elseif iscell(data_input)
        % Input is file paths, load the data
        data = loadTireData(data_input);
    else
        error('data_input must be either a table or cell array of file paths');
    end
    
    % Now process the data
    fprintf('=== Starting Data Preprocessing Pipeline ===\n\n');
    
    % Step 1: Extract features and targets
    fprintf('Step 1: Extracting features and targets...\n');
    X = extractFeatures(data);
    Y = extractTargets(data);
    fprintf('Extracted %d samples with 5 features and 4 targets\n\n', size(X, 1));
    
    % Step 2: Remove missing data
    fprintf('Step 2: Removing missing/infinite values...\n');
    [X_clean, Y_clean, stats] = removeMissingData(X, Y);
    fprintf('\n');
    
    % Step 3: Normalize data
    fprintf('Step 3: Normalizing data...\n');
    [X_norm, input_mean, input_std] = normalizeData(X_clean);
    [Y_norm, output_mean, output_std] = normalizeData(Y_clean);
    fprintf('Features normalized: mean ≈ 0, std ≈ 1\n');
    fprintf('Targets normalized: mean ≈ 0, std ≈ 1\n\n');
    
    % Store normalization parameters
    norm_params = struct();
    norm_params.input_mean = input_mean;
    norm_params.input_std = input_std;
    norm_params.output_mean = output_mean;
    norm_params.output_std = output_std;
    
    % Step 4: Split data
    fprintf('Step 4: Splitting data into train/validation sets...\n');
    [X_train, Y_train, X_val, Y_val] = splitData(X_norm, Y_norm, config.split_ratio);
    fprintf('\n');
    
    % Summary
    fprintf('=== Preprocessing Complete ===\n');
    fprintf('Training set:   %d samples\n', size(X_train, 1));
    fprintf('Validation set: %d samples\n', size(X_val, 1));
    fprintf('Input features: %d (SR, SA, Fz, IA, TSTC)\n', size(X_train, 2));
    fprintf('Output targets: %d (Fx, Fy, Mx, Mz)\n', size(Y_train, 2));
    fprintf('==============================\n\n');
end
