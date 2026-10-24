function [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config)
% PREPROCESSDATA Complete preprocessing pipeline for tire test data
%
% This function performs the complete preprocessing workflow:
% 1. Load data from CSV files
% 2. Extract features and targets
% 3. Remove missing/infinite values
% 4. Normalize features and targets
% 5. Split into training and validation sets
%
% Inputs:
%   data_paths - Cell array of file paths to CSV files
%   config     - Structure with configuration:
%                .split_ratio - Train/val split ratio (default: 0.8)
%
% Outputs:
%   X_train     - Normalized training input features
%   Y_train     - Normalized training output targets
%   X_val       - Normalized validation input features
%   Y_val       - Normalized validation output targets
%   norm_params - Structure with normalization parameters:
%                 .input_mean, .input_std, .output_mean, .output_std
%
% Example:
%   data_paths = {'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'};
%   config = struct('split_ratio', 0.8);
%   [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);
%
% Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7

    fprintf('=== Starting Data Preprocessing Pipeline ===\n\n');
    
    % Default configuration
    if nargin < 2
        config = struct();
    end
    if ~isfield(config, 'split_ratio')
        config.split_ratio = 0.8;
    end
    
    % Step 1: Load data
    fprintf('Step 1: Loading data...\n');
    data = loadTireData(data_paths);
    fprintf('\n');
    
    % Step 2: Extract features and targets
    fprintf('Step 2: Extracting features and targets...\n');
    X = extractFeatures(data);
    Y = extractTargets(data);
    fprintf('Extracted %d samples with 5 features and 4 targets\n\n', size(X, 1));
    
    % Step 3: Remove missing data
    fprintf('Step 3: Removing missing/infinite values...\n');
    [X_clean, Y_clean, stats] = removeMissingData(X, Y);
    fprintf('\n');
    
    % Step 4: Normalize data
    fprintf('Step 4: Normalizing data...\n');
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
    
    % Step 5: Split data
    fprintf('Step 5: Splitting data into train/validation sets...\n');
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
