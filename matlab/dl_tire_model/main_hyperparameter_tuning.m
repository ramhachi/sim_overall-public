% MAIN_HYPERPARAMETER_TUNING Hyperparameter tuning workflow for tire model
%
% This script demonstrates how to perform hyperparameter tuning to find the
% optimal configuration for the tire model. It supports both grid search and
% random search strategies.
%
% Workflow Steps:
%   1. Load and preprocess data
%   2. Define search space
%   3. Run grid/random search
%   4. Generate comparison report
%   5. Save best model
%
% Requirements: 7.3, 7.4
%
% Usage:
%   1. Edit the configuration section below to specify search space
%   2. Choose search type: 'grid' or 'random'
%   3. Run this script in MATLAB: >> main_hyperparameter_tuning
%   4. Review comparison report and best model
%
% Example:
%   >> main_hyperparameter_tuning
%   % Runs hyperparameter search and saves best configuration

%% ========================================================================
%  CONFIGURATION
%  ========================================================================

% Clear workspace and command window
clear; clc; close all;

fprintf('========================================\n');
fprintf('HYPERPARAMETER TUNING WORKFLOW\n');
fprintf('========================================\n');
fprintf('Started: %s\n\n', char(datetime('now')));

% Data paths - specify your TTC tire test data files
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};

% Search configuration
search_type = 'random';  % 'grid' or 'random'
num_random_trials = 10;  % Number of trials for random search (ignored for grid)

% Output directory
output_dir = 'experiments';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('Configuration:\n');
fprintf('  Search type: %s\n', search_type);
if strcmpi(search_type, 'random')
    fprintf('  Number of trials: %d\n', num_random_trials);
end
fprintf('  Output directory: %s\n\n', output_dir);

%% ========================================================================
%  STEP 1: LOAD AND PREPROCESS DATA
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 1: DATA LOADING AND PREPROCESSING\n');
fprintf('========================================\n\n');

% Basic preprocessing configuration
preprocess_config = struct();
preprocess_config.split_ratio = 0.8;

try
    [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, preprocess_config);
catch ME
    fprintf('ERROR: Data preprocessing failed\n');
    fprintf('Message: %s\n', ME.message);
    error('Cannot continue without valid data.');
end

%% ========================================================================
%  STEP 2: DEFINE HYPERPARAMETER SEARCH SPACE
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 2: DEFINING SEARCH SPACE\n');
fprintf('========================================\n\n');

% Define search space - each field is a cell array of values to try
search_space = struct();

% Learning rate options
search_space.learning_rate = {0.001, 0.0005, 0.0001};

% Batch size options
search_space.batch_size = {128, 256, 512};

% Max epochs options
search_space.max_epochs = {150, 200, 250};

% Patience options (early stopping)
search_space.patience = {10, 15, 20};

% Model type options
search_space.model_type = {'feedforward'};  % Add 'lstm' if desired

% Dropout rate options
search_space.dropout_rate = {0.1, 0.2, 0.3};

% Display search space
fprintf('Hyperparameter Search Space:\n');
fprintf('  Learning rate: [%s]\n', strjoin(cellfun(@num2str, search_space.learning_rate, 'UniformOutput', false), ', '));
fprintf('  Batch size: [%s]\n', strjoin(cellfun(@num2str, search_space.batch_size, 'UniformOutput', false), ', '));
fprintf('  Max epochs: [%s]\n', strjoin(cellfun(@num2str, search_space.max_epochs, 'UniformOutput', false), ', '));
fprintf('  Patience: [%s]\n', strjoin(cellfun(@num2str, search_space.patience, 'UniformOutput', false), ', '));
fprintf('  Model type: [%s]\n', strjoin(search_space.model_type, ', '));
fprintf('  Dropout rate: [%s]\n', strjoin(cellfun(@num2str, search_space.dropout_rate, 'UniformOutput', false), ', '));

if strcmpi(search_type, 'grid')
    % Calculate total combinations for grid search
    n_combinations = 1;
    fields = fieldnames(search_space);
    for i = 1:length(fields)
        n_combinations = n_combinations * length(search_space.(fields{i}));
    end
    fprintf('\nTotal combinations (grid search): %d\n', n_combinations);
    
    % Warn if too many combinations
    if n_combinations > 50
        warning('Grid search will evaluate %d combinations. This may take a long time!', n_combinations);
        fprintf('Consider using random search or reducing search space.\n');
        
        % Ask user to confirm
        user_input = input('Continue? (y/n): ', 's');
        if ~strcmpi(user_input, 'y')
            fprintf('Hyperparameter tuning cancelled.\n');
            return;
        end
    end
end

fprintf('\n');

%% ========================================================================
%  STEP 3: RUN HYPERPARAMETER SEARCH
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 3: RUNNING HYPERPARAMETER SEARCH\n');
fprintf('========================================\n\n');

try
    if strcmpi(search_type, 'random')
        [best_config, all_results] = runHyperparameterSearch(X_train, Y_train, X_val, Y_val, ...
                                                              norm_params, search_space, ...
                                                              search_type, num_random_trials);
    else
        [best_config, all_results] = runHyperparameterSearch(X_train, Y_train, X_val, Y_val, ...
                                                              norm_params, search_space, ...
                                                              search_type);
    end
catch ME
    fprintf('ERROR: Hyperparameter search failed\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Identifier: %s\n', ME.identifier);
    error('Search failed. Check error message above.');
end

%% ========================================================================
%  STEP 4: GENERATE COMPARISON REPORT
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 4: GENERATING COMPARISON REPORT\n');
fprintf('========================================\n\n');

try
    % Extract experiment IDs from results
    experiment_ids = cell(length(all_results), 1);
    for i = 1:length(all_results)
        if isfield(all_results{i}, 'experiment_id')
            experiment_ids{i} = all_results{i}.experiment_id;
        end
    end
    
    % Remove empty entries (failed experiments)
    experiment_ids = experiment_ids(~cellfun(@isempty, experiment_ids));
    
    if isempty(experiment_ids)
        warning('No successful experiments to compare.');
    else
        % Generate comparison report
        log_dir = fullfile(output_dir, 'logs');
        report_file = fullfile(output_dir, 'hyperparameter_comparison.txt');
        
        comparison_table = compareExperiments(experiment_ids, log_dir, report_file);
        
        fprintf('Comparison report generated:\n');
        fprintf('  Report file: %s\n', report_file);
        fprintf('  Experiments compared: %d\n\n', length(experiment_ids));
    end
catch ME
    fprintf('WARNING: Failed to generate comparison report\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  STEP 5: TRAIN FINAL MODEL WITH BEST CONFIGURATION
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 5: TRAINING FINAL MODEL\n');
fprintf('========================================\n\n');

fprintf('Training final model with best configuration...\n\n');

try
    % Create model with best configuration
    final_net = createModelFromConfig(best_config);
    
    % Train model
    [final_trained_net, final_training_info] = trainTireModel(final_net, X_train, Y_train, ...
                                                               X_val, Y_val, best_config);
    
    % Evaluate model
    final_metrics = evaluateModel(final_trained_net, X_val, Y_val, norm_params);
    
    % Export final model
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    final_model_path = fullfile(output_dir, sprintf('best_model_%s.mat', timestamp));
    exportModel(final_trained_net, norm_params, final_model_path);
    
    fprintf('Final model saved to: %s\n\n', final_model_path);
    
    % Generate visualizations for final model
    fprintf('Generating visualizations for final model...\n');
    Y_pred_norm = predict(final_trained_net, X_val);
    Y_pred = denormalizeData(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
    Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);
    X_val_denorm = denormalizeData(X_val, norm_params.input_mean, norm_params.input_std);
    
    viz_dir = fullfile(output_dir, sprintf('visualizations_best_%s', timestamp));
    generateAllVisualizations(X_val_denorm, Y_actual, Y_pred, viz_dir);
    
    fprintf('Visualizations saved to: %s\n\n', viz_dir);
    
catch ME
    fprintf('WARNING: Failed to train final model\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  STEP 6: SAVE TUNING SUMMARY
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 6: SAVING TUNING SUMMARY\n');
fprintf('========================================\n\n');

try
    summary_filename = sprintf('tuning_summary_%s.txt', timestamp);
    summary_path = fullfile(output_dir, summary_filename);
    
    fid = fopen(summary_path, 'w');
    fprintf(fid, '========================================\n');
    fprintf(fid, 'HYPERPARAMETER TUNING SUMMARY\n');
    fprintf(fid, '========================================\n');
    fprintf(fid, 'Date: %s\n\n', char(datetime('now')));
    
    fprintf(fid, 'SEARCH CONFIGURATION:\n');
    fprintf(fid, '  Search type: %s\n', search_type);
    if strcmpi(search_type, 'random')
        fprintf(fid, '  Number of trials: %d\n', num_random_trials);
    end
    fprintf(fid, '  Successful experiments: %d\n\n', length(experiment_ids));
    
    fprintf(fid, 'SEARCH SPACE:\n');
    fprintf(fid, '  Learning rate: [%s]\n', strjoin(cellfun(@num2str, search_space.learning_rate, 'UniformOutput', false), ', '));
    fprintf(fid, '  Batch size: [%s]\n', strjoin(cellfun(@num2str, search_space.batch_size, 'UniformOutput', false), ', '));
    fprintf(fid, '  Max epochs: [%s]\n', strjoin(cellfun(@num2str, search_space.max_epochs, 'UniformOutput', false), ', '));
    fprintf(fid, '  Patience: [%s]\n', strjoin(cellfun(@num2str, search_space.patience, 'UniformOutput', false), ', '));
    fprintf(fid, '  Dropout rate: [%s]\n\n', strjoin(cellfun(@num2str, search_space.dropout_rate, 'UniformOutput', false), ', '));
    
    fprintf(fid, 'BEST CONFIGURATION:\n');
    fprintf(fid, '  Learning rate: %.6f\n', best_config.learning_rate);
    fprintf(fid, '  Batch size: %d\n', best_config.batch_size);
    fprintf(fid, '  Max epochs: %d\n', best_config.max_epochs);
    fprintf(fid, '  Patience: %d\n', best_config.patience);
    fprintf(fid, '  Model type: %s\n', best_config.model_type);
    fprintf(fid, '  Dropout rate: %.2f\n\n', best_config.dropout_rate);
    
    if exist('final_metrics', 'var')
        fprintf(fid, 'FINAL MODEL PERFORMANCE:\n');
        fprintf(fid, '  Fx - RMSE: %.2f N, R²: %.6f\n', final_metrics.Fx.RMSE, final_metrics.Fx.R2);
        fprintf(fid, '  Fy - RMSE: %.2f N, R²: %.6f\n', final_metrics.Fy.RMSE, final_metrics.Fy.R2);
        fprintf(fid, '  Mx - RMSE: %.2f Nm, R²: %.6f\n', final_metrics.Mx.RMSE, final_metrics.Mx.R2);
        fprintf(fid, '  Mz - RMSE: %.2f Nm, R²: %.6f\n', final_metrics.Mz.RMSE, final_metrics.Mz.R2);
        fprintf(fid, '  Meets requirements: %s\n\n', mat2str(final_metrics.meets_requirements));
    end
    
    fprintf(fid, 'OUTPUT FILES:\n');
    fprintf(fid, '  Best model: %s\n', final_model_path);
    fprintf(fid, '  Comparison report: %s\n', report_file);
    fprintf(fid, '  Visualizations: %s\n', viz_dir);
    fprintf(fid, '  Experiment logs: %s\n', log_dir);
    fprintf(fid, '========================================\n');
    
    fclose(fid);
    
    fprintf('Tuning summary saved to: %s\n\n', summary_path);
catch ME
    fprintf('WARNING: Failed to save tuning summary\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  WORKFLOW COMPLETE
%  ========================================================================

fprintf('========================================\n');
fprintf('HYPERPARAMETER TUNING COMPLETE\n');
fprintf('========================================\n');
fprintf('Completed: %s\n\n', char(datetime('now')));

fprintf('RESULTS:\n');
fprintf('  Best model: %s\n', final_model_path);
fprintf('  Comparison report: %s\n', report_file);
fprintf('  Visualizations: %s\n', viz_dir);
fprintf('  Tuning summary: %s\n\n', summary_path);

if exist('final_metrics', 'var')
    if final_metrics.meets_requirements
        fprintf('✓ Best model meets all performance requirements!\n');
        fprintf('  Ready for deployment and integration.\n\n');
    else
        fprintf('⚠ Best model does not meet all requirements.\n');
        fprintf('  Consider expanding search space or adjusting architecture.\n\n');
    end
end

fprintf('Next steps:\n');
fprintf('  1. Review comparison report: %s\n', report_file);
fprintf('  2. Check visualizations: %s\n', viz_dir);
fprintf('  3. Use best model: load(''%s'')\n', final_model_path);
fprintf('  4. Review individual experiment logs in: %s\n\n', log_dir);

fprintf('========================================\n\n');

%% ========================================================================
%  NOTES ON SEARCH SPACE CUSTOMIZATION
%  ========================================================================
% 
% To customize the search space, modify the search_space structure above.
% 
% Example 1: Add more learning rate options
%   search_space.learning_rate = {0.01, 0.005, 0.001, 0.0005, 0.0001};
% 
% Example 2: Try different architectures
%   search_space.model_type = {'feedforward', 'lstm'};
% 
% Example 3: Explore different batch sizes
%   search_space.batch_size = {64, 128, 256, 512, 1024};
% 
% Example 4: Test longer training
%   search_space.max_epochs = {200, 300, 400, 500};
% 
% For grid search, be aware that the total number of experiments is the
% product of all option counts. For example:
%   3 learning rates × 3 batch sizes × 3 epochs = 27 experiments
% 
% For large search spaces, use random search instead:
%   search_type = 'random';
%   num_random_trials = 20;  % Sample 20 random configurations
