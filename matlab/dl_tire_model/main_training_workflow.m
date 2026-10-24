% MAIN_TRAINING_WORKFLOW Complete workflow for training tire model
%
% This script demonstrates the complete workflow for training a deep learning
% tire model from data loading to model export. It serves as the main entry
% point for training new models.
%
% Workflow Steps:
%   1. Load data from CSV files
%   2. Preprocess data (normalize, split)
%   3. Create neural network model
%   4. Train model with GPU acceleration
%   5. Evaluate model performance
%   6. Generate visualizations
%   7. Export trained model
%
% Requirements: 10.2
%
% Usage:
%   1. Edit the configuration section below to specify data paths and parameters
%   2. Run this script in MATLAB: >> main_training_workflow
%   3. Check the output directory for results
%
% Example:
%   >> main_training_workflow
%   % Trains model on default TTC data and saves to output/

%% ========================================================================
%  CONFIGURATION
%  ========================================================================

% Clear workspace and command window
clear; clc; close all;

fprintf('========================================\n');
fprintf('TIRE MODEL TRAINING WORKFLOW\n');
fprintf('========================================\n');
fprintf('Started: %s\n\n', char(datetime('now')));

% Data paths - specify your TTC tire test data files
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};

% Output directory for results
output_dir = 'output';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Training configuration
config = struct();
config.split_ratio = 0.8;          % 80% training, 20% validation
config.learning_rate = 0.001;      % Initial learning rate
config.batch_size = 256;           % Mini-batch size
config.max_epochs = 200;           % Maximum training epochs
config.patience = 10;              % Early stopping patience
config.val_frequency = 50;         % Validation frequency (iterations)
config.random_seed = 42;           % Random seed for reproducibility
config.model_type = 'feedforward'; % Model architecture type

fprintf('Configuration:\n');
fprintf('  Data files: %d\n', length(data_paths));
fprintf('  Split ratio: %.2f\n', config.split_ratio);
fprintf('  Learning rate: %.6f\n', config.learning_rate);
fprintf('  Batch size: %d\n', config.batch_size);
fprintf('  Max epochs: %d\n', config.max_epochs);
fprintf('  Model type: %s\n\n', config.model_type);

%% ========================================================================
%  STEP 1: LOAD AND PREPROCESS DATA
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 1: DATA LOADING AND PREPROCESSING\n');
fprintf('========================================\n\n');

try
    [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);
catch ME
    fprintf('ERROR: Data preprocessing failed\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Identifier: %s\n', ME.identifier);
    error('Cannot continue without valid data. Please check data paths and format.');
end

%% ========================================================================
%  STEP 2: CREATE MODEL
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 2: MODEL CREATION\n');
fprintf('========================================\n\n');

try
    net = createModelFromConfig(config);
    fprintf('Model architecture created successfully\n');
    fprintf('Model type: %s\n\n', config.model_type);
catch ME
    fprintf('ERROR: Model creation failed\n');
    fprintf('Message: %s\n', ME.message);
    error('Cannot continue without valid model architecture.');
end

%% ========================================================================
%  STEP 3: TRAIN MODEL
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 3: MODEL TRAINING\n');
fprintf('========================================\n\n');

try
    [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
catch ME
    fprintf('ERROR: Model training failed\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Identifier: %s\n', ME.identifier);
    error('Training failed. Check error message above for details.');
end

%% ========================================================================
%  STEP 4: EVALUATE MODEL
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 4: MODEL EVALUATION\n');
fprintf('========================================\n\n');

try
    metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
catch ME
    fprintf('ERROR: Model evaluation failed\n');
    fprintf('Message: %s\n', ME.message);
    warning('Evaluation failed, but model was trained. Continuing...');
    metrics = struct();
    metrics.meets_requirements = false;
end

%% ========================================================================
%  STEP 5: GENERATE VISUALIZATIONS
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 5: VISUALIZATION GENERATION\n');
fprintf('========================================\n\n');

try
    % Make predictions for visualization
    Y_pred_norm = predict(trained_net, X_val);
    Y_pred = denormalizeData(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
    Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);
    
    % Denormalize input features for operating condition plots
    X_val_denorm = denormalizeData(X_val, norm_params.input_mean, norm_params.input_std);
    
    % Generate all visualizations
    viz_dir = fullfile(output_dir, 'visualizations');
    generateAllVisualizations(X_val_denorm, Y_actual, Y_pred, viz_dir);
    
    fprintf('Visualizations saved to: %s\n\n', viz_dir);
catch ME
    fprintf('WARNING: Visualization generation failed\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('Continuing without visualizations...\n\n');
end

%% ========================================================================
%  STEP 6: EXPORT MODEL
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 6: MODEL EXPORT\n');
fprintf('========================================\n\n');

try
    % Export model with timestamp
    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    model_filename = sprintf('tire_model_%s.mat', timestamp);
    model_path = fullfile(output_dir, model_filename);
    
    exportModel(trained_net, norm_params, model_path);
    
    % Also save with a generic name for easy loading
    generic_model_path = fullfile(output_dir, 'tire_model_latest.mat');
    exportModel(trained_net, norm_params, generic_model_path);
    
    fprintf('Model exported to:\n');
    fprintf('  %s\n', model_path);
    fprintf('  %s\n\n', generic_model_path);
catch ME
    fprintf('ERROR: Model export failed\n');
    fprintf('Message: %s\n', ME.message);
    warning('Model was trained but export failed. Model is still in workspace.');
end

%% ========================================================================
%  STEP 7: SAVE TRAINING SUMMARY
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 7: SAVING TRAINING SUMMARY\n');
fprintf('========================================\n\n');

try
    summary_filename = sprintf('training_summary_%s.txt', timestamp);
    summary_path = fullfile(output_dir, summary_filename);
    
    fid = fopen(summary_path, 'w');
    fprintf(fid, '========================================\n');
    fprintf(fid, 'TIRE MODEL TRAINING SUMMARY\n');
    fprintf(fid, '========================================\n');
    fprintf(fid, 'Date: %s\n\n', char(datetime('now')));
    
    fprintf(fid, 'DATA:\n');
    fprintf(fid, '  Training samples: %d\n', size(X_train, 1));
    fprintf(fid, '  Validation samples: %d\n', size(X_val, 1));
    fprintf(fid, '  Input features: 5 (SR, SA, Fz, IA, TSTC)\n');
    fprintf(fid, '  Output targets: 4 (Fx, Fy, Mx, Mz)\n\n');
    
    fprintf(fid, 'CONFIGURATION:\n');
    fprintf(fid, '  Model type: %s\n', config.model_type);
    fprintf(fid, '  Learning rate: %.6f\n', config.learning_rate);
    fprintf(fid, '  Batch size: %d\n', config.batch_size);
    fprintf(fid, '  Max epochs: %d\n', config.max_epochs);
    fprintf(fid, '  Patience: %d\n', config.patience);
    fprintf(fid, '  Random seed: %d\n\n', config.random_seed);
    
    fprintf(fid, 'TRAINING RESULTS:\n');
    fprintf(fid, '  Training time: %.2f seconds (%.2f minutes)\n', ...
            training_info.TrainingTime, training_info.TrainingTime / 60);
    fprintf(fid, '  Final validation loss: %.6f\n\n', training_info.FinalValidationLoss);
    
    if ~isempty(fieldnames(metrics))
        fprintf(fid, 'PERFORMANCE METRICS:\n');
        fprintf(fid, '  Fx - RMSE: %.2f N, R²: %.6f, TIC: %.6f\n', ...
                metrics.Fx.RMSE, metrics.Fx.R2, metrics.Fx.TIC);
        fprintf(fid, '  Fy - RMSE: %.2f N, R²: %.6f, TIC: %.6f\n', ...
                metrics.Fy.RMSE, metrics.Fy.R2, metrics.Fy.TIC);
        fprintf(fid, '  Mx - RMSE: %.2f Nm, R²: %.6f, TIC: %.6f\n', ...
                metrics.Mx.RMSE, metrics.Mx.R2, metrics.Mx.TIC);
        fprintf(fid, '  Mz - RMSE: %.2f Nm, R²: %.6f, TIC: %.6f\n\n', ...
                metrics.Mz.RMSE, metrics.Mz.R2, metrics.Mz.TIC);
        
        fprintf(fid, 'REQUIREMENTS:\n');
        fprintf(fid, '  Meets requirements: %s\n', mat2str(metrics.meets_requirements));
        if ~metrics.meets_requirements
            fprintf(fid, '  Warning: Some performance thresholds not met\n');
        end
    end
    
    fprintf(fid, '\n========================================\n');
    fprintf(fid, 'OUTPUT FILES:\n');
    fprintf(fid, '  Model: %s\n', model_path);
    fprintf(fid, '  Visualizations: %s\n', viz_dir);
    fprintf(fid, '========================================\n');
    
    fclose(fid);
    
    fprintf('Training summary saved to: %s\n\n', summary_path);
catch ME
    fprintf('WARNING: Failed to save training summary\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  WORKFLOW COMPLETE
%  ========================================================================

fprintf('========================================\n');
fprintf('WORKFLOW COMPLETE\n');
fprintf('========================================\n');
fprintf('Completed: %s\n\n', char(datetime('now')));

fprintf('RESULTS:\n');
fprintf('  Model saved: %s\n', model_path);
fprintf('  Visualizations: %s\n', viz_dir);
fprintf('  Summary: %s\n\n', summary_path);

if ~isempty(fieldnames(metrics)) && metrics.meets_requirements
    fprintf('✓ Model meets all performance requirements!\n');
    fprintf('  Ready for deployment and integration.\n\n');
else
    fprintf('⚠ Model training complete but may not meet all requirements.\n');
    fprintf('  Review metrics and consider hyperparameter tuning.\n\n');
end

fprintf('Next steps:\n');
fprintf('  1. Review visualizations in: %s\n', viz_dir);
fprintf('  2. Check training summary: %s\n', summary_path);
fprintf('  3. Use model for predictions: load(''%s'')\n', model_path);
fprintf('  4. If needed, run hyperparameter tuning: main_hyperparameter_tuning\n\n');

fprintf('========================================\n\n');
