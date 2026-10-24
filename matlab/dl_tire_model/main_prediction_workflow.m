% MAIN_PREDICTION_WORKFLOW Use trained tire model for predictions
%
% This script demonstrates how to load a trained tire model and use it to
% make predictions on new input data. It serves as the main entry point for
% using trained models in production or for testing.
%
% Workflow Steps:
%   1. Load exported model
%   2. Accept new input data
%   3. Make predictions
%   4. Display results
%
% Requirements: 10.5
%
% Usage:
%   1. Edit the configuration section below to specify model path and input data
%   2. Run this script in MATLAB: >> main_prediction_workflow
%   3. View prediction results in console and output files
%
% Example:
%   >> main_prediction_workflow
%   % Loads latest model and makes predictions on sample data

%% ========================================================================
%  CONFIGURATION
%  ========================================================================

% Clear workspace and command window
clear; clc; close all;

fprintf('========================================\n');
fprintf('TIRE MODEL PREDICTION WORKFLOW\n');
fprintf('========================================\n');
fprintf('Started: %s\n\n', char(datetime('now')));

% Model path - specify your trained model file
model_path = 'output/tire_model_latest.mat';

% Check if model exists
if ~isfile(model_path)
    error(['Model file not found: %s\n' ...
           'Please train a model first using main_training_workflow.m'], model_path);
end

%% ========================================================================
%  STEP 1: LOAD TRAINED MODEL
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 1: LOADING TRAINED MODEL\n');
fprintf('========================================\n\n');

try
    fprintf('Loading model from: %s\n', model_path);
    model_data = load(model_path);
    
    % Extract model components
    trained_net = model_data.trained_net;
    norm_params = model_data.norm_params;
    
    fprintf('✓ Model loaded successfully\n');
    fprintf('  Network type: %s\n', class(trained_net));
    fprintf('  Normalization parameters loaded\n\n');
catch ME
    fprintf('ERROR: Failed to load model\n');
    fprintf('Message: %s\n', ME.message);
    error('Cannot continue without valid model.');
end

%% ========================================================================
%  STEP 2: PREPARE INPUT DATA
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 2: PREPARING INPUT DATA\n');
fprintf('========================================\n\n');

% Option 1: Define input data manually
% Each row is one sample with [SR, SA, Fz, IA, TSTC]
fprintf('Creating sample input data...\n\n');

% Example 1: Straight-line braking
input_1 = [-0.1, 0.0, 1500, 0.0, 30.0];  % Braking, no slip angle

% Example 2: Cornering at moderate load
input_2 = [0.0, 5.0, 1500, -2.0, 35.0];  % Pure cornering, 5° slip angle

% Example 3: Combined slip (braking + cornering)
input_3 = [-0.05, 3.0, 1800, -1.5, 40.0];  % Combined slip

% Example 4: High load cornering
input_4 = [0.0, 8.0, 2000, -3.0, 45.0];  % High load, high slip angle

% Example 5: Low load acceleration
input_5 = [0.08, 0.0, 1000, 0.0, 25.0];  % Acceleration, low load

% Combine into input matrix
X_raw = [input_1; input_2; input_3; input_4; input_5];

% Display input data
fprintf('Input data prepared:\n');
fprintf('%-10s | %-8s | %-8s | %-8s | %-8s\n', 'Sample', 'SR', 'SA (°)', 'Fz (N)', 'IA (°)', 'TSTC (°C)');
fprintf('-----------|----------|----------|----------|----------|----------\n');
for i = 1:size(X_raw, 1)
    fprintf('Sample %-3d | %8.3f | %8.2f | %8.1f | %8.2f | %8.1f\n', ...
            i, X_raw(i,1), X_raw(i,2), X_raw(i,3), X_raw(i,4), X_raw(i,5));
end
fprintf('\n');

% Option 2: Load input data from CSV file (uncomment to use)
% fprintf('Loading input data from CSV file...\n');
% input_file = 'input_data.csv';
% if isfile(input_file)
%     input_table = readtable(input_file);
%     X_raw = [input_table.SR, input_table.SA, input_table.Fz, ...
%              input_table.IA, input_table.TSTC];
%     fprintf('Loaded %d samples from %s\n\n', size(X_raw, 1), input_file);
% else
%     error('Input file not found: %s', input_file);
% end

%% ========================================================================
%  STEP 3: MAKE PREDICTIONS
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 3: MAKING PREDICTIONS\n');
fprintf('========================================\n\n');

try
    fprintf('Computing tire forces and moments...\n');
    
    % Make predictions using the prediction function
    Y_pred = predictTireForces(trained_net, X_raw, norm_params);
    
    fprintf('✓ Predictions complete for %d samples\n\n', size(X_raw, 1));
catch ME
    fprintf('ERROR: Prediction failed\n');
    fprintf('Message: %s\n', ME.message);
    error('Cannot continue without valid predictions.');
end

%% ========================================================================
%  STEP 4: DISPLAY RESULTS
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 4: PREDICTION RESULTS\n');
fprintf('========================================\n\n');

% Display results in table format
fprintf('PREDICTION RESULTS:\n');
fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', 'Sample', 'Fx (N)', 'Fy (N)', 'Mx (Nm)', 'Mz (Nm)');
fprintf('-----------|------------|------------|------------|------------\n');
for i = 1:size(Y_pred, 1)
    fprintf('Sample %-3d | %10.2f | %10.2f | %10.2f | %10.2f\n', ...
            i, Y_pred(i,1), Y_pred(i,2), Y_pred(i,3), Y_pred(i,4));
end
fprintf('\n');

% Display detailed results for each sample
fprintf('DETAILED RESULTS:\n');
fprintf('========================================\n\n');

for i = 1:size(X_raw, 1)
    fprintf('Sample %d:\n', i);
    fprintf('  Inputs:\n');
    fprintf('    Slip Ratio (SR):        %8.3f\n', X_raw(i,1));
    fprintf('    Slip Angle (SA):        %8.2f °\n', X_raw(i,2));
    fprintf('    Vertical Load (Fz):     %8.1f N\n', X_raw(i,3));
    fprintf('    Inclination Angle (IA): %8.2f °\n', X_raw(i,4));
    fprintf('    Tire Temperature (TSTC):%8.1f °C\n', X_raw(i,5));
    fprintf('  Predictions:\n');
    fprintf('    Longitudinal Force (Fx):%8.2f N\n', Y_pred(i,1));
    fprintf('    Lateral Force (Fy):     %8.2f N\n', Y_pred(i,2));
    fprintf('    Overturning Moment (Mx):%8.2f Nm\n', Y_pred(i,3));
    fprintf('    Aligning Moment (Mz):   %8.2f Nm\n', Y_pred(i,4));
    fprintf('\n');
end

%% ========================================================================
%  STEP 5: SAVE RESULTS (OPTIONAL)
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 5: SAVING RESULTS\n');
fprintf('========================================\n\n');

% Create output directory
output_dir = 'output/predictions';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Save results to CSV file
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_filename = sprintf('predictions_%s.csv', timestamp);
output_path = fullfile(output_dir, output_filename);

try
    % Create results table
    results_table = table(X_raw(:,1), X_raw(:,2), X_raw(:,3), X_raw(:,4), X_raw(:,5), ...
                          Y_pred(:,1), Y_pred(:,2), Y_pred(:,3), Y_pred(:,4), ...
                          'VariableNames', {'SR', 'SA', 'Fz', 'IA', 'TSTC', ...
                                           'Fx', 'Fy', 'Mx', 'Mz'});
    
    % Write to CSV
    writetable(results_table, output_path);
    
    fprintf('✓ Results saved to: %s\n\n', output_path);
catch ME
    fprintf('WARNING: Failed to save results to CSV\n');
    fprintf('Message: %s\n\n', ME.message);
end

% Save results to MAT file
mat_filename = sprintf('predictions_%s.mat', timestamp);
mat_path = fullfile(output_dir, mat_filename);

try
    save(mat_path, 'X_raw', 'Y_pred', 'timestamp');
    fprintf('✓ Results saved to: %s\n\n', mat_path);
catch ME
    fprintf('WARNING: Failed to save results to MAT file\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  STEP 6: VISUALIZE PREDICTIONS (OPTIONAL)
%  ========================================================================

fprintf('========================================\n');
fprintf('STEP 6: VISUALIZING PREDICTIONS\n');
fprintf('========================================\n\n');

try
    % Create figure for force predictions
    figure('Name', 'Tire Force Predictions', 'Position', [100, 100, 1200, 800]);
    
    % Plot Fx vs SR
    subplot(2, 2, 1);
    scatter(X_raw(:,1), Y_pred(:,1), 100, 'filled');
    xlabel('Slip Ratio (SR)');
    ylabel('Longitudinal Force Fx (N)');
    title('Fx vs Slip Ratio');
    grid on;
    
    % Plot Fy vs SA
    subplot(2, 2, 2);
    scatter(X_raw(:,2), Y_pred(:,2), 100, 'filled');
    xlabel('Slip Angle SA (°)');
    ylabel('Lateral Force Fy (N)');
    title('Fy vs Slip Angle');
    grid on;
    
    % Plot Mx vs IA
    subplot(2, 2, 3);
    scatter(X_raw(:,4), Y_pred(:,3), 100, 'filled');
    xlabel('Inclination Angle IA (°)');
    ylabel('Overturning Moment Mx (Nm)');
    title('Mx vs Inclination Angle');
    grid on;
    
    % Plot Mz vs SA
    subplot(2, 2, 4);
    scatter(X_raw(:,2), Y_pred(:,4), 100, 'filled');
    xlabel('Slip Angle SA (°)');
    ylabel('Aligning Moment Mz (Nm)');
    title('Mz vs Slip Angle');
    grid on;
    
    % Save figure
    fig_filename = sprintf('predictions_plot_%s.png', timestamp);
    fig_path = fullfile(output_dir, fig_filename);
    saveas(gcf, fig_path);
    
    fprintf('✓ Visualization saved to: %s\n\n', fig_path);
catch ME
    fprintf('WARNING: Failed to create visualization\n');
    fprintf('Message: %s\n\n', ME.message);
end

%% ========================================================================
%  WORKFLOW COMPLETE
%  ========================================================================

fprintf('========================================\n');
fprintf('PREDICTION WORKFLOW COMPLETE\n');
fprintf('========================================\n');
fprintf('Completed: %s\n\n', char(datetime('now')));

fprintf('SUMMARY:\n');
fprintf('  Model used: %s\n', model_path);
fprintf('  Samples predicted: %d\n', size(X_raw, 1));
fprintf('  Results saved to: %s\n\n', output_dir);

fprintf('Next steps:\n');
fprintf('  1. Review prediction results above\n');
fprintf('  2. Check output files in: %s\n', output_dir);
fprintf('  3. Use predictions in your simulation or analysis\n');
fprintf('  4. For batch predictions, modify input data section\n\n');

fprintf('========================================\n\n');

%% ========================================================================
%  HELPER FUNCTION: BATCH PREDICTION FROM FILE
%  ========================================================================
% Uncomment and use this function to make predictions on large datasets

% function batch_predict_from_file(model_path, input_csv, output_csv)
%     % Load model
%     model_data = load(model_path);
%     trained_net = model_data.trained_net;
%     norm_params = model_data.norm_params;
%     
%     % Load input data
%     input_table = readtable(input_csv);
%     X_raw = [input_table.SR, input_table.SA, input_table.Fz, ...
%              input_table.IA, input_table.TSTC];
%     
%     % Make predictions
%     Y_pred = predictTireForces(trained_net, X_raw, norm_params);
%     
%     % Create output table
%     output_table = [input_table, array2table(Y_pred, ...
%                     'VariableNames', {'Fx', 'Fy', 'Mx', 'Mz'})];
%     
%     % Save results
%     writetable(output_table, output_csv);
%     
%     fprintf('Batch prediction complete: %d samples\n', size(X_raw, 1));
%     fprintf('Results saved to: %s\n', output_csv);
% end
