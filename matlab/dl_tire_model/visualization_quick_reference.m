% VISUALIZATION QUICK REFERENCE
% Quick reference guide for using visualization functions
%
% This script demonstrates common usage patterns for the visualization
% functions in the MATLAB DL Tire Model.

%% Example 1: Generate All Visualizations (Recommended)
% This is the easiest way to generate all plots at once

% Assume you have:
% - X_features: N×5 matrix of input features [SR, SA, Fz, IA, TSTC]
% - Y_actual: N×4 matrix of actual values [Fx, Fy, Mx, Mz]
% - Y_pred: N×4 matrix of predicted values [Fx, Fy, Mx, Mz]

% Generate all visualizations
output_dir = 'output/visualizations';
generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);

% This creates 20 plot files:
% - 4 scatter plots
% - 4 residual plots
% - 5 time series plots
% - 7 operating condition plots

%% Example 2: Generate Individual Plot Types
% If you only need specific plot types

% Scatter plots only
generateScatterPlots(Y_actual, Y_pred, 'output/scatter');

% Residual plots only
generateResidualPlots(Y_actual, Y_pred, 'output/residual');

% Time series plots only
generateTimeSeriesPlots(Y_actual, Y_pred, 'output/timeseries');

% Operating condition plots only
generateOperatingConditionPlots(X_features, Y_actual, Y_pred, 'output/conditions');

%% Example 3: Custom Parameters
% Customize visualization behavior

% Time series with limited samples (for large datasets)
generateTimeSeriesPlots(Y_actual, Y_pred, 'output/timeseries', ...
                       'MaxSamples', 500);

% Operating condition plots with more bins
generateOperatingConditionPlots(X_features, Y_actual, Y_pred, ...
                               'output/conditions', 'NumBins', 10);

% Time series with custom time vector
time_vector = linspace(0, 10, size(Y_actual, 1));  % 0 to 10 seconds
generateTimeSeriesPlots(Y_actual, Y_pred, 'output/timeseries', ...
                       'TimeVector', time_vector);

%% Example 4: Integration with Model Evaluation
% Complete workflow from training to visualization

% 1. Preprocess data
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
config.split_ratio = 0.8;
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);

% 2. Create and train model
net = createTireModel();
train_config = struct('max_epochs', 200, 'batch_size', 256, 'random_seed', 42);
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, train_config);

% 3. Make predictions on validation set
Y_pred_norm = predict(trained_net, X_val);

% 4. Denormalize predictions and actual values
Y_pred = denormalizeData(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);

% 5. Denormalize features for operating condition plots
X_features = denormalizeData(X_val, norm_params.input_mean, norm_params.input_std);

% 6. Generate all visualizations
generateAllVisualizations(X_features, Y_actual, Y_pred, 'output/model_evaluation');

%% Example 5: Batch Processing Multiple Models
% Compare visualizations from different models or configurations

models = {'model_v1', 'model_v2', 'model_v3'};

for i = 1:length(models)
    % Load model
    model_file = sprintf('models/%s.mat', models{i});
    load(model_file, 'trained_net', 'norm_params');
    
    % Make predictions
    Y_pred_norm = predict(trained_net, X_val);
    Y_pred = denormalizeData(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
    
    % Generate visualizations in separate directory
    output_dir = sprintf('output/comparison/%s', models{i});
    generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);
end

%% Example 6: Error Handling
% Proper error handling when generating visualizations

try
    generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);
    fprintf('Visualizations generated successfully!\n');
catch ME
    fprintf('Error generating visualizations: %s\n', ME.message);
    fprintf('Check that:\n');
    fprintf('  - Y_actual and Y_pred have same dimensions\n');
    fprintf('  - Y_actual and Y_pred have 4 columns [Fx, Fy, Mx, Mz]\n');
    fprintf('  - X_features has 5 columns [SR, SA, Fz, IA, TSTC]\n');
    fprintf('  - All matrices have same number of rows\n');
end

%% Example 7: Selective Visualization
% Generate only specific plots based on needs

% For quick model checking: scatter plots only
generateScatterPlots(Y_actual, Y_pred, 'output/quick_check');

% For error analysis: residual plots only
generateResidualPlots(Y_actual, Y_pred, 'output/error_analysis');

% For temporal analysis: time series only
generateTimeSeriesPlots(Y_actual, Y_pred, 'output/temporal');

% For operating condition analysis: condition plots only
generateOperatingConditionPlots(X_features, Y_actual, Y_pred, 'output/conditions');

%% Example 8: Programmatic Access to Generated Files
% List and process generated visualization files

output_dir = 'output/visualizations';
generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);

% List all generated PNG files
files = dir(fullfile(output_dir, '*.png'));
fprintf('Generated %d visualization files:\n', length(files));
for i = 1:length(files)
    fprintf('  %d. %s\n', i, files(i).name);
end

% Copy specific plots to report directory
report_dir = 'output/report_figures';
if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

% Copy scatter plots for report
scatter_files = {'scatter_Fx.png', 'scatter_Fy.png'};
for i = 1:length(scatter_files)
    src = fullfile(output_dir, scatter_files{i});
    dst = fullfile(report_dir, scatter_files{i});
    copyfile(src, dst);
end

%% Tips and Best Practices

% 1. Always denormalize data before visualization
%    - Visualizations should show physical units (N, Nm, deg, etc.)
%    - Normalized data is harder to interpret

% 2. Use generateAllVisualizations for comprehensive analysis
%    - Provides complete picture of model performance
%    - Only takes a few seconds to generate all plots

% 3. Use individual functions for focused analysis
%    - Faster when you only need specific plot types
%    - Useful for iterative model development

% 4. Customize parameters for large datasets
%    - Use MaxSamples to limit time series plot size
%    - Use NumBins to control operating condition granularity

% 5. Organize output directories by experiment
%    - Use descriptive directory names
%    - Keep visualizations from different models separate

% 6. Check generated plots regularly during development
%    - Scatter plots reveal systematic errors
%    - Residual plots show error patterns
%    - Time series plots reveal temporal issues
%    - Operating condition plots show where model struggles

fprintf('\nVisualization Quick Reference Complete!\n');
fprintf('See individual function help for more details:\n');
fprintf('  help generateScatterPlots\n');
fprintf('  help generateResidualPlots\n');
fprintf('  help generateTimeSeriesPlots\n');
fprintf('  help generateOperatingConditionPlots\n');
fprintf('  help generateAllVisualizations\n');
