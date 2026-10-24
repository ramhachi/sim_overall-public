function generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir, varargin)
% GENERATEALLVISUALIZATIONS Generate all visualization plots for model evaluation
%
% Inputs:
%   X_features - N×5 matrix of input features [SR, SA, Fz, IA, TSTC]
%   Y_actual   - N×4 matrix of actual values [Fx, Fy, Mx, Mz]
%   Y_pred     - N×4 matrix of predicted values [Fx, Fy, Mx, Mz]
%   output_dir - Directory path to save plots
%   varargin   - Optional parameters passed to individual plot functions
%
% Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6
%
% This is a convenience function that generates all visualization types:
% - Scatter plots (predicted vs actual)
% - Residual plots
% - Time series plots
% - Performance by operating condition plots

    fprintf('=== Generating All Visualizations ===\n\n');
    
    % Create main output directory
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
        fprintf('Created output directory: %s\n\n', output_dir);
    end
    
    % Generate scatter plots
    fprintf('1. Generating scatter plots...\n');
    try
        generateScatterPlots(Y_actual, Y_pred, output_dir);
        fprintf('✓ Scatter plots completed\n\n');
    catch ME
        warning('generateAllVisualizations:ScatterPlotError', 'Failed to generate scatter plots: %s', ME.message);
    end
    
    % Generate residual plots
    fprintf('2. Generating residual plots...\n');
    try
        generateResidualPlots(Y_actual, Y_pred, output_dir);
        fprintf('✓ Residual plots completed\n\n');
    catch ME
        warning('generateAllVisualizations:ResidualPlotError', 'Failed to generate residual plots: %s', ME.message);
    end
    
    % Generate time series plots
    fprintf('3. Generating time series plots...\n');
    try
        generateTimeSeriesPlots(Y_actual, Y_pred, output_dir, varargin{:});
        fprintf('✓ Time series plots completed\n\n');
    catch ME
        warning('generateAllVisualizations:TimeSeriesPlotError', 'Failed to generate time series plots: %s', ME.message);
    end
    
    % Generate operating condition plots
    fprintf('4. Generating operating condition plots...\n');
    try
        generateOperatingConditionPlots(X_features, Y_actual, Y_pred, output_dir, varargin{:});
        fprintf('✓ Operating condition plots completed\n\n');
    catch ME
        warning('generateAllVisualizations:OperatingConditionPlotError', 'Failed to generate operating condition plots: %s', ME.message);
    end
    
    fprintf('=== All Visualizations Complete ===\n');
    fprintf('Output directory: %s\n', output_dir);
    
    % List all generated files
    files = dir(fullfile(output_dir, '*.png'));
    fprintf('\nGenerated %d plot files:\n', length(files));
    for i = 1:length(files)
        fprintf('  - %s\n', files(i).name);
    end
end
