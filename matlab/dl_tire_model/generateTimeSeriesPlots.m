function generateTimeSeriesPlots(Y_actual, Y_pred, output_dir, varargin)
% GENERATETIMESERIESPLOTS Create time series plots for sequential data
%
% Inputs:
%   Y_actual - N×4 matrix of actual values [Fx, Fy, Mx, Mz]
%   Y_pred   - N×4 matrix of predicted values [Fx, Fy, Mx, Mz]
%   output_dir - Directory path to save plots
%   varargin - Optional parameters:
%              'TimeVector' - Custom time vector (default: 1:N)
%              'MaxSamples' - Maximum samples to plot (default: 1000)
%
% Requirements: 5.3, 5.5
%
% This function creates time series plots for sequential test data,
% showing both actual and predicted values over time.

    % Parse optional inputs
    p = inputParser;
    addParameter(p, 'TimeVector', [], @isnumeric);
    addParameter(p, 'MaxSamples', 1000, @isnumeric);
    parse(p, varargin{:});
    
    time_vector = p.Results.TimeVector;
    max_samples = p.Results.MaxSamples;
    
    % Validate inputs
    if size(Y_actual, 2) ~= 4 || size(Y_pred, 2) ~= 4
        error('Y_actual and Y_pred must have 4 columns [Fx, Fy, Mx, Mz]');
    end
    
    if size(Y_actual, 1) ~= size(Y_pred, 1)
        error('Y_actual and Y_pred must have the same number of rows');
    end
    
    % Create output directory if it doesn't exist
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Create default time vector if not provided
    n_samples = size(Y_actual, 1);
    if isempty(time_vector)
        time_vector = 1:n_samples;
    end
    
    % Subsample if dataset is too large
    if n_samples > max_samples
        fprintf('Dataset has %d samples. Subsampling to %d for visualization.\n', ...
                n_samples, max_samples);
        indices = round(linspace(1, n_samples, max_samples));
        Y_actual = Y_actual(indices, :);
        Y_pred = Y_pred(indices, :);
        time_vector = time_vector(indices);
        n_samples = max_samples;
    end
    
    % Output names
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    output_units = {'N', 'N', 'Nm', 'Nm'};
    
    % Generate time series plot for each output
    for i = 1:4
        figure('Position', [100, 100, 1000, 600]);
        
        % Plot actual values
        plot(time_vector, Y_actual(:,i), 'b-', 'LineWidth', 1.5, ...
             'DisplayName', 'Actual');
        hold on;
        
        % Plot predicted values
        plot(time_vector, Y_pred(:,i), 'r--', 'LineWidth', 1.5, ...
             'DisplayName', 'Predicted');
        
        % Labels and title
        xlabel('Sample Index', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel([output_names{i}, ' (', output_units{i}, ')'], ...
               'FontSize', 12, 'FontWeight', 'bold');
        title(['Time Series: ', output_names{i}], ...
              'FontSize', 14, 'FontWeight', 'bold');
        
        % Add grid and legend
        grid on;
        legend('Location', 'best');
        
        % Compute and display RMSE
        rmse = sqrt(mean((Y_pred(:,i) - Y_actual(:,i)).^2));
        
        % Add RMSE text box
        text(0.05, 0.95, sprintf('RMSE: %.2f %s', rmse, output_units{i}), ...
             'Units', 'normalized', 'FontSize', 11, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black', ...
             'VerticalAlignment', 'top');
        
        % Save plot
        output_file = fullfile(output_dir, ['timeseries_', output_names{i}, '.png']);
        saveas(gcf, output_file);
        fprintf('Saved time series plot: %s\n', output_file);
        
        % Close figure to free memory
        close(gcf);
    end
    
    % Create combined plot with all outputs
    figure('Position', [100, 100, 1200, 800]);
    
    for i = 1:4
        subplot(2, 2, i);
        
        % Plot actual and predicted
        plot(time_vector, Y_actual(:,i), 'b-', 'LineWidth', 1.2, ...
             'DisplayName', 'Actual');
        hold on;
        plot(time_vector, Y_pred(:,i), 'r--', 'LineWidth', 1.2, ...
             'DisplayName', 'Predicted');
        
        % Labels and title
        xlabel('Sample Index', 'FontSize', 10);
        ylabel([output_names{i}, ' (', output_units{i}, ')'], 'FontSize', 10);
        title(output_names{i}, 'FontSize', 12, 'FontWeight', 'bold');
        
        % Grid and legend
        grid on;
        legend('Location', 'best', 'FontSize', 8);
        
        % Add RMSE
        rmse = sqrt(mean((Y_pred(:,i) - Y_actual(:,i)).^2));
        text(0.05, 0.95, sprintf('RMSE: %.1f', rmse), ...
             'Units', 'normalized', 'FontSize', 9, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black', ...
             'VerticalAlignment', 'top');
    end
    
    % Overall title
    sgtitle('Time Series Comparison: All Outputs', ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    % Save combined plot
    output_file = fullfile(output_dir, 'timeseries_combined.png');
    saveas(gcf, output_file);
    fprintf('Saved combined time series plot: %s\n', output_file);
    
    % Close figure
    close(gcf);
    
    fprintf('All time series plots generated successfully in: %s\n', output_dir);
end
