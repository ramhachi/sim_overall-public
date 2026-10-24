function generateResidualPlots(Y_actual, Y_pred, output_dir)
% GENERATERESIDUALPLOTS Create residual plots for model predictions
%
% Inputs:
%   Y_actual - N×4 matrix of actual values [Fx, Fy, Mx, Mz]
%   Y_pred   - N×4 matrix of predicted values [Fx, Fy, Mx, Mz]
%   output_dir - Directory path to save plots
%
% Requirements: 5.2, 5.5
%
% This function creates residual plots for each output (Fx, Fy, Mx, Mz)
% showing prediction errors with a zero reference line.

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
    
    % Output names
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    output_units = {'N', 'N', 'Nm', 'Nm'};
    
    % Generate residual plot for each output
    for i = 1:4
        figure('Position', [100, 100, 800, 600]);
        
        % Compute residuals
        residuals = Y_pred(:,i) - Y_actual(:,i);
        
        % Create scatter plot with transparency
        scatter(Y_actual(:,i), residuals, 10, 'filled', ...
                'MarkerFaceAlpha', 0.3, 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
        hold on;
        
        % Add zero reference line
        xlim_vals = xlim;
        plot(xlim_vals, [0, 0], 'r--', 'LineWidth', 2, ...
             'DisplayName', 'Zero Error');
        
        % Labels and title
        xlabel(['Actual ', output_names{i}, ' (', output_units{i}, ')'], ...
               'FontSize', 12, 'FontWeight', 'bold');
        ylabel(['Residual (Predicted - Actual) (', output_units{i}, ')'], ...
               'FontSize', 12, 'FontWeight', 'bold');
        title(['Residual Plot: ', output_names{i}], ...
              'FontSize', 14, 'FontWeight', 'bold');
        
        % Add grid and legend
        grid on;
        legend('Residuals', 'Zero Error', 'Location', 'best');
        
        % Compute and display statistics
        mean_residual = mean(residuals);
        std_residual = std(residuals);
        rmse = sqrt(mean(residuals.^2));
        
        % Add statistics text box
        stats_text = sprintf('Mean: %.2f %s\nStd: %.2f %s\nRMSE: %.2f %s', ...
                            mean_residual, output_units{i}, ...
                            std_residual, output_units{i}, ...
                            rmse, output_units{i});
        text(0.05, 0.95, stats_text, ...
             'Units', 'normalized', 'FontSize', 10, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black', ...
             'VerticalAlignment', 'top');
        
        % Save plot
        output_file = fullfile(output_dir, ['residual_', output_names{i}, '.png']);
        saveas(gcf, output_file);
        fprintf('Saved residual plot: %s\n', output_file);
        
        % Close figure to free memory
        close(gcf);
    end
    
    fprintf('All residual plots generated successfully in: %s\n', output_dir);
end
