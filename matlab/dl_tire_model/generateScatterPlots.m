function generateScatterPlots(Y_actual, Y_pred, output_dir)
% GENERATESCATTERPLOTS Create scatter plots of predicted vs actual values
%
% Inputs:
%   Y_actual - N×4 matrix of actual values [Fx, Fy, Mx, Mz]
%   Y_pred   - N×4 matrix of predicted values [Fx, Fy, Mx, Mz]
%   output_dir - Directory path to save plots
%
% Requirements: 5.1, 5.5
%
% This function creates scatter plots for each output (Fx, Fy, Mx, Mz)
% comparing predicted vs actual values with a perfect prediction reference line.

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
    
    % Generate scatter plot for each output
    for i = 1:4
        figure('Position', [100, 100, 800, 600]);
        
        % Create scatter plot with transparency
        scatter(Y_actual(:,i), Y_pred(:,i), 10, 'filled', ...
                'MarkerFaceAlpha', 0.3, 'MarkerFaceColor', [0, 0.4470, 0.7410]);
        hold on;
        
        % Add perfect prediction reference line (y = x)
        min_val = min([Y_actual(:,i); Y_pred(:,i)]);
        max_val = max([Y_actual(:,i); Y_pred(:,i)]);
        plot([min_val, max_val], [min_val, max_val], 'r--', 'LineWidth', 2, ...
             'DisplayName', 'Perfect Prediction');
        
        % Labels and title
        xlabel(['Actual ', output_names{i}, ' (', output_units{i}, ')'], ...
               'FontSize', 12, 'FontWeight', 'bold');
        ylabel(['Predicted ', output_names{i}, ' (', output_units{i}, ')'], ...
               'FontSize', 12, 'FontWeight', 'bold');
        title(['Predicted vs Actual: ', output_names{i}], ...
              'FontSize', 14, 'FontWeight', 'bold');
        
        % Add grid and legend
        grid on;
        legend('Data', 'Perfect Prediction', 'Location', 'best');
        
        % Compute and display R² on plot
        SS_res = sum((Y_actual(:,i) - Y_pred(:,i)).^2);
        SS_tot = sum((Y_actual(:,i) - mean(Y_actual(:,i))).^2);
        R2 = 1 - SS_res / SS_tot;
        
        % Add R² text box
        text(0.05, 0.95, sprintf('R² = %.4f', R2), ...
             'Units', 'normalized', 'FontSize', 11, ...
             'BackgroundColor', 'white', 'EdgeColor', 'black', ...
             'VerticalAlignment', 'top');
        
        % Make plot square for better visualization
        axis square;
        
        % Save plot
        output_file = fullfile(output_dir, ['scatter_', output_names{i}, '.png']);
        saveas(gcf, output_file);
        fprintf('Saved scatter plot: %s\n', output_file);
        
        % Close figure to free memory
        close(gcf);
    end
    
    fprintf('All scatter plots generated successfully in: %s\n', output_dir);
end
