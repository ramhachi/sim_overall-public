function generateOperatingConditionPlots(X_features, Y_actual, Y_pred, output_dir, varargin)
% GENERATEOPERATINGCONDITIONPLOTS Create performance plots by operating condition
%
% Inputs:
%   X_features - N×5 matrix of input features [SR, SA, Fz, IA, TSTC]
%   Y_actual   - N×4 matrix of actual values [Fx, Fy, Mx, Mz]
%   Y_pred     - N×4 matrix of predicted values [Fx, Fy, Mx, Mz]
%   output_dir - Directory path to save plots
%   varargin   - Optional parameters:
%                'NumBins' - Number of bins for grouping (default: 5)
%
% Requirements: 5.4, 5.5
%
% This function creates plots showing prediction performance across different
% operating conditions (varying Fz, SA, SR ranges).

    % Parse optional inputs
    p = inputParser;
    addParameter(p, 'NumBins', 5, @isnumeric);
    parse(p, varargin{:});
    
    num_bins = p.Results.NumBins;
    
    % Validate inputs
    if size(X_features, 2) ~= 5
        error('X_features must have 5 columns [SR, SA, Fz, IA, TSTC]');
    end
    
    if size(Y_actual, 2) ~= 4 || size(Y_pred, 2) ~= 4
        error('Y_actual and Y_pred must have 4 columns [Fx, Fy, Mx, Mz]');
    end
    
    if size(X_features, 1) ~= size(Y_actual, 1) || size(X_features, 1) ~= size(Y_pred, 1)
        error('X_features, Y_actual, and Y_pred must have the same number of rows');
    end
    
    % Create output directory if it doesn't exist
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Feature names and units
    feature_names = {'SR', 'SA', 'Fz', 'IA', 'TSTC'};
    feature_labels = {'Slip Ratio', 'Slip Angle (deg)', 'Vertical Load (N)', ...
                      'Inclination Angle (deg)', 'Temperature (°C)'};
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    
    % Key features to analyze: SR, SA, Fz
    key_features = [1, 2, 3];  % Indices for SR, SA, Fz
    
    % Generate performance plots for each key feature
    for feat_idx = key_features
        fprintf('Generating plots for %s...\n', feature_names{feat_idx});
        
        % Get feature values
        feature_vals = X_features(:, feat_idx);
        
        % Create bins
        [~, edges, bin_indices] = histcounts(feature_vals, num_bins);
        
        % Compute RMSE for each bin and each output
        rmse_by_bin = zeros(num_bins, 4);
        r2_by_bin = zeros(num_bins, 4);
        bin_centers = zeros(num_bins, 1);
        bin_counts = zeros(num_bins, 1);
        
        for bin = 1:num_bins
            bin_mask = (bin_indices == bin);
            bin_counts(bin) = sum(bin_mask);
            
            if bin_counts(bin) > 0
                bin_centers(bin) = mean(feature_vals(bin_mask));
                
                for out = 1:4
                    % Compute RMSE
                    residuals = Y_pred(bin_mask, out) - Y_actual(bin_mask, out);
                    rmse_by_bin(bin, out) = sqrt(mean(residuals.^2));
                    
                    % Compute R²
                    SS_res = sum(residuals.^2);
                    SS_tot = sum((Y_actual(bin_mask, out) - mean(Y_actual(bin_mask, out))).^2);
                    if SS_tot > 0
                        r2_by_bin(bin, out) = 1 - SS_res / SS_tot;
                    else
                        r2_by_bin(bin, out) = NaN;
                    end
                end
            else
                bin_centers(bin) = (edges(bin) + edges(bin+1)) / 2;
                rmse_by_bin(bin, :) = NaN;
                r2_by_bin(bin, :) = NaN;
            end
        end
        
        % Plot RMSE by operating condition
        figure('Position', [100, 100, 1200, 800]);
        
        for out = 1:4
            subplot(2, 2, out);
            
            % Bar plot of RMSE
            bar(bin_centers, rmse_by_bin(:, out), 'FaceColor', [0.2, 0.6, 0.8]);
            
            % Labels and title
            xlabel(feature_labels{feat_idx}, 'FontSize', 10, 'FontWeight', 'bold');
            ylabel('RMSE', 'FontSize', 10, 'FontWeight', 'bold');
            title([output_names{out}, ' RMSE vs ', feature_names{feat_idx}], ...
                  'FontSize', 12, 'FontWeight', 'bold');
            
            % Grid
            grid on;
            
            % Add sample counts as text
            for bin = 1:num_bins
                if bin_counts(bin) > 0 && ~isnan(rmse_by_bin(bin, out))
                    text(bin_centers(bin), rmse_by_bin(bin, out), ...
                         sprintf('n=%d', bin_counts(bin)), ...
                         'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'bottom', ...
                         'FontSize', 8);
                end
            end
        end
        
        % Overall title
        sgtitle(['RMSE by ', feature_labels{feat_idx}], ...
                'FontSize', 16, 'FontWeight', 'bold');
        
        % Save plot
        output_file = fullfile(output_dir, ['rmse_by_', feature_names{feat_idx}, '.png']);
        saveas(gcf, output_file);
        fprintf('Saved RMSE plot: %s\n', output_file);
        close(gcf);
        
        % Plot R² by operating condition
        figure('Position', [100, 100, 1200, 800]);
        
        for out = 1:4
            subplot(2, 2, out);
            
            % Bar plot of R²
            bar(bin_centers, r2_by_bin(:, out), 'FaceColor', [0.8, 0.4, 0.2]);
            
            % Add reference line at R² = 0.95
            hold on;
            xlim_vals = xlim;
            plot(xlim_vals, [0.95, 0.95], 'r--', 'LineWidth', 2, ...
                 'DisplayName', 'Target (0.95)');
            
            % Labels and title
            xlabel(feature_labels{feat_idx}, 'FontSize', 10, 'FontWeight', 'bold');
            ylabel('R²', 'FontSize', 10, 'FontWeight', 'bold');
            title([output_names{out}, ' R² vs ', feature_names{feat_idx}], ...
                  'FontSize', 12, 'FontWeight', 'bold');
            
            % Grid and legend
            grid on;
            legend('Location', 'best', 'FontSize', 8);
            
            % Set y-axis limits
            ylim([0, 1]);
            
            % Add sample counts as text
            for bin = 1:num_bins
                if bin_counts(bin) > 0 && ~isnan(r2_by_bin(bin, out))
                    text(bin_centers(bin), r2_by_bin(bin, out), ...
                         sprintf('n=%d', bin_counts(bin)), ...
                         'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'bottom', ...
                         'FontSize', 8);
                end
            end
        end
        
        % Overall title
        sgtitle(['R² by ', feature_labels{feat_idx}], ...
                'FontSize', 16, 'FontWeight', 'bold');
        
        % Save plot
        output_file = fullfile(output_dir, ['r2_by_', feature_names{feat_idx}, '.png']);
        saveas(gcf, output_file);
        fprintf('Saved R² plot: %s\n', output_file);
        close(gcf);
    end
    
    % Create summary heatmap showing RMSE across all conditions
    fprintf('Generating summary heatmap...\n');
    
    figure('Position', [100, 100, 1000, 800]);
    
    % Prepare data for heatmap (features × outputs)
    heatmap_data = zeros(length(key_features), 4);
    
    for i = 1:length(key_features)
        feat_idx = key_features(i);
        feature_vals = X_features(:, feat_idx);
        
        for out = 1:4
            residuals = Y_pred(:, out) - Y_actual(:, out);
            heatmap_data(i, out) = sqrt(mean(residuals.^2));
        end
    end
    
    % Create heatmap
    h = heatmap(output_names, feature_labels(key_features), heatmap_data);
    h.Title = 'RMSE Summary: Performance Across Operating Conditions';
    h.XLabel = 'Output';
    h.YLabel = 'Operating Condition';
    h.Colormap = parula;
    h.ColorbarVisible = 'on';
    
    % Save heatmap
    output_file = fullfile(output_dir, 'rmse_heatmap_summary.png');
    saveas(gcf, output_file);
    fprintf('Saved summary heatmap: %s\n', output_file);
    close(gcf);
    
    fprintf('All operating condition plots generated successfully in: %s\n', output_dir);
end
