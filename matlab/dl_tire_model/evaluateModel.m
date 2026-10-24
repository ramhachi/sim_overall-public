function metrics = evaluateModel(trained_net, X_val, Y_val, norm_params)
% EVALUATEMODEL Evaluate trained model performance on validation data
%
% This function performs comprehensive model evaluation:
% 1. Makes predictions on validation data
% 2. Denormalizes predictions and targets to physical units
% 3. Computes all performance metrics (RMSE, R², TIC, MAE)
% 4. Checks performance thresholds (RMSE < 100N, R² > 0.95)
% 5. Returns structured metrics
%
% Inputs:
%   trained_net - Trained neural network model
%   X_val       - N×5 matrix of normalized validation input features
%   Y_val       - N×4 matrix of normalized validation output targets
%   norm_params - Structure with normalization parameters:
%                 .input_mean, .input_std, .output_mean, .output_std
%
% Outputs:
%   metrics - Structure containing:
%             .Fx, .Fy, .Mx, .Mz - Structures for each output with:
%                 .RMSE - Root Mean Square Error
%                 .R2   - Coefficient of determination
%                 .TIC  - Theil Inequality Coefficient
%                 .MAE  - Mean Absolute Error
%             .meets_requirements - Boolean indicating if thresholds are met
%             .threshold_flags    - Structure with warning flags
%
% Example:
%   metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
%   fprintf('Fx RMSE: %.2f N, R²: %.4f\n', metrics.Fx.RMSE, metrics.Fx.R2);
%
% Requirements: 4.1, 4.2, 4.3, 4.4, 4.5, 4.7, 4.8

    fprintf('=== Model Evaluation ===\n\n');
    
    % Step 1: Make predictions on validation data
    fprintf('Making predictions on validation data...\n');
    Y_pred_norm = predict(trained_net, X_val);
    fprintf('Predictions complete for %d samples\n\n', size(X_val, 1));
    
    % Step 2: Denormalize predictions and targets to physical units
    fprintf('Denormalizing to physical units...\n');
    Y_pred = denormalizeData(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
    Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);
    fprintf('Denormalization complete\n\n');
    
    % Step 3: Compute all metrics for each output
    fprintf('Computing performance metrics...\n');
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    output_units = {'N', 'N', 'Nm', 'Nm'};
    
    % Compute metrics using vectorized functions
    rmse_values = computeRMSE(Y_actual, Y_pred);
    r2_values = computeR2(Y_actual, Y_pred);
    tic_values = computeTIC(Y_actual, Y_pred);
    mae_values = mean(abs(Y_actual - Y_pred), 1);
    
    % Store metrics in structure
    metrics = struct();
    for i = 1:4
        metrics.(output_names{i}) = struct();
        metrics.(output_names{i}).RMSE = rmse_values(i);
        metrics.(output_names{i}).R2 = r2_values(i);
        metrics.(output_names{i}).TIC = tic_values(i);
        metrics.(output_names{i}).MAE = mae_values(i);
    end
    
    fprintf('Metrics computed for all outputs\n\n');
    
    % Step 4: Check performance thresholds
    fprintf('Checking performance thresholds...\n');
    threshold_flags = struct();
    
    % Force predictions (Fx, Fy) should have RMSE < 100N
    threshold_flags.Fx_RMSE_high = metrics.Fx.RMSE > 100;
    threshold_flags.Fy_RMSE_high = metrics.Fy.RMSE > 100;
    
    % All outputs should have R² > 0.95
    threshold_flags.Fx_R2_low = metrics.Fx.R2 < 0.95;
    threshold_flags.Fy_R2_low = metrics.Fy.R2 < 0.95;
    threshold_flags.Mx_R2_low = metrics.Mx.R2 < 0.95;
    threshold_flags.Mz_R2_low = metrics.Mz.R2 < 0.95;
    
    % Check if all requirements are met
    all_flags = struct2cell(threshold_flags);
    metrics.meets_requirements = ~any([all_flags{:}]);
    metrics.threshold_flags = threshold_flags;
    
    if metrics.meets_requirements
        fprintf('✓ All performance thresholds met!\n\n');
    else
        fprintf('⚠ Some performance thresholds not met:\n');
        if threshold_flags.Fx_RMSE_high
            fprintf('  - Fx RMSE (%.2f N) exceeds 100 N threshold\n', metrics.Fx.RMSE);
        end
        if threshold_flags.Fy_RMSE_high
            fprintf('  - Fy RMSE (%.2f N) exceeds 100 N threshold\n', metrics.Fy.RMSE);
        end
        if threshold_flags.Fx_R2_low
            fprintf('  - Fx R² (%.4f) below 0.95 threshold\n', metrics.Fx.R2);
        end
        if threshold_flags.Fy_R2_low
            fprintf('  - Fy R² (%.4f) below 0.95 threshold\n', metrics.Fy.R2);
        end
        if threshold_flags.Mx_R2_low
            fprintf('  - Mx R² (%.4f) below 0.95 threshold\n', metrics.Mx.R2);
        end
        if threshold_flags.Mz_R2_low
            fprintf('  - Mz R² (%.4f) below 0.95 threshold\n', metrics.Mz.R2);
        end
        fprintf('\n');
    end
    
    % Step 5: Display summary table
    fprintf('=== Performance Summary ===\n\n');
    fprintf('%-6s | %10s | %8s | %8s | %10s\n', 'Output', 'RMSE', 'R²', 'TIC', 'MAE');
    fprintf('-------|------------|----------|----------|------------\n');
    for i = 1:4
        fprintf('%-6s | %8.2f %s | %8.4f | %8.4f | %8.2f %s\n', ...
                output_names{i}, ...
                metrics.(output_names{i}).RMSE, output_units{i}, ...
                metrics.(output_names{i}).R2, ...
                metrics.(output_names{i}).TIC, ...
                metrics.(output_names{i}).MAE, output_units{i});
    end
    fprintf('\n');
    
    fprintf('=========================\n\n');
end
