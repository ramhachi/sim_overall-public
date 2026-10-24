function r2 = computeR2(y_actual, y_pred)
% COMPUTER2 Compute coefficient of determination (R²)
%
% Computes the R² (R-squared) value between actual and predicted values
% for each output. R² = 1 - (SS_res / SS_tot), where:
%   SS_res = sum((y_actual - y_pred)^2)  (residual sum of squares)
%   SS_tot = sum((y_actual - mean(y_actual))^2)  (total sum of squares)
%
% R² ranges from -∞ to 1, where:
%   1.0 = perfect prediction
%   0.0 = model performs as well as predicting the mean
%   < 0 = model performs worse than predicting the mean
%
% Inputs:
%   y_actual - N×D matrix of actual values
%   y_pred   - N×D matrix of predicted values
%
% Outputs:
%   r2 - 1×D vector of R² for each output dimension
%
% Example:
%   y_actual = [1, 2, 3; 4, 5, 6; 7, 8, 9];
%   y_pred = [1.1, 2.2, 2.9; 3.9, 5.1, 6.2; 7.2, 7.9, 8.8];
%   r2 = computeR2(y_actual, y_pred);
%
% Requirements: 4.2

    % Validate inputs
    if size(y_actual, 1) ~= size(y_pred, 1) || size(y_actual, 2) ~= size(y_pred, 2)
        error('y_actual and y_pred must have the same dimensions');
    end
    
    % Compute R² for each column (output dimension)
    n_outputs = size(y_actual, 2);
    r2 = zeros(1, n_outputs);
    
    for i = 1:n_outputs
        % Residual sum of squares
        SS_res = sum((y_actual(:,i) - y_pred(:,i)).^2);
        
        % Total sum of squares
        SS_tot = sum((y_actual(:,i) - mean(y_actual(:,i))).^2);
        
        % R² = 1 - SS_res / SS_tot
        if SS_tot == 0
            % If all actual values are the same, R² is undefined
            % Set to NaN to indicate this
            r2(i) = NaN;
        else
            r2(i) = 1 - SS_res / SS_tot;
        end
    end
end
