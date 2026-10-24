function tic_value = computeTIC(y_actual, y_pred)
% COMPUTETIC Compute Theil Inequality Coefficient
%
% Computes the TIC (Theil Inequality Coefficient) between actual and
% predicted values for each output. TIC is a normalized error metric:
%   TIC = RMSE / (RMS(y_actual) + RMS(y_pred))
%
% where RMS(x) = sqrt(mean(x^2))
%
% TIC ranges from 0 to 1, where:
%   0.0 = perfect prediction
%   1.0 = worst possible prediction
%
% TIC is scale-invariant and useful for comparing models across different
% output scales.
%
% Inputs:
%   y_actual - N×D matrix of actual values
%   y_pred   - N×D matrix of predicted values
%
% Outputs:
%   tic_value - 1×D vector of TIC for each output dimension
%
% Example:
%   y_actual = [1, 2, 3; 4, 5, 6; 7, 8, 9];
%   y_pred = [1.1, 2.2, 2.9; 3.9, 5.1, 6.2; 7.2, 7.9, 8.8];
%   tic_value = computeTIC(y_actual, y_pred);
%
% Requirements: 4.3

    % Validate inputs
    if size(y_actual, 1) ~= size(y_pred, 1) || size(y_actual, 2) ~= size(y_pred, 2)
        error('y_actual and y_pred must have the same dimensions');
    end
    
    % Compute TIC for each column (output dimension)
    n_outputs = size(y_actual, 2);
    tic_value = zeros(1, n_outputs);
    
    for i = 1:n_outputs
        % RMSE
        rmse = sqrt(mean((y_actual(:,i) - y_pred(:,i)).^2));
        
        % RMS of actual values
        rms_actual = sqrt(mean(y_actual(:,i).^2));
        
        % RMS of predicted values
        rms_pred = sqrt(mean(y_pred(:,i).^2));
        
        % TIC = RMSE / (RMS_actual + RMS_pred)
        denominator = rms_actual + rms_pred;
        if denominator == 0
            % If both actual and predicted are all zeros, TIC is undefined
            % Set to NaN to indicate this
            tic_value(i) = NaN;
        else
            tic_value(i) = rmse / denominator;
        end
    end
end
