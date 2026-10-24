function rmse = computeRMSE(y_actual, y_pred)
% COMPUTERMSE Compute Root Mean Square Error
%
% Computes the RMSE between actual and predicted values for each output.
% RMSE = sqrt(mean((y_actual - y_pred)^2))
%
% Inputs:
%   y_actual - N×D matrix of actual values
%   y_pred   - N×D matrix of predicted values
%
% Outputs:
%   rmse - 1×D vector of RMSE for each output dimension
%
% Example:
%   y_actual = [1, 2, 3; 4, 5, 6; 7, 8, 9];
%   y_pred = [1.1, 2.2, 2.9; 3.9, 5.1, 6.2; 7.2, 7.9, 8.8];
%   rmse = computeRMSE(y_actual, y_pred);
%
% Requirements: 4.1

    % Validate inputs
    if size(y_actual, 1) ~= size(y_pred, 1) || size(y_actual, 2) ~= size(y_pred, 2)
        error('y_actual and y_pred must have the same dimensions');
    end
    
    % Compute RMSE for each column (output dimension)
    squared_errors = (y_actual - y_pred).^2;
    rmse = sqrt(mean(squared_errors, 1));
end
