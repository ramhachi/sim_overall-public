function input_ranges = computeInputRanges(X_train)
% COMPUTEINPUTRANGES Compute min/max ranges from training data
%
% Computes the minimum and maximum values for each input feature from
% the training dataset. These ranges are used for out-of-range detection
% during prediction.
%
% Inputs:
%   X_train - N×5 matrix of training input features (raw, not normalized)
%
% Outputs:
%   input_ranges - Structure with fields:
%                  .min   - 1×5 vector of minimum values
%                  .max   - 1×5 vector of maximum values
%                  .names - Cell array of feature names
%
% Example:
%   X_train = randn(1000, 5);
%   input_ranges = computeInputRanges(X_train);
%   fprintf('SR range: [%.4f, %.4f]\n', input_ranges.min(1), input_ranges.max(1));
%
% Requirements: 8.3

    % Validate input
    if size(X_train, 2) ~= 5
        error('X_train must have 5 columns (SR, SA, Fz, IA, TSTC)');
    end
    
    % Compute min and max for each feature
    input_ranges = struct();
    input_ranges.min = min(X_train, [], 1);
    input_ranges.max = max(X_train, [], 1);
    input_ranges.names = {'SR', 'SA', 'Fz', 'IA', 'TSTC'};
    
    % Display ranges
    fprintf('Input feature ranges from training data:\n');
    for i = 1:5
        fprintf('  %5s: [%10.4f, %10.4f]\n', ...
                input_ranges.names{i}, input_ranges.min(i), input_ranges.max(i));
    end
end
