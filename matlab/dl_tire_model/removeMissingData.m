function [X_clean, Y_clean, stats] = removeMissingData(X, Y)
% REMOVEMISSINGDATA Remove rows with NaN or Inf values from data
%
% Inputs:
%   X - N×5 matrix of input features
%   Y - N×4 matrix of output targets
%
% Outputs:
%   X_clean - M×5 matrix of clean input features (M <= N)
%   Y_clean - M×4 matrix of clean output targets
%   stats   - Structure with statistics:
%             .total_samples - Original number of samples
%             .removed_samples - Number of samples removed
%             .clean_samples - Number of clean samples remaining
%             .removal_percentage - Percentage of samples removed
%
% Example:
%   data = loadTireData({'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'});
%   X = extractFeatures(data);
%   Y = extractTargets(data);
%   [X_clean, Y_clean, stats] = removeMissingData(X, Y);
%
% Requirements: 1.6

    % Find rows with NaN or Inf in features or targets
    bad_rows_X = any(isnan(X) | isinf(X), 2);
    bad_rows_Y = any(isnan(Y) | isinf(Y), 2);
    bad_rows = bad_rows_X | bad_rows_Y;
    
    % Calculate statistics
    stats.total_samples = size(X, 1);
    stats.removed_samples = sum(bad_rows);
    stats.clean_samples = stats.total_samples - stats.removed_samples;
    stats.removal_percentage = 100 * stats.removed_samples / stats.total_samples;
    
    % Log removal statistics
    if stats.removed_samples > 0
        fprintf('Removed %d samples (%.2f%%) with missing or infinite values\n', ...
                stats.removed_samples, stats.removal_percentage);
        fprintf('  - Rows with bad features: %d\n', sum(bad_rows_X));
        fprintf('  - Rows with bad targets: %d\n', sum(bad_rows_Y));
        fprintf('Clean samples remaining: %d\n', stats.clean_samples);
    else
        fprintf('No missing or infinite values detected. All %d samples are clean.\n', ...
                stats.total_samples);
    end
    
    % Remove bad rows
    X_clean = X(~bad_rows, :);
    Y_clean = Y(~bad_rows, :);
end
