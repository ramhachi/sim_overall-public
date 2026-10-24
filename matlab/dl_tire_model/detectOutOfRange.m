function [is_out_of_range, warnings] = detectOutOfRange(X_raw, input_ranges)
% DETECTOUTOFRANGE Detect out-of-range inputs for model prediction
%
% Checks if input features are within the training data range. Generates
% warnings for inputs that fall outside the valid range, as predictions
% may be unreliable due to extrapolation.
%
% Inputs:
%   X_raw        - N×5 matrix of raw input features (not normalized)
%   input_ranges - Structure with fields:
%                  .min - 1×5 vector of minimum values for each feature
%                  .max - 1×5 vector of maximum values for each feature
%                  .names - Cell array of feature names (optional)
%
% Outputs:
%   is_out_of_range - N×1 logical vector indicating which samples have
%                     at least one out-of-range feature
%   warnings        - Cell array of warning messages for out-of-range inputs
%
% Example:
%   input_ranges = struct('min', [-0.2, -15, 500, -5, 20], ...
%                         'max', [0.2, 15, 2500, 5, 80], ...
%                         'names', {{'SR', 'SA', 'Fz', 'IA', 'TSTC'}});
%   X_test = [0.3, 10, 1500, 0, 50];  % SR out of range
%   [is_oor, warnings] = detectOutOfRange(X_test, input_ranges);
%
% Requirements: 8.3, 8.5

    % Default feature names if not provided
    if ~isfield(input_ranges, 'names')
        input_ranges.names = {'SR', 'SA', 'Fz', 'IA', 'TSTC'};
    end
    
    % Validate inputs
    if size(X_raw, 2) ~= 5
        error('X_raw must have 5 columns (SR, SA, Fz, IA, TSTC)');
    end
    
    if length(input_ranges.min) ~= 5 || length(input_ranges.max) ~= 5
        error('input_ranges.min and input_ranges.max must have 5 elements');
    end
    
    % Initialize outputs
    n_samples = size(X_raw, 1);
    is_out_of_range = false(n_samples, 1);
    warnings = {};
    
    % Check each feature
    for i = 1:5
        % Find samples where feature i is below minimum
        below_min = X_raw(:,i) < input_ranges.min(i);
        
        % Find samples where feature i is above maximum
        above_max = X_raw(:,i) > input_ranges.max(i);
        
        % Mark samples as out of range
        is_out_of_range = is_out_of_range | below_min | above_max;
        
        % Generate warnings for this feature
        if any(below_min)
            n_below = sum(below_min);
            min_val = min(X_raw(below_min, i));
            warnings{end+1} = sprintf('%s: %d sample(s) below minimum (%.4f < %.4f)', ...
                                      input_ranges.names{i}, n_below, min_val, input_ranges.min(i));
        end
        
        if any(above_max)
            n_above = sum(above_max);
            max_val = max(X_raw(above_max, i));
            warnings{end+1} = sprintf('%s: %d sample(s) above maximum (%.4f > %.4f)', ...
                                      input_ranges.names{i}, n_above, max_val, input_ranges.max(i));
        end
    end
    
    % Summary
    if any(is_out_of_range)
        n_oor = sum(is_out_of_range);
        pct_oor = 100 * n_oor / n_samples;
        warnings = [warnings, {sprintf('Total: %d/%d samples (%.1f%%) have out-of-range features', ...
                                       n_oor, n_samples, pct_oor)}];
    end
end
