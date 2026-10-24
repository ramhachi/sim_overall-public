function stats = validateTireData(data)
% VALIDATETIREDATA Validate tire test data and report statistics
%
% Inputs:
%   data - Table containing tire test data
%
% Outputs:
%   stats - Structure containing validation statistics
%
% Example:
%   data = loadTireData(data_paths);
%   stats = validateTireData(data);

    stats = struct();
    
    % Check data is not empty
    if isempty(data)
        error('validateTireData:EmptyData', 'Data table is empty');
    end
    
    % Required columns
    required_cols = {'SR', 'SA', 'Fz', 'IA', 'TSTC', 'Fx', 'Fy', 'Mx', 'Mz'};
    
    % Verify all required columns exist
    missing_cols = setdiff(required_cols, data.Properties.VariableNames);
    if ~isempty(missing_cols)
        error('validateTireData:MissingColumns', ...
              'Missing required columns: %s', strjoin(missing_cols, ', '));
    end
    
    % Count total samples
    stats.total_samples = height(data);
    
    % Check for missing values (NaN or Inf)
    stats.missing_by_column = struct();
    for i = 1:length(required_cols)
        col_name = required_cols{i};
        col_data = data.(col_name);
        
        % Count NaN and Inf values
        nan_count = sum(isnan(col_data));
        inf_count = sum(isinf(col_data));
        missing_count = nan_count + inf_count;
        
        stats.missing_by_column.(col_name) = missing_count;
        
        if missing_count > 0
            warning('validateTireData:MissingValues', ...
                    'Column %s has %d missing/infinite values (%.2f%%)', ...
                    col_name, missing_count, 100 * missing_count / stats.total_samples);
        end
    end
    
    % Count rows with any missing values
    data_matrix = table2array(data);
    bad_rows = any(isnan(data_matrix) | isinf(data_matrix), 2);
    stats.rows_with_missing = sum(bad_rows);
    stats.clean_samples = stats.total_samples - stats.rows_with_missing;
    
    % Report data ranges
    stats.ranges = struct();
    for i = 1:length(required_cols)
        col_name = required_cols{i};
        col_data = data.(col_name);
        
        % Remove NaN/Inf for range calculation
        clean_data = col_data(~isnan(col_data) & ~isinf(col_data));
        
        if ~isempty(clean_data)
            stats.ranges.(col_name) = struct(...
                'min', min(clean_data), ...
                'max', max(clean_data), ...
                'mean', mean(clean_data), ...
                'std', std(clean_data));
        end
    end
    
    % Display summary
    fprintf('\n=== Data Validation Summary ===\n');
    fprintf('Total samples: %d\n', stats.total_samples);
    fprintf('Clean samples: %d (%.2f%%)\n', stats.clean_samples, ...
            100 * stats.clean_samples / stats.total_samples);
    fprintf('Rows with missing values: %d (%.2f%%)\n', stats.rows_with_missing, ...
            100 * stats.rows_with_missing / stats.total_samples);
    
    fprintf('\n=== Data Ranges ===\n');
    for i = 1:length(required_cols)
        col_name = required_cols{i};
        if isfield(stats.ranges, col_name)
            r = stats.ranges.(col_name);
            fprintf('%s: [%.4f, %.4f], mean=%.4f, std=%.4f\n', ...
                    col_name, r.min, r.max, r.mean, r.std);
        end
    end
    fprintf('==============================\n\n');
end
