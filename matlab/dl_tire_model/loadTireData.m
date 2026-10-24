function data = loadTireData(data_paths)
% LOADTIREDATA Load TTC tire test data from CSV files
%
% Inputs:
%   data_paths - Cell array of file paths to CSV files
%
% Outputs:
%   data - Table containing combined tire test data with columns:
%          SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz
%
% Example:
%   data_paths = {'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
%                 'data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'};
%   data = loadTireData(data_paths);

    % Required columns for tire model
    required_cols = {'SR', 'SA', 'Fz', 'IA', 'TSTC', 'Fx', 'Fy', 'Mx', 'Mz'};
    
    % Initialize empty table
    data = table();
    
    % Load each file
    for i = 1:length(data_paths)
        file_path = data_paths{i};
        
        % Check if file exists
        if ~isfile(file_path)
            error('loadTireData:FileNotFound', ...
                  'Data file not found: %s', file_path);
        end
        
        % Try to read CSV file with preserved variable names
        % TTC data has two formats:
        % - DriveBrake: row 1 = header info, row 2 = column names, row 3+ = data
        % - Cornering: row 1 = column names, row 2 = units, row 3+ = data
        try
            % First, try to detect which format by reading first line
            fid = fopen(file_path, 'r');
            first_line = fgetl(fid);
            fclose(fid);
            
            % Check if first line contains "TIRF Data File" (DriveBrake format)
            if contains(first_line, 'TIRF Data File')
                % DriveBrake format
                opts = detectImportOptions(file_path);
                opts.VariableNamingRule = 'preserve';
                opts.VariableNamesLine = 2;  % Column names are on row 2
                opts.DataLines = [3, Inf];   % Data starts at row 3
            else
                % Cornering format
                opts = detectImportOptions(file_path);
                opts.VariableNamingRule = 'preserve';
                opts.VariableNamesLine = 1;  % Column names are on row 1
                opts.DataLines = [3, Inf];   % Data starts at row 3 (skip units row)
            end
            
            data_table = readtable(file_path, opts);
        catch ME
            error('loadTireData:ReadError', ...
                  'Failed to read CSV file %s: %s', file_path, ME.message);
        end
        
        % Map actual column names to expected names
        % TTC data uses: FX, FY, FZ, MX, MZ (uppercase)
        column_mapping = struct(...
            'FX', 'Fx', ...
            'FY', 'Fy', ...
            'FZ', 'Fz', ...
            'MX', 'Mx', ...
            'MZ', 'Mz' ...
        );
        
        % Rename columns if they exist with different case
        col_names = data_table.Properties.VariableNames;
        for j = 1:length(col_names)
            if isfield(column_mapping, col_names{j})
                data_table.Properties.VariableNames{j} = column_mapping.(col_names{j});
            end
        end
        
        % Validate required columns exist
        missing_cols = setdiff(required_cols, data_table.Properties.VariableNames);
        if ~isempty(missing_cols)
            error('loadTireData:MissingColumns', ...
                  'Missing required columns in %s: %s', ...
                  file_path, strjoin(missing_cols, ', '));
        end
        
        % Extract only required columns
        data_subset = data_table(:, required_cols);
        
        % Append to combined data
        if isempty(data)
            data = data_subset;
        else
            data = [data; data_subset];
        end
        
        fprintf('Loaded %d samples from %s\n', height(data_subset), file_path);
    end
    
    fprintf('Total samples loaded: %d\n', height(data));
end
