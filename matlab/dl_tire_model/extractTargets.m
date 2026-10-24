function Y = extractTargets(data)
% EXTRACTTARGETS Extract output targets from tire test data
%
% Inputs:
%   data - Table containing tire test data with columns:
%          Fx, Fy, Mx, Mz
%
% Outputs:
%   Y - N×4 matrix of output targets [Fx, Fy, Mx, Mz]
%
% Example:
%   data = loadTireData({'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'});
%   Y = extractTargets(data);
%
% Requirements: 1.2

    % Validate input type
    if ~istable(data)
        error('extractTargets:InvalidInput', ...
              'Input must be a table, got %s', class(data));
    end
    
    % Required target columns
    required_cols = {'Fx', 'Fy', 'Mx', 'Mz'};
    
    % Check if all required columns exist
    missing_cols = setdiff(required_cols, data.Properties.VariableNames);
    if ~isempty(missing_cols)
        error('extractTargets:MissingColumns', ...
              'Missing required columns: %s', strjoin(missing_cols, ', '));
    end
    
    % Extract targets in the correct order
    Y = [data.Fx, data.Fy, data.Mx, data.Mz];
end
