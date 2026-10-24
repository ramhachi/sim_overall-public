function X = extractFeatures(data)
% EXTRACTFEATURES Extract input features from tire test data
%
% Inputs:
%   data - Table containing tire test data with columns:
%          SR, SA, Fz, IA, TSTC
%
% Outputs:
%   X - N×5 matrix of input features [SR, SA, Fz, IA, TSTC]
%
% Example:
%   data = loadTireData({'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'});
%   X = extractFeatures(data);
%
% Requirements: 1.2

    % Validate input type
    if ~istable(data)
        error('extractFeatures:InvalidInput', ...
              'Input must be a table, got %s', class(data));
    end
    
    % Required feature columns
    required_cols = {'SR', 'SA', 'Fz', 'IA', 'TSTC'};
    
    % Check if all required columns exist
    missing_cols = setdiff(required_cols, data.Properties.VariableNames);
    if ~isempty(missing_cols)
        error('extractFeatures:MissingColumns', ...
              'Missing required columns: %s', strjoin(missing_cols, ', '));
    end
    
    % Extract features in the correct order
    X = [data.SR, data.SA, data.Fz, data.IA, data.TSTC];
end
