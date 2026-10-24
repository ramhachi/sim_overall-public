function experiment_log = loadExperimentLog(experiment_id, log_dir)
% LOADEXPERIMENTLOG Load experiment log by ID
%
% This function loads a previously saved experiment log from disk.
%
% Inputs:
%   experiment_id - Unique experiment identifier
%   log_dir       - Directory containing log files (default: 'experiments/logs')
%
% Outputs:
%   experiment_log - Structure containing experiment details
%
% Example:
%   log = loadExperimentLog('exp_20260124_143022');
%
% Requirements: 7.2

    % Set default log directory
    if nargin < 2
        log_dir = 'experiments/logs';
    end
    
    % Construct log file path
    log_file = fullfile(log_dir, [experiment_id, '.mat']);
    
    % Check if file exists
    if ~isfile(log_file)
        error('Experiment log not found: %s', log_file);
    end
    
    % Load experiment log
    loaded = load(log_file);
    experiment_log = loaded.experiment_log;
    
    fprintf('Loaded experiment log: %s\n', experiment_id);
end
