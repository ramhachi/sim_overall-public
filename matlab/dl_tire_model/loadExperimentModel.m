function [trained_net, norm_params] = loadExperimentModel(experiment_id, model_dir)
% LOADEXPERIMENTMODEL Load trained model by experiment ID
%
% This function loads a previously saved experiment model from disk.
%
% Inputs:
%   experiment_id - Unique experiment identifier
%   model_dir     - Directory containing model files (default: 'experiments/models')
%
% Outputs:
%   trained_net - Trained neural network
%   norm_params - Normalization parameters structure
%
% Example:
%   [net, params] = loadExperimentModel('exp_20260124_143022');
%
% Requirements: 7.5

    % Set default model directory
    if nargin < 2
        model_dir = 'experiments/models';
    end
    
    % Construct model file path
    model_filename = sprintf('%s_model.mat', experiment_id);
    model_path = fullfile(model_dir, model_filename);
    
    % Check if file exists
    if ~isfile(model_path)
        error('Model file not found: %s', model_path);
    end
    
    % Load model
    loaded = load(model_path);
    trained_net = loaded.trained_net;
    norm_params = loaded.norm_params;
    
    fprintf('Loaded model for experiment: %s\n', experiment_id);
end
