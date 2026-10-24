function model_path = saveExperimentModel(trained_net, norm_params, experiment_id, model_dir)
% SAVEEXPERIMENTMODEL Save trained model with unique experiment identifier
%
% This function saves a trained model with a unique filename based on the
% experiment ID, allowing multiple experiment models to be stored without
% overwriting each other.
%
% Inputs:
%   trained_net   - Trained neural network
%   norm_params   - Normalization parameters structure
%   experiment_id - Unique experiment identifier
%   model_dir     - Directory to save models (default: 'experiments/models')
%
% Outputs:
%   model_path - Full path to saved model file
%
% Example:
%   model_path = saveExperimentModel(trained_net, norm_params, 'exp_20260124_143022');
%
% Requirements: 7.5

    % Set default model directory
    if nargin < 4
        model_dir = 'experiments/models';
    end
    
    % Create model directory if it doesn't exist
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end
    
    % Construct model filename
    model_filename = sprintf('%s_model.mat', experiment_id);
    model_path = fullfile(model_dir, model_filename);
    
    % Save model and normalization parameters
    save(model_path, 'trained_net', 'norm_params', 'experiment_id');
    
    % Also create a prediction function file
    predict_filename = sprintf('%s_predict.mat', experiment_id);
    predict_path = fullfile(model_dir, predict_filename);
    
    % Create prediction function handle
    predict_fn = @(X_raw) predictTireForces(trained_net, X_raw, norm_params);
    save(predict_path, 'predict_fn', 'experiment_id');
    
    fprintf('Model saved successfully:\n');
    fprintf('  Experiment ID: %s\n', experiment_id);
    fprintf('  Model file: %s\n', model_path);
    fprintf('  Prediction function: %s\n\n', predict_path);
end
