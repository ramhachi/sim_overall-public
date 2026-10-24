function Y = loadAndPredict(model_path, X_raw)
% LOADANDPREDICT Load exported model and make predictions
%
% This is a convenience function that loads an exported model and makes
% predictions in a single call. Useful for quick predictions without
% manually loading the model.
%
% Inputs:
%   model_path - Path to exported .mat file
%   X_raw      - N×5 matrix of raw input features [SR, SA, Fz, IA, TSTC]
%
% Outputs:
%   Y - N×4 matrix of predicted outputs [Fx, Fy, Mx, Mz]
%
% Example:
%   X_raw = [0.1, 5.0, 1500, -2.0, 30.0];
%   Y = loadAndPredict('tire_model.mat', X_raw);
%
% Requirements: 6.3

    % Load model
    if ~isfile(model_path)
        error('Model file not found: %s', model_path);
    end
    
    model = load(model_path);
    
    % Verify required fields
    if ~isfield(model, 'trained_net')
        error('Model file missing trained_net');
    end
    if ~isfield(model, 'norm_params')
        error('Model file missing norm_params');
    end
    
    % Make prediction
    Y = predictTireForces(model.trained_net, X_raw, model.norm_params);
end
