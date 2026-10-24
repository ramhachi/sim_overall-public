function Y = predictTireForces(net, X_raw, norm_params)
% PREDICTTIREFORCES Predict tire forces and moments from raw inputs
%
% This function accepts raw (unnormalized) tire state variables and returns
% predicted tire forces and moments in physical units. It handles all
% normalization and denormalization internally.
%
% Inputs:
%   net         - Trained neural network
%   X_raw       - N×5 matrix of raw input features [SR, SA, Fz, IA, TSTC]
%                 SR: Slip ratio (dimensionless)
%                 SA: Slip angle (degrees)
%                 Fz: Vertical load (N)
%                 IA: Inclination angle (degrees)
%                 TSTC: Tire surface temperature (°C)
%   norm_params - Structure with normalization parameters:
%                 .input_mean, .input_std, .output_mean, .output_std
%
% Outputs:
%   Y - N×4 matrix of predicted outputs [Fx, Fy, Mx, Mz]
%       Fx: Longitudinal force (N)
%       Fy: Lateral force (N)
%       Mx: Overturning moment (Nm)
%       Mz: Aligning moment (Nm)
%
% Example:
%   % Load exported model
%   model = load('tire_model.mat');
%   
%   % Predict for single input
%   X_raw = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]
%   Y = predictTireForces(model.trained_net, X_raw, model.norm_params);
%   fprintf('Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', Y(1), Y(2), Y(3), Y(4));
%
% Requirements: 6.3

    % Validate inputs
    if nargin < 3
        error('predictTireForces requires 3 arguments: net, X_raw, norm_params');
    end
    
    if size(X_raw, 2) ~= 5
        error('X_raw must have 5 columns [SR, SA, Fz, IA, TSTC], got %d', size(X_raw, 2));
    end
    
    % Check that norm_params has required fields
    required_fields = {'input_mean', 'input_std', 'output_mean', 'output_std'};
    for i = 1:length(required_fields)
        if ~isfield(norm_params, required_fields{i})
            error('norm_params missing required field: %s', required_fields{i});
        end
    end
    
    % Step 1: Normalize inputs
    X_norm = (X_raw - norm_params.input_mean) ./ norm_params.input_std;
    
    % Step 2: Make prediction
    Y_norm = predict(net, X_norm);
    
    % Step 3: Denormalize outputs
    Y = Y_norm .* norm_params.output_std + norm_params.output_mean;
end
