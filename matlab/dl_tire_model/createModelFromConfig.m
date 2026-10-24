function net = createModelFromConfig(config)
    % createModelFromConfig - Create neural network based on configuration
    %
    % Creates either a feedforward or LSTM neural network for tire force
    % prediction based on the provided configuration structure.
    %
    % Inputs:
    %   config - Structure with fields:
    %            .model_type: 'feedforward' or 'lstm' (default: 'feedforward')
    %
    % Returns:
    %   net - Layer graph defining the network architecture
    %
    % Example:
    %   config.model_type = 'feedforward';
    %   net = createModelFromConfig(config);
    %
    % Requirements: 2.6
    
    % Set default model type if not specified
    if ~isfield(config, 'model_type')
        config.model_type = 'feedforward';
    end
    
    % Create network based on model type
    switch lower(config.model_type)
        case 'feedforward'
            net = createTireModel();
        case 'lstm'
            net = createLSTMTireModel();
        otherwise
            error('Unknown model type: %s. Use ''feedforward'' or ''lstm''.', config.model_type);
    end
end
