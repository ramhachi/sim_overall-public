function net = createTireModel()
    % createTireModel - Create feedforward neural network for tire force prediction
    %
    % Creates a feedforward neural network that maps 5 tire state inputs
    % (SR, SA, Fz, IA, TSTC) to 4 tire force/moment outputs (Fx, Fy, Mx, Mz).
    %
    % Architecture:
    %   - Input layer: 5 neurons
    %   - Hidden layer 1: 128 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 2: 64 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 3: 32 neurons, ReLU activation
    %   - Output layer: 4 neurons, linear activation
    %
    % Returns:
    %   net - Layer graph defining the network architecture
    %
    % Requirements: 2.1, 2.2, 2.3, 2.5, 2.7
    
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        fullyConnectedLayer(128, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.2, 'Name', 'dropout1')
        
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.2, 'Name', 'dropout2')
        
        fullyConnectedLayer(32, 'Name', 'fc3')
        reluLayer('Name', 'relu3')
        
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end
