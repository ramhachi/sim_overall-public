function net = createDeepTireModel()
    % createDeepTireModel - Create deeper feedforward neural network for tire force prediction
    %
    % Creates a deeper feedforward neural network with 5 hidden layers that maps 
    % 5 tire state inputs (SR, SA, Fz, IA, TSTC) to 4 tire force/moment outputs (Fx, Fy, Mx, Mz).
    %
    % Architecture:
    %   - Input layer: 5 neurons
    %   - Hidden layer 1: 256 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 2: 128 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 3: 64 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 4: 32 neurons, ReLU activation, 0.2 dropout
    %   - Hidden layer 5: 16 neurons, ReLU activation
    %   - Output layer: 4 neurons, linear activation
    %
    % Returns:
    %   net - Layer graph defining the network architecture
    %
    % This deeper architecture may capture more complex tire behavior patterns
    % but requires more training data and computational resources.
    
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        fullyConnectedLayer(256, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.2, 'Name', 'dropout1')
        
        fullyConnectedLayer(128, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.2, 'Name', 'dropout2')
        
        fullyConnectedLayer(64, 'Name', 'fc3')
        reluLayer('Name', 'relu3')
        dropoutLayer(0.2, 'Name', 'dropout3')
        
        fullyConnectedLayer(32, 'Name', 'fc4')
        reluLayer('Name', 'relu4')
        dropoutLayer(0.2, 'Name', 'dropout4')
        
        fullyConnectedLayer(16, 'Name', 'fc5')
        reluLayer('Name', 'relu5')
        
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end
