function net = createLSTMTireModel()
    % createLSTMTireModel - Create LSTM neural network for tire force prediction
    %
    % Creates an LSTM-based neural network that maps 5 tire state inputs
    % (SR, SA, Fz, IA, TSTC) to 4 tire force/moment outputs (Fx, Fy, Mx, Mz).
    % This architecture is designed for temporal dependencies in tire behavior.
    %
    % Architecture:
    %   - Sequence input layer: 5 features
    %   - LSTM layer 1: 64 units, sequence output, 0.2 dropout
    %   - LSTM layer 2: 32 units, last output, 0.2 dropout
    %   - Fully connected layer: 4 neurons (output)
    %   - Regression layer
    %
    % Returns:
    %   net - Layer graph defining the LSTM network architecture
    %
    % Requirements: 2.6
    
    layers = [
        sequenceInputLayer(5, 'Name', 'input')
        
        lstmLayer(64, 'Name', 'lstm1', 'OutputMode', 'sequence')
        dropoutLayer(0.2, 'Name', 'dropout1')
        
        lstmLayer(32, 'Name', 'lstm2', 'OutputMode', 'last')
        dropoutLayer(0.2, 'Name', 'dropout2')
        
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end
