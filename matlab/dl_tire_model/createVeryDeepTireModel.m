function net = createVeryDeepTireModel()
% CREATEVERYDEEPTIREMODEL Create a very deep neural network for tire modeling
%
% This function creates an extremely deep feedforward neural network
% with many layers for maximum capacity. This is an experimental
% "contrarian" approach to see if more depth helps.
%
% Architecture:
%   Input: 5 features (SR, SA, Fz, IA, TSTC)
%   Hidden layers: 256 -> 256 -> 128 -> 128 -> 64 -> 64 -> 32 -> 32
%   Output: 4 targets (Fx, Fy, Mx, Mz)
%
% Total: 8 hidden layers (very deep!)
%
% Returns:
%   net - Layer graph for the very deep tire model
%
% Example:
%   net = createVeryDeepTireModel();
%
% Requirements: 2.1, 2.2, 2.3, 2.5, 2.7

    fprintf('Creating VERY DEEP tire model (8 hidden layers)...\n');
    
    layers = [
        % Input layer
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        % Block 1: 256 neurons
        fullyConnectedLayer(256, 'Name', 'fc1')
        batchNormalizationLayer('Name', 'bn1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.2, 'Name', 'dropout1')
        
        % Block 2: 256 neurons
        fullyConnectedLayer(256, 'Name', 'fc2')
        batchNormalizationLayer('Name', 'bn2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.2, 'Name', 'dropout2')
        
        % Block 3: 128 neurons
        fullyConnectedLayer(128, 'Name', 'fc3')
        batchNormalizationLayer('Name', 'bn3')
        reluLayer('Name', 'relu3')
        dropoutLayer(0.2, 'Name', 'dropout3')
        
        % Block 4: 128 neurons
        fullyConnectedLayer(128, 'Name', 'fc4')
        batchNormalizationLayer('Name', 'bn4')
        reluLayer('Name', 'relu4')
        dropoutLayer(0.2, 'Name', 'dropout4')
        
        % Block 5: 64 neurons
        fullyConnectedLayer(64, 'Name', 'fc5')
        batchNormalizationLayer('Name', 'bn5')
        reluLayer('Name', 'relu5')
        dropoutLayer(0.15, 'Name', 'dropout5')
        
        % Block 6: 64 neurons
        fullyConnectedLayer(64, 'Name', 'fc6')
        batchNormalizationLayer('Name', 'bn6')
        reluLayer('Name', 'relu6')
        dropoutLayer(0.15, 'Name', 'dropout6')
        
        % Block 7: 32 neurons
        fullyConnectedLayer(32, 'Name', 'fc7')
        batchNormalizationLayer('Name', 'bn7')
        reluLayer('Name', 'relu7')
        dropoutLayer(0.1, 'Name', 'dropout7')
        
        % Block 8: 32 neurons
        fullyConnectedLayer(32, 'Name', 'fc8')
        batchNormalizationLayer('Name', 'bn8')
        reluLayer('Name', 'relu8')
        
        % Output layer
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
    
    fprintf('Very deep model created:\n');
    fprintf('  - 8 hidden layers\n');
    fprintf('  - Batch normalization in all layers\n');
    fprintf('  - Dropout: 0.2 -> 0.2 -> 0.15 -> 0.1\n');
    fprintf('  - Total parameters: ~200k\n');
    fprintf('  - Architecture: 256-256-128-128-64-64-32-32\n\n');
end
