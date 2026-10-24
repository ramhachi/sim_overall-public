function options = createTrainingOptions(config, X_val, Y_val)
% CREATETRAININGOPTIONS Create training options for neural network training
%
% This function configures the training options for the deep learning model,
% including optimizer settings, learning rate schedule, early stopping,
% and GPU acceleration.
%
% Inputs:
%   config - Structure with training configuration:
%            .max_epochs      - Maximum training epochs (default: 200)
%            .batch_size      - Mini-batch size (default: 256)
%            .learning_rate   - Initial learning rate (default: 0.001)
%            .patience        - Early stopping patience (default: 10)
%            .val_frequency   - Validation frequency in iterations (default: 50)
%   X_val  - Validation input features (N×5 matrix)
%   Y_val  - Validation output targets (N×4 matrix)
%
% Outputs:
%   options - Training options object for trainNetwork
%
% Example:
%   config = struct('max_epochs', 200, 'batch_size', 256, ...
%                   'learning_rate', 0.001, 'patience', 10);
%   options = createTrainingOptions(config, X_val, Y_val);
%
% Requirements: 3.2, 3.3, 3.4, 3.5, 3.8, 9.4, 9.5

    % Set default values if not provided
    if ~isfield(config, 'max_epochs')
        config.max_epochs = 200;
    end
    if ~isfield(config, 'batch_size')
        config.batch_size = 256;
    end
    if ~isfield(config, 'learning_rate')
        config.learning_rate = 0.001;
    end
    if ~isfield(config, 'patience')
        config.patience = 10;
    end
    if ~isfield(config, 'val_frequency')
        config.val_frequency = 50;
    end
    
    % Create training options with Adam optimizer
    options = trainingOptions('adam', ...
        'MaxEpochs', config.max_epochs, ...
        'MiniBatchSize', config.batch_size, ...
        'InitialLearnRate', config.learning_rate, ...
        'LearnRateSchedule', 'piecewise', ...
        'LearnRateDropFactor', 0.5, ...
        'LearnRateDropPeriod', 20, ...
        'ValidationData', {X_val, Y_val}, ...
        'ValidationFrequency', config.val_frequency, ...
        'ValidationPatience', config.patience, ...
        'Shuffle', 'every-epoch', ...
        'Verbose', true, ...
        'Plots', 'training-progress', ...
        'ExecutionEnvironment', 'auto');  % Auto-detect GPU
    
    fprintf('Training options configured:\n');
    fprintf('  Optimizer: Adam\n');
    fprintf('  Max epochs: %d\n', config.max_epochs);
    fprintf('  Batch size: %d\n', config.batch_size);
    fprintf('  Initial learning rate: %.4f\n', config.learning_rate);
    fprintf('  Learning rate schedule: piecewise (drop factor 0.5 every 20 epochs)\n');
    fprintf('  Early stopping patience: %d epochs\n', config.patience);
    fprintf('  Validation frequency: every %d iterations\n', config.val_frequency);
    fprintf('  Execution environment: auto (GPU if available)\n\n');
end
