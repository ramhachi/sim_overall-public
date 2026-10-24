function [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config)
% TRAINTIEREMODEL Train the neural network tire model
%
% This function trains the neural network using the provided training data,
% with GPU acceleration if available. It handles GPU out-of-memory errors,
% detects training divergence, and implements early stopping.
%
% Inputs:
%   net     - Neural network architecture (from createTireModel)
%   X_train - Training input features (N×5 matrix)
%   Y_train - Training output targets (N×4 matrix)
%   X_val   - Validation input features (M×5 matrix)
%   Y_val   - Validation output targets (M×4 matrix)
%   config  - Structure with training configuration:
%             .max_epochs      - Maximum training epochs (default: 200)
%             .batch_size      - Mini-batch size (default: 256)
%             .learning_rate   - Initial learning rate (default: 0.001)
%             .patience        - Early stopping patience (default: 10)
%             .val_frequency   - Validation frequency (default: 50)
%             .random_seed     - Random seed for reproducibility (default: 42)
%
% Outputs:
%   trained_net   - Trained neural network
%   training_info - Structure with training information:
%                   .TrainingLoss      - Training loss history
%                   .ValidationLoss    - Validation loss history
%                   .FinalValidationLoss - Final validation loss
%                   .TrainingTime      - Total training time in seconds
%
% Example:
%   net = createTireModel();
%   config = struct('max_epochs', 200, 'batch_size', 256, 'random_seed', 42);
%   [trained_net, info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
%
% Requirements: 3.1, 3.6, 3.7, 10.3

    fprintf('=== Starting Model Training ===\n\n');
    
    % Set default configuration values
    if nargin < 6
        config = struct();
    end
    if ~isfield(config, 'random_seed')
        config.random_seed = 42;
    end
    
    % Set random seed for reproducibility
    rng(config.random_seed);
    fprintf('Random seed set to %d for reproducibility\n\n', config.random_seed);
    
    % Check GPU availability
    [gpu_available, gpu_info] = checkGPUAvailability();
    
    % Create training options
    options = createTrainingOptions(config, X_val, Y_val);
    
    % Start training with error handling
    training_start_time = tic;
    
    try
        % Attempt training
        fprintf('Starting training...\n');
        fprintf('Training samples: %d\n', size(X_train, 1));
        fprintf('Validation samples: %d\n\n', size(X_val, 1));
        
        [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
        
        % Check for training divergence (NaN loss)
        if any(isnan(training_info.TrainingLoss))
            error('trainTireModel:TrainingDivergence', ...
                  ['Training diverged (NaN loss detected).\n' ...
                   'Suggestions:\n' ...
                   '  1. Reduce learning rate (try 0.0001)\n' ...
                   '  2. Check data for extreme values\n' ...
                   '  3. Verify normalization is applied correctly']);
        end
        
        % Training successful
        training_time = toc(training_start_time);
        training_info.TrainingTime = training_time;
        
        fprintf('\n=== Training Complete ===\n');
        fprintf('Total training time: %.2f seconds (%.2f minutes)\n', ...
                training_time, training_time / 60);
        fprintf('Final validation loss: %.6f\n', training_info.FinalValidationLoss);
        fprintf('========================\n\n');
        
    catch ME
        % Handle GPU out-of-memory errors
        if contains(ME.message, 'out of memory', 'IgnoreCase', true) || ...
           contains(ME.identifier, 'OutOfMemory', 'IgnoreCase', true)
            
            fprintf('\n*** GPU OUT OF MEMORY ERROR ***\n');
            fprintf('Attempting recovery with reduced batch size...\n\n');
            
            % Reduce batch size by half
            config.batch_size = floor(config.batch_size / 2);
            
            if config.batch_size < 16
                % Batch size too small, fall back to CPU
                fprintf('Batch size too small. Falling back to CPU training...\n\n');
                
                % Force CPU execution
                options = createTrainingOptions(config, X_val, Y_val);
                options.ExecutionEnvironment = 'cpu';
                
                try
                    [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
                    training_time = toc(training_start_time);
                    training_info.TrainingTime = training_time;
                    
                    fprintf('\n=== Training Complete (CPU) ===\n');
                    fprintf('Total training time: %.2f seconds (%.2f minutes)\n', ...
                            training_time, training_time / 60);
                    fprintf('Final validation loss: %.6f\n', training_info.FinalValidationLoss);
                    fprintf('==============================\n\n');
                catch ME2
                    fprintf('CPU training also failed: %s\n', ME2.message);
                    rethrow(ME2);
                end
            else
                % Try again with reduced batch size
                fprintf('Retrying with batch size: %d\n\n', config.batch_size);
                options = createTrainingOptions(config, X_val, Y_val);
                
                try
                    [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
                    training_time = toc(training_start_time);
                    training_info.TrainingTime = training_time;
                    
                    fprintf('\n=== Training Complete (Reduced Batch Size) ===\n');
                    fprintf('Total training time: %.2f seconds (%.2f minutes)\n', ...
                            training_time, training_time / 60);
                    fprintf('Final validation loss: %.6f\n', training_info.FinalValidationLoss);
                    fprintf('==============================================\n\n');
                catch ME2
                    fprintf('Training failed even with reduced batch size: %s\n', ME2.message);
                    rethrow(ME2);
                end
            end
        else
            % Other error - rethrow
            fprintf('\n*** TRAINING ERROR ***\n');
            fprintf('Error: %s\n', ME.message);
            fprintf('Identifier: %s\n', ME.identifier);
            rethrow(ME);
        end
    end
end
