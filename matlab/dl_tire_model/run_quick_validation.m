% Quick Validation Script for MATLAB DL Tire Model
% Simplified version for testing with sampled data

% Clear workspace
clear;
clc;

fprintf('\n');
fprintf('=================================================================\n');
fprintf('MATLAB DL Tire Model - Quick Validation\n');
fprintf('=================================================================\n\n');

% Add current directory to path
addpath(pwd);

try
    %% Step 1: Check GPU
    fprintf('Step 1: Checking GPU availability...\n');
    if gpuDeviceCount > 0
        gpu = gpuDevice();
        fprintf('  GPU detected: %s\n', gpu.Name);
        fprintf('  Memory: %.2f GB\n\n', gpu.AvailableMemory / 1e9);
    else
        fprintf('  No GPU detected - will use CPU\n\n');
    end
    
    %% Step 2: Load Data
    fprintf('Step 2: Loading data...\n');
    data_path = '../../data/tiredata/DriveBrake_sampled_quarter.csv';
    
    if ~isfile(data_path)
        error('Data file not found: %s', data_path);
    end
    
    data = loadTireData({data_path});
    fprintf('  Loaded %d samples\n\n', height(data));
    
    %% Step 3: Preprocess Data
    fprintf('Step 3: Preprocessing data...\n');
    config = struct('split_ratio', 0.8);
    [X_train, Y_train, X_val, Y_val, norm_params] = preprocessDataWrapper(data, config);
    
    %% Step 4: Create Model
    fprintf('Step 4: Creating neural network model...\n');
    net = createTireModel();
    fprintf('  Model created successfully\n\n');
    
    %% Step 5: Configure Training
    fprintf('Step 5: Configuring training...\n');
    train_config = struct();
    train_config.max_epochs = 50;  % Reduced for quick validation
    train_config.batch_size = 128;
    train_config.learning_rate = 0.001;
    train_config.patience = 5;
    train_config.val_frequency = 20;
    train_config.random_seed = 42;
    fprintf('  Training configured (50 epochs max)\n\n');
    
    %% Step 6: Train Model
    fprintf('Step 6: Training model...\n');
    fprintf('  This may take 5-15 minutes...\n\n');
    
    tic;
    [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, train_config);
    training_time = toc;
    
    fprintf('\n  Training completed in %.2f minutes\n\n', training_time / 60);
    
    %% Step 7: Evaluate Model
    fprintf('Step 7: Evaluating model performance...\n');
    metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
    
    %% Step 8: Test Prediction Speed
    fprintf('Step 8: Testing prediction speed...\n');
    test_samples = randn(1000, 5);
    tic;
    predict(trained_net, test_samples);
    elapsed = toc;
    time_per_sample_ms = (elapsed / 1000) * 1000;
    fprintf('  Prediction time: %.4f ms per sample\n\n', time_per_sample_ms);
    
    %% Step 9: Save Results
    fprintf('Step 9: Saving results...\n');
    if ~exist('output', 'dir')
        mkdir('output');
    end
    
    save('output/quick_validation_model.mat', 'trained_net', 'norm_params', 'metrics', 'training_info');
    fprintf('  Model saved to: output/quick_validation_model.mat\n\n');
    
    %% Summary
    fprintf('=================================================================\n');
    fprintf('VALIDATION SUMMARY\n');
    fprintf('=================================================================\n\n');
    
    fprintf('Dataset: %d samples\n', height(data));
    fprintf('Training time: %.2f minutes\n', training_time / 60);
    fprintf('Prediction speed: %.4f ms per sample\n\n', time_per_sample_ms);
    
    fprintf('Performance Metrics:\n');
    fprintf('  Fx - RMSE: %.2f N, R²: %.4f\n', metrics.Fx.RMSE, metrics.Fx.R2);
    fprintf('  Fy - RMSE: %.2f N, R²: %.4f\n', metrics.Fy.RMSE, metrics.Fy.R2);
    fprintf('  Mx - RMSE: %.2f Nm, R²: %.4f\n', metrics.Mx.RMSE, metrics.Mx.R2);
    fprintf('  Mz - RMSE: %.2f Nm, R²: %.4f\n\n', metrics.Mz.RMSE, metrics.Mz.R2);
    
    % Check requirements
    rmse_ok = metrics.Fx.RMSE < 100 && metrics.Fy.RMSE < 100;
    r2_ok = metrics.Fx.R2 > 0.95 && metrics.Fy.R2 > 0.95 && ...
            metrics.Mx.R2 > 0.95 && metrics.Mz.R2 > 0.95;
    speed_ok = time_per_sample_ms < 1;
    
    fprintf('Requirements Check:\n');
    if rmse_ok
        fprintf('  RMSE < 100N: PASS\n');
    else
        fprintf('  RMSE < 100N: FAIL\n');
    end
    
    if r2_ok
        fprintf('  R² > 0.95: PASS\n');
    else
        fprintf('  R² > 0.95: FAIL\n');
    end
    
    if speed_ok
        fprintf('  Prediction < 1ms: PASS\n');
    else
        fprintf('  Prediction < 1ms: FAIL\n');
    end
    
    fprintf('\n');
    
    if rmse_ok && r2_ok && speed_ok
        fprintf('=================================================================\n');
        fprintf('VALIDATION PASSED\n');
        fprintf('=================================================================\n\n');
    else
        fprintf('=================================================================\n');
        fprintf('VALIDATION COMPLETED WITH WARNINGS\n');
        fprintf('=================================================================\n\n');
    end
    
catch ME
    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('VALIDATION FAILED\n');
    fprintf('=================================================================\n\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\n');
    rethrow(ME);
end

fprintf('Validation complete!\n\n');
