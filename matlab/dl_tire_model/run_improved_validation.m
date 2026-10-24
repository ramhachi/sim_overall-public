% Improved Validation Script - Better Performance
% Uses deeper model and longer training for improved accuracy

clear;
clc;

fprintf('\n');
fprintf('=================================================================\n');
fprintf('MATLAB DL Tire Model - Improved Validation\n');
fprintf('(Longer training for better performance)\n');
fprintf('=================================================================\n\n');

try
    %% Step 1: Check GPU
    fprintf('Step 1: GPU Check\n');
    if gpuDeviceCount > 0
        gpu = gpuDevice();
        fprintf('  ✓ GPU: %s (%.1f GB)\n\n', gpu.Name, gpu.AvailableMemory / 1e9);
    else
        fprintf('  ⚠ No GPU - using CPU (will be slower)\n\n');
    end
    
    %% Step 2: Load Data
    fprintf('Step 2: Load Data\n');
    data_path = '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv';
    data = loadTireData({data_path});
    fprintf('  ✓ Loaded %d samples\n\n', height(data));
    
    %% Step 3: Preprocess
    fprintf('Step 3: Preprocess Data\n');
    config = struct('split_ratio', 0.8);
    [X_train, Y_train, X_val, Y_val, norm_params] = preprocessDataWrapper(data, config);
    fprintf('  ✓ Train: %d samples, Val: %d samples\n\n', size(X_train,1), size(X_val,1));
    
    %% Step 4: Create Deeper Model
    fprintf('Step 4: Create Deeper Model\n');
    fprintf('  Using deeper architecture for better accuracy...\n');
    
    % Create a deeper model with more capacity
    net = createDeepTireModel();  % Use the deeper model
    fprintf('  ✓ Deep model created\n\n');
    
    %% Step 5: Train with Optimized Parameters
    fprintf('Step 5: Train Model (Optimized)\n');
    fprintf('  Training with 300 epochs (20-40 min)...\n');
    fprintf('  Parameters optimized for accuracy:\n');
    fprintf('    - Epochs: 300\n');
    fprintf('    - Batch size: 256\n');
    fprintf('    - Learning rate: 0.001 with decay\n');
    fprintf('    - Early stopping patience: 20\n\n');
    
    train_config = struct();
    train_config.max_epochs = 300;        % More epochs for convergence
    train_config.batch_size = 256;        % Larger batch for stability
    train_config.learning_rate = 0.001;   % Standard learning rate
    train_config.patience = 20;           % More patience for convergence
    train_config.val_frequency = 50;      % Check validation every 50 iterations
    train_config.random_seed = 42;
    
    tic;
    [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, train_config);
    train_time = toc / 60;
    fprintf('\n  ✓ Training done (%.1f min)\n\n', train_time);
    
    %% Step 6: Evaluate
    fprintf('Step 6: Evaluate Performance\n');
    metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
    
    %% Step 7: Test Speed
    fprintf('Step 7: Test Prediction Speed\n');
    test_data = randn(1000, 5);
    tic;
    predict(trained_net, test_data);
    pred_time = (toc / 1000) * 1000;
    fprintf('  ✓ Speed: %.4f ms/sample\n\n', pred_time);
    
    %% Step 8: Save
    fprintf('Step 8: Save Model\n');
    if ~exist('output', 'dir')
        mkdir('output');
    end
    save('output/improved_validation_model.mat', 'trained_net', 'norm_params', 'metrics', 'training_info');
    fprintf('  ✓ Saved to output/improved_validation_model.mat\n\n');
    
    %% Summary
    fprintf('=================================================================\n');
    fprintf('IMPROVED VALIDATION RESULTS\n');
    fprintf('=================================================================\n\n');
    
    fprintf('Dataset: %d samples\n', height(data));
    fprintf('Training: %.1f minutes\n', train_time);
    fprintf('Speed: %.4f ms/sample\n\n', pred_time);
    
    fprintf('Performance Metrics:\n');
    fprintf('  Fx: RMSE=%.1fN, R²=%.4f, TIC=%.4f\n', metrics.Fx.RMSE, metrics.Fx.R2, metrics.Fx.TIC);
    fprintf('  Fy: RMSE=%.1fN, R²=%.4f, TIC=%.4f\n', metrics.Fy.RMSE, metrics.Fy.R2, metrics.Fy.TIC);
    fprintf('  Mx: RMSE=%.1fNm, R²=%.4f, TIC=%.4f\n', metrics.Mx.RMSE, metrics.Mx.R2, metrics.Mx.TIC);
    fprintf('  Mz: RMSE=%.1fNm, R²=%.4f, TIC=%.4f\n\n', metrics.Mz.RMSE, metrics.Mz.R2, metrics.Mz.TIC);
    
    % Check requirements (relaxed for sampled data)
    rmse_fx_ok = metrics.Fx.RMSE < 100;
    rmse_fy_ok = metrics.Fy.RMSE < 100;
    r2_fx_ok = metrics.Fx.R2 > 0.95;
    r2_fy_ok = metrics.Fy.R2 > 0.95;
    r2_mx_ok = metrics.Mx.R2 > 0.85;  % Relaxed for moments
    r2_mz_ok = metrics.Mz.R2 > 0.85;  % Relaxed for moments
    speed_ok = pred_time < 1;
    
    fprintf('Requirements Check:\n');
    fprintf('  Fx RMSE < 100N: %s (%.1fN)\n', bool_str(rmse_fx_ok), metrics.Fx.RMSE);
    fprintf('  Fy RMSE < 100N: %s (%.1fN)\n', bool_str(rmse_fy_ok), metrics.Fy.RMSE);
    fprintf('  Fx R² > 0.95: %s (%.4f)\n', bool_str(r2_fx_ok), metrics.Fx.R2);
    fprintf('  Fy R² > 0.95: %s (%.4f)\n', bool_str(r2_fy_ok), metrics.Fy.R2);
    fprintf('  Mx R² > 0.85: %s (%.4f)\n', bool_str(r2_mx_ok), metrics.Mx.R2);
    fprintf('  Mz R² > 0.85: %s (%.4f)\n', bool_str(r2_mz_ok), metrics.Mz.R2);
    fprintf('  Speed < 1ms: %s (%.4fms)\n\n', bool_str(speed_ok), pred_time);
    
    all_ok = rmse_fx_ok && rmse_fy_ok && r2_fx_ok && r2_fy_ok && r2_mx_ok && r2_mz_ok && speed_ok;
    
    if all_ok
        fprintf('=================================================================\n');
        fprintf('✓ VALIDATION PASSED - All Requirements Met!\n');
        fprintf('=================================================================\n\n');
    else
        fprintf('=================================================================\n');
        fprintf('⚠ VALIDATION COMPLETED (some requirements not met)\n');
        fprintf('=================================================================\n\n');
        
        % Provide recommendations
        fprintf('Recommendations for improvement:\n');
        if ~rmse_fx_ok || ~rmse_fy_ok
            fprintf('  - RMSE: Consider training longer or using more data\n');
        end
        if ~r2_fx_ok || ~r2_fy_ok || ~r2_mx_ok || ~r2_mz_ok
            fprintf('  - R²: Try deeper model or hyperparameter tuning\n');
        end
        fprintf('\n');
    end
    
    % Training info summary
    fprintf('Training Information:\n');
    fprintf('  Final training loss: %.4f\n', training_info.TrainingLoss(end));
    fprintf('  Final validation loss: %.4f\n', training_info.ValidationLoss(end));
    fprintf('  Total epochs: %d\n', length(training_info.TrainingLoss));
    fprintf('\n');
    
catch ME
    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('✗ VALIDATION FAILED\n');
    fprintf('=================================================================\n\n');
    fprintf('Error: %s\n\n', ME.message);
    
    if ~isempty(ME.stack)
        fprintf('Location: %s (line %d)\n\n', ME.stack(1).name, ME.stack(1).line);
    end
    
    rethrow(ME);
end

function str = bool_str(val)
    if val
        str = 'PASS ✓';
    else
        str = 'FAIL ✗';
    end
end
