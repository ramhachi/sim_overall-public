% Final System Validation Script
% This script performs comprehensive validation of the MATLAB DL Tire Model
% including all tests, performance benchmarks, and requirement verification

function results = final_system_validation()
    fprintf('=================================================================\n');
    fprintf('MATLAB Deep Learning Tire Model - Final System Validation\n');
    fprintf('=================================================================\n\n');
    
    % Initialize results structure
    results = struct();
    results.timestamp = datetime('now');
    results.tests_passed = true;
    results.performance_met = true;
    results.all_requirements_met = true;
    
    %% Step 1: Run all unit tests
    fprintf('Step 1: Running Unit Tests...\n');
    fprintf('-----------------------------------------------------------------\n');
    results.unit_tests = run_unit_tests();
    fprintf('\n');
    
    %% Step 2: Run all property-based tests
    fprintf('Step 2: Running Property-Based Tests...\n');
    fprintf('-----------------------------------------------------------------\n');
    results.property_tests = run_property_tests();
    fprintf('\n');
    
    %% Step 3: Train model on full dataset
    fprintf('Step 3: Training Model on Full TTC Dataset...\n');
    fprintf('-----------------------------------------------------------------\n');
    results.training = train_full_model();
    fprintf('\n');
    
    %% Step 4: Verify performance requirements
    fprintf('Step 4: Verifying Performance Requirements...\n');
    fprintf('-----------------------------------------------------------------\n');
    results.performance = verify_performance_requirements(results.training);
    fprintf('\n');
    
    %% Step 5: Generate final performance report
    fprintf('Step 5: Generating Final Performance Report...\n');
    fprintf('-----------------------------------------------------------------\n');
    results.report_path = generate_final_report(results);
    fprintf('\n');
    
    %% Summary
    fprintf('=================================================================\n');
    fprintf('VALIDATION SUMMARY\n');
    fprintf('=================================================================\n');
    fprintf('Unit Tests: %s\n', status_string(results.unit_tests.all_passed));
    fprintf('Property Tests: %s\n', status_string(results.property_tests.all_passed));
    fprintf('Training: %s\n', status_string(results.training.success));
    fprintf('Performance Requirements: %s\n', status_string(results.performance.all_met));
    fprintf('\nFinal Report: %s\n', results.report_path);
    
    % Overall status
    results.all_requirements_met = results.unit_tests.all_passed && ...
                                    results.property_tests.all_passed && ...
                                    results.training.success && ...
                                    results.performance.all_met;
    
    fprintf('\n=================================================================\n');
    if results.all_requirements_met
        fprintf('✓ ALL VALIDATION CHECKS PASSED\n');
    else
        fprintf('✗ SOME VALIDATION CHECKS FAILED - See report for details\n');
    end
    fprintf('=================================================================\n\n');
end

%% Helper function: Run unit tests
function test_results = run_unit_tests()
    % Initialize structure first
    test_results = struct();
    test_results.tests = {};
    test_results.passed = [];
    test_results.all_passed = true;
    test_results.num_passed = 0;
    test_results.num_failed = 0;
    test_results.num_skipped = 0;
    
    % List of unit test files
    unit_tests = {
        'test_data_loading.m'
        'test_feature_target_extraction.m'
        'test_preprocessing_checkpoint.m'
        'test_model_creation.m'
        'test_training_checkpoint.m'
        'test_training_engine.m'
        'test_validation_functions.m'
        'test_visualization_functions.m'
        'test_model_export.m'
        'test_batch_prediction.m'
    };
    
    for i = 1:length(unit_tests)
        test_file = unit_tests{i};
        fprintf('  Running %s... ', test_file);
        
        try
            % Check if test file exists
            if ~isfile(test_file)
                fprintf('SKIP (file not found)\n');
                test_results.tests{end+1} = test_file;
                test_results.passed(end+1) = -1; % -1 = skipped
                continue;
            end
            
            % Run the test in a safe way
            [~, test_name, ~] = fileparts(test_file);
            
            % Clear any previous test variables
            evalin('base', 'clear test_*');
            
            % Run the test
            evalc(test_name);  % Use evalc to capture output
            
            fprintf('PASS\n');
            test_results.tests{end+1} = test_file;
            test_results.passed(end+1) = 1;
        catch ME
            fprintf('FAIL\n');
            fprintf('    Error: %s\n', ME.message);
            test_results.tests{end+1} = test_file;
            test_results.passed(end+1) = 0;
            test_results.all_passed = false;
        end
    end
    
    % Summary
    test_results.num_passed = sum(test_results.passed == 1);
    test_results.num_failed = sum(test_results.passed == 0);
    test_results.num_skipped = sum(test_results.passed == -1);
    
    fprintf('\n  Unit Tests Summary: %d passed, %d failed, %d skipped\n', ...
            test_results.num_passed, test_results.num_failed, test_results.num_skipped);
end

%% Helper function: Run property-based tests
function test_results = run_property_tests()
    % Initialize structure first
    test_results = struct();
    test_results.properties = {};
    test_results.passed = [];
    test_results.all_passed = true;
    test_results.num_passed = 0;
    test_results.num_failed = 0;
    
    fprintf('  Running property-based tests (100 iterations each)...\n\n');
    
    % Property 1: Normalization produces zero mean and unit variance
    fprintf('  Property 1: Normalization produces zero mean and unit variance... ');
    try
        test_normalization_property();
        fprintf('PASS\n');
        test_results.properties{end+1} = 'Property 1';
        test_results.passed(end+1) = 1;
    catch ME
        fprintf('FAIL\n');
        fprintf('    Error: %s\n', ME.message);
        test_results.properties{end+1} = 'Property 1';
        test_results.passed(end+1) = 0;
        test_results.all_passed = false;
    end
    
    % Property 5: RMSE computation is mathematically correct
    fprintf('  Property 5: RMSE computation is mathematically correct... ');
    try
        test_rmse_property();
        fprintf('PASS\n');
        test_results.properties{end+1} = 'Property 5';
        test_results.passed(end+1) = 1;
    catch ME
        fprintf('FAIL\n');
        fprintf('    Error: %s\n', ME.message);
        test_results.properties{end+1} = 'Property 5';
        test_results.passed(end+1) = 0;
        test_results.all_passed = false;
    end
    
    % Property 6: R² computation is mathematically correct
    fprintf('  Property 6: R² computation is mathematically correct... ');
    try
        test_r2_property();
        fprintf('PASS\n');
        test_results.properties{end+1} = 'Property 6';
        test_results.passed(end+1) = 1;
    catch ME
        fprintf('FAIL\n');
        fprintf('    Error: %s\n', ME.message);
        test_results.properties{end+1} = 'Property 6';
        test_results.passed(end+1) = 0;
        test_results.all_passed = false;
    end
    
    % Property 8: Normalization round-trip preserves values
    fprintf('  Property 8: Normalization round-trip preserves values... ');
    try
        test_normalization_roundtrip_property();
        fprintf('PASS\n');
        test_results.properties{end+1} = 'Property 8';
        test_results.passed(end+1) = 1;
    catch ME
        fprintf('FAIL\n');
        fprintf('    Error: %s\n', ME.message);
        test_results.properties{end+1} = 'Property 8';
        test_results.passed(end+1) = 0;
        test_results.all_passed = false;
    end
    
    % Summary
    test_results.num_passed = sum(test_results.passed == 1);
    test_results.num_failed = sum(test_results.passed == 0);
    
    fprintf('\n  Property Tests Summary: %d passed, %d failed\n', ...
            test_results.num_passed, test_results.num_failed);
end

%% Helper function: Train full model
function training_results = train_full_model()
    training_results = struct();
    training_results.success = false;
    
    try
        % Start timer
        tic;
        
        % Load full TTC dataset
        fprintf('  Loading TTC dataset...\n');
        data_paths = {
            '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
        };
        
        data = loadTireData(data_paths);
        fprintf('  Loaded %d samples\n', height(data));
        training_results.num_samples = height(data);
        
        % Preprocess data
        fprintf('  Preprocessing data...\n');
        config = struct('split_ratio', 0.8);
        [X_train, Y_train, X_val, Y_val, norm_params] = preprocessDataWrapper(data, config);
        
        % Create model
        fprintf('  Creating neural network model...\n');
        net = createTireModel();
        
        % Configure training
        config = struct();
        config.max_epochs = 200;
        config.batch_size = 256;
        config.learning_rate = 0.001;
        config.patience = 10;
        config.val_frequency = 50;
        config.random_seed = 42;
        
        % Train model
        fprintf('  Training model (this may take up to 30 minutes)...\n');
        [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
        
        % Record training time
        training_results.training_time_seconds = toc;
        training_results.training_time_minutes = training_results.training_time_seconds / 60;
        
        fprintf('  Training completed in %.2f minutes\n', training_results.training_time_minutes);
        
        % Evaluate model
        fprintf('  Evaluating model performance...\n');
        metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
        
        % Store results
        training_results.trained_net = trained_net;
        training_results.norm_params = norm_params;
        training_results.metrics = metrics;
        training_results.training_info = training_info;
        training_results.X_val = X_val;
        training_results.Y_val = Y_val;
        training_results.success = true;
        
        % Save trained model
        output_dir = 'output';
        if ~exist(output_dir, 'dir')
            mkdir(output_dir);
        end
        model_path = fullfile(output_dir, 'final_trained_model.mat');
        save(model_path, 'trained_net', 'norm_params', 'metrics', 'training_info');
        training_results.model_path = model_path;
        fprintf('  Model saved to: %s\n', model_path);
        
    catch ME
        fprintf('  ERROR during training: %s\n', ME.message);
        training_results.error = ME.message;
        training_results.success = false;
    end
end

%% Helper function: Verify performance requirements
function perf_results = verify_performance_requirements(training_results)
    perf_results = struct();
    perf_results.all_met = true;
    
    if ~training_results.success
        fprintf('  Cannot verify performance - training failed\n');
        perf_results.all_met = false;
        return;
    end
    
    % Requirement 1: RMSE < 100N for force predictions
    fprintf('  Checking RMSE requirements (< 100N for forces)...\n');
    rmse_fx = training_results.metrics.Fx.RMSE;
    rmse_fy = training_results.metrics.Fy.RMSE;
    
    fprintf('    Fx RMSE: %.2f N %s\n', rmse_fx, check_mark(rmse_fx < 100));
    fprintf('    Fy RMSE: %.2f N %s\n', rmse_fy, check_mark(rmse_fy < 100));
    
    perf_results.rmse_fx_met = rmse_fx < 100;
    perf_results.rmse_fy_met = rmse_fy < 100;
    perf_results.rmse_met = perf_results.rmse_fx_met && perf_results.rmse_fy_met;
    
    if ~perf_results.rmse_met
        perf_results.all_met = false;
    end
    
    % Requirement 2: R² > 0.95 for all outputs
    fprintf('  Checking R² requirements (> 0.95 for all outputs)...\n');
    r2_fx = training_results.metrics.Fx.R2;
    r2_fy = training_results.metrics.Fy.R2;
    r2_mx = training_results.metrics.Mx.R2;
    r2_mz = training_results.metrics.Mz.R2;
    
    fprintf('    Fx R²: %.4f %s\n', r2_fx, check_mark(r2_fx > 0.95));
    fprintf('    Fy R²: %.4f %s\n', r2_fy, check_mark(r2_fy > 0.95));
    fprintf('    Mx R²: %.4f %s\n', r2_mx, check_mark(r2_mx > 0.95));
    fprintf('    Mz R²: %.4f %s\n', r2_mz, check_mark(r2_mz > 0.95));
    
    perf_results.r2_fx_met = r2_fx > 0.95;
    perf_results.r2_fy_met = r2_fy > 0.95;
    perf_results.r2_mx_met = r2_mx > 0.95;
    perf_results.r2_mz_met = r2_mz > 0.95;
    perf_results.r2_met = perf_results.r2_fx_met && perf_results.r2_fy_met && ...
                          perf_results.r2_mx_met && perf_results.r2_mz_met;
    
    if ~perf_results.r2_met
        perf_results.all_met = false;
    end
    
    % Requirement 3: Training time < 30 minutes with GPU
    fprintf('  Checking training time requirement (< 30 minutes)...\n');
    training_time = training_results.training_time_minutes;
    fprintf('    Training time: %.2f minutes %s\n', training_time, check_mark(training_time < 30));
    
    perf_results.training_time_met = training_time < 30;
    if ~perf_results.training_time_met
        perf_results.all_met = false;
    end
    
    % Requirement 4: Prediction time < 1ms per sample
    fprintf('  Checking prediction time requirement (< 1ms per sample)...\n');
    
    % Test prediction time on 1000 samples
    test_samples = randn(1000, 5);
    tic;
    predict(training_results.trained_net, test_samples);
    elapsed = toc;
    time_per_sample_ms = (elapsed / 1000) * 1000;
    
    fprintf('    Prediction time: %.4f ms per sample %s\n', time_per_sample_ms, ...
            check_mark(time_per_sample_ms < 1));
    
    perf_results.prediction_time_ms = time_per_sample_ms;
    perf_results.prediction_time_met = time_per_sample_ms < 1;
    
    if ~perf_results.prediction_time_met
        perf_results.all_met = false;
    end
end

%% Helper function: Generate final report
function report_path = generate_final_report(results)
    output_dir = 'output';
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    report_path = fullfile(output_dir, 'final_validation_report.txt');
    fid = fopen(report_path, 'w');
    
    fprintf(fid, '=================================================================\n');
    fprintf(fid, 'MATLAB Deep Learning Tire Model - Final Validation Report\n');
    fprintf(fid, '=================================================================\n\n');
    fprintf(fid, 'Generated: %s\n\n', char(results.timestamp));
    
    % Unit Tests
    fprintf(fid, 'UNIT TESTS\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    fprintf(fid, 'Passed: %d\n', results.unit_tests.num_passed);
    fprintf(fid, 'Failed: %d\n', results.unit_tests.num_failed);
    fprintf(fid, 'Skipped: %d\n', results.unit_tests.num_skipped);
    fprintf(fid, 'Status: %s\n\n', status_string(results.unit_tests.all_passed));
    
    % Property Tests
    fprintf(fid, 'PROPERTY-BASED TESTS\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    fprintf(fid, 'Passed: %d\n', results.property_tests.num_passed);
    fprintf(fid, 'Failed: %d\n', results.property_tests.num_failed);
    fprintf(fid, 'Status: %s\n\n', status_string(results.property_tests.all_passed));
    
    % Training
    fprintf(fid, 'MODEL TRAINING\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    if results.training.success
        fprintf(fid, 'Dataset size: %d samples\n', results.training.num_samples);
        fprintf(fid, 'Training time: %.2f minutes\n', results.training.training_time_minutes);
        fprintf(fid, 'Status: SUCCESS\n\n');
        
        % Metrics
        fprintf(fid, 'PERFORMANCE METRICS\n');
        fprintf(fid, '-----------------------------------------------------------------\n');
        fprintf(fid, 'Fx - RMSE: %.2f N, R²: %.4f, TIC: %.4f\n', ...
                results.training.metrics.Fx.RMSE, ...
                results.training.metrics.Fx.R2, ...
                results.training.metrics.Fx.TIC);
        fprintf(fid, 'Fy - RMSE: %.2f N, R²: %.4f, TIC: %.4f\n', ...
                results.training.metrics.Fy.RMSE, ...
                results.training.metrics.Fy.R2, ...
                results.training.metrics.Fy.TIC);
        fprintf(fid, 'Mx - RMSE: %.2f Nm, R²: %.4f, TIC: %.4f\n', ...
                results.training.metrics.Mx.RMSE, ...
                results.training.metrics.Mx.R2, ...
                results.training.metrics.Mx.TIC);
        fprintf(fid, 'Mz - RMSE: %.2f Nm, R²: %.4f, TIC: %.4f\n\n', ...
                results.training.metrics.Mz.RMSE, ...
                results.training.metrics.Mz.R2, ...
                results.training.metrics.Mz.TIC);
    else
        fprintf(fid, 'Status: FAILED\n');
        fprintf(fid, 'Error: %s\n\n', results.training.error);
    end
    
    % Performance Requirements
    fprintf(fid, 'PERFORMANCE REQUIREMENTS\n');
    fprintf(fid, '-----------------------------------------------------------------\n');
    if results.training.success
        fprintf(fid, 'RMSE < 100N: %s\n', status_string(results.performance.rmse_met));
        fprintf(fid, 'R² > 0.95: %s\n', status_string(results.performance.r2_met));
        fprintf(fid, 'Training time < 30 min: %s\n', status_string(results.performance.training_time_met));
        fprintf(fid, 'Prediction time < 1ms: %s\n\n', status_string(results.performance.prediction_time_met));
    else
        fprintf(fid, 'Cannot verify - training failed\n\n');
    end
    
    % Overall Status
    fprintf(fid, '=================================================================\n');
    fprintf(fid, 'OVERALL STATUS: %s\n', status_string(results.all_requirements_met));
    fprintf(fid, '=================================================================\n');
    
    fclose(fid);
    fprintf('  Report saved to: %s\n', report_path);
end

%% Property test implementations
function test_normalization_property()
    num_iterations = 100;
    tolerance = 1e-6;
    
    for iter = 1:num_iterations
        % Generate random data
        n_samples = randi([100, 1000]);
        n_features = 5;
        X = randn(n_samples, n_features) * 10 + 5;
        
        % Normalize
        [X_norm, mu, sigma] = normalizeData(X);
        
        % Verify zero mean and unit variance
        computed_mean = mean(X_norm, 1);
        computed_std = std(X_norm, 0, 1);
        
        if any(abs(computed_mean) > tolerance) || any(abs(computed_std - 1) > tolerance)
            error('Iteration %d: Normalization failed', iter);
        end
    end
end

function test_rmse_property()
    num_iterations = 100;
    tolerance = 1e-10;
    
    for iter = 1:num_iterations
        % Generate random predictions and actuals
        n_samples = randi([50, 500]);
        y_actual = randn(n_samples, 1) * 100;
        y_pred = y_actual + randn(n_samples, 1) * 10;
        
        % Compute RMSE using function
        rmse_computed = computeRMSE(y_actual, y_pred);
        
        % Compute RMSE manually
        rmse_expected = sqrt(mean((y_pred - y_actual).^2));
        
        if abs(rmse_computed - rmse_expected) > tolerance
            error('Iteration %d: RMSE computation incorrect', iter);
        end
    end
end

function test_r2_property()
    num_iterations = 100;
    tolerance = 1e-10;
    
    for iter = 1:num_iterations
        % Generate random predictions and actuals
        n_samples = randi([50, 500]);
        y_actual = randn(n_samples, 1) * 100;
        y_pred = y_actual + randn(n_samples, 1) * 10;
        
        % Compute R² using function
        r2_computed = computeR2(y_actual, y_pred);
        
        % Compute R² manually
        SS_res = sum((y_actual - y_pred).^2);
        SS_tot = sum((y_actual - mean(y_actual)).^2);
        r2_expected = 1 - SS_res / SS_tot;
        
        if abs(r2_computed - r2_expected) > tolerance
            error('Iteration %d: R² computation incorrect', iter);
        end
    end
end

function test_normalization_roundtrip_property()
    num_iterations = 100;
    tolerance = 1e-10;
    
    for iter = 1:num_iterations
        % Generate random data
        n_samples = randi([100, 1000]);
        n_features = 5;
        X_original = randn(n_samples, n_features) * 100 + 50;
        
        % Normalize
        [X_norm, mu, sigma] = normalizeData(X_original);
        
        % Denormalize
        X_recovered = denormalizeData(X_norm, mu, sigma);
        
        % Check round-trip
        max_rel_error = max(abs((X_recovered - X_original) ./ (X_original + 1e-10)), [], 'all');
        
        if max_rel_error > tolerance
            error('Iteration %d: Round-trip normalization failed', iter);
        end
    end
end

%% Utility functions
function str = status_string(passed)
    if passed
        str = 'PASS';
    else
        str = 'FAIL';
    end
end

function mark = check_mark(condition)
    if condition
        mark = '✓';
    else
        mark = '✗';
    end
end
