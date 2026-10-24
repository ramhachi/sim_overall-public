%% VALIDATION FUNCTIONS QUICK REFERENCE
% This script demonstrates the usage of all validation and metrics functions
% implemented in Task 7.

%% Setup: Load and preprocess data (example)
% data_paths = {'../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'};
% config = struct('split_ratio', 0.8);
% [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);

%% Example 1: Compute individual metrics
fprintf('=== Example 1: Individual Metrics ===\n\n');

% Create example data
y_actual = randn(100, 4) * 500;  % 100 samples, 4 outputs
y_pred = y_actual + randn(100, 4) * 50;  % Add some noise

% Compute metrics
rmse = computeRMSE(y_actual, y_pred);
r2 = computeR2(y_actual, y_pred);
tic_value = computeTIC(y_actual, y_pred);

fprintf('RMSE: [%.2f, %.2f, %.2f, %.2f]\n', rmse);
fprintf('R²:   [%.4f, %.4f, %.4f, %.4f]\n', r2);
fprintf('TIC:  [%.4f, %.4f, %.4f, %.4f]\n\n', tic_value);

%% Example 2: Complete model evaluation
fprintf('=== Example 2: Complete Model Evaluation ===\n\n');

% Note: This requires a trained model
% metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
% 
% % Check requirements
% if metrics.meets_requirements
%     fprintf('✓ Model meets all performance requirements!\n');
% else
%     fprintf('⚠ Model does not meet some requirements\n');
% end
% 
% % Access specific metrics
% fprintf('Fx: RMSE=%.2f N, R²=%.4f, TIC=%.4f\n', ...
%         metrics.Fx.RMSE, metrics.Fx.R2, metrics.Fx.TIC);

fprintf('(Requires trained model - see evaluateModel.m)\n\n');

%% Example 3: Input range computation
fprintf('=== Example 3: Input Range Computation ===\n\n');

% Create example training data
X_train_example = [
    -0.15, -10, 1000, -3, 25;
    -0.05,  -5, 1500, -1, 35;
     0.00,   0, 2000,  0, 45;
     0.05,   5, 2000,  1, 55;
     0.15,  10, 2500,  3, 75
];

input_ranges = computeInputRanges(X_train_example);
fprintf('\n');

%% Example 4: Out-of-range detection
fprintf('=== Example 4: Out-of-Range Detection ===\n\n');

% Test with in-range inputs
X_in_range = [0.0, 0, 1500, 0, 50];
[is_oor, warnings] = detectOutOfRange(X_in_range, input_ranges);
fprintf('In-range test: %d samples flagged\n', sum(is_oor));

% Test with out-of-range inputs
X_out_range = [
    0.25,  10, 1500, 0, 50;   % SR out of range
    0.00, -15, 1500, 0, 50;   % SA out of range
    0.00,   0,  800, 0, 50    % Fz out of range
];
[is_oor, warnings] = detectOutOfRange(X_out_range, input_ranges);
fprintf('\nOut-of-range test: %d/%d samples flagged\n', sum(is_oor), size(X_out_range, 1));
fprintf('Warnings:\n');
for i = 1:length(warnings)
    fprintf('  %s\n', warnings{i});
end
fprintf('\n');

%% Example 5: Typical workflow
fprintf('=== Example 5: Typical Workflow ===\n\n');
fprintf('1. Preprocess data:\n');
fprintf('   [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(...);\n\n');

fprintf('2. Compute input ranges (before normalization):\n');
fprintf('   X_train_raw = denormalizeData(X_train, norm_params.input_mean, norm_params.input_std);\n');
fprintf('   input_ranges = computeInputRanges(X_train_raw);\n\n');

fprintf('3. Train model:\n');
fprintf('   trained_net = trainTireModel(...);\n\n');

fprintf('4. Evaluate model:\n');
fprintf('   metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);\n\n');

fprintf('5. Make predictions with range checking:\n');
fprintf('   [is_oor, warnings] = detectOutOfRange(X_new, input_ranges);\n');
fprintf('   if ~any(is_oor)\n');
fprintf('       Y_pred = predict(trained_net, normalize(X_new));\n');
fprintf('   end\n\n');

%% Summary
fprintf('=== Summary ===\n');
fprintf('Available functions:\n');
fprintf('  - computeRMSE(y_actual, y_pred)\n');
fprintf('  - computeR2(y_actual, y_pred)\n');
fprintf('  - computeTIC(y_actual, y_pred)\n');
fprintf('  - evaluateModel(trained_net, X_val, Y_val, norm_params)\n');
fprintf('  - computeInputRanges(X_train)\n');
fprintf('  - detectOutOfRange(X_raw, input_ranges)\n');
fprintf('\nFor detailed documentation, see individual function files.\n');
