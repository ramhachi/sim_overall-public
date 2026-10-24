function exportModelWithValidation(trained_net, norm_params, output_path, X_test)
% EXPORTMODELWITHVALIDATION Export model with validation check
%
% This function exports the trained model and validates that the exported
% model produces identical predictions to the original model. If validation
% fails, the export is aborted.
%
% Inputs:
%   trained_net  - Trained neural network
%   norm_params  - Structure with normalization parameters
%   output_path  - Path where the model should be saved
%   X_test       - Optional test data for validation (default: random data)
%
% Outputs:
%   None (saves file to disk if validation passes)
%
% Example:
%   % Export with automatic validation
%   exportModelWithValidation(trained_net, norm_params, 'model.mat');
%   
%   % Export with specific test data
%   X_test = [0.1, 5.0, 1500, -2.0, 30.0];
%   exportModelWithValidation(trained_net, norm_params, 'model.mat', X_test);
%
% Requirements: 6.6

    fprintf('=== Exporting Model with Validation ===\n');
    
    % Validate inputs
    if nargin < 3
        error('exportModelWithValidation requires at least 3 arguments');
    end
    
    % Generate test data if not provided
    if nargin < 4 || isempty(X_test)
        fprintf('Generating random test data for validation...\n');
        rng(42);  % Fixed seed for reproducibility
        X_test = randn(100, 5);  % 100 random test samples
    end
    
    % Step 1: Make predictions with original model
    fprintf('Step 1: Computing predictions with original model...\n');
    X_test_norm = (X_test - norm_params.input_mean) ./ norm_params.input_std;
    Y_original = predict(trained_net, X_test_norm);
    
    % Step 2: Export model
    fprintf('Step 2: Exporting model to %s...\n', output_path);
    exportModel(trained_net, norm_params, output_path);
    
    % Step 3: Reload exported model
    fprintf('Step 3: Reloading exported model...\n');
    loaded = load(output_path);
    
    % Verify required fields exist
    if ~isfield(loaded, 'trained_net')
        error('Exported model missing trained_net field');
    end
    if ~isfield(loaded, 'norm_params')
        error('Exported model missing norm_params field');
    end
    
    % Step 4: Make predictions with reloaded model
    fprintf('Step 4: Computing predictions with reloaded model...\n');
    X_test_norm_loaded = (X_test - loaded.norm_params.input_mean) ./ loaded.norm_params.input_std;
    Y_loaded = predict(loaded.trained_net, X_test_norm_loaded);
    
    % Step 5: Compare predictions
    fprintf('Step 5: Validating predictions match...\n');
    max_diff = max(abs(Y_original - Y_loaded), [], 'all');
    mean_diff = mean(abs(Y_original - Y_loaded), 'all');
    
    fprintf('  Maximum absolute difference: %.2e\n', max_diff);
    fprintf('  Mean absolute difference:    %.2e\n', mean_diff);
    
    % Check if differences are within acceptable tolerance
    tolerance = 1e-10;
    if max_diff > tolerance
        % Validation failed - delete exported file
        delete(output_path);
        error(['Export validation FAILED: predictions differ by %.2e (tolerance: %.2e)\n' ...
               'Exported file has been deleted. Please check the model and try again.'], ...
               max_diff, tolerance);
    end
    
    % Validation passed
    fprintf('\n✓ Export validation PASSED\n');
    fprintf('  Model exported successfully to: %s\n', output_path);
    fprintf('  Predictions match within tolerance (%.2e)\n', tolerance);
    fprintf('=======================================\n\n');
end
