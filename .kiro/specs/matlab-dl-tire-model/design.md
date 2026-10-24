# Design Document: MATLAB Deep Learning Tire Model

## Overview

This design document describes a deep learning-based tire model implemented in MATLAB that predicts tire forces (Fx, Fy) and moments (Mx, Mz) from tire state variables. The model uses a feedforward neural network trained on approximately 140,000 samples from TTC tire test data, providing an alternative to traditional physics-based models like TMeasy and Magic Formula.

The system consists of five main components:
1. **Data Preprocessor**: Loads, cleans, normalizes, and splits tire test data
2. **Neural Network Model**: Feedforward architecture mapping 5 inputs to 4 outputs
3. **Training Engine**: Trains the network using GPU acceleration with early stopping
4. **Validation Engine**: Evaluates performance using RMSE, R², and TIC metrics
5. **Model Exporter**: Saves trained models for integration with vehicle simulators

The design prioritizes prediction accuracy (RMSE < 100N, R² > 0.95), computational efficiency (< 1ms per prediction), and ease of integration with existing simulation workflows.

## Architecture

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     MATLAB DL Tire Model System                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐  │
│  │   CSV Data   │─────>│     Data     │─────>│   Training   │  │
│  │   (TTC)      │      │ Preprocessor │      │    Engine    │  │
│  └──────────────┘      └──────────────┘      └──────┬───────┘  │
│                              │                       │           │
│                              │                       v           │
│                              │              ┌──────────────┐    │
│                              │              │  DL Model    │    │
│                              │              │  (Network)   │    │
│                              │              └──────┬───────┘    │
│                              │                     │            │
│                              v                     v            │
│                        ┌──────────────┐    ┌──────────────┐   │
│                        │ Validation   │<───│    Model     │   │
│                        │   Engine     │    │   Exporter   │   │
│                        └──────────────┘    └──────────────┘   │
│                              │                     │            │
│                              v                     v            │
│                        ┌──────────────┐    ┌──────────────┐   │
│                        │ Visualizations│    │  Exported    │   │
│                        │  & Metrics    │    │   Model      │   │
│                        └──────────────┘    └──────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Data Loading**: CSV files → Data Preprocessor → Normalized arrays
2. **Training**: Training data → Training Engine → Trained network weights
3. **Validation**: Validation data → Trained model → Predictions → Metrics
4. **Export**: Trained model + normalization params → .mat file or C++ code

### Technology Stack

- **MATLAB R2023a or later**: Core platform
- **Deep Learning Toolbox**: Neural network creation and training
- **Parallel Computing Toolbox**: GPU acceleration (NVIDIA RTX 4070 Ti)
- **Statistics and Machine Learning Toolbox**: Data preprocessing and metrics
- **MATLAB Coder** (optional): C++ code generation for simulator integration

## Components and Interfaces

### Component 1: Data Preprocessor

**Purpose**: Load, clean, normalize, and split tire test data for training and validation.

**Inputs**:
- CSV file paths for DriveBrake and Cornering test data
- Configuration parameters (train/validation split ratio, normalization method)

**Outputs**:
- Training dataset: `X_train` (N×5), `Y_train` (N×4)
- Validation dataset: `X_val` (M×5), `Y_val` (M×4)
- Normalization parameters: `input_mean`, `input_std`, `output_mean`, `output_std`

**Key Functions**:

```matlab
function [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config)
    % Load CSV data
    data = loadTireData(data_paths);
    
    % Extract features and targets
    X = [data.SR, data.SA, data.Fz, data.IA, data.TSTC];  % 5 inputs
    Y = [data.Fx, data.Fy, data.Mx, data.Mz];             % 4 outputs
    
    % Handle missing values
    [X, Y] = removeMissingData(X, Y);
    
    % Normalize inputs and outputs
    [X_norm, input_mean, input_std] = normalizeInputs(X);
    [Y_norm, output_mean, output_std] = normalizeOutputs(Y);
    
    % Split into train/validation
    [X_train, Y_train, X_val, Y_val] = splitData(X_norm, Y_norm, config.split_ratio);
    
    % Store normalization parameters
    norm_params = struct('input_mean', input_mean, 'input_std', input_std, ...
                         'output_mean', output_mean, 'output_std', output_std);
end
```

**Normalization Strategy**:
- Inputs: Z-score normalization (zero mean, unit variance)
- Outputs: Z-score normalization to balance loss contributions from different outputs

### Component 2: Neural Network Model

**Purpose**: Define the neural network architecture that maps tire state variables to forces and moments.

**Architecture**:

```
Input Layer (5 neurons)
    ↓
Hidden Layer 1 (128 neurons, ReLU activation)
    ↓
Dropout (0.2)
    ↓
Hidden Layer 2 (64 neurons, ReLU activation)
    ↓
Dropout (0.2)
    ↓
Hidden Layer 3 (32 neurons, ReLU activation)
    ↓
Output Layer (4 neurons, linear activation)
```

**Rationale**:
- **3 hidden layers**: Sufficient depth to capture nonlinear tire behavior
- **Decreasing layer sizes (128→64→32)**: Hierarchical feature extraction
- **ReLU activation**: Prevents vanishing gradients, computationally efficient
- **Dropout (0.2)**: Regularization to prevent overfitting
- **Linear output**: Regression task requires unbounded outputs

**MATLAB Implementation**:

```matlab
function net = createTireModel()
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        fullyConnectedLayer(128, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.2, 'Name', 'dropout1')
        
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.2, 'Name', 'dropout2')
        
        fullyConnectedLayer(32, 'Name', 'fc3')
        reluLayer('Name', 'relu3')
        
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end
```

**Alternative Architecture (LSTM for Temporal Dependencies)**:

If temporal dependencies are important (e.g., tire temperature dynamics), an LSTM architecture can be used:

```matlab
function net = createLSTMTireModel()
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
```

### Component 3: Training Engine

**Purpose**: Train the neural network using GPU acceleration with proper optimization and early stopping.

**Training Configuration**:

```matlab
function options = createTrainingOptions(config)
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
end
```

**Key Parameters**:
- **Optimizer**: Adam (adaptive learning rate, momentum)
- **Learning rate**: 0.001 (initial), with decay schedule
- **Batch size**: 256 (balance between speed and stability)
- **Max epochs**: 200
- **Early stopping**: Patience of 10 epochs (stop if validation loss doesn't improve)
- **Execution environment**: 'auto' (uses GPU if available, falls back to CPU)

**Training Function**:

```matlab
function [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config)
    % Set random seed for reproducibility
    rng(config.random_seed);
    
    % Check GPU availability
    if gpuDeviceCount > 0
        fprintf('GPU detected: %s\n', gpuDevice().Name);
    else
        warning('No GPU detected. Training will use CPU.');
    end
    
    % Create training options
    options = createTrainingOptions(config);
    
    % Train the network
    [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
    
    % Report final metrics
    fprintf('Training complete. Final validation loss: %.4f\n', training_info.FinalValidationLoss);
end
```

### Component 4: Validation Engine

**Purpose**: Evaluate model performance using standard metrics and generate visualizations.

**Performance Metrics**:

```matlab
function metrics = evaluateModel(trained_net, X_val, Y_val, norm_params)
    % Make predictions
    Y_pred_norm = predict(trained_net, X_val);
    
    % Denormalize predictions and targets
    Y_pred = denormalize(Y_pred_norm, norm_params.output_mean, norm_params.output_std);
    Y_actual = denormalize(Y_val, norm_params.output_mean, norm_params.output_std);
    
    % Compute metrics for each output
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    for i = 1:4
        metrics.(output_names{i}).RMSE = sqrt(mean((Y_pred(:,i) - Y_actual(:,i)).^2));
        metrics.(output_names{i}).R2 = computeR2(Y_actual(:,i), Y_pred(:,i));
        metrics.(output_names{i}).TIC = computeTIC(Y_actual(:,i), Y_pred(:,i));
        metrics.(output_names{i}).MAE = mean(abs(Y_pred(:,i) - Y_actual(:,i)));
    end
    
    % Check performance thresholds
    metrics.meets_requirements = checkRequirements(metrics);
end

function R2 = computeR2(y_actual, y_pred)
    SS_res = sum((y_actual - y_pred).^2);
    SS_tot = sum((y_actual - mean(y_actual)).^2);
    R2 = 1 - SS_res / SS_tot;
end

function TIC = computeTIC(y_actual, y_pred)
    TIC = sqrt(mean((y_pred - y_actual).^2)) / ...
          (sqrt(mean(y_actual.^2)) + sqrt(mean(y_pred.^2)));
end
```

**Visualization Functions**:

```matlab
function generateVisualizations(Y_actual, Y_pred, output_dir)
    output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    
    for i = 1:4
        % Scatter plot: predicted vs actual
        figure;
        scatter(Y_actual(:,i), Y_pred(:,i), 10, 'filled', 'MarkerFaceAlpha', 0.3);
        hold on;
        plot([min(Y_actual(:,i)), max(Y_actual(:,i))], ...
             [min(Y_actual(:,i)), max(Y_actual(:,i))], 'r--', 'LineWidth', 2);
        xlabel(['Actual ', output_names{i}]);
        ylabel(['Predicted ', output_names{i}]);
        title(['Predicted vs Actual: ', output_names{i}]);
        grid on;
        saveas(gcf, fullfile(output_dir, ['scatter_', output_names{i}, '.png']));
        
        % Residual plot
        figure;
        residuals = Y_pred(:,i) - Y_actual(:,i);
        scatter(Y_actual(:,i), residuals, 10, 'filled', 'MarkerFaceAlpha', 0.3);
        hold on;
        yline(0, 'r--', 'LineWidth', 2);
        xlabel(['Actual ', output_names{i}]);
        ylabel('Residual');
        title(['Residual Plot: ', output_names{i}]);
        grid on;
        saveas(gcf, fullfile(output_dir, ['residual_', output_names{i}, '.png']));
    end
end
```

### Component 5: Model Exporter

**Purpose**: Export the trained model for integration with vehicle simulators.

**Export Formats**:

1. **MATLAB .mat file** (for MATLAB-based simulators):

```matlab
function exportModel(trained_net, norm_params, output_path)
    % Save network and normalization parameters
    save(output_path, 'trained_net', 'norm_params');
    
    % Create prediction function
    predict_fn = @(X_raw) predictTireForces(trained_net, X_raw, norm_params);
    save([output_path(1:end-4), '_predict.mat'], 'predict_fn');
end

function Y = predictTireForces(net, X_raw, norm_params)
    % Normalize inputs
    X_norm = (X_raw - norm_params.input_mean) ./ norm_params.input_std;
    
    % Predict
    Y_norm = predict(net, X_norm);
    
    % Denormalize outputs
    Y = Y_norm .* norm_params.output_std + norm_params.output_mean;
end
```

2. **MATLAB Coder for C++ integration** (optional):

```matlab
function generateCppCode(trained_net, norm_params, output_dir)
    % Create a wrapper function for code generation
    function Y = tireModelPredict(X_raw)
        X_norm = (X_raw - norm_params.input_mean) ./ norm_params.input_std;
        Y_norm = predict(trained_net, X_norm);
        Y = Y_norm .* norm_params.output_std + norm_params.output_mean;
    end
    
    % Generate C++ code
    codegen tireModelPredict -args {zeros(1,5)} -config:lib -o tire_model_cpp -d output_dir
end
```

3. **Lookup table** (for fast C++ integration without MATLAB Coder):

```matlab
function generateLookupTable(trained_net, norm_params, output_path, grid_resolution)
    % Define input ranges
    SR_range = linspace(-0.2, 0.2, grid_resolution(1));
    SA_range = linspace(-15, 15, grid_resolution(2));
    Fz_range = linspace(500, 2500, grid_resolution(3));
    IA_range = linspace(-5, 5, grid_resolution(4));
    TSTC_range = linspace(20, 80, grid_resolution(5));
    
    % Generate grid
    [SR, SA, Fz, IA, TSTC] = ndgrid(SR_range, SA_range, Fz_range, IA_range, TSTC_range);
    X_grid = [SR(:), SA(:), Fz(:), IA(:), TSTC(:)];
    
    % Predict on grid
    Y_grid = predictTireForces(trained_net, X_grid, norm_params);
    
    % Save lookup table
    lookup_table = struct('SR_range', SR_range, 'SA_range', SA_range, ...
                          'Fz_range', Fz_range, 'IA_range', IA_range, ...
                          'TSTC_range', TSTC_range, 'Y_grid', Y_grid);
    save(output_path, 'lookup_table');
end
```

## Data Models

### Input Features (5 dimensions)

| Feature | Symbol | Unit | Typical Range | Description |
|---------|--------|------|---------------|-------------|
| Slip Ratio | SR | - | [-0.2, 0.2] | Longitudinal slip ratio |
| Slip Angle | SA | deg | [-15, 15] | Lateral slip angle |
| Vertical Load | Fz | N | [500, 2500] | Normal force on tire |
| Inclination Angle | IA | deg | [-5, 5] | Camber angle |
| Tire Temperature | TSTC | °C | [20, 80] | Tire surface temperature |

### Output Predictions (4 dimensions)

| Output | Symbol | Unit | Typical Range | Description |
|--------|--------|------|---------------|-------------|
| Longitudinal Force | Fx | N | [-2000, 2000] | Force in x-direction |
| Lateral Force | Fy | N | [-2000, 2000] | Force in y-direction |
| Overturning Moment | Mx | Nm | [-100, 100] | Moment about x-axis |
| Aligning Moment | Mz | Nm | [-50, 50] | Moment about z-axis |

### Normalization Parameters

```matlab
struct NormalizationParams
    input_mean: [5×1 double]   % Mean of each input feature
    input_std: [5×1 double]    % Std dev of each input feature
    output_mean: [4×1 double]  % Mean of each output
    output_std: [4×1 double]   % Std dev of each output
end
```

### Training Configuration

```matlab
struct TrainingConfig
    max_epochs: int            % Maximum training epochs (default: 200)
    batch_size: int            % Mini-batch size (default: 256)
    learning_rate: double      % Initial learning rate (default: 0.001)
    patience: int              % Early stopping patience (default: 10)
    val_frequency: int         % Validation frequency in iterations (default: 50)
    split_ratio: double        % Train/val split (default: 0.8)
    random_seed: int           % Random seed for reproducibility (default: 42)
end
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*


### Property 1: Normalization produces zero mean and unit variance

*For any* dataset with input features and output targets, after normalization, the normalized data should have mean approximately zero (within numerical tolerance) and standard deviation approximately one.

**Validates: Requirements 1.3, 1.4**

### Property 2: Data extraction preserves column mapping

*For any* tire test dataset, extracting input features should produce a matrix where columns correspond to [SR, SA, Fz, IA, TSTC] in order, and extracting output targets should produce a matrix where columns correspond to [Fx, Fy, Mx, Mz] in order.

**Validates: Requirements 1.2**

### Property 3: Data splitting maintains specified ratio

*For any* dataset and split ratio r, partitioning the data should result in training set size approximately equal to r × total size and validation set size approximately equal to (1-r) × total size.

**Validates: Requirements 1.5**

### Property 4: Normalization parameters are stored and retrievable

*For any* dataset that has been normalized, the normalization parameters (mean and standard deviation for inputs and outputs) should be stored and retrievable for later use during inference.

**Validates: Requirements 1.7**

### Property 5: RMSE computation is mathematically correct

*For any* set of predictions and actual values, the computed RMSE should equal the square root of the mean of squared differences between predictions and actual values.

**Validates: Requirements 4.1**

### Property 6: R² computation is mathematically correct

*For any* set of predictions and actual values, the computed R² should equal 1 minus the ratio of residual sum of squares to total sum of squares.

**Validates: Requirements 4.2**

### Property 7: TIC computation is mathematically correct

*For any* set of predictions and actual values, the computed TIC should equal the RMSE divided by the sum of RMS of predictions and RMS of actual values.

**Validates: Requirements 4.3**

### Property 8: Normalization round-trip preserves values

*For any* dataset, normalizing then denormalizing using the same normalization parameters should recover the original values within numerical precision (relative error < 1e-10).

**Validates: Requirements 4.5**

### Property 9: Prediction function applies correct transformations

*For any* raw input, the prediction function should normalize the input, pass it through the network, and denormalize the output, producing the same result as manually applying these steps.

**Validates: Requirements 6.3**

### Property 10: Model export-reload preserves predictions

*For any* trained model and test input, exporting the model to a file and reloading it should produce identical predictions to the original model (within numerical precision).

**Validates: Requirements 6.6**

### Property 11: Batch predictions match individual predictions

*For any* batch of input samples, making predictions on the entire batch should produce the same results as making predictions on each sample individually and concatenating the results.

**Validates: Requirements 9.2**

### Property 12: Reproducibility with fixed random seed

*For any* training configuration with a fixed random seed, training the model twice with the same seed and same data should produce identical trained weights and identical predictions on test data.

**Validates: Requirements 10.3**

### Property 13: Out-of-range input detection

*For any* input where at least one feature is outside the training data range, the prediction function should either flag the input as out-of-range or provide a warning about potential extrapolation.

**Validates: Requirements 8.3**

## Error Handling

### Data Loading Errors

**Missing or corrupted CSV files**:
- Check file existence before loading
- Validate CSV structure (expected columns present)
- Provide clear error messages indicating which file and what's wrong

```matlab
function data = loadTireData(data_paths)
    for i = 1:length(data_paths)
        if ~isfile(data_paths{i})
            error('Data file not found: %s', data_paths{i});
        end
        
        try
            data_table = readtable(data_paths{i});
        catch ME
            error('Failed to read CSV file %s: %s', data_paths{i}, ME.message);
        end
        
        % Validate required columns
        required_cols = {'SR', 'SA', 'Fz', 'IA', 'TSTC', 'Fx', 'Fy', 'Mx', 'Mz'};
        missing_cols = setdiff(required_cols, data_table.Properties.VariableNames);
        if ~isempty(missing_cols)
            error('Missing required columns in %s: %s', data_paths{i}, strjoin(missing_cols, ', '));
        end
    end
end
```

### Missing Data Handling

**NaN or Inf values in dataset**:
- Detect missing values during preprocessing
- Option 1: Remove rows with missing values
- Option 2: Impute using mean/median (for small amounts of missing data)
- Log how many samples were removed/imputed

```matlab
function [X_clean, Y_clean, stats] = removeMissingData(X, Y)
    % Find rows with NaN or Inf
    bad_rows = any(isnan(X) | isinf(X), 2) | any(isnan(Y) | isinf(Y), 2);
    
    stats.total_samples = size(X, 1);
    stats.removed_samples = sum(bad_rows);
    stats.clean_samples = stats.total_samples - stats.removed_samples;
    
    if stats.removed_samples > 0
        warning('Removed %d samples (%.2f%%) with missing or infinite values', ...
                stats.removed_samples, 100 * stats.removed_samples / stats.total_samples);
    end
    
    X_clean = X(~bad_rows, :);
    Y_clean = Y(~bad_rows, :);
end
```

### Training Errors

**GPU out of memory**:
- Catch GPU memory errors
- Suggest reducing batch size or model size
- Fall back to CPU training if GPU fails

```matlab
function [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config)
    try
        % Attempt GPU training
        options = createTrainingOptions(config);
        [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
    catch ME
        if contains(ME.message, 'out of memory', 'IgnoreCase', true)
            warning('GPU out of memory. Trying with reduced batch size...');
            config.batch_size = floor(config.batch_size / 2);
            options = createTrainingOptions(config);
            try
                [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
            catch ME2
                warning('Still out of memory. Falling back to CPU training...');
                config.execution_environment = 'cpu';
                options = createTrainingOptions(config);
                [trained_net, training_info] = trainNetwork(X_train, Y_train, net, options);
            end
        else
            rethrow(ME);
        end
    end
end
```

**Training divergence (NaN loss)**:
- Monitor for NaN in loss values
- Stop training and suggest reducing learning rate
- Check for numerical instability in data

```matlab
function checkTrainingStability(training_info)
    if any(isnan(training_info.TrainingLoss))
        error(['Training diverged (NaN loss detected). Suggestions:\n' ...
               '1. Reduce learning rate (try 0.0001)\n' ...
               '2. Check data for extreme values\n' ...
               '3. Verify normalization is applied correctly']);
    end
end
```

### Prediction Errors

**Out-of-range inputs**:
- Check if inputs are within training data range
- Warn user about extrapolation
- Optionally clip inputs to valid range

```matlab
function [Y, warnings] = predictWithValidation(net, X_raw, norm_params, input_ranges)
    warnings = {};
    
    % Check input ranges
    for i = 1:size(X_raw, 2)
        if any(X_raw(:,i) < input_ranges.min(i)) || any(X_raw(:,i) > input_ranges.max(i))
            warnings{end+1} = sprintf('Input feature %d is outside training range [%.2f, %.2f]', ...
                                      i, input_ranges.min(i), input_ranges.max(i));
        end
    end
    
    % Make prediction
    Y = predictTireForces(net, X_raw, norm_params);
    
    % Display warnings
    if ~isempty(warnings)
        warning('Prediction may be unreliable:\n%s', strjoin(warnings, '\n'));
    end
end
```

### Model Export Errors

**Export validation failure**:
- After export, reload model and verify predictions match
- If mismatch detected, abort export and report error

```matlab
function exportModel(trained_net, norm_params, output_path)
    % Save model
    save(output_path, 'trained_net', 'norm_params');
    
    % Validate export
    loaded = load(output_path);
    
    % Test on random inputs
    X_test = randn(10, 5);
    Y_original = predict(trained_net, X_test);
    Y_loaded = predict(loaded.trained_net, X_test);
    
    max_diff = max(abs(Y_original - Y_loaded), [], 'all');
    if max_diff > 1e-10
        error('Export validation failed: predictions differ by %.2e', max_diff);
    end
    
    fprintf('Model exported successfully to %s\n', output_path);
end
```

## Testing Strategy

### Dual Testing Approach

This system requires both **unit tests** and **property-based tests** for comprehensive validation:

**Unit Tests** focus on:
- Specific examples with known inputs and expected outputs
- Edge cases (empty datasets, single samples, boundary values)
- Error conditions (missing files, invalid data, GPU unavailable)
- Integration points (data loading → preprocessing → training → export)

**Property-Based Tests** focus on:
- Universal properties that hold for all inputs (normalization, metrics computation)
- Mathematical correctness (RMSE, R², TIC formulas)
- Round-trip properties (normalize→denormalize, export→reload)
- Batch vs individual prediction consistency

### MATLAB Testing Framework

MATLAB provides built-in testing frameworks:
- **matlab.unittest**: Standard unit testing framework
- **Property-based testing**: Can be implemented using random data generation

### Test Organization

```
tests/
├── unit/
│   ├── test_data_loader.m
│   ├── test_preprocessor.m
│   ├── test_model_architecture.m
│   ├── test_training_engine.m
│   ├── test_validation_engine.m
│   └── test_model_exporter.m
├── property/
│   ├── test_normalization_properties.m
│   ├── test_metrics_properties.m
│   ├── test_prediction_properties.m
│   └── test_reproducibility_properties.m
└── integration/
    ├── test_full_workflow.m
    └── test_export_integration.m
```

### Property-Based Testing Configuration

Each property test should:
- Run minimum **100 iterations** with randomly generated data
- Use MATLAB's `rng()` to set random seed for reproducibility
- Tag each test with a comment referencing the design property

**Example property test structure**:

```matlab
classdef test_normalization_properties < matlab.unittest.TestCase
    methods (Test)
        function testNormalizationProducesZeroMeanUnitVariance(testCase)
            % Feature: matlab-dl-tire-model, Property 1: Normalization produces zero mean and unit variance
            
            num_iterations = 100;
            tolerance = 1e-6;
            
            for iter = 1:num_iterations
                % Generate random data
                n_samples = randi([100, 1000]);
                n_features = 5;
                X = randn(n_samples, n_features) * 10 + 5;  % Random mean and scale
                
                % Normalize
                [X_norm, mu, sigma] = normalizeInputs(X);
                
                % Verify zero mean and unit variance
                computed_mean = mean(X_norm, 1);
                computed_std = std(X_norm, 0, 1);
                
                testCase.verifyLessThan(abs(computed_mean), tolerance, ...
                    sprintf('Iteration %d: Mean not close to zero', iter));
                testCase.verifyLessThan(abs(computed_std - 1), tolerance, ...
                    sprintf('Iteration %d: Std not close to one', iter));
            end
        end
    end
end
```

### Unit Test Examples

**Test data loading with specific CSV file**:

```matlab
function testLoadTireDataWithValidCSV(testCase)
    % Create temporary CSV file
    test_data = table([0.1; 0.2], [5; 10], [1000; 1500], [0; -2], [25; 30], ...
                      [500; 600], [200; 300], [10; 15], [5; 8], ...
                      'VariableNames', {'SR', 'SA', 'Fz', 'IA', 'TSTC', 'Fx', 'Fy', 'Mx', 'Mz'});
    temp_file = [tempname, '.csv'];
    writetable(test_data, temp_file);
    
    % Test loading
    data = loadTireData({temp_file});
    
    % Verify
    testCase.verifyEqual(size(data, 1), 2);
    testCase.verifyEqual(data.SR(1), 0.1);
    
    % Cleanup
    delete(temp_file);
end
```

**Test GPU detection**:

```matlab
function testGPUDetection(testCase)
    if gpuDeviceCount > 0
        % GPU available - verify it's detected
        config = struct('max_epochs', 1, 'batch_size', 32);
        options = createTrainingOptions(config);
        testCase.verifyEqual(options.ExecutionEnvironment, 'auto');
    else
        % No GPU - verify fallback to CPU
        warning('No GPU detected for testing');
    end
end
```

**Test model architecture**:

```matlab
function testModelArchitecture(testCase)
    net = createTireModel();
    layers = net.Layers;
    
    % Verify input layer
    testCase.verifyEqual(layers(1).InputSize, 5);
    
    % Verify output layer
    output_layer = layers(end-1);  % Before regression layer
    testCase.verifyEqual(output_layer.OutputSize, 4);
    
    % Verify at least 2 hidden layers
    fc_layers = layers(arrayfun(@(x) isa(x, 'nnet.cnn.layer.FullyConnectedLayer'), layers));
    testCase.verifyGreaterThanOrEqual(length(fc_layers), 4);  % Input + 2 hidden + output
end
```

### Integration Tests

**Full workflow test**:

```matlab
function testFullWorkflow(testCase)
    % Generate synthetic data
    n_samples = 1000;
    X = randn(n_samples, 5);
    Y = randn(n_samples, 4);
    
    % Preprocess
    [X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(X, Y, struct('split_ratio', 0.8));
    
    % Create and train model (1 epoch for speed)
    net = createTireModel();
    config = struct('max_epochs', 1, 'batch_size', 32, 'learning_rate', 0.001, ...
                    'patience', 5, 'val_frequency', 10, 'random_seed', 42);
    [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
    
    % Validate
    metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
    
    % Export
    temp_file = [tempname, '.mat'];
    exportModel(trained_net, norm_params, temp_file);
    
    % Verify export
    testCase.verifyTrue(isfile(temp_file));
    
    % Cleanup
    delete(temp_file);
end
```

### Test Execution

Run all tests:
```matlab
% Run all tests
results = runtests('tests', 'IncludeSubfolders', true);

% Display summary
summary(results)

% Run only property tests
results_property = runtests('tests/property');
```

### Continuous Validation

During development:
1. Run unit tests after each component implementation
2. Run property tests after completing each major feature
3. Run integration tests before committing changes
4. Verify all tests pass before model export

### Performance Benchmarking

While not part of automated testing, performance should be manually verified:
- Training time on full dataset (target: < 30 minutes with GPU)
- Prediction time per sample (target: < 1 ms)
- Memory usage during training and inference
- GPU utilization during training

```matlab
% Benchmark prediction time
n_samples = 1000;
X_test = randn(n_samples, 5);
tic;
Y_pred = predict(trained_net, X_test);
elapsed = toc;
time_per_sample = elapsed / n_samples * 1000;  % Convert to ms
fprintf('Prediction time: %.4f ms per sample\n', time_per_sample);
```
