# MATLAB Deep Learning Tire Model

A deep learning-based tire model that predicts tire forces (Fx, Fy) and moments (Mx, Mz) from tire state variables using neural networks. This model provides an alternative to traditional physics-based models (TMeasy, Magic Formula) by learning complex tire behavior patterns from approximately 140,000 TTC tire test data points.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [Model Architecture](#model-architecture)
- [Training Procedure](#training-procedure)
- [Using Trained Models](#using-trained-models)
- [Directory Structure](#directory-structure)
- [Data Format](#data-format)
- [Complete Workflows](#complete-workflows)
- [Testing](#testing)
- [Performance Metrics](#performance-metrics)
- [Troubleshooting](#troubleshooting)
- [References](#references)

## Overview

This implementation provides a complete pipeline for training deep learning tire models in MATLAB, from data loading through model export. The system consists of five main components:

1. **Data Preprocessor**: Loads, cleans, normalizes, and splits tire test data
2. **Neural Network Model**: Feedforward or LSTM architecture mapping 5 inputs to 4 outputs
3. **Training Engine**: Trains networks using GPU acceleration with early stopping
4. **Validation Engine**: Evaluates performance using RMSE, R², and TIC metrics
5. **Model Exporter**: Saves trained models for integration with vehicle simulators

## Features

- **Multiple architectures**: Feedforward (default) and LSTM networks
- **GPU acceleration**: Automatic GPU detection with CPU fallback
- **Robust preprocessing**: Handles missing data, normalization, and train/validation splitting
- **Comprehensive validation**: RMSE, R², TIC, and MAE metrics
- **Rich visualizations**: Scatter plots, residuals, time series, and operating condition analysis
- **Flexible export**: .mat files, prediction functions, and lookup tables for C++ integration
- **Hyperparameter tuning**: Grid search and random search with experiment tracking
- **Reproducibility**: Fixed random seeds for consistent results

## Requirements

### Software Dependencies

- **MATLAB R2023a or later** (required)
- **Deep Learning Toolbox** (required) - For neural network creation and training
- **Parallel Computing Toolbox** (optional but recommended) - For GPU acceleration
- **Statistics and Machine Learning Toolbox** (required) - For data preprocessing and metrics

### Hardware Recommendations

- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 16 GB minimum, 32 GB recommended for full dataset
- **GPU**: NVIDIA GPU with CUDA support (e.g., RTX 4070 Ti) for faster training
  - Training time with GPU: ~10-20 minutes on full dataset
  - Training time without GPU: ~1-2 hours on full dataset

### Data Requirements

- TTC tire test data in CSV format
- Required data files (located in `../../data/tiredata/`):
  - `DriveBrake-Hoosier18.0×6.0-10-R20.csv`
  - `Cornering-Hoosier18.0×6.0-10-R20.csv`

## Quick Start

### 1. Train a Model (Simplest Approach)

```matlab
% Navigate to directory
cd matlab/dl_tire_model

% Run complete training workflow
main_training_workflow

% This will:
% - Load TTC tire test data
% - Preprocess and normalize data
% - Create neural network
% - Train with GPU acceleration
% - Evaluate performance
% - Generate visualizations
% - Export trained model
```

### 2. Use a Trained Model

```matlab
% Load exported model
model = load('output/tire_model.mat');

% Make predictions
X_raw = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]
Y = predictTireForces(model.trained_net, X_raw, model.norm_params);

fprintf('Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', ...
        Y(1), Y(2), Y(3), Y(4));
```

### 3. Hyperparameter Tuning

```matlab
% Run hyperparameter tuning workflow
main_hyperparameter_tuning

% This will:
% - Define search space
% - Run grid or random search
% - Log all experiments
% - Compare results
% - Save best model
```

## Model Architecture

### Feedforward Network (Default)

The standard architecture uses a feedforward neural network:

```
Input Layer (5 neurons: SR, SA, Fz, IA, TSTC)
    ↓
Hidden Layer 1 (128 neurons, ReLU, 20% dropout)
    ↓
Hidden Layer 2 (64 neurons, ReLU, 20% dropout)
    ↓
Hidden Layer 3 (32 neurons, ReLU)
    ↓
Output Layer (4 neurons: Fx, Fy, Mx, Mz)
```

**Rationale:**
- 3 hidden layers provide sufficient depth for nonlinear tire behavior
- Decreasing layer sizes (128→64→32) enable hierarchical feature extraction
- ReLU activation prevents vanishing gradients
- Dropout (20%) provides regularization to prevent overfitting
- Linear output layer for unbounded regression outputs

**Create the model:**
```matlab
net = createTireModel();
```

### LSTM Network (Optional)

For temporal dependencies (e.g., tire temperature dynamics):

```
Sequence Input Layer (5 features)
    ↓
LSTM Layer 1 (64 units, sequence output, 20% dropout)
    ↓
LSTM Layer 2 (32 units, last output, 20% dropout)
    ↓
Fully Connected Layer (4 neurons)
    ↓
Regression Layer
```

**Create the model:**
```matlab
net = createLSTMTireModel();
```

## Training Procedure

### Basic Training

```matlab
% 1. Load and preprocess data
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
config.split_ratio = 0.8;  % 80% train, 20% validation
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);

% 2. Create model
net = createTireModel();

% 3. Configure training
train_config = struct();
train_config.max_epochs = 200;
train_config.batch_size = 256;
train_config.learning_rate = 0.001;
train_config.patience = 10;
train_config.random_seed = 42;

% 4. Train model
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, ...
                                               X_val, Y_val, train_config);

% 5. Evaluate model
metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);

% 6. Generate visualizations
X_features = denormalizeData(X_val, norm_params.input_mean, norm_params.input_std);
Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);
Y_pred = predict(trained_net, X_val);
Y_pred = denormalizeData(Y_pred, norm_params.output_mean, norm_params.output_std);
generateAllVisualizations(X_features, Y_actual, Y_pred, 'output/visualizations');

% 7. Export model
exportModelWithValidation(trained_net, norm_params, 'output/tire_model.mat');
```

### Training Configuration Options

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_epochs` | 200 | Maximum training epochs |
| `batch_size` | 256 | Mini-batch size |
| `learning_rate` | 0.001 | Initial learning rate (Adam optimizer) |
| `patience` | 10 | Early stopping patience (epochs) |
| `val_frequency` | 50 | Validation frequency (iterations) |
| `random_seed` | 42 | Random seed for reproducibility |

### GPU Acceleration

The training engine automatically detects and uses GPU if available:

```matlab
% Check GPU availability
[gpu_available, gpu_info] = checkGPUAvailability();
if gpu_available
    fprintf('Training will use GPU: %s\n', gpu_info.Name);
    fprintf('Available memory: %.2f GB\n', gpu_info.AvailableMemory / 1e9);
else
    warning('No GPU detected. Training will use CPU (slower).');
end
```

**GPU Memory Management:**
- If GPU runs out of memory, batch size is automatically reduced
- If still insufficient, training falls back to CPU
- Monitor GPU memory usage during training

### Training Monitoring

During training, MATLAB displays:
- Training progress plot (loss curves)
- Current epoch and iteration
- Training and validation loss
- Learning rate schedule
- Estimated time remaining

**Early Stopping:**
- Training stops if validation loss doesn't improve for `patience` epochs
- Best model (lowest validation loss) is automatically saved

## Using Trained Models

### Loading and Prediction

```matlab
% Load exported model
model = load('output/tire_model.mat');

% Single prediction
X_raw = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]
Y = predictTireForces(model.trained_net, X_raw, model.norm_params);

% Batch prediction
X_batch = [
    0.1,  5.0, 1500, -2.0, 30.0;
    0.05, 3.0, 1200, -1.0, 25.0;
    -0.1, -5.0, 1800, 1.0, 35.0
];
Y_batch = predictTireForces(model.trained_net, X_batch, model.norm_params);

% Display results
fprintf('Predictions:\n');
for i = 1:size(Y_batch, 1)
    fprintf('Sample %d: Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', ...
            i, Y_batch(i,1), Y_batch(i,2), Y_batch(i,3), Y_batch(i,4));
end
```

### Convenience Function

```matlab
% Load and predict in one call
Y = loadAndPredict('output/tire_model.mat', X_raw);
```

### Integration with Simulators

**Option 1: MATLAB-based simulators**
```matlab
% Load model once at simulation start
model = load('output/tire_model.mat');

% In simulation loop
for t = 1:num_timesteps
    % Get current tire state
    X_current = [SR(t), SA(t), Fz(t), IA(t), TSTC(t)];
    
    % Predict forces
    Y = predictTireForces(model.trained_net, X_current, model.norm_params);
    
    % Use predictions in vehicle dynamics
    Fx(t) = Y(1);
    Fy(t) = Y(2);
    Mx(t) = Y(3);
    Mz(t) = Y(4);
end
```

**Option 2: C++ integration via lookup table**
```matlab
% Generate lookup table
generateLookupTable(trained_net, norm_params, 'output/tire_lookup_table.mat');

% In C++, load the .mat file and use 5D interpolation
% See export_quick_reference.m for details
```

## Performance Metrics

The model is evaluated using multiple metrics to ensure accuracy:

### Metrics Computed

1. **RMSE (Root Mean Square Error)**: Measures average prediction error
   - Target: < 100 N for forces (Fx, Fy)
   - Target: < 10 Nm for moments (Mx, Mz)

2. **R² (Coefficient of Determination)**: Measures model fit quality
   - Target: > 0.95 for all outputs
   - R² = 1.0 indicates perfect fit

3. **TIC (Theil Inequality Coefficient)**: Normalized error metric
   - Range: [0, 1], where 0 is perfect
   - Target: < 0.1

4. **MAE (Mean Absolute Error)**: Average absolute prediction error
   - Provides intuitive error magnitude

### Example Metrics Output

```matlab
metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);

% Display results
fprintf('Performance Metrics:\n');
fprintf('Fx: RMSE=%.2f N, R²=%.4f, TIC=%.4f\n', ...
        metrics.Fx.RMSE, metrics.Fx.R2, metrics.Fx.TIC);
fprintf('Fy: RMSE=%.2f N, R²=%.4f, TIC=%.4f\n', ...
        metrics.Fy.RMSE, metrics.Fy.R2, metrics.Fy.TIC);
fprintf('Mx: RMSE=%.2f Nm, R²=%.4f, TIC=%.4f\n', ...
        metrics.Mx.RMSE, metrics.Mx.R2, metrics.Mx.TIC);
fprintf('Mz: RMSE=%.2f Nm, R²=%.4f, TIC=%.4f\n', ...
        metrics.Mz.RMSE, metrics.Mz.R2, metrics.Mz.TIC);

if metrics.meets_requirements
    fprintf('✓ Model meets all performance requirements\n');
else
    fprintf('✗ Model does not meet performance requirements\n');
end
```

## Troubleshooting

### Common Issues and Solutions

#### GPU Out of Memory

**Symptoms:** Training fails with "Out of memory" error

**Solutions:**
1. Reduce batch size: `config.batch_size = 128;` (or lower)
2. Use smaller model: Reduce hidden layer sizes
3. Close other GPU applications
4. Fall back to CPU: `config.execution_environment = 'cpu';`

#### Training Divergence (NaN Loss)

**Symptoms:** Loss becomes NaN during training

**Solutions:**
1. Reduce learning rate: `config.learning_rate = 0.0001;`
2. Check data for extreme values or NaN
3. Verify normalization is applied correctly
4. Increase batch size for more stable gradients

#### Slow Training on CPU

**Symptoms:** Training takes hours instead of minutes

**Solutions:**
1. Install Parallel Computing Toolbox for GPU support
2. Reduce dataset size for testing: Use subset of data
3. Reduce max_epochs: `config.max_epochs = 50;`
4. Use smaller model architecture

#### Poor Model Performance

**Symptoms:** RMSE > 100 N or R² < 0.95

**Solutions:**
1. Train for more epochs: `config.max_epochs = 300;`
2. Increase model capacity: Larger hidden layers
3. Reduce dropout: `config.dropout_rate = 0.1;`
4. Check data quality: Verify no missing or corrupted data
5. Try hyperparameter tuning: Run `main_hyperparameter_tuning`

#### Missing Data Files

**Symptoms:** "File not found" error when loading data

**Solutions:**
1. Verify data files exist in `../../data/tiredata/`
2. Check file names match exactly (case-sensitive)
3. Use absolute paths if relative paths fail
4. Download TTC tire test data if missing

#### Export Validation Fails

**Symptoms:** "Export validation failed" error

**Solutions:**
1. Check disk space: Ensure sufficient space for model file
2. Verify write permissions in output directory
3. Try different output path
4. Check for file system corruption

### Getting Help

If you encounter issues not covered here:

1. Check MATLAB documentation: `doc trainNetwork`
2. Review error messages carefully
3. Enable verbose output: `config.verbose = true;`
4. Test with synthetic data first: See test scripts
5. Consult the design document: `.kiro/specs/matlab-dl-tire-model/design.md`

## Data Format

### CSV File Structure

The system expects CSV files with the following columns:

| Column | Name | Unit | Description | Typical Range |
|--------|------|------|-------------|---------------|
| SR | Slip Ratio | - | Longitudinal slip ratio | [-0.2, 0.2] |
| SA | Slip Angle | deg | Lateral slip angle | [-15, 15] |
| Fz | Vertical Load | N | Normal force on tire | [500, 2500] |
| IA | Inclination Angle | deg | Camber angle | [-5, 5] |
| TSTC | Tire Temperature | °C | Tire surface temperature | [20, 80] |
| Fx | Longitudinal Force | N | Force in x-direction | [-2000, 2000] |
| Fy | Lateral Force | N | Force in y-direction | [-2000, 2000] |
| Mx | Overturning Moment | Nm | Moment about x-axis | [-100, 100] |
| Mz | Aligning Moment | Nm | Moment about z-axis | [-50, 50] |

### Example CSV Format

```csv
SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz
0.0,0.0,1000.0,0.0,25.0,0.0,0.0,0.0,0.0
0.05,2.0,1200.0,-1.0,28.0,450.2,180.5,12.3,5.8
0.1,5.0,1500.0,-2.0,30.0,820.5,520.8,25.6,15.2
-0.05,-3.0,1800.0,1.0,32.0,-380.1,-280.3,-15.4,-8.9
```

### Data Preprocessing Steps

The preprocessing pipeline automatically:

1. **Loads CSV files**: Reads multiple CSV files and combines them
2. **Validates columns**: Checks for required columns, reports missing ones
3. **Extracts features and targets**: Separates inputs (5 features) from outputs (4 targets)
4. **Handles missing data**: Removes rows with NaN or Inf values
5. **Normalizes data**: Applies z-score normalization (zero mean, unit variance)
6. **Splits data**: Partitions into training (80%) and validation (20%) sets
7. **Stores parameters**: Saves normalization parameters for inference

### Data Quality Requirements

- **No missing values**: NaN or Inf values are automatically removed
- **Reasonable ranges**: Data should be within physical limits
- **Sufficient samples**: Minimum 1,000 samples recommended, 100,000+ ideal
- **Balanced coverage**: Data should cover full operating range (SR, SA, Fz, IA, TSTC)

### Loading Custom Data

```matlab
% Load your own CSV files
data_paths = {
    'path/to/your/data1.csv', ...
    'path/to/your/data2.csv'
};

% Validate data quality
data = loadTireData(data_paths);
stats = validateTireData(data);

% Check statistics
fprintf('Total samples: %d\n', stats.total_samples);
fprintf('Clean samples: %d (%.1f%%)\n', stats.clean_samples, ...
        100 * stats.clean_samples / stats.total_samples);

% Proceed with preprocessing
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);
```

## Complete Workflows

### Workflow 1: Training from Scratch

Complete workflow for training a new model:

```matlab
%% 1. Setup
cd matlab/dl_tire_model
addpath(genpath('.'));

%% 2. Load and validate data
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};

fprintf('Loading data...\n');
data = loadTireData(data_paths);
stats = validateTireData(data);

%% 3. Preprocess data
fprintf('Preprocessing data...\n');
config.split_ratio = 0.8;
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);

fprintf('Training samples: %d\n', size(X_train, 1));
fprintf('Validation samples: %d\n', size(X_val, 1));

%% 4. Create model
fprintf('Creating neural network...\n');
net = createTireModel();

%% 5. Check GPU availability
[gpu_available, gpu_info] = checkGPUAvailability();

%% 6. Configure training
train_config = struct();
train_config.max_epochs = 200;
train_config.batch_size = 256;
train_config.learning_rate = 0.001;
train_config.patience = 10;
train_config.val_frequency = 50;
train_config.random_seed = 42;

%% 7. Train model
fprintf('Training model...\n');
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, ...
                                               X_val, Y_val, train_config);

fprintf('Training completed in %.2f minutes\n', training_info.TrainingTime / 60);

%% 8. Evaluate model
fprintf('Evaluating model...\n');
metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);

fprintf('\nPerformance Metrics:\n');
fprintf('Fx: RMSE=%.2f N, R²=%.4f\n', metrics.Fx.RMSE, metrics.Fx.R2);
fprintf('Fy: RMSE=%.2f N, R²=%.4f\n', metrics.Fy.RMSE, metrics.Fy.R2);
fprintf('Mx: RMSE=%.2f Nm, R²=%.4f\n', metrics.Mx.RMSE, metrics.Mx.R2);
fprintf('Mz: RMSE=%.2f Nm, R²=%.4f\n', metrics.Mz.RMSE, metrics.Mz.R2);

%% 9. Generate visualizations
fprintf('Generating visualizations...\n');
X_features = denormalizeData(X_val, norm_params.input_mean, norm_params.input_std);
Y_actual = denormalizeData(Y_val, norm_params.output_mean, norm_params.output_std);
Y_pred = predict(trained_net, X_val);
Y_pred = denormalizeData(Y_pred, norm_params.output_mean, norm_params.output_std);

output_dir = 'output/visualizations';
generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);

%% 10. Export model
fprintf('Exporting model...\n');
output_path = 'output/tire_model.mat';
exportModelWithValidation(trained_net, norm_params, output_path);

fprintf('\n✓ Training workflow complete!\n');
fprintf('Model saved to: %s\n', output_path);
fprintf('Visualizations saved to: %s\n', output_dir);
```

### Workflow 2: Using a Trained Model

Complete workflow for using an existing model:

```matlab
%% 1. Setup
cd matlab/dl_tire_model
addpath(genpath('.'));

%% 2. Load trained model
fprintf('Loading trained model...\n');
model = load('output/tire_model.mat');

%% 3. Prepare input data
% Example: Single prediction
X_single = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]

% Example: Batch predictions
X_batch = [
    0.0,   0.0, 1000,  0.0, 25.0;
    0.05,  2.0, 1200, -1.0, 28.0;
    0.1,   5.0, 1500, -2.0, 30.0;
    -0.05, -3.0, 1800, 1.0, 32.0
];

%% 4. Make predictions
fprintf('Making predictions...\n');

% Single prediction
Y_single = predictTireForces(model.trained_net, X_single, model.norm_params);
fprintf('\nSingle Prediction:\n');
fprintf('Input: SR=%.2f, SA=%.1f°, Fz=%.0f N, IA=%.1f°, TSTC=%.1f°C\n', X_single);
fprintf('Output: Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', Y_single);

% Batch predictions
Y_batch = predictTireForces(model.trained_net, X_batch, model.norm_params);
fprintf('\nBatch Predictions:\n');
for i = 1:size(X_batch, 1)
    fprintf('Sample %d: Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', ...
            i, Y_batch(i,1), Y_batch(i,2), Y_batch(i,3), Y_batch(i,4));
end

%% 5. Visualize predictions (optional)
figure;
subplot(2,2,1); bar(Y_batch(:,1)); title('Fx'); ylabel('Force (N)');
subplot(2,2,2); bar(Y_batch(:,2)); title('Fy'); ylabel('Force (N)');
subplot(2,2,3); bar(Y_batch(:,3)); title('Mx'); ylabel('Moment (Nm)');
subplot(2,2,4); bar(Y_batch(:,4)); title('Mz'); ylabel('Moment (Nm)');

fprintf('\n✓ Prediction workflow complete!\n');
```

### Workflow 3: Hyperparameter Tuning

Complete workflow for optimizing model hyperparameters:

```matlab
%% 1. Setup
cd matlab/dl_tire_model
addpath(genpath('.'));

%% 2. Load and preprocess data
fprintf('Loading and preprocessing data...\n');
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, struct('split_ratio', 0.8));

%% 3. Define search space
fprintf('Defining hyperparameter search space...\n');
search_space = struct();
search_space.learning_rate = {0.001, 0.0005, 0.0001};
search_space.batch_size = {128, 256, 512};
search_space.max_epochs = {100, 150, 200};
search_space.dropout_rate = {0.1, 0.2, 0.3};

%% 4. Run hyperparameter search
fprintf('Running hyperparameter search...\n');
fprintf('This will test %d configurations\n', ...
        length(search_space.learning_rate) * length(search_space.batch_size) * ...
        length(search_space.max_epochs) * length(search_space.dropout_rate));

% Option A: Grid search (exhaustive)
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'grid');

% Option B: Random search (faster, 20 trials)
% [best_config, all_results] = runHyperparameterSearch(...
%     X_train, Y_train, X_val, Y_val, norm_params, ...
%     search_space, 'random', 20);

%% 5. Compare experiments
fprintf('Comparing experiments...\n');
comparison_table = compareExperiments('all', 'experiments/logs', 'experiments/comparison.txt');

%% 6. Load and test best model
fprintf('Loading best model...\n');
best_exp_id = all_results{1}.experiment_id;
[best_net, best_params] = loadExperimentModel(best_exp_id);

% Test predictions
X_test = [0.1, 5.0, 1500, -2.0, 30.0];
Y_test = predictTireForces(best_net, X_test, best_params);

fprintf('\nBest Model Configuration:\n');
fprintf('Learning rate: %.5f\n', best_config.learning_rate);
fprintf('Batch size: %d\n', best_config.batch_size);
fprintf('Max epochs: %d\n', best_config.max_epochs);
fprintf('Dropout rate: %.2f\n', best_config.dropout_rate);

fprintf('\n✓ Hyperparameter tuning complete!\n');
fprintf('Best model ID: %s\n', best_exp_id);
fprintf('Comparison report: experiments/comparison.txt\n');
```

### Workflow 4: Export for C++ Integration

Complete workflow for exporting model for C++ simulators:

```matlab
%% 1. Setup
cd matlab/dl_tire_model
addpath(genpath('.'));

%% 2. Load trained model
fprintf('Loading trained model...\n');
model = load('output/tire_model.mat');

%% 3. Generate lookup table
fprintf('Generating lookup table for C++ integration...\n');

% Define grid resolution (balance between accuracy and file size)
grid_config = struct();
grid_config.SR_range = [-0.2, 0.2, 50];      % 50 points
grid_config.SA_range = [-15, 15, 60];        % 60 points
grid_config.Fz_range = [500, 2500, 40];      % 40 points
grid_config.IA_range = [-5, 5, 20];          % 20 points
grid_config.TSTC_range = [20, 80, 20];       % 20 points

output_path = 'output/tire_lookup_table.mat';
generateLookupTable(model.trained_net, model.norm_params, output_path, grid_config);

fprintf('Lookup table saved to: %s\n', output_path);

%% 4. Test lookup table interpolation
fprintf('Testing lookup table interpolation...\n');
lut_data = load(output_path);
lut = lut_data.lookup_table;

% Test point
SR = 0.05; SA = 5.0; Fz = 1500; IA = -2.0; TSTC = 30.0;

% Interpolate from lookup table
Fx_lut = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
                 lut.IA_array, lut.TSTC_array, lut.Fx_grid, ...
                 SR, SA, Fz, IA, TSTC, 'linear');
Fy_lut = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
                 lut.IA_array, lut.TSTC_array, lut.Fy_grid, ...
                 SR, SA, Fz, IA, TSTC, 'linear');

% Compare with direct prediction
X_test = [SR, SA, Fz, IA, TSTC];
Y_direct = predictTireForces(model.trained_net, X_test, model.norm_params);

fprintf('\nComparison:\n');
fprintf('Direct prediction: Fx=%.1f N, Fy=%.1f N\n', Y_direct(1), Y_direct(2));
fprintf('Lookup table:      Fx=%.1f N, Fy=%.1f N\n', Fx_lut, Fy_lut);
fprintf('Difference:        Fx=%.1f N, Fy=%.1f N\n', ...
        abs(Y_direct(1) - Fx_lut), abs(Y_direct(2) - Fy_lut));

fprintf('\n✓ Export workflow complete!\n');
fprintf('Use the lookup table in C++ with 5D interpolation\n');
```

## Directory Structure

```
matlab/dl_tire_model/
├── README.md                          # This file
├── loadTireData.m                     # Load TTC tire test data from CSV files
├── validateTireData.m                 # Validate data and report statistics
├── extractFeatures.m                  # Extract input features from data
├── extractTargets.m                   # Extract output targets from data
├── removeMissingData.m                # Remove rows with missing/infinite values
├── normalizeData.m                    # Normalize data to zero mean, unit variance
├── denormalizeData.m                  # Denormalize data back to original scale
├── splitData.m                        # Split data into train/validation sets
├── preprocessData.m                   # Complete preprocessing pipeline
├── createTireModel.m                  # Create feedforward neural network
├── createLSTMTireModel.m              # Create LSTM neural network
├── createModelFromConfig.m            # Create model based on configuration
├── checkGPUAvailability.m             # Check GPU availability and display info
├── createTrainingOptions.m            # Configure training options
├── trainTireModel.m                   # Train neural network with GPU support
├── generateScatterPlots.m             # Generate scatter plots (predicted vs actual)
├── generateResidualPlots.m            # Generate residual error plots
├── generateTimeSeriesPlots.m          # Generate time series comparison plots
├── generateOperatingConditionPlots.m  # Generate performance by condition plots
├── generateAllVisualizations.m        # Generate all visualization types
├── exportModel.m                      # Export trained model to .mat file
├── predictTireForces.m                # Prediction function for exported model
├── loadAndPredict.m                   # Convenience function for loading and predicting
├── exportModelWithValidation.m        # Export model with validation check
├── generateLookupTable.m              # Generate lookup table for C++ integration
├── export_quick_reference.m           # Quick reference examples for export functions
├── test_data_loading.m                # Test script for data loading
├── test_feature_target_extraction.m   # Test script for feature/target extraction
├── test_preprocessing_checkpoint.m    # Comprehensive preprocessing test
├── test_model_creation.m              # Test script for model creation
├── test_training_engine.m             # Test script for training engine
├── test_visualization_functions.m     # Test script for visualization functions
└── test_model_export.m                # Test script for model export functionality
```

## Data Loading

### loadTireData.m

Loads TTC tire test data from CSV files and combines them into a single table.

**Usage:**
```matlab
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
data = loadTireData(data_paths);
```

**Required Columns:**
- SR: Slip Ratio
- SA: Slip Angle (deg)
- Fz: Vertical Load (N)
- IA: Inclination Angle (deg)
- TSTC: Tire Surface Temperature (°C)
- Fx: Longitudinal Force (N)
- Fy: Lateral Force (N)
- Mx: Overturning Moment (Nm)
- Mz: Aligning Moment (Nm)

**Error Handling:**
- Throws `loadTireData:FileNotFound` if file doesn't exist
- Throws `loadTireData:ReadError` if CSV file is corrupted
- Throws `loadTireData:MissingColumns` if required columns are missing

### validateTireData.m

Validates tire test data and reports statistics including missing values, data ranges, and summary information.

**Usage:**
```matlab
data = loadTireData(data_paths);
stats = validateTireData(data);
```

**Returns:**
- `stats.total_samples`: Total number of samples
- `stats.clean_samples`: Number of samples without missing values
- `stats.rows_with_missing`: Number of rows with NaN or Inf
- `stats.missing_by_column`: Missing value count per column
- `stats.ranges`: Min, max, mean, std for each column

## Data Preprocessing

### Complete Preprocessing Pipeline

The `preprocessData.m` function provides a complete pipeline that:
1. Loads data from CSV files
2. Extracts features and targets
3. Removes missing/infinite values
4. Normalizes data to zero mean and unit variance
5. Splits data into training and validation sets

**Usage:**
```matlab
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', ...
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};

config.split_ratio = 0.8;  % 80% training, 20% validation

[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);
```

**Returns:**
- `X_train`: Training features (N×5 matrix, normalized)
- `Y_train`: Training targets (N×4 matrix, normalized)
- `X_val`: Validation features (M×5 matrix, normalized)
- `Y_val`: Validation targets (M×4 matrix, normalized)
- `norm_params`: Structure containing normalization parameters
  - `input_mean`: Mean of input features (5×1)
  - `input_std`: Std dev of input features (5×1)
  - `output_mean`: Mean of output targets (4×1)
  - `output_std`: Std dev of output targets (4×1)

### Individual Preprocessing Functions

**extractFeatures.m** - Extract input features (SR, SA, Fz, IA, TSTC)
```matlab
features = extractFeatures(data);
```

**extractTargets.m** - Extract output targets (Fx, Fy, Mx, Mz)
```matlab
targets = extractTargets(data);
```

**removeMissingData.m** - Remove rows with NaN or Inf values
```matlab
[X_clean, Y_clean, stats] = removeMissingData(X, Y);
```

**normalizeData.m** - Normalize data to zero mean, unit variance
```matlab
[X_norm, mu, sigma] = normalizeData(X);
```

**denormalizeData.m** - Denormalize data back to original scale
```matlab
X_original = denormalizeData(X_norm, mu, sigma);
```

**splitData.m** - Split data into training and validation sets
```matlab
[X_train, Y_train, X_val, Y_val] = splitData(X, Y, 0.8);
```

## Neural Network Architecture

### Feedforward Network

The `createTireModel.m` function creates a feedforward neural network with the following architecture:

- **Input layer**: 5 neurons (SR, SA, Fz, IA, TSTC)
- **Hidden layer 1**: 128 neurons, ReLU activation, 0.2 dropout
- **Hidden layer 2**: 64 neurons, ReLU activation, 0.2 dropout
- **Hidden layer 3**: 32 neurons, ReLU activation
- **Output layer**: 4 neurons (Fx, Fy, Mx, Mz), linear activation

**Usage:**
```matlab
net = createTireModel();
```

### LSTM Network (Optional)

The `createLSTMTireModel.m` function creates an LSTM-based network for temporal dependencies:

- **Sequence input layer**: 5 features
- **LSTM layer 1**: 64 units, sequence output, 0.2 dropout
- **LSTM layer 2**: 32 units, last output, 0.2 dropout
- **Fully connected layer**: 4 neurons
- **Regression layer**

**Usage:**
```matlab
net = createLSTMTireModel();
```

### Configuration-Based Model Creation

The `createModelFromConfig.m` function allows you to create either architecture based on configuration:

**Usage:**
```matlab
% Create feedforward model
config.model_type = 'feedforward';
net = createModelFromConfig(config);

% Create LSTM model
config.model_type = 'lstm';
net = createModelFromConfig(config);

% Default (feedforward)
net = createModelFromConfig(struct());
```

## Testing

### Data Loading Tests

Run the data loading test script:

```matlab
cd matlab/dl_tire_model
test_data_loading
```

This will:
1. Load DriveBrake data
2. Load Cornering data
3. Load combined data
4. Test error handling
5. Verify data structure
6. Display sample data

### Preprocessing Tests

Run the comprehensive preprocessing checkpoint test:

```matlab
cd matlab/dl_tire_model
test_preprocessing_checkpoint
```

This will verify:
1. Data loading from actual TTC files
2. Feature and target extraction
3. Missing data removal
4. Normalization (zero mean, unit variance)
5. Data splitting (80/20 train/validation)
6. Complete preprocessing pipeline
7. Normalization round-trip accuracy
8. Reasonable output ranges

### Model Creation Tests

Run the model creation test script:

```matlab
cd matlab/dl_tire_model
test_model_creation
```

This will verify:
1. Feedforward model creation
2. LSTM model creation
3. Configuration-based model creation
4. Input/output layer sizes
5. Dropout layer presence
6. Error handling for invalid configurations

### Training Engine Tests

Run the training engine test script:

```matlab
cd matlab/dl_tire_model
test_training_engine
```

This will verify:
1. GPU detection and availability
2. Training options configuration
3. Model training with synthetic data
4. Error handling (GPU OOM, training divergence)
5. Training info structure completeness
6. Reproducibility with random seeds

**Note:** This test uses a small synthetic dataset and only 3 epochs for quick verification. For full training on TTC data, use the complete workflow.

## Requirements

- MATLAB R2023a or later
- Deep Learning Toolbox (for neural network creation and training)
- Data files in `data/tiredata/` directory

## Training Engine

### GPU Detection

The `checkGPUAvailability.m` function checks for GPU availability and provides detailed information:

**Usage:**
```matlab
[gpu_available, gpu_info] = checkGPUAvailability();
if gpu_available
    fprintf('Using GPU: %s\n', gpu_info.Name);
end
```

**Returns:**
- `gpu_available`: Boolean indicating if GPU is available
- `gpu_info`: Structure with GPU details (Name, Index, ComputeCapability, TotalMemory, AvailableMemory)

### Training Options Configuration

The `createTrainingOptions.m` function configures training parameters:

**Usage:**
```matlab
config = struct();
config.max_epochs = 200;
config.batch_size = 256;
config.learning_rate = 0.001;
config.patience = 10;
config.val_frequency = 50;

options = createTrainingOptions(config, X_val, Y_val);
```

**Configuration Parameters:**
- `max_epochs`: Maximum training epochs (default: 200)
- `batch_size`: Mini-batch size (default: 256)
- `learning_rate`: Initial learning rate (default: 0.001)
- `patience`: Early stopping patience in epochs (default: 10)
- `val_frequency`: Validation frequency in iterations (default: 50)

**Features:**
- Adam optimizer
- Piecewise learning rate schedule (drop factor 0.5 every 20 epochs)
- Early stopping based on validation loss
- Automatic GPU detection ('auto' execution environment)

### Model Training

The `trainTireModel.m` function trains the neural network with comprehensive error handling:

**Usage:**
```matlab
net = createTireModel();
config = struct('max_epochs', 200, 'batch_size', 256, 'random_seed', 42);
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
```

**Returns:**
- `trained_net`: Trained neural network
- `training_info`: Structure with training details
  - `TrainingLoss`: Training loss history
  - `ValidationLoss`: Validation loss history
  - `FinalValidationLoss`: Final validation loss
  - `TrainingTime`: Total training time in seconds

**Error Handling:**
- GPU out-of-memory: Automatically reduces batch size or falls back to CPU
- Training divergence: Detects NaN loss and provides suggestions
- Reproducibility: Sets random seed for consistent results

**Example:**
```matlab
% Complete training workflow
data_paths = {'../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'};
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, struct('split_ratio', 0.8));

net = createTireModel();
config = struct('max_epochs', 200, 'batch_size', 256, 'random_seed', 42);
[trained_net, info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);

fprintf('Training completed in %.2f minutes\n', info.TrainingTime / 60);
```

## Visualization Functions

### Complete Visualization Suite

The `generateAllVisualizations.m` function provides a comprehensive visualization suite that generates all plot types:

**Usage:**
```matlab
% After model evaluation
X_features = [...];  % N×5 input features
Y_actual = [...];    % N×4 actual values (denormalized)
Y_pred = [...];      # N×4 predicted values (denormalized)

output_dir = 'output/visualizations';
generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir);
```

This generates 20 plot files covering all visualization types.

### Individual Visualization Functions

**generateScatterPlots.m** - Predicted vs actual scatter plots
```matlab
generateScatterPlots(Y_actual, Y_pred, output_dir);
```
- Creates scatter plots for each output (Fx, Fy, Mx, Mz)
- Includes perfect prediction reference line (y = x)
- Displays R² value on each plot
- Saves 4 PNG files: `scatter_Fx.png`, `scatter_Fy.png`, `scatter_Mx.png`, `scatter_Mz.png`

**generateResidualPlots.m** - Residual error plots
```matlab
generateResidualPlots(Y_actual, Y_pred, output_dir);
```
- Creates residual plots showing prediction errors
- Includes zero error reference line
- Displays mean, std, and RMSE statistics
- Saves 4 PNG files: `residual_Fx.png`, `residual_Fy.png`, `residual_Mx.png`, `residual_Mz.png`

**generateTimeSeriesPlots.m** - Time series comparison plots
```matlab
generateTimeSeriesPlots(Y_actual, Y_pred, output_dir, 'MaxSamples', 1000);
```
- Creates time series plots for sequential data
- Plots both actual and predicted values
- Includes combined plot with all outputs
- Automatic subsampling for large datasets
- Saves 5 PNG files: individual plots + `timeseries_combined.png`

**generateOperatingConditionPlots.m** - Performance by operating condition
```matlab
generateOperatingConditionPlots(X_features, Y_actual, Y_pred, output_dir, 'NumBins', 5);
```
- Analyzes performance across different operating conditions (SR, SA, Fz)
- Generates RMSE and R² bar plots by condition
- Includes target threshold line (R² = 0.95)
- Creates summary heatmap
- Saves 7 PNG files: RMSE/R² plots for SR, SA, Fz + heatmap

### Visualization Testing

Run the visualization test script:

```matlab
cd matlab/dl_tire_model
test_visualization_functions
```

This will:
1. Generate synthetic test data
2. Test all visualization functions
3. Verify file creation
4. Test error handling
5. Test custom parameters
6. Create test plots in `test_visualizations/` directory

## Implementation Status

### Completed
- ✅ Data loading from CSV files
- ✅ Data validation and statistics
- ✅ Feature and target extraction
- ✅ Missing data handling
- ✅ Data normalization and denormalization
- ✅ Train/validation data splitting
- ✅ Complete preprocessing pipeline
- ✅ Feedforward neural network architecture
- ✅ LSTM neural network architecture (optional)
- ✅ Configuration-based model creation
- ✅ GPU detection and availability checking
- ✅ Training options configuration
- ✅ Model training with GPU support
- ✅ Error handling (GPU OOM, training divergence)
- ✅ Reproducibility with random seeds
- ✅ Scatter plot generation (predicted vs actual)
- ✅ Residual plot generation
- ✅ Time series plot generation
- ✅ Performance by operating condition plots
- ✅ Complete visualization suite
- ✅ Model export to .mat file
- ✅ Prediction functions for exported models
- ✅ Export validation
- ✅ Lookup table generation for C++ integration
- ✅ Batch prediction support

## Model Export and Integration

### Basic Model Export

The `exportModel.m` function saves the trained model and normalization parameters to a .mat file:

**Usage:**
```matlab
output_path = 'output/tire_model.mat';
exportModel(trained_net, norm_params, output_path);
```

**Saved Contents:**
- `trained_net`: Trained neural network
- `norm_params`: Normalization parameters (input_mean, input_std, output_mean, output_std)

### Prediction Functions

**predictTireForces.m** - Make predictions with raw inputs

This function handles all normalization/denormalization internally:

```matlab
% Load exported model
model = load('output/tire_model.mat');

% Single prediction
X_raw = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]
Y = predictTireForces(model.trained_net, X_raw, model.norm_params);
fprintf('Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', Y(1), Y(2), Y(3), Y(4));

% Batch prediction
X_batch = [
    0.1,  5.0, 1500, -2.0, 30.0;
    0.05, 3.0, 1200, -1.0, 25.0;
    -0.1, -5.0, 1800, 1.0, 35.0
];
Y_batch = predictTireForces(model.trained_net, X_batch, model.norm_params);
```

**loadAndPredict.m** - Convenience function for quick predictions

```matlab
% Load and predict in one call
X_raw = [0.1, 5.0, 1500, -2.0, 30.0];
Y = loadAndPredict('output/tire_model.mat', X_raw);
```

### Export with Validation

The `exportModelWithValidation.m` function exports the model and automatically validates that predictions match:

**Usage:**
```matlab
% Automatic validation with random test data
output_path = 'output/tire_model_validated.mat';
exportModelWithValidation(trained_net, norm_params, output_path);

% Validation with specific test data
X_test = [0.1, 5.0, 1500, -2.0, 30.0];
exportModelWithValidation(trained_net, norm_params, output_path, X_test);
```

**Validation Process:**
1. Makes predictions with original model
2. Exports model to file
3. Reloads exported model
4. Makes predictions with reloaded model
5. Compares predictions (tolerance: 1e-10)
6. Deletes file if validation fails

### Lookup Table Generation

The `generateLookupTable.m` function creates a multi-dimensional lookup table for C++ integration:

**Usage:**
```matlab
% Default grid (coarse, fast)
output_path = 'output/tire_lookup_table.mat';
generateLookupTable(trained_net, norm_params, output_path);

% Custom grid resolution
grid_config = struct();
grid_config.SR_range = [-0.2, 0.2, 50];      % 50 points
grid_config.SA_range = [-15, 15, 60];        % 60 points
grid_config.Fz_range = [500, 2500, 40];      % 40 points
grid_config.IA_range = [-5, 5, 20];          # 20 points
grid_config.TSTC_range = [20, 80, 20];       % 20 points
generateLookupTable(trained_net, norm_params, output_path, grid_config);
```

**Using the Lookup Table:**
```matlab
% Load lookup table
data = load('output/tire_lookup_table.mat');
lut = data.lookup_table;

% Interpolate at specific point
SR = 0.05; SA = 5.0; Fz = 1500; IA = -2.0; TSTC = 30.0;

Fx = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Fx_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');
Fy = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Fy_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');
```

**Lookup Table Structure:**
- `SR_array`, `SA_array`, `Fz_array`, `IA_array`, `TSTC_array`: 1D arrays for each dimension
- `Fx_grid`, `Fy_grid`, `Mx_grid`, `Mz_grid`: 5D grids of predictions
- `grid_dimensions`: Size of each dimension
- `input_names`, `output_names`: Variable names
- `input_units`, `output_units`: Physical units

### Export Testing

Run the export functionality test script:

```matlab
cd matlab/dl_tire_model
test_model_export
```

This will verify:
1. Basic model export to .mat file
2. Prediction function correctness
3. Load and predict convenience function
4. Export with validation
5. Lookup table generation
6. Batch vs individual prediction consistency

### Quick Reference

See `export_quick_reference.m` for copy-paste examples of:
- Basic model export
- Export with validation
- Loading and making predictions
- Generating lookup tables
- Using lookup tables for interpolation
- Complete workflow from training to export

## Hyperparameter Tuning

### Overview

The hyperparameter tuning framework provides tools for experimenting with different model configurations, logging results, comparing experiments, and managing trained models. It supports both grid search (exhaustive) and random search (sampling-based) approaches.

### Configuration Structure

The `createHyperparameterConfig.m` function creates structured hyperparameter configurations:

**Usage:**
```matlab
% Default configuration
config = createHyperparameterConfig();

% Custom configuration
config = createHyperparameterConfig(...
    'learning_rate', 0.0001, ...
    'batch_size', 128, ...
    'max_epochs', 150, ...
    'patience', 15, ...
    'model_type', 'feedforward', ...
    'dropout_rate', 0.3, ...
    'split_ratio', 0.8);
```

**Supported Parameters:**
- Training: `learning_rate`, `batch_size`, `max_epochs`, `patience`, `val_frequency`, `random_seed`
- Model: `model_type` ('feedforward' or 'lstm'), `hidden_sizes`, `dropout_rate`
- Data: `split_ratio`

### Experiment Logging

**logExperiment.m** - Log experiment details

Saves experiment configuration, training information, and performance metrics:

```matlab
% After training and evaluation
experiment_id = logExperiment(config, metrics, training_info);

% Custom log directory
experiment_id = logExperiment(config, metrics, training_info, 'my_experiments/logs');
```

**Saved Files:**
- `exp_YYYYMMDD_HHMMSS.mat`: Machine-readable log
- `exp_YYYYMMDD_HHMMSS.txt`: Human-readable log

**loadExperimentLog.m** - Load saved experiment logs

```matlab
exp_log = loadExperimentLog('exp_20260124_143022');
```

### Hyperparameter Search

**runHyperparameterSearch.m** - Automated hyperparameter optimization

Performs grid search or random search over a defined search space:

**Grid Search Example:**
```matlab
% Define search space
search_space.learning_rate = {0.001, 0.0005, 0.0001};
search_space.batch_size = {128, 256, 512};
search_space.max_epochs = {100, 150, 200};

% Run grid search (tests all 27 combinations)
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'grid');
```

**Random Search Example:**
```matlab
% Define larger search space
search_space.learning_rate = {0.001, 0.0005, 0.0001, 0.00005};
search_space.batch_size = {64, 128, 256, 512};
search_space.max_epochs = {50, 100, 150, 200};
search_space.patience = {5, 10, 15, 20};
search_space.dropout_rate = {0.1, 0.2, 0.3, 0.4};

% Run random search with 20 trials
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'random', 20);
```

**Features:**
- Automatic model training and evaluation for each configuration
- Automatic experiment logging
- Automatic model saving with unique identifiers
- Error handling for failed trials
- Progress reporting
- Best configuration tracking

### Experiment Comparison

**compareExperiments.m** - Compare multiple experiments

Generates comprehensive comparison tables:

```matlab
% Compare specific experiments
experiment_ids = {'exp_20260124_143022', 'exp_20260124_143145'};
comparison_table = compareExperiments(experiment_ids);

% Compare all experiments in directory
comparison_table = compareExperiments('all');

% Compare and save report
comparison_table = compareExperiments('all', 'experiments/logs', 'comparison.txt');
```

**Comparison Table Includes:**
- Hyperparameter configurations
- Training time and validation loss
- Performance metrics (RMSE, R² for Fx, Fy)
- Requirements compliance status
- Sorted by validation loss (best first)

### Model Management

**saveExperimentModel.m** - Save model with unique identifier

```matlab
model_path = saveExperimentModel(trained_net, norm_params, experiment_id);

% Custom directory
model_path = saveExperimentModel(trained_net, norm_params, experiment_id, 'my_models');
```

**loadExperimentModel.m** - Load saved experiment model

```matlab
[trained_net, norm_params] = loadExperimentModel('exp_20260124_143022');

% Custom directory
[trained_net, norm_params] = loadExperimentModel('exp_20260124_143022', 'my_models');
```

### Complete Workflow Example

```matlab
% Step 1: Load and preprocess data
data_paths = {'../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'};
data = loadTireData(data_paths);
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data);

% Step 2: Define search space
search_space.learning_rate = {0.001, 0.0005};
search_space.batch_size = {128, 256};
search_space.max_epochs = {100, 150};

% Step 3: Run hyperparameter search
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'grid');

% Step 4: Compare all experiments
comparison_table = compareExperiments('all', 'experiments/logs', 'comparison.txt');

% Step 5: Load and use best model
best_exp_id = all_results{1}.experiment_id;
[best_net, best_params] = loadExperimentModel(best_exp_id);

% Step 6: Make predictions with best model
predictions = predictTireForces(best_net, new_data, best_params);
```

### Directory Structure

Hyperparameter tuning creates the following structure:

```
experiments/
├── logs/                           # Experiment logs
│   ├── exp_20260124_143022.mat    # Machine-readable log
│   ├── exp_20260124_143022.txt    # Human-readable log
│   └── ...
├── models/                         # Trained models
│   ├── exp_20260124_143022_model.mat      # Model + normalization params
│   ├── exp_20260124_143022_predict.mat    # Prediction function
│   └── ...
└── comparison_report.txt           # Experiment comparison results
```

### Example Scripts

**example_hyperparameter_tuning.m** - Interactive demonstration

Comprehensive example showing:
- Data loading and preprocessing
- Search space definition
- Grid search, random search, and single experiments
- Experiment comparison
- Best model loading and testing

**hyperparameter_tuning_quick_reference.m** - Quick reference guide

Copy-paste examples for:
- Creating configurations
- Logging experiments
- Running searches
- Comparing experiments
- Managing models
- Complete workflows
- Tips and best practices

### Tips and Best Practices

1. **Start with random search** to explore the space quickly
2. **Use grid search** for fine-tuning around promising regions
3. **Monitor training time** and adjust max_epochs accordingly
4. **Check GPU memory usage** if training fails
5. **Always compare experiments** before selecting final model
6. **Save comparison reports** for documentation
7. **Use meaningful experiment IDs** for tracking

### Next Steps

The following components will be added in subsequent tasks:
- Complete workflow scripts
- Documentation and examples

## References

- Spec: `.kiro/specs/matlab-dl-tire-model/`
- Design Document: `.kiro/specs/matlab-dl-tire-model/design.md`
- Requirements: `.kiro/specs/matlab-dl-tire-model/requirements.md`
