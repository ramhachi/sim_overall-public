% EXPORT_QUICK_REFERENCE Quick reference for model export functionality
%
% This script provides examples of how to use the model export functions.
% Copy and modify these examples for your own use.

%% Example 1: Basic model export
% Export a trained model to a .mat file

% Assuming you have trained_net and norm_params from training
output_path = 'output/tire_model.mat';
exportModel(trained_net, norm_params, output_path);

%% Example 2: Export with validation
% Export and automatically validate that predictions match

output_path = 'output/tire_model_validated.mat';
exportModelWithValidation(trained_net, norm_params, output_path);

% Or provide specific test data for validation
X_test = [0.1, 5.0, 1500, -2.0, 30.0];  % [SR, SA, Fz, IA, TSTC]
exportModelWithValidation(trained_net, norm_params, output_path, X_test);

%% Example 3: Load and make predictions
% Load an exported model and make predictions

% Method 1: Load manually
model = load('output/tire_model.mat');
X_raw = [0.1, 5.0, 1500, -2.0, 30.0];  % Single input
Y = predictTireForces(model.trained_net, X_raw, model.norm_params);
fprintf('Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', Y(1), Y(2), Y(3), Y(4));

% Method 2: Use convenience function
Y = loadAndPredict('output/tire_model.mat', X_raw);

% Batch prediction
X_batch = [
    0.1,  5.0, 1500, -2.0, 30.0;
    0.05, 3.0, 1200, -1.0, 25.0;
    -0.1, -5.0, 1800, 1.0, 35.0
];
Y_batch = loadAndPredict('output/tire_model.mat', X_batch);

%% Example 4: Generate lookup table for C++ integration
% Create a lookup table for fast interpolation in C++

output_path = 'output/tire_lookup_table.mat';

% Use default grid (coarse)
generateLookupTable(trained_net, norm_params, output_path);

% Or specify custom grid resolution
grid_config = struct();
grid_config.SR_range = [-0.2, 0.2, 50];      % 50 points from -0.2 to 0.2
grid_config.SA_range = [-15, 15, 60];        % 60 points from -15 to 15 deg
grid_config.Fz_range = [500, 2500, 40];      % 40 points from 500 to 2500 N
grid_config.IA_range = [-5, 5, 20];          % 20 points from -5 to 5 deg
grid_config.TSTC_range = [20, 80, 20];       % 20 points from 20 to 80 °C
generateLookupTable(trained_net, norm_params, output_path, grid_config);

%% Example 5: Use lookup table for interpolation
% Load and interpolate from lookup table

data = load('output/tire_lookup_table.mat');
lut = data.lookup_table;

% Single point interpolation
SR = 0.05;
SA = 5.0;
Fz = 1500;
IA = -2.0;
TSTC = 30.0;

Fx = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Fx_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');
Fy = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Fy_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');
Mx = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Mx_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');
Mz = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...
             lut.IA_array, lut.TSTC_array, lut.Mz_grid, ...
             SR, SA, Fz, IA, TSTC, 'linear');

fprintf('Interpolated: Fx=%.1f N, Fy=%.1f N, Mx=%.1f Nm, Mz=%.1f Nm\n', ...
        Fx, Fy, Mx, Mz);

%% Example 6: Complete workflow from training to export
% Full example showing training and export

% 1. Load and preprocess data
data_paths = {
    'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv',
    'data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
config = struct('split_ratio', 0.8);
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config);

% 2. Create model
net = createTireModel();

% 3. Train model
train_config = struct();
train_config.max_epochs = 100;
train_config.batch_size = 256;
train_config.learning_rate = 0.001;
train_config.patience = 10;
train_config.val_frequency = 50;
train_config.random_seed = 42;
[trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, train_config);

% 4. Evaluate model
metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);

% 5. Export model with validation
output_path = 'output/final_tire_model.mat';
exportModelWithValidation(trained_net, norm_params, output_path);

% 6. Generate lookup table
lookup_path = 'output/final_tire_lookup_table.mat';
generateLookupTable(trained_net, norm_params, lookup_path);

fprintf('Model training and export complete!\n');
