% HYPERPARAMETER TUNING QUICK REFERENCE
%
% This file provides quick examples of how to use the hyperparameter tuning
% functionality for the MATLAB deep learning tire model.
%
% Requirements: 7.1, 7.2, 7.3, 7.4, 7.5

%% 1. CREATE HYPERPARAMETER CONFIGURATION
% Create default configuration
config = createHyperparameterConfig();

% Create custom configuration
config = createHyperparameterConfig(...
    'learning_rate', 0.0001, ...
    'batch_size', 128, ...
    'max_epochs', 150, ...
    'patience', 15, ...
    'model_type', 'feedforward', ...
    'dropout_rate', 0.3);

%% 2. LOG EXPERIMENT
% After training and evaluating a model
experiment_id = logExperiment(config, metrics, training_info);

% Log to custom directory
experiment_id = logExperiment(config, metrics, training_info, 'my_experiments/logs');

%% 3. LOAD EXPERIMENT LOG
% Load specific experiment
exp_log = loadExperimentLog('exp_20260124_143022');

% Load from custom directory
exp_log = loadExperimentLog('exp_20260124_143022', 'my_experiments/logs');

%% 4. RUN GRID SEARCH
% Define search space
search_space.learning_rate = {0.001, 0.0005, 0.0001};
search_space.batch_size = {128, 256, 512};
search_space.max_epochs = {100, 150, 200};

% Run grid search (tests all combinations)
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'grid');

%% 5. RUN RANDOM SEARCH
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

%% 6. COMPARE EXPERIMENTS
% Compare specific experiments
experiment_ids = {'exp_20260124_143022', 'exp_20260124_143145', 'exp_20260124_143301'};
comparison_table = compareExperiments(experiment_ids);

% Compare all experiments in directory
comparison_table = compareExperiments('all');

% Compare and save results to file
comparison_table = compareExperiments('all', 'experiments/logs', 'comparison.txt');

%% 7. SAVE EXPERIMENT MODEL
% Save model with unique identifier
model_path = saveExperimentModel(trained_net, norm_params, experiment_id);

% Save to custom directory
model_path = saveExperimentModel(trained_net, norm_params, experiment_id, 'my_models');

%% 8. LOAD EXPERIMENT MODEL
% Load specific experiment model
[trained_net, norm_params] = loadExperimentModel('exp_20260124_143022');

% Load from custom directory
[trained_net, norm_params] = loadExperimentModel('exp_20260124_143022', 'my_models');

%% 9. COMPLETE WORKFLOW EXAMPLE
% Step 1: Load and preprocess data
data = loadTireData(data_paths);
[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data);

% Step 2: Define search space
search_space.learning_rate = {0.001, 0.0005};
search_space.batch_size = {128, 256};

% Step 3: Run hyperparameter search
[best_config, all_results] = runHyperparameterSearch(...
    X_train, Y_train, X_val, Y_val, norm_params, ...
    search_space, 'grid');

% Step 4: Compare all experiments
comparison_table = compareExperiments('all', 'experiments/logs', 'comparison.txt');

% Step 5: Load best model
best_exp_id = all_results{1}.experiment_id;
[best_net, best_params] = loadExperimentModel(best_exp_id);

% Step 6: Use best model for predictions
predictions = predictTireForces(best_net, new_data, best_params);

%% 10. TIPS AND BEST PRACTICES
% - Start with random search to explore the space quickly
% - Use grid search for fine-tuning around promising regions
% - Monitor training time and adjust max_epochs accordingly
% - Check GPU memory usage if training fails
% - Always compare experiments before selecting final model
% - Save comparison reports for documentation
% - Use meaningful experiment IDs for tracking
