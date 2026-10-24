function [best_config, all_results] = runHyperparameterSearch(X_train, Y_train, X_val, Y_val, norm_params, search_space, search_type, num_trials)
% RUNHYPERPARAMETERSEARCH Perform hyperparameter search
%
% This function performs either grid search or random search over a specified
% hyperparameter search space. It trains models with different configurations,
% evaluates their performance, and tracks the best configuration.
%
% Inputs:
%   X_train      - Training input features (N×5 matrix)
%   Y_train      - Training output targets (N×4 matrix)
%   X_val        - Validation input features (M×5 matrix)
%   Y_val        - Validation output targets (M×4 matrix)
%   norm_params  - Normalization parameters structure
%   search_space - Structure defining hyperparameter search space:
%                  Each field should be a cell array of values to try
%                  Example: search_space.learning_rate = {0.001, 0.0001};
%                           search_space.batch_size = {128, 256, 512};
%   search_type  - 'grid' for grid search, 'random' for random search
%   num_trials   - Number of trials for random search (ignored for grid search)
%
% Outputs:
%   best_config  - Configuration that achieved best validation performance
%   all_results  - Cell array of all experiment results
%
% Examples:
%   % Grid search
%   search_space.learning_rate = {0.001, 0.0001};
%   search_space.batch_size = {128, 256};
%   [best_config, results] = runHyperparameterSearch(X_train, Y_train, ...
%       X_val, Y_val, norm_params, search_space, 'grid');
%
%   % Random search
%   search_space.learning_rate = {0.001, 0.0005, 0.0001};
%   search_space.batch_size = {64, 128, 256, 512};
%   search_space.max_epochs = {100, 150, 200};
%   [best_config, results] = runHyperparameterSearch(X_train, Y_train, ...
%       X_val, Y_val, norm_params, search_space, 'random', 10);
%
% Requirements: 7.3

    fprintf('========================================\n');
    fprintf('HYPERPARAMETER SEARCH\n');
    fprintf('========================================\n');
    fprintf('Search type: %s\n', search_type);
    
    % Generate configurations based on search type
    if strcmpi(search_type, 'grid')
        configs = generateGridSearchConfigs(search_space);
        fprintf('Grid search: %d configurations to evaluate\n\n', length(configs));
    elseif strcmpi(search_type, 'random')
        if nargin < 8
            num_trials = 10;
        end
        configs = generateRandomSearchConfigs(search_space, num_trials);
        fprintf('Random search: %d random configurations\n\n', length(configs));
    else
        error('Invalid search_type. Use ''grid'' or ''random''.');
    end
    
    % Initialize results storage
    all_results = cell(length(configs), 1);
    best_val_loss = inf;
    best_config = [];
    best_idx = 0;
    
    % Create log directory
    log_dir = 'experiments/logs';
    if ~exist(log_dir, 'dir')
        mkdir(log_dir);
    end
    
    % Iterate through configurations
    for i = 1:length(configs)
        fprintf('========================================\n');
        fprintf('Trial %d/%d\n', i, length(configs));
        fprintf('========================================\n');
        
        config = configs{i};
        
        try
            % Create model
            net = createModelFromConfig(config);
            
            % Train model
            [trained_net, training_info] = trainTireModel(net, X_train, Y_train, X_val, Y_val, config);
            
            % Evaluate model
            metrics = evaluateModel(trained_net, X_val, Y_val, norm_params);
            
            % Log experiment
            experiment_id = logExperiment(config, metrics, training_info, log_dir);
            
            % Save model with unique identifier
            model_dir = 'experiments/models';
            saveExperimentModel(trained_net, norm_params, experiment_id, model_dir);
            
            % Store results
            result = struct();
            result.config = config;
            result.metrics = metrics;
            result.training_info = training_info;
            result.experiment_id = experiment_id;
            result.val_loss = training_info.FinalValidationLoss;
            all_results{i} = result;
            
            % Check if this is the best configuration so far
            if training_info.FinalValidationLoss < best_val_loss
                best_val_loss = training_info.FinalValidationLoss;
                best_config = config;
                best_idx = i;
                
                fprintf('\n*** NEW BEST CONFIGURATION ***\n');
                fprintf('Validation loss: %.6f\n', best_val_loss);
                fprintf('Experiment ID: %s\n\n', experiment_id);
            end
            
        catch ME
            fprintf('\n*** ERROR IN TRIAL %d ***\n', i);
            fprintf('Error: %s\n', ME.message);
            fprintf('Skipping this configuration...\n\n');
            
            % Store error result
            result = struct();
            result.config = config;
            result.error = ME.message;
            result.val_loss = inf;
            all_results{i} = result;
        end
    end
    
    % Display final summary
    fprintf('\n========================================\n');
    fprintf('HYPERPARAMETER SEARCH COMPLETE\n');
    fprintf('========================================\n');
    fprintf('Total trials: %d\n', length(configs));
    fprintf('Successful trials: %d\n', sum(cellfun(@(x) ~isfield(x, 'error'), all_results)));
    fprintf('\nBest configuration (Trial %d):\n', best_idx);
    fprintf('  Validation loss: %.6f\n', best_val_loss);
    fprintf('  Experiment ID: %s\n', all_results{best_idx}.experiment_id);
    fprintf('  Learning rate: %.6f\n', best_config.learning_rate);
    fprintf('  Batch size: %d\n', best_config.batch_size);
    fprintf('  Max epochs: %d\n', best_config.max_epochs);
    fprintf('========================================\n\n');
end


function configs = generateGridSearchConfigs(search_space)
% Generate all combinations for grid search
    
    % Get field names and values
    field_names = fieldnames(search_space);
    field_values = cell(length(field_names), 1);
    
    for i = 1:length(field_names)
        field_values{i} = search_space.(field_names{i});
    end
    
    % Generate all combinations using ndgrid
    n_fields = length(field_names);
    grid_outputs = cell(1, n_fields);
    [grid_outputs{:}] = ndgrid(1:length(field_values{1}), ...
                                arrayfun(@(i) 1:length(field_values{i}), 2:n_fields, 'UniformOutput', false));
    
    % Flatten grid to get all combinations
    n_combinations = prod(cellfun(@length, field_values));
    configs = cell(n_combinations, 1);
    
    for i = 1:n_combinations
        config = createHyperparameterConfig();  % Start with defaults
        
        for j = 1:n_fields
            field_name = field_names{j};
            value_idx = grid_outputs{j}(i);
            value = field_values{j}{value_idx};
            config.(field_name) = value;
        end
        
        configs{i} = config;
    end
end


function configs = generateRandomSearchConfigs(search_space, num_trials)
% Generate random configurations for random search
    
    field_names = fieldnames(search_space);
    configs = cell(num_trials, 1);
    
    for i = 1:num_trials
        config = createHyperparameterConfig();  % Start with defaults
        
        % Randomly select value for each hyperparameter
        for j = 1:length(field_names)
            field_name = field_names{j};
            possible_values = search_space.(field_name);
            random_idx = randi(length(possible_values));
            config.(field_name) = possible_values{random_idx};
        end
        
        configs{i} = config;
    end
end
