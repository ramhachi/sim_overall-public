function comparison_table = compareExperiments(experiment_ids, log_dir, output_file)
% COMPAREEXPERIMENTS Compare multiple experiments and generate comparison table
%
% This function loads multiple experiment logs, compares their configurations
% and performance metrics, and generates a comprehensive comparison table.
% The results can be saved to a file for documentation.
%
% Inputs:
%   experiment_ids - Cell array of experiment IDs to compare
%   log_dir        - Directory containing log files (default: 'experiments/logs')
%   output_file    - Optional file path to save comparison results
%
% Outputs:
%   comparison_table - Table containing comparison of all experiments
%
% Examples:
%   % Compare specific experiments
%   ids = {'exp_20260124_143022', 'exp_20260124_143145'};
%   table = compareExperiments(ids);
%
%   % Compare and save results
%   table = compareExperiments(ids, 'experiments/logs', 'comparison.txt');
%
%   % Compare all experiments in directory
%   table = compareExperiments('all');
%
% Requirements: 7.4

    % Set default log directory
    if nargin < 2 || isempty(log_dir)
        log_dir = 'experiments/logs';
    end
    
    % Handle 'all' option - load all experiments
    if ischar(experiment_ids) && strcmpi(experiment_ids, 'all')
        % Find all experiment log files
        log_files = dir(fullfile(log_dir, 'exp_*.mat'));
        experiment_ids = cell(length(log_files), 1);
        for i = 1:length(log_files)
            [~, exp_id, ~] = fileparts(log_files(i).name);
            experiment_ids{i} = exp_id;
        end
        
        if isempty(experiment_ids)
            error('No experiment logs found in %s', log_dir);
        end
        
        fprintf('Found %d experiments to compare\n\n', length(experiment_ids));
    end
    
    % Load all experiment logs
    n_experiments = length(experiment_ids);
    experiments = cell(n_experiments, 1);
    
    fprintf('Loading experiment logs...\n');
    for i = 1:n_experiments
        try
            experiments{i} = loadExperimentLog(experiment_ids{i}, log_dir);
        catch ME
            warning('Failed to load experiment %s: %s', experiment_ids{i}, ME.message);
            experiments{i} = [];
        end
    end
    
    % Remove failed loads
    valid_experiments = ~cellfun(@isempty, experiments);
    experiments = experiments(valid_experiments);
    experiment_ids = experiment_ids(valid_experiments);
    n_experiments = length(experiments);
    
    fprintf('Successfully loaded %d experiments\n\n', n_experiments);
    
    % Extract data for comparison table
    exp_id_col = experiment_ids;
    learning_rate_col = zeros(n_experiments, 1);
    batch_size_col = zeros(n_experiments, 1);
    max_epochs_col = zeros(n_experiments, 1);
    patience_col = zeros(n_experiments, 1);
    model_type_col = cell(n_experiments, 1);
    dropout_rate_col = zeros(n_experiments, 1);
    
    training_time_col = zeros(n_experiments, 1);
    val_loss_col = zeros(n_experiments, 1);
    
    rmse_fx_col = zeros(n_experiments, 1);
    rmse_fy_col = zeros(n_experiments, 1);
    r2_fx_col = zeros(n_experiments, 1);
    r2_fy_col = zeros(n_experiments, 1);
    
    meets_req_col = false(n_experiments, 1);
    
    for i = 1:n_experiments
        exp = experiments{i};
        
        % Configuration
        learning_rate_col(i) = exp.config.learning_rate;
        batch_size_col(i) = exp.config.batch_size;
        max_epochs_col(i) = exp.config.max_epochs;
        patience_col(i) = exp.config.patience;
        model_type_col{i} = exp.config.model_type;
        dropout_rate_col(i) = exp.config.dropout_rate;
        
        % Training info
        training_time_col(i) = exp.training_info.TrainingTime;
        val_loss_col(i) = exp.training_info.FinalValidationLoss;
        
        % Metrics
        rmse_fx_col(i) = exp.metrics.Fx.RMSE;
        rmse_fy_col(i) = exp.metrics.Fy.RMSE;
        r2_fx_col(i) = exp.metrics.Fx.R2;
        r2_fy_col(i) = exp.metrics.Fy.R2;
        
        meets_req_col(i) = exp.metrics.meets_requirements;
    end
    
    % Create comparison table
    comparison_table = table(exp_id_col, learning_rate_col, batch_size_col, ...
                            max_epochs_col, patience_col, model_type_col, ...
                            dropout_rate_col, training_time_col, val_loss_col, ...
                            rmse_fx_col, rmse_fy_col, r2_fx_col, r2_fy_col, ...
                            meets_req_col, ...
                            'VariableNames', {'ExperimentID', 'LearningRate', 'BatchSize', ...
                                             'MaxEpochs', 'Patience', 'ModelType', ...
                                             'DropoutRate', 'TrainingTime_s', 'ValLoss', ...
                                             'RMSE_Fx', 'RMSE_Fy', 'R2_Fx', 'R2_Fy', ...
                                             'MeetsRequirements'});
    
    % Sort by validation loss (best first)
    comparison_table = sortrows(comparison_table, 'ValLoss', 'ascend');
    
    % Display comparison table
    fprintf('========================================\n');
    fprintf('EXPERIMENT COMPARISON\n');
    fprintf('========================================\n\n');
    disp(comparison_table);
    
    % Find best experiment
    [best_val_loss, best_idx] = min(val_loss_col);
    best_exp_id = experiment_ids{best_idx};
    
    fprintf('\n========================================\n');
    fprintf('BEST EXPERIMENT\n');
    fprintf('========================================\n');
    fprintf('Experiment ID: %s\n', best_exp_id);
    fprintf('Validation Loss: %.6f\n', best_val_loss);
    fprintf('Learning Rate: %.6f\n', learning_rate_col(best_idx));
    fprintf('Batch Size: %d\n', batch_size_col(best_idx));
    fprintf('RMSE Fx: %.4f N\n', rmse_fx_col(best_idx));
    fprintf('RMSE Fy: %.4f N\n', rmse_fy_col(best_idx));
    fprintf('R² Fx: %.6f\n', r2_fx_col(best_idx));
    fprintf('R² Fy: %.6f\n', r2_fy_col(best_idx));
    fprintf('Meets Requirements: %s\n', mat2str(meets_req_col(best_idx)));
    fprintf('========================================\n\n');
    
    % Save to file if requested
    if nargin >= 3 && ~isempty(output_file)
        % Create output directory if needed
        [output_dir, ~, ~] = fileparts(output_file);
        if ~isempty(output_dir) && ~exist(output_dir, 'dir')
            mkdir(output_dir);
        end
        
        % Write to text file
        fid = fopen(output_file, 'w');
        try
            fprintf(fid, '========================================\n');
            fprintf(fid, 'EXPERIMENT COMPARISON REPORT\n');
            fprintf(fid, '========================================\n');
            fprintf(fid, 'Generated: %s\n', char(datetime('now')));
            fprintf(fid, 'Number of experiments: %d\n\n', n_experiments);
            
            % Write table
            fprintf(fid, '%s\n', evalc('disp(comparison_table)'));
            
            % Write best experiment summary
            fprintf(fid, '\n========================================\n');
            fprintf(fid, 'BEST EXPERIMENT\n');
            fprintf(fid, '========================================\n');
            fprintf(fid, 'Experiment ID: %s\n', best_exp_id);
            fprintf(fid, 'Validation Loss: %.6f\n', best_val_loss);
            fprintf(fid, 'Learning Rate: %.6f\n', learning_rate_col(best_idx));
            fprintf(fid, 'Batch Size: %d\n', batch_size_col(best_idx));
            fprintf(fid, 'RMSE Fx: %.4f N\n', rmse_fx_col(best_idx));
            fprintf(fid, 'RMSE Fy: %.4f N\n', rmse_fy_col(best_idx));
            fprintf(fid, 'R² Fx: %.6f\n', r2_fx_col(best_idx));
            fprintf(fid, 'R² Fy: %.6f\n', r2_fy_col(best_idx));
            fprintf(fid, 'Meets Requirements: %s\n', mat2str(meets_req_col(best_idx)));
            fprintf(fid, '========================================\n');
            
            fclose(fid);
            fprintf('Comparison report saved to: %s\n\n', output_file);
        catch ME
            fclose(fid);
            rethrow(ME);
        end
    end
end
