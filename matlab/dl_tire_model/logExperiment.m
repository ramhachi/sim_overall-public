function experiment_id = logExperiment(config, metrics, training_info, log_dir)
% LOGEXPERIMENT Log hyperparameter configuration and performance metrics
%
% This function logs experiment details including hyperparameter configuration,
% performance metrics, and training information to a structured log file.
% Each experiment is assigned a unique identifier based on timestamp.
%
% Inputs:
%   config        - Hyperparameter configuration structure
%   metrics       - Performance metrics structure (from evaluateModel)
%   training_info - Training information structure (from trainTireModel)
%   log_dir       - Directory to save log files (default: 'experiments/logs')
%
% Outputs:
%   experiment_id - Unique identifier for this experiment (timestamp-based)
%
% Example:
%   config = createHyperparameterConfig('learning_rate', 0.001);
%   % ... train model and evaluate ...
%   experiment_id = logExperiment(config, metrics, training_info);
%
% Requirements: 7.2

    % Set default log directory
    if nargin < 4
        log_dir = 'experiments/logs';
    end
    
    % Create log directory if it doesn't exist
    if ~exist(log_dir, 'dir')
        mkdir(log_dir);
    end
    
    % Generate unique experiment ID based on timestamp
    experiment_id = sprintf('exp_%s', datestr(now, 'yyyymmdd_HHMMSS'));
    
    % Create experiment log structure
    experiment_log = struct();
    experiment_log.experiment_id = experiment_id;
    experiment_log.timestamp = datetime('now');
    experiment_log.config = config;
    experiment_log.metrics = metrics;
    experiment_log.training_info = training_info;
    
    % Save log as MAT file
    log_file_mat = fullfile(log_dir, [experiment_id, '.mat']);
    save(log_file_mat, 'experiment_log');
    
    % Also save as human-readable text file
    log_file_txt = fullfile(log_dir, [experiment_id, '.txt']);
    fid = fopen(log_file_txt, 'w');
    
    try
        % Write header
        fprintf(fid, '========================================\n');
        fprintf(fid, 'EXPERIMENT LOG\n');
        fprintf(fid, '========================================\n');
        fprintf(fid, 'Experiment ID: %s\n', experiment_id);
        fprintf(fid, 'Timestamp: %s\n\n', char(experiment_log.timestamp));
        
        % Write hyperparameter configuration
        fprintf(fid, '--- HYPERPARAMETER CONFIGURATION ---\n');
        fprintf(fid, 'Learning Rate:     %.6f\n', config.learning_rate);
        fprintf(fid, 'Batch Size:        %d\n', config.batch_size);
        fprintf(fid, 'Max Epochs:        %d\n', config.max_epochs);
        fprintf(fid, 'Patience:          %d\n', config.patience);
        fprintf(fid, 'Val Frequency:     %d\n', config.val_frequency);
        fprintf(fid, 'Random Seed:       %d\n', config.random_seed);
        fprintf(fid, 'Model Type:        %s\n', config.model_type);
        fprintf(fid, 'Hidden Sizes:      [%s]\n', num2str(config.hidden_sizes));
        fprintf(fid, 'Dropout Rate:      %.2f\n', config.dropout_rate);
        fprintf(fid, 'Split Ratio:       %.2f\n\n', config.split_ratio);
        
        % Write training information
        fprintf(fid, '--- TRAINING INFORMATION ---\n');
        fprintf(fid, 'Training Time:     %.2f seconds (%.2f minutes)\n', ...
                training_info.TrainingTime, training_info.TrainingTime / 60);
        fprintf(fid, 'Final Val Loss:    %.6f\n', training_info.FinalValidationLoss);
        fprintf(fid, 'Epochs Completed:  %d\n\n', length(training_info.TrainingLoss));
        
        % Write performance metrics
        fprintf(fid, '--- PERFORMANCE METRICS ---\n');
        output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
        for i = 1:length(output_names)
            output_name = output_names{i};
            fprintf(fid, '\n%s:\n', output_name);
            fprintf(fid, '  RMSE:  %.4f\n', metrics.(output_name).RMSE);
            fprintf(fid, '  R²:    %.6f\n', metrics.(output_name).R2);
            fprintf(fid, '  TIC:   %.6f\n', metrics.(output_name).TIC);
            fprintf(fid, '  MAE:   %.4f\n', metrics.(output_name).MAE);
        end
        
        % Write requirements check
        fprintf(fid, '\n--- REQUIREMENTS CHECK ---\n');
        fprintf(fid, 'Meets Requirements: %s\n', ...
                mat2str(metrics.meets_requirements));
        
        fprintf(fid, '\n========================================\n');
        
        fclose(fid);
        
        % Display summary
        fprintf('Experiment logged successfully:\n');
        fprintf('  Experiment ID: %s\n', experiment_id);
        fprintf('  Log files:\n');
        fprintf('    - %s\n', log_file_mat);
        fprintf('    - %s\n\n', log_file_txt);
        
    catch ME
        fclose(fid);
        rethrow(ME);
    end
end
