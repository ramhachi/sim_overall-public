function config = createHyperparameterConfig(varargin)
% CREATEHYPERPARAMETERCONFIG Create hyperparameter configuration structure
%
% This function creates a configuration structure for hyperparameter tuning
% experiments. It supports all training-related hyperparameters and provides
% sensible defaults.
%
% Inputs (Name-Value pairs):
%   'learning_rate'   - Initial learning rate (default: 0.001)
%   'batch_size'      - Mini-batch size (default: 256)
%   'max_epochs'      - Maximum training epochs (default: 200)
%   'patience'        - Early stopping patience (default: 10)
%   'val_frequency'   - Validation frequency in iterations (default: 50)
%   'random_seed'     - Random seed for reproducibility (default: 42)
%   'model_type'      - Model architecture: 'feedforward' or 'lstm' (default: 'feedforward')
%   'hidden_sizes'    - Hidden layer sizes (default: [128, 64, 32])
%   'dropout_rate'    - Dropout rate (default: 0.2)
%   'split_ratio'     - Train/validation split ratio (default: 0.8)
%
% Outputs:
%   config - Structure containing all hyperparameter settings
%
% Examples:
%   % Create default configuration
%   config = createHyperparameterConfig();
%
%   % Create custom configuration
%   config = createHyperparameterConfig('learning_rate', 0.0001, ...
%                                       'batch_size', 128, ...
%                                       'max_epochs', 100);
%
%   % Create configuration for LSTM model
%   config = createHyperparameterConfig('model_type', 'lstm', ...
%                                       'hidden_sizes', [64, 32]);
%
% Requirements: 7.1

    % Parse input arguments
    p = inputParser;
    
    % Training hyperparameters
    addParameter(p, 'learning_rate', 0.001, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'batch_size', 256, @(x) isnumeric(x) && x > 0 && mod(x,1)==0);
    addParameter(p, 'max_epochs', 200, @(x) isnumeric(x) && x > 0 && mod(x,1)==0);
    addParameter(p, 'patience', 10, @(x) isnumeric(x) && x > 0 && mod(x,1)==0);
    addParameter(p, 'val_frequency', 50, @(x) isnumeric(x) && x > 0 && mod(x,1)==0);
    addParameter(p, 'random_seed', 42, @(x) isnumeric(x) && mod(x,1)==0);
    
    % Model architecture hyperparameters
    addParameter(p, 'model_type', 'feedforward', @(x) ischar(x) || isstring(x));
    addParameter(p, 'hidden_sizes', [128, 64, 32], @(x) isnumeric(x) && isvector(x));
    addParameter(p, 'dropout_rate', 0.2, @(x) isnumeric(x) && x >= 0 && x < 1);
    
    % Data preprocessing hyperparameters
    addParameter(p, 'split_ratio', 0.8, @(x) isnumeric(x) && x > 0 && x < 1);
    
    parse(p, varargin{:});
    
    % Create configuration structure
    config = struct();
    config.learning_rate = p.Results.learning_rate;
    config.batch_size = p.Results.batch_size;
    config.max_epochs = p.Results.max_epochs;
    config.patience = p.Results.patience;
    config.val_frequency = p.Results.val_frequency;
    config.random_seed = p.Results.random_seed;
    config.model_type = p.Results.model_type;
    config.hidden_sizes = p.Results.hidden_sizes;
    config.dropout_rate = p.Results.dropout_rate;
    config.split_ratio = p.Results.split_ratio;
    
    % Add metadata
    config.created_at = datetime('now');
    config.config_version = '1.0';
    
    % Display configuration summary
    fprintf('Hyperparameter Configuration Created:\n');
    fprintf('=====================================\n');
    fprintf('Training Parameters:\n');
    fprintf('  Learning rate:     %.6f\n', config.learning_rate);
    fprintf('  Batch size:        %d\n', config.batch_size);
    fprintf('  Max epochs:        %d\n', config.max_epochs);
    fprintf('  Patience:          %d\n', config.patience);
    fprintf('  Val frequency:     %d\n', config.val_frequency);
    fprintf('  Random seed:       %d\n', config.random_seed);
    fprintf('\nModel Architecture:\n');
    fprintf('  Model type:        %s\n', config.model_type);
    fprintf('  Hidden sizes:      [%s]\n', num2str(config.hidden_sizes));
    fprintf('  Dropout rate:      %.2f\n', config.dropout_rate);
    fprintf('\nData Preprocessing:\n');
    fprintf('  Split ratio:       %.2f\n', config.split_ratio);
    fprintf('=====================================\n\n');
end
