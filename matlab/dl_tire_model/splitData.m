function [X_train, Y_train, X_val, Y_val] = splitData(X, Y, split_ratio)
% SPLITDATA Split data into training and validation sets
%
% Inputs:
%   X           - N×D matrix of input features
%   Y           - N×M matrix of output targets
%   split_ratio - Fraction of data to use for training (default: 0.8)
%
% Outputs:
%   X_train - Training input features
%   Y_train - Training output targets
%   X_val   - Validation input features
%   Y_val   - Validation output targets
%
% Example:
%   X = randn(1000, 5);
%   Y = randn(1000, 4);
%   [X_train, Y_train, X_val, Y_val] = splitData(X, Y, 0.8);
%   % X_train will have ~800 samples, X_val will have ~200 samples
%
% Requirements: 1.5

    % Default split ratio
    if nargin < 3
        split_ratio = 0.8;
    end
    
    % Validate split ratio
    if split_ratio <= 0 || split_ratio >= 1
        error('splitData:InvalidRatio', ...
              'split_ratio must be between 0 and 1, got %.2f', split_ratio);
    end
    
    % Get number of samples
    n_samples = size(X, 1);
    
    % Verify X and Y have same number of samples
    if size(Y, 1) ~= n_samples
        error('splitData:SizeMismatch', ...
              'X and Y must have the same number of samples. X: %d, Y: %d', ...
              n_samples, size(Y, 1));
    end
    
    % Calculate split index
    n_train = round(n_samples * split_ratio);
    
    % Create random permutation for shuffling
    rng('default');  % For reproducibility
    idx = randperm(n_samples);
    
    % Split indices
    train_idx = idx(1:n_train);
    val_idx = idx(n_train+1:end);
    
    % Split data
    X_train = X(train_idx, :);
    Y_train = Y(train_idx, :);
    X_val = X(val_idx, :);
    Y_val = Y(val_idx, :);
    
    % Log split statistics
    fprintf('Data split: %d training samples (%.1f%%), %d validation samples (%.1f%%)\n', ...
            size(X_train, 1), 100 * size(X_train, 1) / n_samples, ...
            size(X_val, 1), 100 * size(X_val, 1) / n_samples);
end
