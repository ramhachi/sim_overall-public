function [X_norm, mu, sigma] = normalizeData(X)
% NORMALIZEDATA Normalize data to zero mean and unit variance (Z-score)
%
% Inputs:
%   X - N×D matrix of data to normalize
%
% Outputs:
%   X_norm - N×D matrix of normalized data
%   mu     - 1×D vector of means for each feature
%   sigma  - 1×D vector of standard deviations for each feature
%
% Example:
%   X = randn(1000, 5) * 10 + 5;
%   [X_norm, mu, sigma] = normalizeData(X);
%   % Verify: mean(X_norm) should be ~0, std(X_norm) should be ~1
%
% Requirements: 1.3, 1.4, 1.7

    % Compute mean and standard deviation for each column
    mu = mean(X, 1);
    sigma = std(X, 0, 1);  % 0 = normalize by N-1 (sample std)
    
    % Handle case where std is zero (constant column)
    % Replace zero std with 1 to avoid division by zero
    sigma(sigma == 0) = 1;
    
    % Normalize: (X - mu) / sigma
    X_norm = (X - mu) ./ sigma;
end
