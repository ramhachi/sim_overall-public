function X = denormalizeData(X_norm, mu, sigma)
% DENORMALIZEDATA Denormalize data back to original scale
%
% This function reverses the Z-score normalization to recover the original
% data values.
%
% Inputs:
%   X_norm - N×D matrix of normalized data
%   mu     - 1×D vector of means for each feature
%   sigma  - 1×D vector of standard deviations for each feature
%
% Outputs:
%   X - N×D matrix of denormalized data
%
% Example:
%   X_original = randn(1000, 5) * 10 + 5;
%   [X_norm, mu, sigma] = normalizeData(X_original);
%   X_recovered = denormalizeData(X_norm, mu, sigma);
%   % X_recovered should equal X_original (within numerical precision)
%
% Requirements: 4.5

    % Denormalize: X = X_norm * sigma + mu
    X = X_norm .* sigma + mu;
end
