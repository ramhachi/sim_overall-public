%% TEST_CUSTOM_MIXED_GA_ELITES - elites are re-scored with the full objective (#51)
% Cheap objective prefers x1 = +3; the full objective forbids x1 > 0. The returned
% best must be fully scored, i.e. respect the full-only constraint.

fprintf('=== custom_mixed_ga elite re-scoring ===\n');
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'matlab'));
rng(1);
cheap = @(x) (x(1) - 3)^2 + x(2)^2;
full = @(x) cheap(x) + 100 * (x(1) > 0);
opts = struct('PopulationSize', 20, 'MaxGenerations', 15, 'UseParallel', false, ...
    'EliteFitnessFcn', full);
evalc('[best_x, best_fval] = custom_mixed_ga(cheap, 2, [-5 -5], [5 5], 2, opts);');
assert(best_x(1) <= 0, 'Best design must satisfy the full-case objective (x1 <= 0).');
assert(abs(best_fval - full(best_x)) < 1e-12, 'Reported best must carry its full-case score.');
fprintf('custom_mixed_ga elite re-scoring OK\n');
