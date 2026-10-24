%% 簡単なTCPテスト
clear; close all; clc;

fprintf('=== TCP FEM Solver Simple Test ===\n\n');

% Add path
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(fullfile(project_dir, 'matlab'));

% Create minimal test case: 2 nodes, 1 element
nodes = [
    0.0, 0.0, 0.0;
    1.0, 0.0, 0.0;
];

elements = [
    1, 2, 25.4, 1.65, 205, 80, 7850, 1, 0.7
];

bcs = [
    1, 1, 0;
    1, 2, 0;
    1, 3, 0;
    1, 4, 0;
    1, 5, 0;
    1, 6, 0;
];

loads = [
    2, 1, 1000;
];

fprintf('Test case: 2 nodes, 1 element\n\n');

% Run TCP solver
fprintf('Calling TCP solver...\n');
tic;
result = beam_fem_solver_tcp(nodes, elements, bcs, loads);
elapsed = toc;

fprintf('✓ Success! Time: %.3f seconds\n', elapsed);
fprintf('Mass: %.3f kg\n', result.mass);
fprintf('K_torsion: %.3f Nm/deg\n', result.K_torsion);
fprintf('Max displacement: %.6f m\n', max(abs(result.U(:))));

% Run again to test cached connection
fprintf('\nSecond call (cached connection)...\n');
tic;
result2 = beam_fem_solver_tcp(nodes, elements, bcs, loads);
elapsed2 = toc;

fprintf('✓ Success! Time: %.1f ms\n', elapsed2*1000);
fprintf('Mass: %.3f kg\n', result2.mass);

% Shutdown server
fprintf('\nShutting down server...\n');
beam_fem_solver_tcp('shutdown');
fprintf('✓ Test complete\n');
