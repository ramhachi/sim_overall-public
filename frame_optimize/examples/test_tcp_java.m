%% Java Socket版TCPテスト
clear; close all; clc;

fprintf('=== Java Socket版 FEM Solver Test ===\n\n');

% Add path
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(fullfile(project_dir, 'matlab'));

% Create minimal test case
nodes = [
    0.0, 0.0, 0.0;
    1.0, 0.0, 0.0;
];

elements = [
    1, 2, 25.4, 1.65, 205, 80, 7850, 1, 0.7
];

bcs = [
    1, 1, 0; 1, 2, 0; 1, 3, 0; 1, 4, 0; 1, 5, 0; 1, 6, 0;
];

loads = [
    2, 1, 1000;
];

fprintf('Test case: 2 nodes, 1 element\n\n');

% Test 1: Initial call (server startup)
fprintf('1. Initial call (server startup)...\n');
tic;
result1 = beam_fem_solver_tcp_java(nodes, elements, bcs, loads);
elapsed1 = toc;
fprintf('   Time: %.3f seconds\n', elapsed1);
fprintf('   Mass: %.3f kg, K_torsion: %.3f Nm/deg\n\n', result1.mass, result1.K_torsion);

% Test 2: 100 rapid calls (performance test)
fprintf('2. Performance test (100 calls)...\n');
times = zeros(100, 1);
for i = 1:100
    tic;
    result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads);
    times(i) = toc;
end

fprintf('   Mean: %.2f ms\n', mean(times)*1000);
fprintf('   Std:  %.2f ms\n', std(times)*1000);
fprintf('   Min:  %.2f ms\n', min(times)*1000);
fprintf('   Max:  %.2f ms\n', max(times)*1000);
fprintf('   Median: %.2f ms\n\n', median(times)*1000);

% Test 3: Compare with tcpclient version (if available)
if exist('beam_fem_solver_tcp', 'file')
    fprintf('3. Comparison with tcpclient version...\n');

    % Stop Java version
    beam_fem_solver_tcp_java('stop');
    pause(1);

    % Test tcpclient version
    tcp_times = zeros(20, 1);
    for i = 1:20
        tic;
        result_tcp = beam_fem_solver_tcp(nodes, elements, bcs, loads);
        tcp_times(i) = toc;
    end
    beam_fem_solver_tcp('stop');

    fprintf('   tcpclient mean: %.2f ms\n', mean(tcp_times(2:end))*1000);
    fprintf('   Java Socket mean: %.2f ms\n', mean(times)*1000);
    fprintf('   Speedup: %.1fx\n\n', mean(tcp_times(2:end))/mean(times));
else
    fprintf('3. tcpclient version not available for comparison\n\n');

    % Shutdown Java version
    beam_fem_solver_tcp_java('stop');
end

% Visualization
figure('Position', [100, 100, 1000, 400]);

subplot(1,2,1);
histogram(times*1000, 30);
xlabel('Time [ms]');
ylabel('Frequency');
title(sprintf('Java Socket Performance (Mean: %.2f ms)', mean(times)*1000));
grid on;

subplot(1,2,2);
plot(times*1000, 'o-', 'LineWidth', 1, 'MarkerSize', 4);
xlabel('Call number');
ylabel('Time [ms]');
title('Performance Stability');
grid on;

fprintf('✓ Test complete\n');
