%% TEST_WEEK4_FUNCTIONS - Unit tests for Week 4 Visualization/UI
%
% This script tests:
%   1. plot_pareto
%   2. plot_frame
%   3. fsae_viewer

close all; clc;

fprintf('=== Frame Optimization Week 4 Tests ===\n\n');

% Setup paths
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

total_tests = 0;
passed_tests = 0;

%% 1. Load Dummy Data
try
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
    
    % Simple Square Pyramid
    nodes = [0,0,0; 1,0,0; 1,1,0; 0,1,0; 0.5,0.5,1];
    conn = [1,2; 2,3; 3,4; 4,1; 1,5; 2,5; 3,5; 4,5];
    groups = [1;1;1;1; 2;2;2;2];
    group_names = {'Base', 'Legs'};
    
    elems_geom = struct('connectivity', conn, ...
                        'group_indices', groups, ...
                        'group_names', {group_names});
                        
    % Pipe assignments (Index 5 for Base, Index 2 over Legs)
    pipe_assign = [5, 2];
    
    % Mock Pareto Data
    history_data = [15, 200; 14, 180; 16, 250; 12, 100; 13, 150];
    pareto_front = [12, 100; 14, 180; 16, 250];
    p_results = struct('history', history_data, 'front', pareto_front);
    
catch ME
    error('Setup failed: %s', ME.message);
end

%% Test 1: plot_pareto
fprintf('Test 1: plot_pareto (Silent execution check)...\n');
total_tests = total_tests + 1;
try
    fig_p = figure('Visible', 'off');
    plot_pareto(p_results.history, p_results.front, 'Unit Test Pareto');
    close(fig_p);
    
    fprintf('  ✓ PASSED: Pareto front plot rendered without crashing.\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 2: plot_frame
fprintf('\nTest 2: plot_frame (Silent execution check)...\n');
total_tests = total_tests + 1;
try
    fig_f = figure('Visible', 'off');
    plot_frame(nodes, elems_geom, pipe_assign, catalog, true);
    close(fig_f);
    
    fprintf('  ✓ PASSED: 3D Frame Plot (cylinders) rendered without crashing.\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 3: fsae_viewer
fprintf('\nTest 3: fsae_viewer integration display...\n');
total_tests = total_tests + 1;
try
    % Generate dummy view rendering in invisible mode
    set(0, 'DefaultFigureVisible', 'off');
    fsae_viewer(nodes, elems_geom, pipe_assign, catalog, p_results);
    set(0, 'DefaultFigureVisible', 'on');
    
    % Close all invisible test figures
    close all;
    
    fprintf('  ✓ PASSED: Combined dashboard viewer executes properly.\n');
    passed_tests = passed_tests + 1;
catch ME
    set(0, 'DefaultFigureVisible', 'on'); % Reset on fail
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Summary
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('Passed: %d / %d\n', passed_tests, total_tests);

if passed_tests == total_tests
    fprintf('✓ ALL TESTS PASSED\n');
else
    error('test_week4_functions:Failed', '%d tests failed', total_tests - passed_tests);
end
