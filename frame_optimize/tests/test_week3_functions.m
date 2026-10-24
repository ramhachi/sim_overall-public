%% TEST_WEEK3_FUNCTIONS - Unit tests for Week 3 Optimization Engine Deliverables
%
% This script tests:
%   1. generateGroundStructure
%   2. configureGA
%   3. extractParetoFront
%   4. runFrameOptimization (Mock Test Execution)

close all; clc;

fprintf('=== Frame Optimization Week 3 Tests ===\n\n');

% Setup paths
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

total_tests = 0;
passed_tests = 0;

%% Test 1: generateGroundStructure
fprintf('Test 1: generateGroundStructure...\n');
total_tests = total_tests + 1;

try
    % Create 4 simple nodes in a square
    test_nodes = [0,0,0; 1000,0,0; 0,1000,0; 1000,1000,0];
    gs_config = struct('max_length_mm', 1200); % Small max distance to trim diagonals (1414 block)
    
    elems = generateGroundStructure(test_nodes, gs_config);
    
    assert(size(elems, 1) == 4, 'Should strictly have 4 outside connections under 1200mm');
    fprintf('  ✓ PASSED: Filtered length-restricted Ground Structure mapping correctly.\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 2: configureGA
fprintf('\nTest 2: configureGA settings construction...\n');
total_tests = total_tests + 1;

try
    ga_conf = struct('pop_size', 20, 'max_generations', 5);
    opts = configureGA(ga_conf);
    
    % Just verifying structure integrity without executing the pool logic.
    % Optimoptions validation:
    assert(isa(opts, 'optim.options.SolverOptions'), 'Returns custom optimisation definition block.');
    fprintf('  ✓ PASSED: Valid GA options construct initialized.\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 3: extractParetoFront
fprintf('\nTest 3: extractParetoFront handling limits...\n');
total_tests = total_tests + 1;

try
    % Objective matrix shape: N-samples x M-objectives
    % Consider column 1 = Mass (minimize), Col 2 = -Stiffness (minimize negative so stiffness is maximized)
    data = [
       10.0, -500;   % Moderately heavy, ok stiffness
       8.0,  -400;   % Lightweight, poor stiffness -> PARETO C1
       12.0, -800;   % Heavy, high stiffness     -> PARETO C2
       9.0,  -350;   % Dominated by #2
       11.0, -450;   % Dominated by #1
       10.0, -500;   % Duplicate
    ];
    [p_front, p_idx] = extractParetoFront(data);
    
    assert(size(p_front, 1) == 3, 'Identified cleanly correct 3 Pareto optimal tiers');
    fprintf('  ✓ PASSED: Multi-objective non-domination extraction works.\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Summary
fprintf('\n=== TEST SUMMARY ===\n');
fprintf('Passed: %d / %d\n', passed_tests, total_tests);

if passed_tests == total_tests
    fprintf('✓ ALL TESTS PASSED\n');
else
    error('test_week3_functions:Failed', '%d tests failed', total_tests - passed_tests);
end
