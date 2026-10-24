%% TEST_BASIC_FUNCTIONALITY - Unit tests for Week 1 deliverables
%
% This script tests:
%   1. Material loading
%   2. Pipe catalog loading
%   3. Section property calculations
%   4. Size class determination
%   5. Catalog filtering

close all; clc;

fprintf('=== Frame Optimization Week 1 Tests ===\n\n');

% Setup paths
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

total_tests = 0;
passed_tests = 0;

%% Test 1: Load Materials
fprintf('Test 1: Load materials from JSON...\n');
total_tests = total_tests + 1;

try
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));

    % Check that AISI4130 exists
    assert(isfield(materials, 'AISI4130'), 'AISI4130 not found');

    % Check properties
    assert(materials.AISI4130.E_GPa == 205, 'Incorrect E for AISI4130');
    assert(materials.AISI4130.rho_kg_m3 == 7850, 'Incorrect density for AISI4130');

    fprintf('  ✓ PASSED: Loaded %d materials\n', length(fieldnames(materials)));
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 2: Load Pipe Catalog
fprintf('\nTest 2: Load pipe catalog...\n');
total_tests = total_tests + 1;

try
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

    % Check that we have pipes
    assert(~isempty(catalog), 'Catalog is empty');

    % Check that first pipe has required fields
    assert(isfield(catalog(1), 'A_mm2'), 'Missing A_mm2 field');
    assert(isfield(catalog(1), 'Iy_mm4'), 'Missing Iy_mm4 field');
    assert(isfield(catalog(1), 'size_class'), 'Missing size_class field');

    fprintf('  ✓ PASSED: Loaded %d pipes\n', length(catalog));
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 3: Section Properties (Round Tube)
fprintf('\nTest 3: Compute section properties for round tube...\n');
total_tests = total_tests + 1;

try
    % AISI 4130 25.4mm × 1.65mm round tube (Size A reference)
    props = computeTubeSectionProperties(25.4, 1.65, 'round');

    % Expected values (hand-calculated)
    D_outer = 25.4;
    t_val = 1.65;
    D_inner = D_outer - 2*t_val;

    A_expected = pi/4 * (D_outer^2 - D_inner^2);
    I_expected = pi/64 * (D_outer^4 - D_inner^4);

    % Check within 0.1% tolerance
    assert(abs(props.A_mm2 - A_expected) / A_expected < 0.001, ...
           'Area calculation error');
    assert(abs(props.Iy_mm4 - I_expected) / I_expected < 0.001, ...
           'Moment of inertia calculation error');

    fprintf('  ✓ PASSED: A=%.2f mm², I=%.1f mm⁴\n', props.A_mm2, props.Iy_mm4);
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 4: Section Properties (Square Tube)
fprintf('\nTest 4: Compute section properties for square tube...\n');
total_tests = total_tests + 1;

try
    props = computeTubeSectionProperties(25.4, 1.65, 'square');

    % Expected values
    D_outer = 25.4;
    t_val = 1.65;
    D_inner = D_outer - 2*t_val;

    A_expected = D_outer^2 - D_inner^2;
    I_expected = (D_outer^4 - D_inner^4) / 12;

    assert(abs(props.A_mm2 - A_expected) / A_expected < 0.001, ...
           'Area calculation error for square');
    assert(abs(props.Iy_mm4 - I_expected) / I_expected < 0.001, ...
           'Moment of inertia calculation error for square');
    % Bredt: J = b^3 t (b = D - t) = 22,104 mm^4 for 25.4 x 1.65 (#47)
    b = D_outer - t_val;
    assert(abs(props.J_mm4 - b^3*t_val) < 1e-6, 'Square J must be Bredt b^3 t');
    assert(abs(props.J_mm4 - 22104) < 5, 'Square J 25.4x1.65 must be ~22,104 mm^4');
    assert(abs(props.Asy_mm2 - A_expected/2) < 1e-9, 'Square shear area must be web area A/2');
    round_props = computeTubeSectionProperties(25.4, 1.65, 'round');
    assert(abs(round_props.Asy_mm2 - round_props.A_mm2/2) < 1e-9, 'Round shear area must be A/2');

    fprintf('  ✓ PASSED: A=%.2f mm², I=%.1f mm⁴\n', props.A_mm2, props.Iy_mm4);
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 5: Size Class Determination
fprintf('\nTest 5: Determine FSAE size classes...\n');
total_tests = total_tests + 1;

try
    % Test Size A (31.8mm × 2.41mm round)
    props = computeTubeSectionProperties(31.8, 2.41, 'round');
    size_class = determineSizeClass(props.Iy_mm4, props.A_mm2, 31.8, 2.41);
    assert(strcmp(size_class, 'A'), sprintf('Expected Size A, got %s', size_class));

    % Test Size D (35.0mm × 1.20mm round; synthetic D envelope)
    props = computeTubeSectionProperties(35.0, 1.20, 'round');
    size_class = determineSizeClass(props.Iy_mm4, props.A_mm2, 35.0, 1.20);
    assert(strcmp(size_class, 'D'), sprintf('Expected Size D, got %s', size_class));

    % A/D overlap is resolved in favor of the separate D specification.
    props = computeTubeSectionProperties(35.0, 2.41, 'round');
    size_class = determineSizeClass(props.Iy_mm4, props.A_mm2, 35.0, 2.41);
    assert(strcmp(size_class, 'D'), sprintf('Expected D precedence, got %s', size_class));

    % Test None (too small)
    props = computeTubeSectionProperties(15.0, 0.5, 'round');
    size_class = determineSizeClass(props.Iy_mm4, props.A_mm2, 15.0, 0.5);
    assert(strcmp(size_class, 'None'), sprintf('Expected None, got %s', size_class));

    fprintf('  ✓ PASSED: Size classification correct\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 6: Catalog Filtering
fprintf('\nTest 6: Filter pipe catalog...\n');
total_tests = total_tests + 1;

try
    % Filter for Size A, round, AISI4130
    opts = struct('min_size_class', 'A', ...
                  'shape', 'round', ...
                  'material_id', 'AISI4130', ...
                  'available_only', true);
    idx = filterPipeCatalog(catalog, opts);

    assert(~isempty(idx), 'No Size A round AISI4130 pipes found');

    % Check that all filtered pipes meet criteria
    for i = idx'
        assert(strcmp(catalog(i).material_id, 'AISI4130'), 'Wrong material');
        assert(strcmp(catalog(i).shape, 'round'), 'Wrong shape');

        % A/B/C strength hierarchy; D is a separate side-impact spec and is
        % never returned by an A/B/C filter.
        size_map = containers.Map({'None', 'C', 'B', 'A'}, {0, 1, 2, 3});
        assert(~strcmp(catalog(i).size_class, 'D'), 'Size D leaked into A filter');
        assert(size_map(catalog(i).size_class) >= size_map('A'), 'Size too small');
    end

    % Size D is a separate envelope and must be selectable explicitly.
    idx_d = filterPipeCatalog(catalog, struct('min_size_class', 'D', 'available_only', true));
    assert(~isempty(idx_d), 'No Size D pipes found');
    assert(all(strcmp({catalog(idx_d).size_class}, 'D')), 'D filter returned non-D pipe');

    fprintf('  ✓ PASSED: Found %d matching pipes (%d Size D)\n', ...
        length(idx), length(idx_d));
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 7: Catalog Statistics
fprintf('\nTest 7: Catalog statistics...\n');
total_tests = total_tests + 1;

try
    % Count pipes by size class
    size_counts = struct('A', 0, 'B', 0, 'C', 0, 'D', 0, 'None', 0);
    for i = 1:length(catalog)
        sc = catalog(i).size_class;
        if isfield(size_counts, sc)
            size_counts.(sc) = size_counts.(sc) + 1;
        end
    end

    fprintf('  Size distribution:\n');
    fprintf('    Size A: %d pipes\n', size_counts.A);
    fprintf('    Size B: %d pipes\n', size_counts.B);
    fprintf('    Size C: %d pipes\n', size_counts.C);
    fprintf('    Size D: %d pipes\n', size_counts.D);
    fprintf('    None:   %d pipes\n', size_counts.None);

    assert(size_counts.A > 0, 'No Size A pipes in catalog');

    fprintf('  ✓ PASSED\n');
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
    error('test_basic_functionality:Failed', '%d tests failed', total_tests - passed_tests);
end
