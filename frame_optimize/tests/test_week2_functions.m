%% TEST_WEEK2_FUNCTIONS - Unit tests for Week 2 deliverables
%
% This script tests:
%   1. getRequiredSizeClass
%   2. checkMaterialRestriction
%   3. checkExclusionZones
%
% Canonical frame objective coverage now lives in test_frame_obj.m because
% the SSOT flow depends on parametric geometry + suspension artifact input.

close all; clc;

fprintf('=== Frame Optimization Week 2 Tests ===\n\n');

% Setup paths
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

total_tests = 0;
passed_tests = 0;

% Load prerequisites
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
reg_json_text = fileread(fullfile(data_dir, 'regulation_params.json'));
reg_params = jsondecode(reg_json_text);

%% Test 1: getRequiredSizeClass
fprintf('Test 1: getRequiredSizeClass...\n');
total_tests = total_tests + 1;

try
    size_mh = getRequiredSizeClass('main_hoop', reg_params);
    assert(strcmp(size_mh, 'A'), 'main_hoop should be Size A');
    
    size_fbh = getRequiredSizeClass('front_bulkhead', reg_params);
    assert(strcmp(size_fbh, 'B'), 'front_bulkhead should be Size B');
    
    size_diag = getRequiredSizeClass('diagonal_brace', reg_params);
    assert(strcmp(size_diag, 'C'), 'diagonal_brace should be Size C');

    size_struct = getRequiredSizeClass('structural', reg_params);
    assert(strcmp(size_struct, 'B'), 'V1 structural should be Size B');

    size_shoulder = getRequiredSizeClass('shoulder', reg_params);
    assert(strcmp(size_shoulder, 'A'), 'V1 shoulder should be Size A');

    try
        getRequiredSizeClass('unknown_group', reg_params);
        error('test_week2:UnknownGroup', 'unknown_group should raise an error');
    catch ME
        assert(strcmp(ME.identifier, 'getRequiredSizeClass:UnknownGroup'), ...
            'unknown_group should raise getRequiredSizeClass:UnknownGroup');
    end

    fprintf('  ✓ PASSED\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 2: checkMaterialRestriction
fprintf('\nTest 2: checkMaterialRestriction...\n');
total_tests = total_tests + 1;

try
    % Find specific pipes
    steel_pipe_idx = find(arrayfun(@(p) strcmp(p.material_id, 'AISI4130'), catalog), 1);
    al_thick_pipe_idx = find(arrayfun(@(p) strcmp(p.material_id, 'Al6061T6') && p.t_mm >= 3.0, catalog), 1);
    al_thin_pipe_idx = find(arrayfun(@(p) strcmp(p.material_id, 'Al6061T6') && p.t_mm < 3.0, catalog), 1);
    
    steel_pipe = catalog(steel_pipe_idx);
    al_thick_pipe = catalog(al_thick_pipe_idx);
    al_thin_pipe = catalog(al_thin_pipe_idx);

    % Test 2.1: Main hoop with steel (Should PASS)
    [is_valid, reason] = checkMaterialRestriction('main_hoop', steel_pipe, materials.(steel_pipe.material_id), reg_params);
    assert(is_valid, 'Main hoop with steel should be valid');

    % Test 2.2: Main hoop with aluminum (Should FAIL)
    [is_valid, reason] = checkMaterialRestriction('main_hoop', al_thick_pipe, materials.(al_thick_pipe.material_id), reg_params);
    assert(~is_valid, 'Main hoop with aluminum should be invalid');

    % Test 2.3: Non-restricted group with thick aluminum (Should PASS)
    [is_valid, reason] = checkMaterialRestriction('front_bulkhead', al_thick_pipe, materials.(al_thick_pipe.material_id), reg_params);
    assert(is_valid, 'Front bulkhead with thick Al should be valid');

    % Test 2.4: Non-restricted group with thin aluminum (Should FAIL minimum thickness)
    [is_valid, reason] = checkMaterialRestriction('front_bulkhead', al_thin_pipe, materials.(al_thin_pipe.material_id), reg_params);
    assert(~is_valid, 'Front bulkhead with thin Al should fail thickness check');

    % Test 2.5: CFRP is not subject to the aluminum 3.0 mm welded minimum
    cfrp_pipe_idx = find(arrayfun(@(p) strcmp(p.material_id, 'CFRP_T700') && p.t_mm < 3.0, catalog), 1);
    cfrp_pipe = catalog(cfrp_pipe_idx);
    [is_valid, reason] = checkMaterialRestriction('front_bulkhead', cfrp_pipe, materials.(cfrp_pipe.material_id), reg_params);
    assert(is_valid, sprintf('CFRP should not inherit aluminum thickness rules: %s', reason));

    % Test 2.6: CFRP is rejected where steel is mandatory
    [is_valid, ~] = checkMaterialRestriction('main_hoop', cfrp_pipe, materials.(cfrp_pipe.material_id), reg_params);
    assert(~is_valid, 'Main hoop with CFRP should be invalid');

    fprintf('  ✓ PASSED\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 3: checkExclusionZones
fprintf('\nTest 3: checkExclusionZones...\n');
total_tests = total_tests + 1;

try
    nodes = [
        0, 0, 0;    % N1
        1000, 0, 0; % N2
        500, 500, 0 % N3
    ];
    
    % Element 1 intersects AABB, Element 2 does not
    elements = [1, 2; 1, 3]; 
    
    config.exclusion_zones = [
        struct('type', 'AABB', 'min_bounds', [400, -100, -100], 'max_bounds', [600, 100, 100])
    ];

    penalties = checkExclusionZones(nodes, elements, config);
    
    assert(penalties(1) > 0, 'Element 1 passing through AABB should have penalty');
    assert(abs(penalties(1) - 200 * 1e6) < 1e-6, 'Intersection length should be exactly 200mm');
    assert(penalties(2) == 0, 'Element 2 not passing through AABB should have 0 penalty');

    fprintf('  ✓ PASSED\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 4: exclusion-zone name exemptions and unsupported types
fprintf('\nTest 4: checkExclusionZones name exemptions...\n');
total_tests = total_tests + 1;

try
    nodes = [0, 0, 0; 1000, 0, 0];
    elements = [1, 2];
    config.exclusion_zones = struct('name', 'driver_cockpit', 'type', 'AABB', ...
        'min_bounds', [400, -100, -100], 'max_bounds', [600, 100, 100]);

    p_exempt = checkExclusionZones(nodes, elements, config, 2, {'unused', 'main_hoop'});
    assert(p_exempt(1) == 0, 'main_hoop must be exempt inside driver_cockpit');

    p_hit = checkExclusionZones(nodes, elements, config, 2, {'unused', 'front_bulkhead'});
    assert(p_hit(1) > 0, 'front_bulkhead must not be exempt inside driver_cockpit');

    config.exclusion_zones = struct('name', 'front_hoop_sweep', 'type', 'AABB', ...
        'min_bounds', [400, -100, -100], 'max_bounds', [600, 100, 100]);
    p_fh = checkExclusionZones(nodes, elements, config, 2, {'unused', 'front_hoop'});
    assert(p_fh(1) == 0, 'front_hoop must be exempt inside front_hoop_sweep');

    % driver_cockpit must be clipped to the actual front-hoop x, not the
    % static artifact min x (FRAME-CODE-23 / #109).
    nodes2 = [500, 0, 0; 600, 0, 0; 400, 0, 0; 450, 0, 0];
    elements2 = [1, 2; 3, 4];
    config.exclusion_zones = struct('name', 'driver_cockpit', 'type', 'AABB', ...
        'min_bounds', [0, -100, -100], 'max_bounds', [1000, 100, 100]);
    p_clip = checkExclusionZones(nodes2, elements2, config, [1; 2], ...
        {'front_hoop', 'structural'});
    assert(p_clip(2) == 0, 'driver_cockpit must be clipped aft of the front hoop');

    config.exclusion_zones = struct('type', 'AABB', ...
        'min_bounds', [400, -100, -100], 'max_bounds', [600, 100, 100]);
    [p_unnamed, names_unnamed] = checkExclusionZones(nodes, elements, config, 2, ...
        {'unused', 'front_bulkhead'});
    assert(p_unnamed(1) > 0, 'Unnamed zone must still be checked');
    assert(~isempty(names_unnamed{1}), 'Unnamed zone must get a fallback name');

    config.exclusion_zones = struct('name', 'unsupported', 'type', 'ConvexHull');
    caught = false;
    try
        checkExclusionZones(nodes, elements, config, 2, {'unused', 'front_bulkhead'});
    catch ME
        caught = strcmp(ME.identifier, 'checkExclusionZones:UnsupportedZoneType');
    end
    assert(caught, 'Unsupported zone type must fail fast');

    fprintf('  ✓ PASSED\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  ✗ FAILED: %s\n', ME.message);
end

%% Test 5: compliance size/material fail-closed behavior
fprintf('\nTest 5: checkFrameCompliance size/material...\n');
total_tests = total_tests + 1;

try
    frame.nodes = [0, 0, 0; 1, 0, 0];
    frame.elements = [1, 2, 31.8, 2.41, 205, 80, 7850, 1, 0.85];
    frame.element_groups = struct('main_hoop', 1, 'front_hoop', 1);
    frame.element_materials = {'AISI4130'};
    [pass, pen, ~] = checkFrameCompliance(frame, catalog, materials, reg_params);
    assert(pass && pen == 0, 'Size-A steel hoop should pass compliance');

    % 31.8x1.65 is Size B and must not slip through main_hoop (Size A).
    frame.elements = [1, 2, 31.8, 1.65, 205, 80, 7850, 1, 0.85];
    [pass, ~, report] = checkFrameCompliance(frame, catalog, materials, reg_params);
    assert(~pass && ~isempty(report.size_violations), ...
        'Undersized main hoop must be rejected');

    % CFRP is governed by equivalence, not the aluminum 3.0mm thickness rule.
    frame.elements = [1, 2, 32.0, 2.0, 70, 26, 1600, 1, 0.85];
    frame.element_groups = struct('front_bulkhead', 1, 'front_hoop', 1);
    frame.element_materials = {'CFRP_T700'};
    [~, ~, report] = checkFrameCompliance(frame, catalog, materials, reg_params);
    assert(isempty(report.aluminum_thickness_violations), ...
        'CFRP must not inherit aluminum thickness violations');

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
    error('test_week2_functions:Failed', '%d tests failed', total_tests - passed_tests);
end
