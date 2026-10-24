%% QUICK TEST - Real Vehicle Geometry Loading
%
% Quick validation test for the real vehicle implementation.
% Tests data loading and load case generation without running FEM.

clear; close all; clc;

fprintf('=== Real Vehicle Implementation Quick Test ===\n\n');

%% 1. Test geometry loading

script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

fprintf('1. Testing real_vehicle_geometry.json loading...\n');

geom_file = fullfile(data_dir, 'real_vehicle_geometry.json');
if ~isfile(geom_file)
    error('✗ File not found: %s', geom_file);
end

geom_text = fileread(geom_file);
geom = jsondecode(geom_text);

fprintf('   ✓ JSON loaded successfully\n');

% Validate structure
assert(isfield(geom, 'dimensions'), 'Missing field: dimensions');
assert(isfield(geom, 'exclusion_zones'), 'Missing field: exclusion_zones');
assert(isfield(geom, 'frame_nodes'), 'Missing field: frame_nodes');

fprintf('   ✓ Required fields present\n');

% Display dimensions
fprintf('\n   Vehicle dimensions:\n');
fprintf('     Wheelbase:     %.3f m\n', geom.dimensions.wheelbase_m);
fprintf('     Track front:   %.3f m\n', geom.dimensions.track_front_m);
fprintf('     Track rear:    %.3f m\n', geom.dimensions.track_rear_m);
fprintf('     CG height:     %.3f m\n', geom.dimensions.cg_height_m);
fprintf('     Total mass:    %.0f kg\n', geom.dimensions.total_mass_kg);

% Display exclusion zones
fprintf('\n   Exclusion zones (%d):\n', length(geom.exclusion_zones));
for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);
    fprintf('     %d. %s: [%.2f,%.2f,%.2f] to [%.2f,%.2f,%.2f]\n', ...
            i, zone.name, zone.min(1), zone.min(2), zone.min(3), ...
            zone.max(1), zone.max(2), zone.max(3));
end

%% 2. Test load case generation

fprintf('\n2. Testing generate_load_cases.m...\n');

% Create simple test nodes
nodes = [
    0, 0, 0;      % 1
    1, 0, 0;      % 2
    0, 0, 1;      % 3
    1, 0, 1;      % 4
];

load_cases = generate_load_cases(nodes, geom.dimensions.total_mass_kg, 9.81);

fprintf('   ✓ Generated %d load cases\n', length(load_cases));

% Validate structure
for i = 1:length(load_cases)
    assert(isfield(load_cases(i), 'name'), 'Load case %d missing name', i);
    assert(isfield(load_cases(i), 'loads'), 'Load case %d missing loads', i);
    assert(isfield(load_cases(i), 'description'), 'Load case %d missing description', i);

    fprintf('\n   Case %d: %s\n', i, load_cases(i).name);
    fprintf('     Description: %s\n', load_cases(i).description);
    fprintf('     Loads: %d rows\n', size(load_cases(i).loads, 1));

    % Display loads
    for j = 1:size(load_cases(i).loads, 1)
        node_id = load_cases(i).loads(j, 1);
        dof = load_cases(i).loads(j, 2);
        force = load_cases(i).loads(j, 3);
        dof_names = {'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'};
        fprintf('       Node %d, %s = %.1f N/Nm\n', node_id, dof_names{dof}, force);
    end
end

%% 3. Test materials and pipe catalog

fprintf('\n3. Testing materials and pipe catalog loading...\n');

materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

fprintf('   ✓ Loaded %d materials\n', length(fieldnames(materials)));
fprintf('   ✓ Loaded %d pipe sizes\n', length(catalog));

% Check for required materials
assert(isfield(materials, 'AISI4130'), 'Missing material: AISI4130');
assert(isfield(materials, 'Al6061T6'), 'Missing material: Al6061T6');

fprintf('   ✓ Required materials present (AISI4130, Al6061T6)\n');

%% 4. Validate exclusion zone data types

fprintf('\n4. Validating exclusion zone data types...\n');

for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);

    % Check type
    assert(isfield(zone, 'type'), 'Zone %d missing type', i);
    assert(strcmp(zone.type, 'AABB'), 'Zone %d type must be AABB', i);

    % Check bounds
    assert(isfield(zone, 'min'), 'Zone %d missing min', i);
    assert(isfield(zone, 'max'), 'Zone %d missing max', i);
    assert(length(zone.min) == 3, 'Zone %d min must be 3D', i);
    assert(length(zone.max) == 3, 'Zone %d max must be 3D', i);

    % Check name
    assert(isfield(zone, 'name'), 'Zone %d missing name', i);
end

fprintf('   ✓ All exclusion zones have valid structure\n');

%% 5. Summary

fprintf('\n=== ALL TESTS PASSED ===\n');
fprintf('\nReady to run full demo:\n');
fprintf('  >> demo_real_vehicle_frame\n');
fprintf('\nTo enable real vehicle in optimization:\n');
fprintf('  >> edit demo_full_pipeline.m\n');
fprintf('  >> % Set use_real_vehicle = true;\n');
