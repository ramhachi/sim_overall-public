%% Diagnostic: Check what frameObjectiveFunction sees
clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..', 'matlab');
data_dir = fullfile(script_dir, '..', 'data');
addpath(matlab_dir);

geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
project_root = fullfile(script_dir, '..', '..');
suspension_file = resolve_suspension_geometry_source(project_root);

Z_floor = 0.050;
for i = 1:length(geom.exclusion_zones)
    geom.exclusion_zones(i).min(3) = geom.exclusion_zones(i).min(3) + Z_floor;
    geom.exclusion_zones(i).max(3) = geom.exclusion_zones(i).max(3) + Z_floor;
end

% Use baseline parameters
shape_params = [0.95, 0.35, 0.45+Z_floor, 0.35];
node_toggles = [1, 1];
node_ratios = [0.5, 0.5, 0.5, 0.5];

[nodes, ~, ~, ~, fh_x, mh_x, fbh_idx, susp_idx, rear_susp_idx] = ...
    generate_parametric_nodes(geom, suspension_file, shape_params, node_toggles, node_ratios);

fprintf('Total nodes: %d\n', size(nodes, 1));
fprintf('FBH idx: [%s]\n', num2str(fbh_idx));
fprintf('Front susp idx: [%s]\n', num2str(susp_idx));
fprintf('Rear susp idx: [%s]\n', num2str(rear_susp_idx));
fprintf('FH X: %.3f, MH X: %.3f\n', fh_x, mh_x);

% Check rear susp indexing
if ~isempty(rear_susp_idx)
    fprintf('\nRear susp mount positions:\n');
    for i = 1:length(rear_susp_idx)
        n = rear_susp_idx(i);
        fprintf('  Node %d: [%.3f, %.3f, %.3f]\n', n, nodes(n,:));
    end
    
    rear_inner_start = rear_susp_idx(end) + 1;
    rear_inner_end = rear_inner_start + length(rear_susp_idx) - 1;
    fprintf('\nRear inner brackets (expected %d-%d):\n', rear_inner_start, rear_inner_end);
    for i = rear_inner_start:min(rear_inner_end, size(nodes,1))
        fprintf('  Node %d: [%.3f, %.3f, %.3f]\n', i, nodes(i,:));
    end
end

% Generate BCs as the objective function would
node_config = struct();
if ~isempty(rear_susp_idx)
    rear_left = rear_susp_idx([1, 3, 5, 7]);
    rear_right = rear_susp_idx([2, 4, 6, 8]);
    node_config.rear_susp_left_nodes = rear_left;
    node_config.rear_susp_right_nodes = rear_right;
    node_config.rear_lower_nodes = [rear_susp_idx(1), rear_susp_idx(2)];
    node_config.rear_upper_nodes = [rear_susp_idx(3), rear_susp_idx(4)];
    fprintf('\nBC fixed nodes (rear_susp_left):  [%s]\n', num2str(rear_left));
    fprintf('BC fixed nodes (rear_susp_right): [%s]\n', num2str(rear_right));
else
    node_config.rear_lower_nodes = [9, 10];
    node_config.rear_upper_nodes = [13, 14];
    fprintf('\nBC fallback: rear_lower=[9,10], rear_upper=[13,14]\n');
end

node_config.front_susp_left_nodes = susp_idx([1, 3, 5, 7]);
node_config.front_susp_right_nodes = susp_idx([2, 4, 6, 8]);
node_config.total_mass_kg = geom.dimensions.total_mass_kg;

fprintf('Load nodes (front_left):  [%s]\n', num2str(node_config.front_susp_left_nodes));
fprintf('Load nodes (front_right): [%s]\n', num2str(node_config.front_susp_right_nodes));

cases = generate_load_cases(nodes, node_config.total_mass_kg, 9.81, node_config);
fprintf('\nTorsion case:\n');
fprintf('  BCs: %d constraints\n', size(cases(1).bcs, 1));
fprintf('  Loads: %d forces\n', size(cases(1).loads, 1));
fprintf('  BC nodes: [%s]\n', num2str(unique(cases(1).bcs(:,1))'));
fprintf('  Load nodes: [%s]\n', num2str(unique(cases(1).loads(:,1))'));

% Check topology
distance_thresholds = [0.6, 0.65, 0.6];
config = struct();
config.exclusion_zones = {};
elements_geom = generate_adaptive_topology(nodes, fbh_idx, susp_idx, fh_x, mh_x, config, distance_thresholds, rear_susp_idx);

fprintf('\nTopology: %d elements\n', size(elements_geom.connectivity, 1));

% Check if BC/load nodes appear in elements
all_elem_nodes = unique([elements_geom.connectivity(:,1); elements_geom.connectivity(:,2)]);
bc_nodes = unique(cases(1).bcs(:,1));
load_nodes = unique(cases(1).loads(:,1));

bc_connected = ismember(bc_nodes, all_elem_nodes);
load_connected = ismember(load_nodes, all_elem_nodes);

fprintf('\nBC nodes connected to topology:\n');
for i = 1:length(bc_nodes)
    fprintf('  Node %d: %s\n', bc_nodes(i), string(bc_connected(i)));
end
fprintf('Load nodes connected to topology:\n');
for i = 1:length(load_nodes)
    fprintf('  Node %d: %s\n', load_nodes(i), string(load_connected(i)));
end

% Try FEM solve
fprintf('\nStarting FEM solver...\n');
beam_fem_solver_tcp_java('start');
pause(1);

elements_fem = elements_geom.base_elements;
result = beam_fem_solver_tcp_java(nodes, elements_fem, cases(1).bcs, cases(1).loads);
fprintf('FEM result: K_torsion = %.2f Nm/deg, mass = %.2f kg, success = %d\n', ...
    result.K_torsion, result.mass, result.success);

beam_fem_solver_tcp_java('stop');
