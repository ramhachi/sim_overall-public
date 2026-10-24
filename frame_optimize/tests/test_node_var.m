script_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'examples');
addpath(script_dir);
addpath(fullfile(script_dir, '..', 'matlab'));

data_dir = fullfile(script_dir, '..', 'data');
geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
suspension_file = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));

% Test Case 1: Minimum bounds from GA setup
% shape_params = [0.50, 0.25, 0.35, 0.25]
% node_toggles = [0, 0]
% node_ratios = [0.2, 0.2, 0.2, 0.2]
nodes1 = generate_parametric_nodes(geom, suspension_file, [0.50, 0.25, 0.35, 0.25], [0, 0], [0.2, 0.2, 0.2, 0.2]);

% Test Case 2: Maximum bounds from GA setup
% shape_params = [0.80, 0.45, 0.55, 0.75]
% node_toggles = [1, 1]
% node_ratios = [0.8, 0.8, 0.8, 0.8]
nodes2 = generate_parametric_nodes(geom, suspension_file, [0.80, 0.45, 0.55, 0.75], [1, 1], [0.8, 0.8, 0.8, 0.8]);

fprintf('Nodes 1 count: %d\n', size(nodes1, 1));
fprintf('Nodes 1 FH top (Node 3): X=%.2f, Y=%.2f, Z=%.2f\n', nodes1(3,1), nodes1(3,2), nodes1(3,3));
fprintf('Nodes 1 SI top (Node 7): X=%.2f, Y=%.2f, Z=%.2f\n', nodes1(7,1), nodes1(7,2), nodes1(7,3));

fprintf('Nodes 2 count: %d\n', size(nodes2, 1));
fprintf('Nodes 2 FH top (Node 3): X=%.2f, Y=%.2f, Z=%.2f\n', nodes2(3,1), nodes2(3,2), nodes2(3,3));
fprintf('Nodes 2 SI top (Node 7): X=%.2f, Y=%.2f, Z=%.2f\n', nodes2(7,1), nodes2(7,2), nodes2(7,3));

diff = abs(nodes1(1:23,:) - nodes2(1:23,:));
max_diff = max(diff(:));
fprintf('Maximum displacement of base nodes between two extremes: %.3f m\n', max_diff);
