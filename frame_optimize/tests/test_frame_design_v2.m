fprintf('=== Frame design V2 schema/generator test ===\n');

test_dir = fileparts(mfilename('fullpath'));
frame_root = fullfile(test_dir, '..');
project_root = fullfile(frame_root, '..');
addpath(fullfile(frame_root, 'matlab'));
addpath(fullfile(project_root, 'matlab'));

data_dir = fullfile(frame_root, 'data');
fixture_file = fullfile(test_dir, 'fixtures', 'canonical_suspension_fixture.json');
geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
artifact = load_vehicle_artifact(fixture_file);

schema = frame_design_schema_v2(8);
assert(strcmp(schema.schema_version, 'frame_design/v2'));
assert(numel(schema.initial) == schema.num_variables);
assert(any(strcmp(schema.group_names, 'mh_bracing')));
assert(any(strcmp(schema.group_names, 'mh_brace_support')));
assert(any(strcmp(schema.group_names, 'shoulder_bar')));

design = decode_frame_design_vector(schema.initial, schema);
assert(strcmp(design.schema_version, 'frame_design/v2'));
assert(isfield(design.geometry, 'fbh_half_width_m'));
assert(isfield(design.geometry, 'mh_brace_end_bottom_half_width_m'));

diff_config = load_diff_mount_config(artifact.params, geom);
assert(~isempty(diff_config.diff_mount_points), 'Fixture must include diff mount points.');

config = struct();
config.diff_mount_points = diff_config.diff_mount_points;
config.diff_envelope = diff_config.diff_envelope;
config.exclusion_zones = {};

[nodes, ~, ~, ~, ~, ~, ~, ~, rear_susp_idx, meta] = ...
    generate_parametric_nodes_v2(geom, fixture_file, design, config);
assert(size(nodes, 2) == 3);
assert(numel(meta.brace_trapezoid_idx) == 4);
assert(~isempty(meta.diff_mount_idx));
assert(~isempty(rear_susp_idx));
assert(all(nodes(meta.diff_mount_idx, 1) >= meta.X_brace_end));
assert(isfield(meta, 'design_adjustments'));

adjusted_design = design;
adjusted_design.geometry.shoulder_bar_z_m = 0.0;
[~, ~, ~, ~, ~, ~, ~, ~, ~, adjusted_meta] = ...
    generate_parametric_nodes_v2(geom, fixture_file, adjusted_design, config);
assert(isfield(adjusted_meta.design_adjustments, 'shoulder_bar_z_m'), ...
    'Clamped shoulder_bar_z_m must be recorded, not silently applied.');

elements_geom = generate_adaptive_topology_v2(nodes, meta, config, design.distance_thresholds);
assert(any(strcmp(elements_geom.group_names, 'mh_bracing')));
assert(any(elements_geom.group_indices == find(strcmp(elements_geom.group_names, 'mh_bracing'), 1)));
assert(any(elements_geom.group_indices == find(strcmp(elements_geom.group_names, 'mh_brace_support'), 1)));
assert(any(elements_geom.group_indices == find(strcmp(elements_geom.group_names, 'shoulder_bar'), 1)));

materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
reg_params = jsondecode(fileread(fullfile(data_dir, 'regulation_params.json')));
frame_struct = struct();
frame_struct.nodes = nodes;
frame_struct.elements = elements_geom.base_elements;
frame_struct.element_groups = struct();
for gi = 1:numel(elements_geom.group_names)
    frame_struct.element_groups.(elements_geom.group_names{gi}) = ...
        find(elements_geom.group_indices == gi);
end
n_frame_elements = size(elements_geom.base_elements, 1);
frame_struct.element_materials = repmat({'AISI4130'}, n_frame_elements, 1);
[~, ~, compliance_report] = checkFrameCompliance(frame_struct, catalog, materials, reg_params);
assert(isempty(compliance_report.section_violations), ...
    'Initial V2 topology must satisfy F.6 group counts.');
assert(isempty(compliance_report.driver_template_violations), ...
    'Initial V2 topology must satisfy the cockpit template proxy.');

missing_support = frame_struct;
missing_support.element_groups.fbh_support = [];
[~, ~, missing_report] = checkFrameCompliance(missing_support, catalog, materials, reg_params);
assert(~isempty(missing_report.section_violations), ...
    'Missing FBH support members must fail the F.6 section check.');

[pass, penalty, report] = check_frame_geometry_constraints_v2(nodes, meta, design, config, elements_geom);
assert(pass, 'V2 initial geometry should pass cheap checks: %s', evalc('disp(report)'));
assert(penalty == 0);
assert(report.metrics.main_hoop_centerline_width_m >= 0.380);
assert(report.metrics.mh_brace_angle_deg >= 30.0);

node_config = struct();
node_config.rear_susp_left_nodes = rear_susp_idx([1, 3, 5, 7]);
node_config.rear_susp_right_nodes = rear_susp_idx([2, 4, 6, 8]);
node_config.rear_lower_nodes = [rear_susp_idx(1), rear_susp_idx(2)];
node_config.rear_upper_nodes = [rear_susp_idx(3), rear_susp_idx(4)];
node_config.front_susp_left_nodes = meta.susp_idx([1, 3, 5, 7]);
node_config.front_susp_right_nodes = meta.susp_idx([2, 4, 6, 8]);
node_config.diff_mount_nodes = meta.diff_mount_idx;
cases = generate_load_cases(nodes, geom.dimensions.total_mass_kg, 9.81, node_config);
case_names = {cases.name};
assert(any(strcmp(case_names, 'torsion')));
assert(any(strcmp(case_names, 'lateral_bending')));
assert(any(strcmp(case_names, 'longitudinal_bending')));
assert(any(strcmp(case_names, 'rollover')));
assert(any(strcmp(case_names, 'diff_reaction')));

lat_case = cases(strcmp(case_names, 'lateral_bending'));
lat_total = sum(lat_case.loads(lat_case.loads(:, 2) == 2, 3));
assert(abs(abs(lat_total) - geom.dimensions.total_mass_kg * 1.5 * 9.81) < 1e-6, ...
    'Lateral load must be the full vehicle inertial force.');

roll_case = cases(strcmp(case_names, 'rollover'));
roll_lat = sum(roll_case.loads(roll_case.loads(:, 2) == 2, 3));
roll_vert = sum(roll_case.loads(roll_case.loads(:, 2) == 3, 3));
roll_expected = geom.dimensions.total_mass_kg * 1.5 * 9.81;
assert(abs(abs(roll_lat) - roll_expected) < 1e-6, 'Rollover lateral load mismatch.');
assert(abs(abs(roll_vert) - roll_expected) < 1e-6, 'Rollover vertical load mismatch.');

missing_diff_failed = false;
try
    generate_parametric_nodes_v2(geom, fixture_file, design, struct());
catch ME
    missing_diff_failed = strcmp(ME.identifier, 'generate_parametric_nodes_v2:MissingDiffMountConfig');
end
assert(missing_diff_failed, 'V2 generator must fail fast without diff mount config.');

% Struct arrays (e.g. suspension.kc.corners) must not expand to a
% comma-separated list when nested artifacts are merged.
merge_a = struct('suspension', struct('kc', struct('corners', ...
    struct('x', {1, 2}))));
merge_b = struct('suspension', struct('kc', struct('corners', ...
    struct('x', {3, 4}))));
merge_out = merge_nested_structs(merge_a, merge_b);
assert(merge_out.suspension.kc.corners(1).x == 3);
assert(merge_out.suspension.kc.corners(2).x == 4);

fprintf('Frame design V2 schema/generator OK\n');
