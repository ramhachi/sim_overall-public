fprintf('=== Canonical frame objective smoke test ===\n');
% Initialize small test to check if the canonical parametric objective runs
test_dir = fileparts(mfilename('fullpath'));
script_dir = fullfile(test_dir, '..', 'examples');
project_root = fullfile(script_dir, '..', '..');
addpath(script_dir);
addpath(fullfile(script_dir, '..', 'matlab'));

data_dir = fullfile(script_dir, '..', 'data');
geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

fixture_file = fullfile(test_dir, 'fixtures', 'canonical_suspension_fixture.json');
assert(isfile(fixture_file), 'Missing local suspension fixture: %s', fixture_file);
suspension_file = resolve_suspension_geometry_source(project_root, fixture_file);
assert(strcmp(suspension_file, fixture_file), ...
    'Expected canonical resolver to select the checked-in suspension fixture.');
fprintf('Using checked-in suspension fixture: %s\n', suspension_file);

Z_floor = 0.050;
for i = 1:length(geom.exclusion_zones)
    geom.exclusion_zones(i).min(3) = geom.exclusion_zones(i).min(3) + Z_floor;
    geom.exclusion_zones(i).max(3) = geom.exclusion_zones(i).max(3) + Z_floor;
end

config = struct();
config.W_mass = 5.0;
config.W_cg = 1.0;
config.mass_ref = 22.0;
config.cg_ref = 0.300;
config.K_min = 500;
config.K_target = 1200;
config.gamma_below = 8.0;
config.gamma_hard = 50.0;
config.joint_factor = 0.85;
config.max_cg_z = 0.380;
config.geom = geom;
config.suspension_file = suspension_file;
config.require_rear_suspension_hardpoints = true;
config.require_total_mass_kg = true;
config.exclusion_zones = {};
for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);
    exclusion = struct();
    if isfield(zone, 'type'), exclusion.type = zone.type; else, exclusion.type = 'AABB'; end
    if isfield(zone, 'min')
        exclusion.min_bounds = zone.min;
        exclusion.max_bounds = zone.max;
    else
        exclusion.min_bounds = zone.min_bounds;
        exclusion.max_bounds = zone.max_bounds;
    end
    if isfield(zone, 'name'), exclusion.name = zone.name; else, exclusion.name = sprintf('Zone_%d', i); end
    config.exclusion_zones{end+1} = exclusion;
end

[~, init_fbh, ~, ~, X_fh_ref, X_mh_ref] = generate_parametric_nodes( ...
    geom, suspension_file, [0.95, 0.35, 0.45+Z_floor, 0.35], [1, 1], [0.5, 0.5, 0.5, 0.5]);
X_fbh_ref = mean(init_fbh(:, 1));
W_fbh_ref = max(abs(init_fbh(:, 2)));
Z_fbh_min = min(init_fbh(:, 3));
Z_fbh_max = max(init_fbh(:, 3));
M = 0.030;

cockpit_lower = struct();
cockpit_lower.type = 'AABB';
cockpit_lower.min_bounds = [X_fbh_ref + M, -(W_fbh_ref - M), Z_fbh_min + M];
cockpit_lower.max_bounds = [X_fh_ref - M, W_fbh_ref - M, Z_fbh_max - M];
cockpit_lower.name = 'Cockpit Lower (FBH-FH Legs)';
config.exclusion_zones{end+1} = cockpit_lower;

W_fh_ref = 0.250;
H_si_upper_ref = 0.45 + Z_floor;
cockpit_upper = struct();
cockpit_upper.type = 'AABB';
cockpit_upper.min_bounds = [X_fh_ref + M, -(W_fh_ref - M), H_si_upper_ref + M];
cockpit_upper.max_bounds = [X_mh_ref - M, W_fh_ref - M, 1.20];
cockpit_upper.name = 'Cockpit Upper (FH-MH Driver)';
config.exclusion_zones{end+1} = cockpit_upper;

reg_params = jsondecode(fileread(fullfile(data_dir, 'regulation_params.json')));

% Test variables
x = [5, 5, 5, 5, 5, 0.70, 0.35, 0.45+Z_floor, 0.35, 0.6, 0.65, 0.6, 1, 1, 0.5, 0.5, 0.5, 0.5];

% Reuse an already-running FEM server; only start/stop one we own.
cleanup_obj = [];
if ~beam_fem_server_manager('check_alive')
    beam_fem_solver_tcp_java('start');
    cleanup_obj = onCleanup(@() beam_fem_solver_tcp_java('stop')); %#ok<NASGU>
end

% Run canonical objective once
[obj_val, res] = frameObjectiveFunction(x, [], [], [], config, catalog, materials, reg_params);
assert(res.success, 'Canonical frame objective should complete successfully.');
assert(isfield(res, 'cg_z'), 'Canonical frame objective should return cg_z.');
assert(isfield(res, 'P_stiff'), 'Canonical frame objective should return P_stiff.');

fprintf('\n=== Test Result ===\n');
fprintf('Obj Val: %.4f\n', obj_val);
fprintf('CG Height: %.4f m\n', res.cg_z);
fprintf('Mass: %.2f kg\n', res.mass);
fprintf('K_torsion: %.1f Nm/deg\n', res.K_torsion);
fprintf('f_mass: %.4f\n', res.f_mass);
fprintf('f_cg: %.4f\n', res.f_cg);
fprintf('P_stiff: %.4f\n', res.P_stiff);
disp('Canonical path OK');
