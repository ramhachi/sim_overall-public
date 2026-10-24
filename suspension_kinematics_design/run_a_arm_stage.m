function run_a_arm_stage(axleKey, json_file, run_options)
% RUN_A_ARM_STAGE Runtime-driven double-wishbone inverse design for one axle.
% Shared core of main_a_arm_optimizer (front) and main_rear_a_arm_optimizer.
%
% run_options: rng_seed (42), n_starts (50), allow_poor_fit (false) plus the
% resolve_axle_geometry overrides. Design-space constants come from
% suspension_design_space(params, axleKey).

isFront = strcmpi(axleKey, 'front');
if isFront
    idx = [1, 2];
    names = {'FL', 'FR'};
    stageScript = 'suspension_kinematics_design/main_a_arm_optimizer.m';
else
    idx = [3, 4];
    names = {'RL', 'RR'};
    stageScript = 'suspension_kinematics_design/main_rear_a_arm_optimizer.m';
end

fprintf('================================================================\n');
fprintf(' A-Arm Kinematics Optimizer (%s)\n', axleKey);
fprintf('================================================================\n');

script_dir = fileparts(mfilename('fullpath'));
project_matlab_dir = fullfile(script_dir, '..', 'matlab');
if exist('load_vehicle_artifact', 'file') ~= 2
    addpath(project_matlab_dir);
end

if nargin < 3 || isempty(run_options)
    run_options = struct();
end
defaults = struct('rng_seed', 42, 'n_starts', 50, 'allow_poor_fit', false);
for f = fieldnames(defaults)'
    if ~isfield(run_options, f{1})
        run_options.(f{1}) = defaults.(f{1});
    end
end
rng(run_options.rng_seed, 'twister');

if nargin < 2 || isempty(json_file)
    json_file = resolve_default_vehicle_json(fullfile(script_dir, '..'), 'RequireExportReady', true);
    if isempty(json_file)
        error('No vehicle artifact candidate found.');
    end
end
if ~isfile(json_file)
    error('Cannot find %s', json_file);
end

artifact = load_vehicle_artifact(json_file);
p = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
targets = build_inverse_targets(p, axleKey);
ds = suspension_design_space(p, axleKey);

axle = resolve_axle_geometry(p, run_options, axleKey);
W_C = axle.W_C;
R_tire = axle.R_tire;
static_camber = p.suspension.static_camber(idx);
static_camber_deg = rad2deg(static_camber(1));
if isFront
    track = p.mass.track_f;
else
    track = p.mass.track_r;
end

fprintf('Targets Loaded from JSON:\n');
fprintf('  Track Width:          %.3f m\n', track);
fprintf('  Static Camber:        %.2f deg\n', static_camber_deg);
fprintf('  Target Camber Gain:   %.2f rad/m\n', targets.camber_vs_travel(2));
fprintf('  Target Roll Center Z: %.3f m\n', targets.roll_center_z);
fprintf('  Loaded tire radius:   %.4f m (deflection %.1f mm)\n', axle.R_loaded, 1000 * axle.tire_deflection_m);

lb = ds.lb;
ub = ds.ub;
if isempty(ds.U_U_local_z_min_m)
    lb(7) = -R_tire;
else
    lb(7) = ds.U_U_local_z_min_m;
end
ub(7) = ds.L_U_local_z_max_m - ds.min_kingpin_z_sep_m;
ub(10) = ds.L_U_local_z_max_m;
lb([5, 8]) = -ds.upright_x_max_m;
ub([5, 8]) = ds.upright_x_max_m;
x0 = min(max(ds.x0, lb), ub);

hpIn = ds.hardpoints_input;
fixedBoth = isfield(hpIn, names{1}) && isfield(hpIn, names{2});
fixedLeft = isfield(hpIn, names{1});
use_cache = ~isfield(run_options, 'use_cache') || run_options.use_cache;
cost_cache = [];
if fixedLeft
    % Per-corner hardpoint input (CAD/measured): evaluate only (#45).
    x_opt = [];
    fval = NaN;
    exitflag = NaN;
    ms_solutions = struct('Exitflag', {});
    optimizer_elapsed_s = 0.0;
    optimizerName = 'none_fixed_hardpoints';
    geom_l = geom_from_hardpoints(hpIn.(names{1}), true, isFront, static_camber(1), R_tire, axle.R_loaded);
else
    if use_cache
        cost_cache = containers.Map('KeyType', 'char', 'ValueType', 'double');
    end
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
        'StepTolerance', 1e-12, 'ConstraintTolerance', 1e-6);
    obj_fun = @(x) cost_function(x, p, targets, axle, static_camber_deg, isFront, idx(1), ds, cost_cache);
    nonlcon_fun = @(x) physical_constraints(x, axle, static_camber_deg, isFront, ds);
    problem = createOptimProblem('fmincon', 'objective', obj_fun, 'x0', x0, ...
        'lb', lb, 'ub', ub, 'options', options, 'nonlcon', nonlcon_fun);
    ms = MultiStart('Display', 'iter', 'UseParallel', false);
    fprintf('\nRunning Runtime-Driven %s Geometry Optimization (MultiStart)...\n', axleKey);
    t_opt_start = tic;
    [x_opt, fval, exitflag, ~, ms_solutions] = run(ms, problem, run_options.n_starts);
    optimizer_elapsed_s = toc(t_opt_start);
    if exitflag <= 0
        error('%s A-arm optimization did not converge (MultiStart exitflag = %d).', axleKey, exitflag);
    end
    optimizerName = 'MultiStart/fmincon/sqp';
    geom_l = build_wishbone_geom(x_opt, axle, static_camber_deg, true, isFront, ds);
end
if ~isFront && ~isfield(geom_l, 'tie_rod_chassis')
    geom_l = add_zero_bump_steer_toe_link(geom_l, ds);
end
if fixedBoth
    geom_r = geom_from_hardpoints(hpIn.(names{2}), false, isFront, static_camber(2), R_tire, axle.R_loaded);
    if ~isFront && ~isfield(geom_r, 'tie_rod_chassis')
        geom_r = add_zero_bump_steer_toe_link(geom_r, ds);
    end
    symmetry = struct('mode', 'per_corner_input', 'left_right_hardpoints', 'independent', ...
        'asymmetric_static_alignment_supported', true, ...
        'description', 'Both corners supplied in design_space.hardpoints_input; no mirroring.');
else
    geom_r = mirror_wishbone_geometry(geom_l);
    symmetry = struct('mode', 'mirrored', 'left_right_hardpoints', 'exact_y_mirror', ...
        'asymmetric_static_alignment_supported', true, ...
        'description', 'Left corner is optimized/supplied; right hardpoints are an exact Y mirror. Static toe/camber remain per-corner.');
end

% Export-grade dense sweep (#40) on the final geometry only.
dense = kc_dense_samples(p, ds);
metrics_l = evaluate_metrics(geom_l, p, idx(1), isFront, dense);
metrics_r = evaluate_metrics(geom_r, p, idx(2), isFront, dense);
if ~metrics_l.converged || ~metrics_r.converged
    error('Kinematics solve did not converge for the %s geometry.', axleKey);
end
fitReasons = {};
if ~metrics_l.polyfit_quality.fit_ok || ~metrics_r.polyfit_quality.fit_ok
    if ~run_options.allow_poor_fit
        error('KC:PoorPolyfit', ['%s quadratic K&C fits exceed tolerance on the dense sweep ', ...
            '(polyfit_quality.fit_ok=false). Use the exported sample_values LUT, revise the ', ...
            'geometry, or pass run_options.allow_poor_fit=true to save as provisional.'], axleKey);
    end
    warning('KC:PoorPolyfit', 'Saving %s K&C with poor polynomial fit as provisional.', axleKey);
end

fprintf('\nFinal Performance:\n');
fprintf('  Roll Center Z:   %.3f m (Target: %.3f)\n', metrics_l.roll_center_z, targets.roll_center_z);
fprintf('  KPI:             %.2f deg (Target: %.2f)\n', metrics_l.kpi_deg, targets.kpi_deg);
fprintf('  Caster:          %.2f deg (Target: %.2f)\n', metrics_l.caster_deg, targets.caster_deg);
fprintf('  Scrub Radius:    %.3f m (Target: %.3f)\n', metrics_l.scrub_radius_m, targets.scrub_radius_m);
fprintf('  Mech Trail:      %.3f m (Target: %.3f)\n', metrics_l.mechanical_trail_m, targets.mechanical_trail_m);
fprintf('  Camber Gain:     %.2f rad/m (Target: %.2f)\n', metrics_l.camber_gain_rad_m, targets.camber_vs_travel(2));

params_update = p;
params_update.suspension.kc.(axleKey) = build_axle_kc(metrics_l, p.suspension.motion_ratio(idx), isFront);
params_update.suspension.kc.corners.(names{1}) = build_corner_kc(geom_l, metrics_l, ...
    p.suspension.static_toe(idx(1)), static_camber(1), stageScript, fitReasons);
params_update.suspension.kc.corners.(names{2}) = build_corner_kc(geom_r, metrics_r, ...
    p.suspension.static_toe(idx(2)), static_camber(2), stageScript, fitReasons);

rc_target = roll_center_frame(targets.roll_center_z);
rc_achieved = roll_center_frame(metrics_l.roll_center_z);
sg = struct( ...
    'schema_version', 'suspension_geometry/v1', ...
    'updated_at', datestr(now, 'yyyy-mm-dd HH:MM:SS'), ...
    'source_script', stageScript, ...
    'axle_geometry', struct('wheel_center', W_C(:)', 'tire_radius_m', R_tire, ...
        'loaded_tire_radius_m', axle.R_loaded, 'static_fz_n', axle.static_fz_n, ...
        'arm_x_offset_m', axle.arm_x_offset, 'source', axle.source), ...
    'symmetry_contract', symmetry, ...
    'optimizer', optimizerName, ...
    'solver_options', struct('Algorithm', 'sqp', 'StepTolerance', 1e-12, ...
        'ConstraintTolerance', 1e-6, 'MultiStartDisplay', 'iter', 'UseParallel', false), ...
    'code_version', get_code_version(project_matlab_dir), ...
    'n_starts', run_options.n_starts, ...
    'random_seed', run_options.rng_seed, ...
    'optimizer_cost', fval, ...
    'optimizer_exitflag', exitflag, ...
    'optimizer_diagnostics', struct( ...
        'local_exitflags', [ms_solutions.Exitflag], ...
        'n_local_starts', numel(ms_solutions), ...
        'n_local_converged', sum([ms_solutions.Exitflag] > 0), ...
        'elapsed_s', optimizer_elapsed_s, ...
        'evaluation_cache_entries', cache_count(cost_cache), ...
        'evaluation_cache_enabled', use_cache), ...
    'design_vector', x_opt(:)', ...
    'bounds_lb', lb, ...
    'bounds_ub', ub, ...
    'design_space', rmfield(ds, 'hardpoints_input'), ...
    'dense_sampling', struct('travel_knots', numel(dense.TravelSamples), ...
        'roll_knots', numel(dense.RollSamples), 'steer_knots', numel(dense.SteerSamples)), ...
    'targets', merge_nested_structs(geometry_summary(targets), rc_target), ...
    'achieved', merge_nested_structs(geometry_summary(metrics_l), rc_achieved));

[~, final_cost_terms] = evaluate_inverse_cost(metrics_l, targets, axleKey);
stageResult = merge_nested_structs(struct( ...
    'objective_value', fval, ...
    'cost_terms', final_cost_terms, ...
    'kpi_deg', metrics_l.kpi_deg, ...
    'caster_deg', metrics_l.caster_deg, ...
    'scrub_radius_m', metrics_l.scrub_radius_m, ...
    'mechanical_trail_m', metrics_l.mechanical_trail_m, ...
    'camber_gain_rad_per_m', metrics_l.camber_gain_rad_m), rc_achieved);

target_values = rmfield(targets, {'weights', 'tolerances', 'weight_policy', 'sources', ...
    'source_quality', 'quality', 'fallback_fields', 'provisional', 'model_version', 'model_scope'});
sg.target_contract = struct( ...
    'targets', target_values, ...
    'weights', targets.weights, ...
    'tolerances', targets.tolerances, ...
    'weight_policy', targets.weight_policy, ...
    'sources', targets.sources, ...
    'source_quality', targets.source_quality, ...
    'quality', targets.quality, ...
    'fallback_fields', {targets.fallback_fields}, ...
    'provisional', targets.provisional, ...
    'model_version', targets.model_version, ...
    'model_scope', targets.model_scope);
sg.constraints = struct( ...
    'max_kpi_deg', ds.max_kpi_deg, 'max_caster_deg', ds.max_caster_deg, 'units', 'deg', ...
    'min_arm_length_m', ds.min_arm_length_m, 'max_arm_length_m', ds.max_arm_length_m, ...
    'min_inboard_clearance_m', ds.min_inboard_clearance_m, ...
    'wheel_envelope_margin_m', ds.wheel_envelope_margin_m, ...
    'definition', 'KPI in Y-Z plane; caster in X-Z plane');

suspensionGeometry = struct();
suspensionGeometry.(axleKey) = sg;
save_suspension_geometry_artifact( ...
    'BaseArtifactPath', json_file, ...
    'InPlace', true, ...
    'CurrentParams', params_update, ...
    'SuspensionGeometry', suspensionGeometry, ...
    'StageName', ['suspension_geometry_' axleKey], ...
    'StageOptimizer', 'multistart_fmincon', ...
    'StageResult', stageResult);

fprintf('\n%s suspension geometry appended to %s\n', axleKey, json_file);
end

function s = geometry_summary(m)
% Canonical unit-suffixed summary of either a targets or a metrics struct.
% Roll-centre fields come from roll_center_frame (Z-down and height).
if isfield(m, 'camber_gain_rad_m')
    camberGain = m.camber_gain_rad_m;
else
    camberGain = m.camber_vs_travel(2);
end
s = struct( ...
    'kpi_deg', m.kpi_deg, ...
    'caster_deg', m.caster_deg, ...
    'camber_gain_rad_per_m', camberGain, ...
    'scrub_radius_m', m.scrub_radius_m, ...
    'mechanical_trail_m', m.mechanical_trail_m, ...
    'roll_center_height_per_travel_m_per_m', -m.roll_center_height_per_travel, ...
    'roll_center_height_per_roll_m_per_rad', -m.roll_center_height_per_roll, ...
    'jacking_force_per_fy_n_per_n', m.jacking_force_per_fy);
end

function geom = build_wishbone_geom(x, axle, static_camber_deg, is_left, is_front, ds)
W_C = axle.W_C;
off = axle.arm_x_offset;
U_C_y = x(1); U_C_z = x(2);
L_C_y = x(3); L_C_z = x(4);
U_U_loc = [x(5); x(6); x(7)];
L_U_loc = [x(8); x(9); x(10)];

c = cosd(static_camber_deg);
s = sind(static_camber_deg);
R_camb = [1, 0, 0; 0, c, s; 0, -s, c];

U_U = W_C + R_camb * U_U_loc;
L_U = W_C + R_camb * L_U_loc;
dzu = off * ds.upper_axis_pitch_tan;
dzl = off * ds.lower_axis_pitch_tan;
U_F = [W_C(1) + off; U_C_y; U_C_z + dzu];
U_R = [W_C(1) - off; U_C_y; U_C_z - dzu];
L_F = [W_C(1) + off; L_C_y; L_C_z + dzl];
L_R = [W_C(1) - off; L_C_y; L_C_z - dzl];

geom = struct();
geom.U_F = U_F; geom.U_R = U_R; geom.U_U = U_U;
geom.L_F = L_F; geom.L_R = L_R; geom.L_U = L_U;
geom.W_C = W_C;
geom.R_tire = axle.R_tire;
geom.R_loaded = axle.R_loaded;
geom.is_left = is_left;
geom.is_front = is_front;
spindle_local = is_left * [0; -1; 0] + (~is_left) * [0; 1; 0];
geom.Spindle_Dir = R_camb * spindle_local;
geom.upper_arm_axis = (U_R - U_F) / norm(U_R - U_F);
geom.upper_arm_axis_origin = U_F;
geom.lower_arm_axis = (L_R - L_F) / norm(L_R - L_F);
geom.lower_arm_axis_origin = L_F;
geom.kingpin_axis.origin = L_U;
geom.kingpin_axis.direction = (U_U - L_U) / norm(U_U - L_U);
end

function geom = add_zero_bump_steer_toe_link(geom, ds)
% Rear toe link (#42). Outer point at a fixed offset from the lower ball
% joint; inner point from the classic front-view construction: on the line
% from the front-view IC through the outer point, where it crosses the line
% through the inboard pivots (zero first-order bump steer). Same X (link
% parallel to Y in plan view at static).
outer = geom.L_U + [ds.toe_link_outer_dx_m; 0; ds.toe_link_outer_dz_m];
uc = (geom.U_F + geom.U_R) / 2;
lc = (geom.L_F + geom.L_R) / 2;
yz = @(p) p(2:3);
dUp = yz(geom.U_U) - yz(uc);
dLow = yz(geom.L_U) - yz(lc);
A = [dUp, -dLow];
if abs(det(A)) < 1e-9
    dirIc = dUp;                                    % IC at infinity
else
    s = A \ (yz(lc) - yz(uc));
    dirIc = yz(outer) - (yz(uc) + s(1) * dUp);
end
dPiv = yz(lc) - yz(uc);
t = [dirIc, -dPiv] \ (yz(uc) - yz(outer));
inner = [outer(1); yz(outer) + t(1) * dirIc];
geom.tie_rod_chassis = inner;
geom.tie_rod_upright = outer;
end

function metrics = evaluate_metrics(geom, params, wheelIdx, isFront, samples)
if isFront
    track = params.mass.track_f;
    cgToAxle = params.mass.a;
else
    track = params.mass.track_r;
    cgToAxle = params.mass.b;
end
args = {};
if nargin >= 5 && ~isempty(samples)
    args = namedargs2cell(samples);
end
if ~isFront
    % Rear toe link is chassis-fixed: no rack/steer sweep.
    args = [args, {'RackSamples', [], 'SteerSamples', 0.0}];
end
metrics = evaluate_double_wishbone_3d(geom, ...
    'TrackWidth', track, ...
    'Wheelbase', params.mass.wheelbase, ...
    'CGHeight', params.mass.h_cg, ...
    'CGToAxle', cgToAxle, ...
    'BrakeBiasFront', params.brake.balance, ...
    'DriveBiasRear', 1.0, ... % RWD: all drive torque at the rear axle
    'StaticToe', params.suspension.static_toe(wheelIdx), ...
    'StaticCamber', 0.0, ...
    'SteeringStiffness', params.steering.stiffness, ...
    'UseTieRod', isfield(geom, 'tie_rod_chassis'), ...
    args{:});
end

function cost = cost_function(x, params, targets, axle, static_camber_deg, isFront, wheelIdx, ds, cost_cache)
use_cache = isa(cost_cache, 'containers.Map');
cache_key = sprintf('%.12g|', x(:));
if use_cache && isKey(cost_cache, cache_key)
    cost = cost_cache(cache_key);
    return;
end
geom = build_wishbone_geom(x, axle, static_camber_deg, true, isFront, ds);
metrics = evaluate_metrics(geom, params, wheelIdx, isFront, []);
if isFront
    axleKey = 'front';
else
    axleKey = 'rear';
end
cost = evaluate_inverse_cost(metrics, targets, axleKey);
if metrics.IC_y > 0
    cost = cost + 1e5;
end
if ~metrics.converged
    cost = cost + 1e10;
end
if use_cache
    cost_cache(cache_key) = cost;
end
end

function [c, ceq] = physical_constraints(x, axle, static_camber_deg, isFront, ds)
geom = build_wishbone_geom(x, axle, static_camber_deg, true, isFront, ds);
W_C = axle.W_C;
[kpi_deg, caster_deg] = kingpin_angles((geom.L_U - geom.U_U) / norm(geom.L_U - geom.U_U));
max_upper_z = max(x(2), geom.U_U(3));
min_lower_z = min(x(4), geom.L_U(3));
upper_arm_length = norm(geom.U_U - (geom.U_F + geom.U_R) / 2);
lower_arm_length = norm(geom.L_U - (geom.L_F + geom.L_R) / 2);
upper_inboard = (geom.U_F(2) + geom.U_R(2)) / 2 - W_C(2);
lower_inboard = (geom.L_F(2) + geom.L_R(2)) / 2 - W_C(2);
c = [ ...
    kpi_deg - ds.max_kpi_deg; ...
    caster_deg - ds.max_caster_deg; ...
    ds.min_kingpin_z_sep_m - (geom.L_U(3) - geom.U_U(3)); ...
    (max_upper_z - min_lower_z) + ds.arm_clear_gap_m; ...
    norm(x(5:7)) - ds.max_upright_radius_m; ...
    norm(x(8:10)) - ds.max_upright_radius_m; ...
    ds.min_arm_length_m - upper_arm_length; ...
    upper_arm_length - ds.max_arm_length_m; ...
    ds.min_arm_length_m - lower_arm_length; ...
    lower_arm_length - ds.max_arm_length_m; ...
    ds.min_inboard_clearance_m - upper_inboard; ...
    ds.min_inboard_clearance_m - lower_inboard; ...
    norm(geom.U_U - W_C) - (axle.R_tire + ds.wheel_envelope_margin_m); ...
    norm(geom.L_U - W_C) - (axle.R_tire + ds.wheel_envelope_margin_m) ...
];
ceq = [];
end

function n = cache_count(cache)
if isempty(cache)
    n = 0;
else
    n = cache.Count;
end
end
