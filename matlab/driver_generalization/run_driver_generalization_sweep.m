function analysis = run_driver_generalization_sweep(config)
% RUN_DRIVER_GENERALIZATION_SWEEP Evaluate driver robustness across vehicle specs.
%
% This sweep is intentionally driver-focused. It varies plant parameters
% that affect flexible-body, steering-torque control and records both lap
% performance and driver-health metrics such as steering lag, saturation,
% frame twist, and off-track failures.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_project_paths(config.project_root);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);
ensure_dir(config.params_output_dir);

base_params = resolve_base_params(config);
samples = build_samples(base_params, config);
events = normalize_events(config);

manifest = struct();
manifest.schema_version = 'driver_generalization_sweep/v1';
manifest.generated_by = 'run_driver_generalization_sweep';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.project_root = config.project_root;
manifest.runtime_output_dir = config.runtime_output_dir;
manifest.report_output_dir = config.report_output_dir;
manifest.samples_csv_path = fullfile(config.runtime_output_dir, 'samples.csv');
manifest.summary_csv_path = fullfile(config.runtime_output_dir, 'summary.csv');
manifest.report_path = fullfile(config.report_output_dir, sprintf('driver_generalization_%s.md', config.run_id));
manifest.params_output_dir = config.params_output_dir;
manifest.base_params = base_params;
manifest.samples = samples;
manifest.events = events;
write_json(fullfile(config.runtime_output_dir, 'manifest.json'), manifest);

completed = read_completed_keys(manifest.samples_csv_path);
rows = {};

for sample_idx = 1:numel(samples)
    sample = samples(sample_idx);
    params = apply_sample(base_params, sample);
    for event_idx = 1:numel(events)
        event = events(event_idx);
        key = make_key(sample.sample_id, event.name);
        if isKey(completed, key)
            continue;
        end

        params_file = fullfile(config.params_output_dir, sprintf('%03d_%s_%s.json', ...
            numel(completed) + numel(rows) + 1, sample.sample_id, event.name));
        write_json(params_file, params);

        started = tic;
        if ~isempty(config.execute_event_fn)
            [result, success] = config.execute_event_fn(params, event, sample, config);
        else
            [result, success] = execute_laptime(config, params_file, event);
        end
        runtime_sec = toc(started);

        row = build_row(config, sample, event, params_file, result, success, runtime_sec);
        append_row(manifest.samples_csv_path, row);
        completed(key) = true;
        rows{end + 1} = row; %#ok<AGROW>
    end
end

samples_table = read_samples_table(manifest.samples_csv_path);
summary = summarize_results(samples_table, samples, events, manifest.summary_csv_path);
write_report(manifest, summary, config);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = fullfile(config.runtime_output_dir, 'manifest.json');
analysis.samples_csv_path = manifest.samples_csv_path;
analysis.summary_csv_path = manifest.summary_csv_path;
analysis.report_path = manifest.report_path;
analysis.runtime_output_dir = config.runtime_output_dir;
analysis.executed_rows = numel(rows);
analysis.samples = samples;
analysis.events = events;
analysis.summary = summary;
end

function config = normalize_config(config)
if ~isfield(config, 'project_root') || isempty(config.project_root)
    this_file = mfilename('fullpath');
    config.project_root = fileparts(fileparts(fileparts(this_file)));
end
if ~isfield(config, 'run_id') || isempty(config.run_id)
    config.run_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~isfield(config, 'runtime_output_dir') || isempty(config.runtime_output_dir)
    config.runtime_output_dir = fullfile(config.project_root, 'output', 'runtime', ...
        'driver_generalization', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end
if ~isfield(config, 'params_output_dir') || isempty(config.params_output_dir)
    config.params_output_dir = fullfile(config.runtime_output_dir, 'params');
end
if ~isfield(config, 'driver_mode') || isempty(config.driver_mode)
    config.driver_mode = 'pid';
end
if ~isfield(config, 'events') || isempty(config.events)
    config.events = default_events();
end
if ~isfield(config, 'parameter_names') || isempty(config.parameter_names)
    config.parameter_names = default_parameter_names();
end
if ~isfield(config, 'factor_low') || isempty(config.factor_low)
    config.factor_low = 0.85;
end
if ~isfield(config, 'factor_high') || isempty(config.factor_high)
    config.factor_high = 1.15;
end
if ~isfield(config, 'base_params') || isempty(config.base_params)
    config.base_params = [];
end
if ~isfield(config, 'base_params_file') || isempty(config.base_params_file)
    config.base_params_file = '';
end
if ~isfield(config, 'force_flexible_torque_defaults') || isempty(config.force_flexible_torque_defaults)
    config.force_flexible_torque_defaults = true;
end
if ~isfield(config, 'execute_event_fn')
    config.execute_event_fn = [];
end
if ~isfield(config, 'extra_cli_args') || isempty(config.extra_cli_args)
    config.extra_cli_args = '';
end
end

function params = resolve_base_params(config)
if ~isempty(config.base_params)
    params = config.base_params;
    params = enforce_flexible_torque_defaults(params, config.force_flexible_torque_defaults);
    return;
end

if ~isempty(config.base_params_file)
    params = jsondecode(fileread(config.base_params_file));
else
    params = create_vehicle_baseline_template();
end
params = enforce_flexible_torque_defaults(params, config.force_flexible_torque_defaults);
end

function params = enforce_flexible_torque_defaults(params, force_defaults)
if ~isfield(params, 'suspension')
    params.suspension = struct();
end
if ~isfield(params, 'steering')
    params.steering = struct();
end

if force_defaults || ~isfield(params.suspension, 'K_chassis')
    params.suspension.K_chassis = 2000.0;
end
if force_defaults || ~isfield(params.suspension, 'C_chassis')
    params.suspension.C_chassis = 5.0;
end
if force_defaults || ~isfield(params.suspension, 'I_twist')
    params.suspension.I_twist = 1.5;
end

params.steering.torque_mode = true;
if force_defaults || ~isfield(params.steering, 'I_steer')
    params.steering.I_steer = 0.02;
end
if force_defaults || ~isfield(params.steering, 'R_steer')
    params.steering.R_steer = 4.5;
end
if force_defaults || ~isfield(params.steering, 'T_fric')
    params.steering.T_fric = 0.3;
end
if force_defaults || ~isfield(params.steering, 'C_steer_torque')
    params.steering.C_steer_torque = 0.3;
end
if force_defaults || ~isfield(params.steering, 'T_driver_max')
    params.steering.T_driver_max = 25.0;
end
end

function value = field_or(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default_value;
end
end

function row = backfill_intervention_breakdown(row)
if isnan(row.vehicle_health_penalty)
    row.vehicle_health_penalty = row.mean_chassis_twist_risk;
end

if isnan(row.control_intervention_cost) && ~isnan(row.driver_intervention_cost)
    row.control_intervention_cost = max(row.driver_intervention_cost - nan_to_zero(row.vehicle_health_penalty), 0.0);
end

if isnan(row.driver_intervention_cost)
    row.driver_intervention_cost = nan_to_zero(row.control_intervention_cost) ...
        + nan_to_zero(row.vehicle_health_penalty);
end

if isnan(row.driver_masking_risk)
    row.driver_masking_risk = nan_to_zero(row.vehicle_health_penalty) ...
        * (nan_to_zero(row.mean_torque_speed_loss) ...
           + nan_to_zero(row.mean_track_margin_speed_loss) ...
           + nan_to_zero(row.mean_corner_throttle_loss));
end
end

function value = nan_to_zero(value)
if isnan(value)
    value = 0.0;
end
end

function events = default_events()
events = struct( ...
    'name', 'autocross', ...
    'track_name', 'autocross', ...
    'sim_time', 180.0, ...
    'initial_speed', -1.0, ...
    'timeout_sec', 240.0);
end

function events = normalize_events(config)
events = config.events;
if ~isstruct(events)
    error('events must be a struct array.');
end
for i = 1:numel(events)
    events(i).name = char(string(field_or(events(i), 'name', events(i).track_name)));
    events(i).track_name = char(string(field_or(events(i), 'track_name', events(i).name)));
    events(i).sim_time = double(field_or(events(i), 'sim_time', 180.0));
    events(i).initial_speed = double(field_or(events(i), 'initial_speed', -1.0));
    events(i).timeout_sec = double(field_or(events(i), 'timeout_sec', 240.0));
end
end

function names = default_parameter_names()
names = { ...
    'MassTotal', ...
    'TireMuY', ...
    'TireMuX', ...
    'AeroCLA', ...
    'AeroCDA', ...
    'ChassisStiffness', ...
    'ChassisDamping', ...
    'TwistInertia', ...
    'SteeringInertia', ...
    'SteeringRatio', ...
    'SteeringFriction', ...
    'SteeringDamping', ...
    'DriverTorqueMax', ...
    'FrontSpring', ...
    'FrontARB'};
end

function catalog = parameter_catalog(base_params)
catalog = [ ...
    entry('MassTotal', {'mass', 'm_total'}, 180.0, 360.0, 'mass'), ...
    entry('TireMuY', {'tire', 'mu_y_max'}, 1.0, 2.8, 'tire'), ...
    entry('TireMuX', {'tire', 'mu_x_max'}, 0.9, 2.4, 'tire'), ...
    entry('AeroCLA', {'aero', 'CL_A'}, 0.0, 5.0, 'aero'), ...
    entry('AeroCDA', {'aero', 'CD_A'}, 0.3, 1.8, 'aero'), ...
    entry('ChassisStiffness', {'suspension', 'K_chassis'}, 800.0, 1000000.0, 'frame'), ...
    entry('ChassisDamping', {'suspension', 'C_chassis'}, 1.0, 80.0, 'frame'), ...
    entry('TwistInertia', {'suspension', 'I_twist'}, 0.5, 5.0, 'frame'), ...
    entry('SteeringInertia', {'steering', 'I_steer'}, 0.005, 0.12, 'steering'), ...
    entry('SteeringRatio', {'steering', 'R_steer'}, 2.5, 8.0, 'steering'), ...
    entry('SteeringFriction', {'steering', 'T_fric'}, 0.0, 4.0, 'steering'), ...
    entry('SteeringDamping', {'steering', 'C_steer_torque'}, 0.05, 2.0, 'steering'), ...
    entry('DriverTorqueMax', {'steering', 'T_driver_max'}, 8.0, 60.0, 'steering'), ...
    entry('FrontSpring', {'suspension', 'k_spring', 1}, 8000.0, 60000.0, 'suspension'), ...
    entry('FrontARB', {'suspension', 'k_arb_f'}, 0.0, 2500.0, 'suspension') ...
];

for i = 1:numel(catalog)
    catalog(i).base_value = get_path_value(base_params, decode_path(catalog(i).path));
end
end

function e = entry(name, path, min_value, max_value, group)
e = struct();
e.name = name;
e.path = encode_path(path);
e.min_value = min_value;
e.max_value = max_value;
e.group = group;
e.base_value = NaN;
end

function samples = build_samples(base_params, config)
catalog = parameter_catalog(base_params);
selected = {};
for i = 1:numel(config.parameter_names)
    name = char(string(config.parameter_names{i}));
    idx = find(strcmp({catalog.name}, name), 1);
    if isempty(idx)
        error('Unknown driver-generalization parameter: %s', name);
    end
    selected{end + 1} = catalog(idx); %#ok<AGROW>
end

samples = sample_entry('baseline', 'baseline', '', '', NaN, NaN, NaN, '', '', '');
for i = 1:numel(selected)
    item = selected{i};
    base_value = item.base_value;
    low_value = clamp(base_value * config.factor_low, item.min_value, item.max_value);
    high_value = clamp(base_value * config.factor_high, item.min_value, item.max_value);
    samples(end + 1) = sample_entry( ...
        sprintf('%s_low', item.name), 'perturbation', item.name, 'low', ...
        base_value, low_value, low_value - base_value, item.path, item.group, item.name);
    samples(end + 1) = sample_entry( ...
        sprintf('%s_high', item.name), 'perturbation', item.name, 'high', ...
        base_value, high_value, high_value - base_value, item.path, item.group, item.name);
end
end

function s = sample_entry(sample_id, role, parameter_name, direction, base_value, applied_value, delta_value, path, group, label)
s = struct();
s.sample_id = char(sample_id);
s.sample_role = char(role);
s.parameter_name = char(parameter_name);
s.parameter_label = char(label);
s.direction = char(direction);
s.base_value = double(base_value);
s.applied_value = double(applied_value);
s.delta_value = double(delta_value);
s.path = char(path);
s.group = char(group);
end

function params = apply_sample(base_params, sample)
params = base_params;
if strcmp(sample.sample_role, 'baseline')
    return;
end
params = set_path_value(params, decode_path(sample.path), sample.applied_value);

if strcmp(sample.parameter_name, 'FrontSpring')
    params.suspension.k_spring(2) = sample.applied_value;
end
end

function path_text = encode_path(path)
if isempty(path)
    path_text = '';
    return;
end
parts = cell(1, numel(path));
for i = 1:numel(path)
    token = path{i};
    if isnumeric(token)
        parts{i} = sprintf('#%d', token);
    else
        parts{i} = char(token);
    end
end
path_text = strjoin(parts, '.');
end

function path = decode_path(path_text)
path_text = char(string(path_text));
if isempty(path_text)
    path = {};
    return;
end
parts = strsplit(path_text, '.');
path = cell(1, numel(parts));
for i = 1:numel(parts)
    token = parts{i};
    if startsWith(token, '#')
        path{i} = str2double(extractAfter(token, 1));
    else
        path{i} = token;
    end
end
end

function value = get_path_value(s, path)
value = s;
for i = 1:numel(path)
    key = path{i};
    if isnumeric(key)
        value = value(key);
    else
        value = value.(key);
    end
end
end

function s = set_path_value(s, path, value)
if isscalar(path)
    key = path{1};
    if isnumeric(key)
        s(key) = value;
    else
        s.(key) = value;
    end
    return;
end

key = path{1};
if isnumeric(key)
    child = s(key);
    child = set_path_value(child, path(2:end), value);
    s(key) = child;
else
    child = s.(key);
    child = set_path_value(child, path(2:end), value);
    s.(key) = child;
end
end

function [result, success] = execute_laptime(config, params_file, event)
exe = fullfile(config.project_root, 'build', 'cpp', 'laptime_sim');
if ispc
    exe = [exe, '.exe'];
end
if ~isfile(exe)
    build_cmd = sprintf('cmake --build %s --target laptime_sim', shell_quote(fullfile(config.project_root, 'build')));
    [build_status, build_out] = system(build_cmd);
    if build_status ~= 0
        error('Failed to build laptime_sim: %s', build_out);
    end
end

cmd = sprintf('%s --track %s --time %.6g --params_file %s --driver %s --output_json %s', ...
    shell_quote(exe), shell_quote(event.track_name), event.sim_time, ...
    shell_quote(params_file), shell_quote(config.driver_mode), char(string(config.extra_cli_args)));
if event.initial_speed >= 0
    cmd = sprintf('%s --initial-speed %.6g', cmd, event.initial_speed);
end

previous_dir = pwd;
cleanup = onCleanup(@() cd(previous_dir));
cd(config.project_root);
[status, out] = system(cmd);
result = parse_result(out);
success = (status == 0) && isfield(result, 'lap_completed') && logical(result.lap_completed);
end

function result = parse_result(out)
start_token = '%%JSON_BEGIN%%';
end_token = '%%JSON_END%%';
start_idx = strfind(out, start_token);
end_idx = strfind(out, end_token);
if isempty(start_idx) || isempty(end_idx)
    error('No JSON result markers found in laptime output.');
end
json_text = out(start_idx(1) + strlength(start_token):end_idx(end) - 1);
result = jsondecode(strtrim(json_text));
end

function row = build_row(config, sample, event, params_file, result, success, runtime_sec)
if isempty(result)
    result = struct();
end
row = struct();
row.timestamp_utc = utc_now();
row.run_id = config.run_id;
row.sample_id = sample.sample_id;
row.sample_role = sample.sample_role;
row.parameter_name = sample.parameter_name;
row.parameter_label = sample.parameter_label;
row.parameter_group = sample.group;
row.direction = sample.direction;
row.base_value = sample.base_value;
row.applied_value = sample.applied_value;
row.delta_value = sample.delta_value;
row.event_name = event.name;
row.track_name = event.track_name;
row.success = logical(success);
row.lap_completed = logical(field_or(result, 'lap_completed', false));
row.status = char(string(field_or(result, 'status', 'failed')));
row.failure_code = char(string(field_or(result, 'failure_code', '')));
row.failure_message = char(string(field_or(result, 'failure_message', '')));
row.failure_time = double(field_or(result, 'failure_time', NaN));
row.lap_time = double(field_or(result, 'lap_time', NaN));
row.distance_traveled = double(field_or(result, 'distance_traveled', NaN));
row.max_lateral_error = double(field_or(result, 'max_lateral_error', NaN));
row.avg_lateral_error = double(field_or(result, 'avg_lateral_error', NaN));
row.max_track_limit_excess = double(field_or(result, 'max_track_limit_excess', NaN));
row.max_speed = double(field_or(result, 'max_speed', NaN));
row.avg_speed = double(field_or(result, 'avg_speed', NaN));
row.rms_heading_error = double(field_or(result, 'rms_heading_error', NaN));
row.steering_smoothness = double(field_or(result, 'steering_smoothness', NaN));
row.throttle_smoothness = double(field_or(result, 'throttle_smoothness', NaN));
row.brake_smoothness = double(field_or(result, 'brake_smoothness', NaN));
row.steering_saturation_ratio = double(field_or(result, 'steering_saturation_ratio', NaN));
row.mean_abs_sw_error = double(field_or(result, 'mean_abs_sw_error', NaN));
row.max_abs_sw_error = double(field_or(result, 'max_abs_sw_error', NaN));
row.mean_steering_lag_ratio = double(field_or(result, 'mean_steering_lag_ratio', NaN));
row.max_steering_lag_ratio = double(field_or(result, 'max_steering_lag_ratio', NaN));
row.avg_torque_speed_multiplier = double(field_or(result, 'avg_torque_speed_multiplier', NaN));
row.mean_torque_speed_loss = double(field_or(result, 'mean_torque_speed_loss', NaN));
row.mean_steering_cap_speed_loss = double(field_or(result, 'mean_steering_cap_speed_loss', NaN));
row.mean_track_margin_speed_loss = double(field_or(result, 'mean_track_margin_speed_loss', NaN));
row.mean_corner_throttle_loss = double(field_or(result, 'mean_corner_throttle_loss', NaN));
row.mean_chassis_twist_risk = double(field_or(result, 'mean_chassis_twist_risk', NaN));
row.chassis_twist_risk_integral = double(field_or(result, 'chassis_twist_risk_integral', NaN));
row.driver_intervention_cost = double(field_or(result, 'driver_intervention_cost', NaN));
row.control_intervention_cost = double(field_or(result, 'control_intervention_cost', NaN));
row.vehicle_health_penalty = double(field_or(result, 'vehicle_health_penalty', NaN));
row.driver_masking_risk = double(field_or(result, 'driver_masking_risk', NaN));
row = backfill_intervention_breakdown(row);
row.mean_mpc_rollout_progress_deficit = double(field_or(result, 'mean_mpc_rollout_progress_deficit', NaN));
row.max_mpc_rollout_progress_deficit = double(field_or(result, 'max_mpc_rollout_progress_deficit', NaN));
row.min_steering_actuator_speed_cap = double(field_or(result, 'min_steering_actuator_speed_cap', NaN));
row.min_track_margin_speed_cap = double(field_or(result, 'min_track_margin_speed_cap', NaN));
row.min_velocity_profile_steering_cap = double(field_or(result, 'min_velocity_profile_steering_cap', NaN));
row.velocity_profile_steering_cap_ratio = double(field_or(result, 'velocity_profile_steering_cap_ratio', NaN));
row.max_abs_phi_twist = double(field_or(result, 'max_abs_phi_twist', NaN));
row.max_abs_dphi_twist = double(field_or(result, 'max_abs_dphi_twist', NaN));

% Compound robustness penalty score
alpha = 100.0; % chassis twist (rad) penalty scaling
beta = 20.0;   % steering lag ratio penalty scaling
gamma = 30.0;  % control intervention penalty scaling [s / cost]
delta = 30.0;  % vehicle health penalty scaling [s / cost]
chassis_twist_penalty = row.max_abs_phi_twist * alpha;
steering_lag_penalty = row.mean_steering_lag_ratio * beta;
control_intervention_penalty = row.control_intervention_cost * gamma;
vehicle_health_penalty = row.vehicle_health_penalty * delta;
if isnan(chassis_twist_penalty), chassis_twist_penalty = 0; end
if isnan(steering_lag_penalty), steering_lag_penalty = 0; end
if isnan(control_intervention_penalty), control_intervention_penalty = 0; end
if isnan(vehicle_health_penalty), vehicle_health_penalty = 0; end
row.robustness_score = row.lap_time + chassis_twist_penalty + steering_lag_penalty ...
    + control_intervention_penalty + vehicle_health_penalty;

row.runtime_sec = runtime_sec;
row.params_file = params_file;
end

function summary = summarize_results(tbl, samples, events, summary_path)
summary = struct();
summary.sample_count = height(tbl);
summary.event_count = numel(events);
summary.failure_count = sum(~tbl.success);
summary.baseline = tbl(strcmp(tbl.sample_role, 'baseline'), :);

rows = struct([]);
for i = 1:numel(samples)
    sample = samples(i);
    if strcmp(sample.sample_role, 'baseline')
        continue;
    end
    st = tbl(strcmp(tbl.sample_id, sample.sample_id), :);
    if isempty(st)
        continue;
    end
    rows(end + 1).sample_id = sample.sample_id; %#ok<AGROW>
    rows(end).parameter_name = sample.parameter_name;
    rows(end).direction = sample.direction;
    rows(end).parameter_group = sample.group;
    rows(end).success_rate = mean(double(st.success));
    rows(end).mean_lap_time = mean(st.lap_time, 'omitnan');
    rows(end).mean_robustness_score = mean(st.robustness_score, 'omitnan');
    rows(end).max_failure_time = max(st.failure_time, [], 'omitnan');
    rows(end).max_distance_traveled = max(st.distance_traveled, [], 'omitnan');
    rows(end).max_lateral_error = max(st.max_lateral_error);
    rows(end).max_track_limit_excess = max(st.max_track_limit_excess);
    rows(end).mean_steering_smoothness = mean(st.steering_smoothness, 'omitnan');
    rows(end).mean_throttle_smoothness = mean(st.throttle_smoothness, 'omitnan');
    rows(end).mean_brake_smoothness = mean(st.brake_smoothness, 'omitnan');
    rows(end).max_steering_saturation_ratio = max(st.steering_saturation_ratio);
    rows(end).max_mean_steering_lag_ratio = max(st.mean_steering_lag_ratio);
    rows(end).max_driver_intervention_cost = max(st.driver_intervention_cost);
    rows(end).max_control_intervention_cost = max(st.control_intervention_cost);
    rows(end).max_vehicle_health_penalty = max(st.vehicle_health_penalty);
    rows(end).max_driver_masking_risk = max(st.driver_masking_risk);
    rows(end).max_mpc_rollout_progress_deficit = max(st.max_mpc_rollout_progress_deficit);
    rows(end).max_mean_torque_speed_loss = max(st.mean_torque_speed_loss);
    rows(end).max_mean_chassis_twist_risk = max(st.mean_chassis_twist_risk);
    rows(end).min_steering_actuator_speed_cap = min(st.min_steering_actuator_speed_cap);
    rows(end).min_track_margin_speed_cap = min(st.min_track_margin_speed_cap);
    rows(end).min_velocity_profile_steering_cap = min(st.min_velocity_profile_steering_cap);
    rows(end).max_velocity_profile_steering_cap_ratio = max(st.velocity_profile_steering_cap_ratio);
    rows(end).max_abs_phi_twist = max(st.max_abs_phi_twist);
end

if isempty(rows)
    summary.table = table();
else
    summary.table = struct2table(rows);
    writetable(summary.table, summary_path);
end
end

function write_report(manifest, summary, config)
fid = fopen(manifest.report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', manifest.report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Driver Generalization Sweep\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- driver_mode: `%s`\n', config.driver_mode);
fprintf(fid, '- samples_csv: `%s`\n', manifest.samples_csv_path);
fprintf(fid, '- summary_csv: `%s`\n\n', manifest.summary_csv_path);

fprintf(fid, '## Purpose\n\n');
fprintf(fid, 'Evaluate whether the driver stack remains feasible and fast when flexible-body and steering-torque plant parameters change.\n\n');

fprintf(fid, '## Baseline\n\n');
if ~isempty(summary.baseline)
    b = summary.baseline(1, :);
    fprintf(fid, '- lap_time: %.3f s\n', b.lap_time);
    fprintf(fid, '- robustness_score: %.3f\n', b.robustness_score);
    fprintf(fid, '- max_lateral_error: %.3f m\n', b.max_lateral_error);
    fprintf(fid, '- steering_saturation_ratio: %.4f\n', b.steering_saturation_ratio);
    fprintf(fid, '- mean_steering_lag_ratio: %.4f\n\n', b.mean_steering_lag_ratio);
    fprintf(fid, '- driver_intervention_cost: %.4f\n', b.driver_intervention_cost);
    fprintf(fid, '- control_intervention_cost: %.4f\n', b.control_intervention_cost);
    fprintf(fid, '- vehicle_health_penalty: %.4f\n', b.vehicle_health_penalty);
    fprintf(fid, '- driver_masking_risk: %.4f\n', b.driver_masking_risk);
    fprintf(fid, '- mean_torque_speed_loss: %.4f\n', b.mean_torque_speed_loss);
    fprintf(fid, '- mean_chassis_twist_risk: %.4f\n\n', b.mean_chassis_twist_risk);
    fprintf(fid, '- min_steering_actuator_speed_cap: %.3f m/s\n', b.min_steering_actuator_speed_cap);
    fprintf(fid, '- min_track_margin_speed_cap: %.3f m/s\n', b.min_track_margin_speed_cap);
    fprintf(fid, '- min_velocity_profile_steering_cap: %.3f m/s\n', b.min_velocity_profile_steering_cap);
    fprintf(fid, '- velocity_profile_steering_cap_ratio: %.3f\n\n', b.velocity_profile_steering_cap_ratio);
else
    fprintf(fid, 'No baseline row was found.\n\n');
end

fprintf(fid, '## Parameter Rows\n\n');
if isempty(summary.table)
    fprintf(fid, 'No perturbation rows were collected.\n');
else
    fprintf(fid, '| parameter | direction | success_rate | mean_lap_time | mean_robustness_score | max_lat_err | max_offtrack_excess | max_sat_ratio | max_lag_ratio | max_control | max_vehicle | max_masking | max_mpc_progress_deficit | max_speed_loss | max_twist_risk | min_actuator_cap | min_margin_cap | min_profile_cap | profile_cap_ratio |\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
    for i = 1:height(summary.table)
        r = summary.table(i, :);
        fprintf(fid, '| %s | %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
            char(r.parameter_name), char(r.direction), r.success_rate, r.mean_lap_time, r.mean_robustness_score, ...
            r.max_lateral_error, r.max_track_limit_excess, ...
            r.max_steering_saturation_ratio, r.max_mean_steering_lag_ratio, ...
            r.max_control_intervention_cost, r.max_vehicle_health_penalty, r.max_driver_masking_risk, ...
            r.max_mpc_rollout_progress_deficit, r.max_mean_torque_speed_loss, r.max_mean_chassis_twist_risk, ...
            r.min_steering_actuator_speed_cap, r.min_track_margin_speed_cap, ...
            r.min_velocity_profile_steering_cap, r.max_velocity_profile_steering_cap_ratio);
    end
end
end

function append_row(path, row)
row_table = struct2table(row, 'AsArray', true);
if isfile(path)
    writetable(row_table, path, 'WriteMode', 'append', 'WriteVariableNames', false);
else
    writetable(row_table, path);
end
end

function tbl = read_samples_table(path)
if isfile(path)
    tbl = readtable(path, 'TextType', 'string');
else
    tbl = table();
end
end

function completed = read_completed_keys(path)
completed = containers.Map('KeyType', 'char', 'ValueType', 'logical');
tbl = read_samples_table(path);
if isempty(tbl)
    return;
end
for i = 1:height(tbl)
    completed(make_key(tbl.sample_id(i), tbl.event_name(i))) = true;
end
end

function key = make_key(sample_id, event_name)
key = sprintf('%s::%s', char(string(sample_id)), char(string(event_name)));
end

function ensure_dir(path)
if ~exist(path, 'dir')
    mkdir(path);
end
end

function ensure_project_paths(project_root)
addpath(fullfile(project_root, 'matlab'));
addpath(fullfile(project_root, 'matlab', 'driver_generalization'));
end

function write_json(path, data)
fid = fopen(path, 'w');
if fid == -1
    error('Failed to open %s for writing.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(data));
end

function out = utc_now()
out = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
end

function value = clamp(value, low, high)
value = min(max(value, low), high);
end

function quoted = shell_quote(value)
value = char(string(value));
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
