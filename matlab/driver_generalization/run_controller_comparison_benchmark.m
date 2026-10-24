function analysis = run_controller_comparison_benchmark(config)
% RUN_CONTROLLER_COMPARISON_BENCHMARK Compare PID, MPC, and optional RL drivers.
%
% This is the phase-1 gate for the flexible-chassis torque-steering driver
% work. It keeps the vehicle plant and event fixed, runs each controller
% through laptime_sim, and writes comparable health metrics.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_project_paths(config.project_root);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);
ensure_dir(config.trace_output_dir);

base_params = resolve_base_params(config);
params_file = fullfile(config.runtime_output_dir, 'effective_vehicle_params.json');
write_json(params_file, base_params);

variants = normalize_variants(config);
events = normalize_events(config);

manifest = struct();
manifest.schema_version = 'controller_comparison_benchmark/v1';
manifest.generated_by = 'run_controller_comparison_benchmark';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.project_root = config.project_root;
manifest.runtime_output_dir = config.runtime_output_dir;
manifest.report_output_dir = config.report_output_dir;
manifest.trace_output_dir = config.trace_output_dir;
manifest.samples_csv_path = fullfile(config.runtime_output_dir, 'samples.csv');
manifest.summary_csv_path = fullfile(config.runtime_output_dir, 'summary.csv');
manifest.report_path = fullfile(config.report_output_dir, sprintf('controller_comparison_%s.md', config.run_id));
manifest.base_params_file = config.base_params_file;
manifest.effective_params_file = params_file;
manifest.reference_lap_time_sec = config.reference_lap_time_sec;
manifest.variants = variants;
manifest.events = events;
write_json(fullfile(config.runtime_output_dir, 'manifest.json'), manifest);

completed = read_completed_keys(manifest.samples_csv_path);
rows = {};

for variant_idx = 1:numel(variants)
    variant = variants(variant_idx);
    if ~variant.enabled
        continue;
    end

    for event_idx = 1:numel(events)
        event = events(event_idx);
        key = make_key(variant.name, event.name);
        if isKey(completed, key)
            continue;
        end

        trace_csv_path = fullfile(config.trace_output_dir, sprintf('%03d_%s_%s.csv', ...
            completed.Count + numel(rows) + 1, variant.name, event.name));

        started = tic;
        if ~isempty(config.execute_variant_fn)
            [result, success] = config.execute_variant_fn(variant, event, params_file, trace_csv_path, config);
            command_text = 'mock';
        else
            [result, success, command_text] = execute_laptime(config, variant, event, params_file, trace_csv_path);
        end
        runtime_sec = toc(started);

        row = build_row(config, variant, event, result, success, runtime_sec, trace_csv_path, command_text);
        append_row(manifest.samples_csv_path, row);
        completed(key) = true;
        rows{end + 1} = row; %#ok<AGROW>
    end
end

samples_table = read_samples_table(manifest.samples_csv_path);
summary = summarize_results(samples_table, variants, config.reference_lap_time_sec, manifest.summary_csv_path);
write_report(manifest, summary, config);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = fullfile(config.runtime_output_dir, 'manifest.json');
analysis.samples_csv_path = manifest.samples_csv_path;
analysis.summary_csv_path = manifest.summary_csv_path;
analysis.report_path = manifest.report_path;
analysis.runtime_output_dir = config.runtime_output_dir;
analysis.trace_output_dir = config.trace_output_dir;
analysis.effective_params_file = params_file;
analysis.executed_rows = numel(rows);
analysis.variants = variants;
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
        'driver_generalization', 'controller_comparison', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end
if ~isfield(config, 'trace_output_dir') || isempty(config.trace_output_dir)
    config.trace_output_dir = fullfile(config.runtime_output_dir, 'traces');
end
if ~isfield(config, 'base_params') || isempty(config.base_params)
    config.base_params = [];
end
if ~isfield(config, 'base_params_file') || isempty(config.base_params_file)
    config.base_params_file = fullfile(config.project_root, 'matlab', 'parameter_presets', ...
        'params_winged_flexible.json');
end
if ~isfield(config, 'force_attack_steering_defaults') || isempty(config.force_attack_steering_defaults)
    config.force_attack_steering_defaults = true;
end
if ~isfield(config, 'driver_torque_max') || isempty(config.driver_torque_max)
    config.driver_torque_max = 40.0;
end
if ~isfield(config, 'driver_torque_rate_max') || isempty(config.driver_torque_rate_max)
    config.driver_torque_rate_max = 0.0;
end
if ~isfield(config, 'rl_steer_torque_rate_limit') || isempty(config.rl_steer_torque_rate_limit)
    config.rl_steer_torque_rate_limit = -1.0;
end
if ~isfield(config, 'reference_lap_time_sec') || isempty(config.reference_lap_time_sec)
    config.reference_lap_time_sec = 66.0;
end
if ~isfield(config, 'events') || isempty(config.events)
    config.events = default_events();
end
if ~isfield(config, 'variants')
    config.variants = [];
end
if ~isfield(config, 'rl_actor_dir') || isempty(config.rl_actor_dir)
    config.rl_actor_dir = '';
end
if ~isfield(config, 'include_rl') || isempty(config.include_rl)
    config.include_rl = ~isempty(config.rl_actor_dir);
end
if ~isfield(config, 'extra_cli_args') || isempty(config.extra_cli_args)
    config.extra_cli_args = '';
end
if ~isfield(config, 'execute_variant_fn')
    config.execute_variant_fn = [];
end
end

function params = resolve_base_params(config)
if ~isempty(config.base_params)
    params = config.base_params;
else
    params_file = resolve_project_path(config.project_root, config.base_params_file);
    raw = jsondecode(fileread(params_file));
    if isfield(raw, 'params')
        assert_valid_vehicle_parameter_artifact(raw);
        params = raw.params;
    else
        params = raw;
    end
end

if config.force_attack_steering_defaults
    if ~isfield(params, 'steering')
        params.steering = struct();
    end
    params.steering.torque_mode = true;
    params.steering.T_driver_max = double(config.driver_torque_max);
    params.steering.T_driver_rate_max = double(config.driver_torque_rate_max);
end
end

function variants = normalize_variants(config)
if ~isempty(config.variants)
    variants = config.variants;
else
    variants = struct( ...
        'name', {'pid_baseline', 'mpc_preview', 'mpc_v2_preview'}, ...
        'driver_mode', {'pid', 'mpc', 'mpc_v2'}, ...
        'driver_role', {'fallback_teacher', 'legacy_model_based', 'model_based_candidate'}, ...
        'extra_cli_args', {'', '', ''}, ...
        'rl_actor_dir', {'', '', ''}, ...
        'enabled', {true, true, true});

    if config.include_rl
        variants(end + 1) = struct( ...
            'name', 'rl_actor', ...
            'driver_mode', 'rl', ...
            'driver_role', 'learned_candidate', ...
            'extra_cli_args', '', ...
            'rl_actor_dir', config.rl_actor_dir, ...
            'enabled', true);
    end
end

if ~isfield(variants, 'driver_role')
    [variants.driver_role] = deal('');
end
for i = 1:numel(variants)
    variants(i).name = char(string(field_or(variants(i), 'name', sprintf('variant_%d', i))));
    variants(i).driver_mode = char(string(field_or(variants(i), 'driver_mode', 'pid')));
    variants(i).driver_role = char(string(field_or(variants(i), 'driver_role', ...
        default_driver_role(variants(i).driver_mode))));
    variants(i).extra_cli_args = char(string(field_or(variants(i), 'extra_cli_args', '')));
    variants(i).enabled = logical(field_or(variants(i), 'enabled', true));
    if ~isfield(variants(i), 'rl_actor_dir') || isempty(variants(i).rl_actor_dir)
        variants(i).rl_actor_dir = '';
    else
        variants(i).rl_actor_dir = resolve_project_path(config.project_root, variants(i).rl_actor_dir);
    end
end
end

function events = default_events()
events = struct( ...
    'name', 'autocross', ...
    'track_name', 'autocross', ...
    'sim_time', 180.0, ...
    'initial_speed', -1.0);
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
end
end

function [result, success, command_text] = execute_laptime(config, variant, event, params_file, trace_csv_path)
exe = fullfile(config.project_root, 'build', 'cpp', 'laptime_sim');
if ispc
    exe = [exe, '.exe'];
end
if ~isfile(exe)
    build_cmd = sprintf('cmake --build %s --target laptime_sim', ...
        shell_quote(fullfile(config.project_root, 'build')));
    [build_status, build_out] = system(build_cmd);
    if build_status ~= 0
        error('Failed to build laptime_sim: %s', build_out);
    end
end

command_text = sprintf(['%s --track %s --time %.6g --params_file %s --driver %s ' ...
    '--driver-torque-max %.6g --driver-torque-rate-max %.6g --output_json --output %s'], ...
    shell_quote(exe), shell_quote(event.track_name), event.sim_time, ...
    shell_quote(params_file), shell_quote(variant.driver_mode), ...
    config.driver_torque_max, config.driver_torque_rate_max, shell_quote(trace_csv_path));
if event.initial_speed >= 0
    command_text = sprintf('%s --initial-speed %.6g', command_text, event.initial_speed);
end
if strcmp(variant.driver_mode, 'rl')
    if isempty(variant.rl_actor_dir)
        error('RL variant %s requires rl_actor_dir.', variant.name);
    end
    command_text = sprintf('%s --rl-actor-dir %s', command_text, shell_quote(variant.rl_actor_dir));
    if config.rl_steer_torque_rate_limit >= 0.0
        command_text = sprintf('%s --rl-steer-torque-rate-limit %.6g', ...
            command_text, config.rl_steer_torque_rate_limit);
    end
end
command_text = strtrim(strjoin({command_text, config.extra_cli_args, variant.extra_cli_args}, ' '));

previous_dir = pwd;
cleanup = onCleanup(@() cd(previous_dir));
cd(config.project_root);
[status, out] = system(command_text);
result = parse_result(out);
success = (status == 0) && isfield(result, 'lap_completed') && logical(result.lap_completed);
end

function result = parse_result(out)
start_token = '%%JSON_BEGIN%%';
end_token = '%%JSON_END%%';
start_idx = strfind(out, start_token);
end_idx = strfind(out, end_token);
if isempty(start_idx) || isempty(end_idx)
    error('No JSON result markers found in laptime output:\n%s', out);
end
json_text = out(start_idx(1) + strlength(start_token):end_idx(end) - 1);
result = jsondecode(strtrim(json_text));
end

function row = build_row(config, variant, event, result, success, runtime_sec, trace_csv_path, command_text)
if isempty(result)
    result = struct();
end
row = struct();
row.timestamp_utc = utc_now();
row.run_id = config.run_id;
row.variant = variant.name;
row.driver_mode = variant.driver_mode;
row.driver_role = variant.driver_role;
row.event_name = event.name;
row.track_name = event.track_name;
row.reference_lap_time_sec = double(config.reference_lap_time_sec);
row.success = logical(success);
row.lap_completed = logical(field_or(result, 'lap_completed', false));
row.status = char(string(field_or(result, 'status', 'failed')));
row.failure_code = char(string(field_or(result, 'failure_code', '')));
row.failure_message = char(string(field_or(result, 'failure_message', '')));
row.failure_time = double(field_or(result, 'failure_time', NaN));
row.lap_time = double(field_or(result, 'lap_time', NaN));
row.lap_time_delta_to_reference = row.lap_time - row.reference_lap_time_sec;
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
row.mean_steering_reach_speed_loss = double(field_or(result, 'mean_steering_reach_speed_loss', NaN));
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
row.min_steering_reach_speed_cap = double(field_or(result, 'min_steering_reach_speed_cap', NaN));
row.min_track_margin_speed_cap = double(field_or(result, 'min_track_margin_speed_cap', NaN));
row.min_velocity_profile_steering_cap = double(field_or(result, 'min_velocity_profile_steering_cap', NaN));
row.velocity_profile_steering_cap_ratio = double(field_or(result, 'velocity_profile_steering_cap_ratio', NaN));
row.max_abs_phi_twist = double(field_or(result, 'max_abs_phi_twist', NaN));
row.max_abs_dphi_twist = double(field_or(result, 'max_abs_dphi_twist', NaN));
row.max_track_limit_excess = double(field_or(result, 'max_track_limit_excess', NaN));
row.mpc_solve_count = double(field_or(result, 'mpc_solve_count', NaN));
row.mpc_avg_solve_time_ms = double(field_or(result, 'mpc_avg_solve_time_ms', NaN));
row.mpc_max_solve_time_ms = double(field_or(result, 'mpc_max_solve_time_ms', NaN));
row.mpc_fail_count = double(field_or(result, 'mpc_fail_count', NaN));
row.dominant_driver_loss = char(string(field_or(result, 'dominant_driver_loss', ...
    field_or(result, 'dominant_loss', ''))));
row.runtime_sec = runtime_sec;
row.trace_csv_path = trace_csv_path;
row.command = command_text;
end

function summary = summarize_results(tbl, variants, reference_lap_time_sec, summary_path)
summary = struct();
summary.sample_count = height(tbl);
summary.failure_count = sum(~logical(tbl.success));
summary.best_success = table();

if isempty(tbl)
    summary.table = table();
    return;
end

rows = struct([]);
for i = 1:numel(variants)
    variant_name = string(variants(i).name);
    st = tbl(tbl.variant == variant_name, :);
    if isempty(st)
        continue;
    end

    success_mask = logical(st.success);
    success_rows = st(success_mask, :);
    rows(end + 1).variant = char(variant_name); %#ok<AGROW>
    rows(end).driver_mode = variants(i).driver_mode;
    rows(end).driver_role = variants(i).driver_role;
    rows(end).case_count = height(st);
    rows(end).success_rate = mean(double(success_mask));
    rows(end).best_lap_time = min_or_nan(success_rows.lap_time);
    rows(end).mean_lap_time = mean_or_nan(success_rows.lap_time);
    rows(end).best_delta_to_reference = rows(end).best_lap_time - reference_lap_time_sec;
    rows(end).max_failure_time = max_or_nan(st.failure_time);
    rows(end).max_distance_traveled = max_or_nan(st.distance_traveled);
    rows(end).max_track_limit_excess = max(st.max_track_limit_excess);
    rows(end).mean_steering_smoothness = mean_or_nan(st.steering_smoothness);
    rows(end).mean_throttle_smoothness = mean_or_nan(st.throttle_smoothness);
    rows(end).mean_brake_smoothness = mean_or_nan(st.brake_smoothness);
    rows(end).max_steering_saturation_ratio = max(st.steering_saturation_ratio);
    rows(end).mean_steering_lag_ratio = mean(st.mean_steering_lag_ratio, 'omitnan');
    rows(end).mean_driver_intervention_cost = mean(st.driver_intervention_cost, 'omitnan');
    rows(end).mean_control_intervention_cost = mean(st.control_intervention_cost, 'omitnan');
    rows(end).mean_vehicle_health_penalty = mean(st.vehicle_health_penalty, 'omitnan');
    rows(end).max_driver_masking_risk = max(st.driver_masking_risk);
    rows(end).mean_mpc_rollout_progress_deficit = mean(st.mean_mpc_rollout_progress_deficit, 'omitnan');
    rows(end).max_mpc_rollout_progress_deficit = max(st.max_mpc_rollout_progress_deficit);
    rows(end).mpc_avg_solve_time_ms = mean(st.mpc_avg_solve_time_ms, 'omitnan');
    rows(end).mpc_fail_count = sum(st.mpc_fail_count, 'omitnan');
end

if isempty(rows)
    summary.table = table();
else
    summary.table = struct2table(rows);
    writetable(summary.table, summary_path);
end

success_tbl = tbl(logical(tbl.success), :);
if ~isempty(success_tbl)
    [~, idx] = min(success_tbl.lap_time);
    summary.best_success = success_tbl(idx, :);
end
end

function write_report(manifest, summary, config)
fid = fopen(manifest.report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', manifest.report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Controller Comparison Benchmark\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- effective_params_file: `%s`\n', manifest.effective_params_file);
fprintf(fid, '- samples_csv: `%s`\n', manifest.samples_csv_path);
fprintf(fid, '- summary_csv: `%s`\n', manifest.summary_csv_path);
fprintf(fid, '- trace_output_dir: `%s`\n', manifest.trace_output_dir);
fprintf(fid, '- reference_lap_time_sec: %.3f\n\n', config.reference_lap_time_sec);

fprintf(fid, '## Positioning\n\n');
fprintf(fid, ['PID is treated as the stable fallback and diagnostic teacher. ' ...
    'MPC is the model-based path toward the general driver. RL is evaluated ' ...
    'through the same gate when an exported actor is supplied.\n\n']);

fprintf(fid, '## Summary\n\n');
if isempty(summary.table)
    fprintf(fid, 'No controller rows were collected.\n');
else
    fprintf(fid, '| variant | driver | role | success_rate | best_lap | mean_lap | delta_to_ref | max_failure_time | max_distance | max_offtrack | steer_smooth | brake_smooth | max_sat | mean_lag | control_cost | vehicle_health | masking | mpc_progress_deficit | mpc_avg_ms | mpc_fails |\n');
    fprintf(fid, '|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
    for i = 1:height(summary.table)
        r = summary.table(i, :);
        fprintf(fid, '| %s | %s | %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.1f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.0f |\n', ...
            char(r.variant), char(r.driver_mode), char(r.driver_role), r.success_rate, ...
            r.best_lap_time, r.mean_lap_time, r.best_delta_to_reference, ...
            r.max_failure_time, r.max_distance_traveled, r.max_track_limit_excess, ...
            r.mean_steering_smoothness, r.mean_brake_smoothness, ...
            r.max_steering_saturation_ratio, ...
            r.mean_steering_lag_ratio, r.mean_control_intervention_cost, ...
            r.mean_vehicle_health_penalty, r.max_driver_masking_risk, ...
            r.mean_mpc_rollout_progress_deficit, r.mpc_avg_solve_time_ms, r.mpc_fail_count);
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
    completed(make_key(tbl.variant(i), tbl.event_name(i))) = true;
end
end

function key = make_key(variant_name, event_name)
key = sprintf('%s::%s', char(string(variant_name)), char(string(event_name)));
end

function value = field_or(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default_value;
end
end

function role = default_driver_role(driver_mode)
switch char(string(driver_mode))
    case 'pid'
        role = 'fallback_teacher';
    case 'mpc'
        role = 'legacy_model_based';
    case 'mpc_v2'
        role = 'model_based_candidate';
    case 'rl'
        role = 'learned_candidate';
    otherwise
        role = 'candidate';
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

function value = mean_or_nan(values)
if isempty(values)
    value = NaN;
else
    value = mean(values, 'omitnan');
end
end

function value = min_or_nan(values)
if isempty(values)
    value = NaN;
else
    value = min(values, [], 'omitnan');
end
end

function value = max_or_nan(values)
if isempty(values)
    value = NaN;
else
    value = max(values, [], 'omitnan');
end
end

function ensure_project_paths(project_root)
addpath(fullfile(project_root, 'matlab'));
addpath(fullfile(project_root, 'matlab', 'driver_generalization'));
end

function ensure_dir(path)
if ~exist(path, 'dir')
    mkdir(path);
end
end

function write_json(path, data)
fid = fopen(path, 'w');
if fid == -1
    error('Failed to open %s for writing.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(data, 'PrettyPrint', true));
end

function path_out = resolve_project_path(project_root, path_in)
path_in = char(string(path_in));
if startsWith(path_in, filesep) || ~isempty(regexp(path_in, '^[A-Za-z]:[\\/]', 'once'))
    path_out = path_in;
else
    path_out = fullfile(project_root, path_in);
end
end

function q = shell_quote(s)
s = char(string(s));
if ispc
    q = ['"', strrep(s, '"', '\"'), '"'];
else
    q = ['''', strrep(s, '''', '''"''"'''), ''''];
end
end

function out = utc_now()
out = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
end
