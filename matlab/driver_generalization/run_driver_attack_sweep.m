function analysis = run_driver_attack_sweep(config)
% RUN_DRIVER_ATTACK_SWEEP Sweep attack-lap driver torque and speed settings.
%
% This sweep keeps the vehicle plant fixed and varies driver-side attack
% assumptions such as available steering torque and torque-mode speed scale.
% It is intended to make 25/40/50 Nm attack-lap comparisons repeatable
% without changing the driver implementation itself.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);
ensure_dir(config.trace_output_dir);

base_params_file = resolve_base_params_file(config);
cases = build_attack_cases(config);
events = normalize_events(config);

manifest = struct();
manifest.schema_version = 'driver_attack_sweep/v1';
manifest.generated_by = 'run_driver_attack_sweep';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.project_root = config.project_root;
manifest.runtime_output_dir = config.runtime_output_dir;
manifest.report_output_dir = config.report_output_dir;
manifest.trace_output_dir = config.trace_output_dir;
manifest.samples_csv_path = fullfile(config.runtime_output_dir, 'samples.csv');
manifest.summary_csv_path = fullfile(config.runtime_output_dir, 'summary.csv');
manifest.report_path = fullfile(config.report_output_dir, sprintf('driver_attack_%s.md', config.run_id));
manifest.base_params_file = base_params_file;
manifest.cases = cases;
manifest.events = events;
write_json(fullfile(config.runtime_output_dir, 'manifest.json'), manifest);

completed = read_completed_keys(manifest.samples_csv_path);
rows = {};

for case_idx = 1:numel(cases)
    case_spec = cases(case_idx);
    for event_idx = 1:numel(events)
        event = events(event_idx);
        key = make_key(case_spec.case_id, event.name);
        if isKey(completed, key)
            continue;
        end

        trace_csv_path = fullfile(config.trace_output_dir, sprintf('%03d_%s_%s.csv', ...
            completed.Count + 1, case_spec.case_id, event.name));

        started = tic;
        if ~isempty(config.execute_case_fn)
            [result, success] = config.execute_case_fn(case_spec, event, config);
        else
            [result, success] = execute_laptime(config, base_params_file, event, case_spec, trace_csv_path);
        end
        runtime_sec = toc(started);

        row = build_row(config, case_spec, event, result, success, runtime_sec, trace_csv_path);
        append_row(manifest.samples_csv_path, row);
        completed(key) = true;
        rows{end + 1} = row; %#ok<AGROW>
    end
end

samples_table = read_samples_table(manifest.samples_csv_path);
summary = summarize_attack_results(samples_table, config.driver_torque_max_values, manifest.summary_csv_path);
write_report(manifest, summary, config);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = fullfile(config.runtime_output_dir, 'manifest.json');
analysis.samples_csv_path = manifest.samples_csv_path;
analysis.summary_csv_path = manifest.summary_csv_path;
analysis.report_path = manifest.report_path;
analysis.runtime_output_dir = config.runtime_output_dir;
analysis.trace_output_dir = config.trace_output_dir;
analysis.executed_rows = numel(rows);
analysis.cases = cases;
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
        'driver_generalization', 'attack_sweep', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end
if ~isfield(config, 'trace_output_dir') || isempty(config.trace_output_dir)
    config.trace_output_dir = fullfile(config.runtime_output_dir, 'traces');
end
if ~isfield(config, 'driver_mode') || isempty(config.driver_mode)
    config.driver_mode = 'pid';
end
if ~isfield(config, 'events') || isempty(config.events)
    config.events = default_events();
end
if ~isfield(config, 'driver_torque_max_values') || isempty(config.driver_torque_max_values)
    config.driver_torque_max_values = [25.0, 40.0, 50.0];
end
if ~isfield(config, 'torque_speed_scales') || isempty(config.torque_speed_scales)
    config.torque_speed_scales = [0.78, 0.80, 0.82];
end
if ~isfield(config, 'base_params') || isempty(config.base_params)
    config.base_params = [];
end
if ~isfield(config, 'base_params_file') || isempty(config.base_params_file)
    config.base_params_file = '';
end
if ~isfield(config, 'extra_cli_args') || isempty(config.extra_cli_args)
    config.extra_cli_args = '';
end
if ~isfield(config, 'acceptance') || isempty(config.acceptance)
    config.acceptance = struct();
end
config.acceptance = normalize_acceptance(config.acceptance);
if ~isfield(config, 'execute_case_fn')
    config.execute_case_fn = [];
end
end

function acceptance = normalize_acceptance(acceptance)
if ~isfield(acceptance, 'max_track_limit_excess') || isempty(acceptance.max_track_limit_excess)
    acceptance.max_track_limit_excess = 1e-6;
end
if ~isfield(acceptance, 'max_steering_saturation_ratio') || isempty(acceptance.max_steering_saturation_ratio)
    acceptance.max_steering_saturation_ratio = 0.15;
end
if ~isfield(acceptance, 'max_mean_steering_lag_ratio') || isempty(acceptance.max_mean_steering_lag_ratio)
    acceptance.max_mean_steering_lag_ratio = 0.20;
end
if ~isfield(acceptance, 'max_lateral_error') || isempty(acceptance.max_lateral_error)
    acceptance.max_lateral_error = Inf;
end
end

function params_file = resolve_base_params_file(config)
if ~isempty(config.base_params_file)
    params_file = resolve_project_path(config.project_root, config.base_params_file);
    return;
end

if ~isempty(config.base_params)
    params = config.base_params;
else
    params = create_vehicle_baseline_template();
end

params_file = fullfile(config.runtime_output_dir, 'base_params.json');
write_json(params_file, params);
end

function cases = build_attack_cases(config)
torque_values = double(config.driver_torque_max_values(:)');
speed_scales = double(config.torque_speed_scales(:)');
cases = repmat(struct( ...
    'case_id', '', ...
    'driver_torque_max', 0.0, ...
    'torque_speed_scale', 0.0), ...
    1, numel(torque_values) * numel(speed_scales));
case_count = 0;

for torque_idx = 1:numel(torque_values)
    for scale_idx = 1:numel(speed_scales)
        torque_max = torque_values(torque_idx);
        speed_scale = speed_scales(scale_idx);
        if torque_max <= 0.0
            error('driver_torque_max_values must be positive.');
        end
        if speed_scale <= 0.0
            error('torque_speed_scales must be positive.');
        end

        case_count = case_count + 1;
        cases(case_count).case_id = sprintf('torque%s_scale%s', ...
            value_token(torque_max), value_token(speed_scale));
        cases(case_count).driver_torque_max = torque_max;
        cases(case_count).torque_speed_scale = speed_scale;
    end
end
end

function token = value_token(value)
token = regexprep(sprintf('%.3f', value), '\.?0+$', '');
token = strrep(token, '.', 'p');
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

function [result, success] = execute_laptime(config, params_file, event, case_spec, trace_csv_path)
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

cmd = sprintf(['%s --track %s --time %.6g --params_file %s --driver %s ' ...
    '--driver-torque-max %.6g --torque-speed-scale %.6g --output_json --output %s %s'], ...
    shell_quote(exe), shell_quote(event.track_name), event.sim_time, ...
    shell_quote(params_file), shell_quote(config.driver_mode), ...
    case_spec.driver_torque_max, case_spec.torque_speed_scale, ...
    shell_quote(trace_csv_path), char(string(config.extra_cli_args)));
if event.initial_speed >= 0
    cmd = sprintf('%s --initial-speed %.6g', cmd, event.initial_speed);
end

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

function row = build_row(config, case_spec, event, result, success, runtime_sec, trace_csv_path)
if isempty(result)
    result = struct();
end
row = struct();
row.timestamp_utc = utc_now();
row.run_id = config.run_id;
row.case_id = case_spec.case_id;
row.event_name = event.name;
row.track_name = event.track_name;
row.driver_mode = config.driver_mode;
row.driver_torque_max = case_spec.driver_torque_max;
row.torque_speed_scale = case_spec.torque_speed_scale;
row.extra_cli_args = char(string(config.extra_cli_args));
row.success = logical(success);
row.lap_completed = logical(field_or(result, 'lap_completed', false));
row.status = char(string(field_or(result, 'status', 'failed')));
row.failure_code = char(string(field_or(result, 'failure_code', '')));
row.failure_message = char(string(field_or(result, 'failure_message', '')));
row.lap_time = double(field_or(result, 'lap_time', NaN));
row.distance_traveled = double(field_or(result, 'distance_traveled', NaN));
row.max_lateral_error = double(field_or(result, 'max_lateral_error', NaN));
row.avg_lateral_error = double(field_or(result, 'avg_lateral_error', NaN));
row.max_track_limit_excess = double(field_or(result, 'max_track_limit_excess', NaN));
row.max_speed = double(field_or(result, 'max_speed', NaN));
row.avg_speed = double(field_or(result, 'avg_speed', NaN));
row.rms_heading_error = double(field_or(result, 'rms_heading_error', NaN));
row.steering_saturation_ratio = double(field_or(result, 'steering_saturation_ratio', NaN));
row.mean_abs_sw_error = double(field_or(result, 'mean_abs_sw_error', NaN));
row.max_abs_sw_error = double(field_or(result, 'max_abs_sw_error', NaN));
row.mean_steering_lag_ratio = double(field_or(result, 'mean_steering_lag_ratio', NaN));
row.max_steering_lag_ratio = double(field_or(result, 'max_steering_lag_ratio', NaN));
row.avg_torque_speed_multiplier = double(field_or(result, 'avg_torque_speed_multiplier', NaN));
row.min_steering_actuator_speed_cap = double(field_or(result, 'min_steering_actuator_speed_cap', NaN));
row.min_track_margin_speed_cap = double(field_or(result, 'min_track_margin_speed_cap', NaN));
row.min_velocity_profile_steering_cap = double(field_or(result, 'min_velocity_profile_steering_cap', NaN));
row.velocity_profile_steering_cap_ratio = double(field_or(result, 'velocity_profile_steering_cap_ratio', NaN));
row.max_abs_phi_twist = double(field_or(result, 'max_abs_phi_twist', NaN));
row.max_abs_dphi_twist = double(field_or(result, 'max_abs_dphi_twist', NaN));
row.runtime_sec = runtime_sec;
row.trace_csv_path = trace_csv_path;
[accepted, reasons] = evaluate_acceptance(row, config.acceptance);
row.accepted = accepted;
row.acceptance_reasons = reasons;
end

function summary = summarize_attack_results(tbl, torque_values, summary_path)
summary = struct();
summary.sample_count = height(tbl);
summary.failure_count = sum(~logical(tbl.success));
summary.best_overall = table();
summary.best_accepted_overall = table();

if isempty(tbl)
    summary.table = table();
    return;
end

rows = struct([]);
for i = 1:numel(torque_values)
    torque_max = double(torque_values(i));
    st = tbl(abs(tbl.driver_torque_max - torque_max) < 1e-9, :);
    if isempty(st)
        continue;
    end

    success_mask = logical(st.success);
    success_rows = st(success_mask, :);
    accepted_mask = logical(st.accepted);
    accepted_rows = st(accepted_mask, :);
    rows(end + 1).driver_torque_max = torque_max; %#ok<AGROW>
    rows(end).case_count = height(st);
    rows(end).success_rate = mean(double(success_mask));
    rows(end).accepted_count = sum(double(accepted_mask));
    rows(end).accepted_rate = mean(double(accepted_mask));
    rows(end).best_lap_time = NaN;
    rows(end).best_torque_speed_scale = NaN;
    rows(end).best_accepted_lap_time = NaN;
    rows(end).best_accepted_torque_speed_scale = NaN;
    rows(end).min_lap_time_success = NaN;
    rows(end).max_track_limit_excess = max(st.max_track_limit_excess);
    rows(end).max_steering_saturation_ratio = max(st.steering_saturation_ratio);
    rows(end).max_mean_steering_lag_ratio = max(st.mean_steering_lag_ratio);

    if ~isempty(success_rows)
        [best_lap, idx] = min(success_rows.lap_time);
        rows(end).best_lap_time = best_lap;
        rows(end).best_torque_speed_scale = success_rows.torque_speed_scale(idx);
        rows(end).min_lap_time_success = best_lap;
    end
    if ~isempty(accepted_rows)
        [best_lap, idx] = min(accepted_rows.lap_time);
        rows(end).best_accepted_lap_time = best_lap;
        rows(end).best_accepted_torque_speed_scale = accepted_rows.torque_speed_scale(idx);
    end
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
    summary.best_overall = success_tbl(idx, :);
end

accepted_tbl = tbl(logical(tbl.accepted), :);
if ~isempty(accepted_tbl)
    [~, idx] = min(accepted_tbl.lap_time);
    summary.best_accepted_overall = accepted_tbl(idx, :);
end
end

function [accepted, reasons] = evaluate_acceptance(row, acceptance)
reasons = strings(0, 1);
if ~row.success
    reasons(end + 1) = "not_completed";
end
if row.max_track_limit_excess > acceptance.max_track_limit_excess
    reasons(end + 1) = "track_limit_excess";
end
if row.steering_saturation_ratio > acceptance.max_steering_saturation_ratio
    reasons(end + 1) = "steering_saturation";
end
if row.mean_steering_lag_ratio > acceptance.max_mean_steering_lag_ratio
    reasons(end + 1) = "steering_lag";
end
if row.max_lateral_error > acceptance.max_lateral_error
    reasons(end + 1) = "lateral_error";
end

accepted = isempty(reasons);
if accepted
    reasons = "accepted";
else
    reasons = strjoin(reasons, ";");
end
reasons = char(reasons);
end

function write_report(manifest, summary, config)
fid = fopen(manifest.report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', manifest.report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Driver Attack Sweep\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- driver_mode: `%s`\n', config.driver_mode);
fprintf(fid, '- base_params_file: `%s`\n', manifest.base_params_file);
fprintf(fid, '- samples_csv: `%s`\n', manifest.samples_csv_path);
fprintf(fid, '- summary_csv: `%s`\n', manifest.summary_csv_path);
fprintf(fid, '- trace_output_dir: `%s`\n\n', manifest.trace_output_dir);

fprintf(fid, '## Purpose\n\n');
fprintf(fid, ['Compare attack-lap steering torque assumptions and torque-mode speed ' ...
    'scales without changing driver control logic.\n\n']);

fprintf(fid, '## Acceptance Criteria\n\n');
fprintf(fid, '- max_track_limit_excess <= %.6g m\n', config.acceptance.max_track_limit_excess);
fprintf(fid, '- steering_saturation_ratio <= %.3f\n', config.acceptance.max_steering_saturation_ratio);
fprintf(fid, '- mean_steering_lag_ratio <= %.3f\n', config.acceptance.max_mean_steering_lag_ratio);
if isfinite(config.acceptance.max_lateral_error)
    fprintf(fid, '- max_lateral_error <= %.3f m\n\n', config.acceptance.max_lateral_error);
else
    fprintf(fid, '- max_lateral_error: not bounded beyond track_limit_excess\n\n');
end

fprintf(fid, '## Best Overall\n\n');
if ~isempty(summary.best_overall)
    b = summary.best_overall(1, :);
    fprintf(fid, '- driver_torque_max: %.3f Nm\n', b.driver_torque_max);
    fprintf(fid, '- torque_speed_scale: %.3f\n', b.torque_speed_scale);
    fprintf(fid, '- lap_time: %.3f s\n', b.lap_time);
    fprintf(fid, '- max_lateral_error: %.3f m\n', b.max_lateral_error);
    fprintf(fid, '- max_track_limit_excess: %.3f m\n', b.max_track_limit_excess);
    fprintf(fid, '- steering_saturation_ratio: %.4f\n\n', b.steering_saturation_ratio);
else
    fprintf(fid, 'No successful attack-lap case was found.\n\n');
end

fprintf(fid, '## Best Accepted Overall\n\n');
if ~isempty(summary.best_accepted_overall)
    b = summary.best_accepted_overall(1, :);
    fprintf(fid, '- driver_torque_max: %.3f Nm\n', b.driver_torque_max);
    fprintf(fid, '- torque_speed_scale: %.3f\n', b.torque_speed_scale);
    fprintf(fid, '- lap_time: %.3f s\n', b.lap_time);
    fprintf(fid, '- max_lateral_error: %.3f m\n', b.max_lateral_error);
    fprintf(fid, '- max_track_limit_excess: %.3f m\n', b.max_track_limit_excess);
    fprintf(fid, '- steering_saturation_ratio: %.4f\n\n', b.steering_saturation_ratio);
else
    fprintf(fid, 'No accepted attack-lap case was found.\n\n');
end

fprintf(fid, '## Torque Summary\n\n');
if isempty(summary.table)
    fprintf(fid, 'No attack rows were collected.\n');
else
    fprintf(fid, '| driver_torque_max | cases | success_rate | accepted_rate | best_lap_time | best_speed_scale | best_accepted_lap | best_accepted_scale | max_offtrack_excess | max_sat_ratio | max_lag_ratio |\n');
    fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
    for i = 1:height(summary.table)
        r = summary.table(i, :);
        fprintf(fid, '| %.3f | %d | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
            r.driver_torque_max, r.case_count, r.success_rate, r.accepted_rate, ...
            r.best_lap_time, r.best_torque_speed_scale, ...
            r.best_accepted_lap_time, r.best_accepted_torque_speed_scale, ...
            r.max_track_limit_excess, ...
            r.max_steering_saturation_ratio, r.max_mean_steering_lag_ratio);
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
    completed(make_key(tbl.case_id(i), tbl.event_name(i))) = true;
end
end

function key = make_key(case_id, event_name)
key = sprintf('%s::%s', char(string(case_id)), char(string(event_name)));
end

function value = field_or(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default_value;
end
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
fprintf(fid, '%s', jsonencode(data));
end

function out = utc_now()
out = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
end

function resolved = resolve_project_path(project_root, input_path)
input_path = char(string(input_path));
if startsWith(input_path, filesep) || (~ispc && startsWith(input_path, '~'))
    resolved = input_path;
elseif ispc && numel(input_path) >= 2 && input_path(2) == ':'
    resolved = input_path;
else
    resolved = fullfile(project_root, input_path);
end
end

function quoted = shell_quote(value)
value = char(string(value));
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
