function analysis = benchmarkRLCheckpoints(config)
% BENCHMARKRLCHECKPOINTS Export and benchmark saved RL agent checkpoints.
%
% analysis = benchmarkRLCheckpoints(struct( ...
%     'checkpoint_dir', 'rl_results/...', ...
%     'episodes', [200 220 234]))
%
% The function evaluates checkpoints through the same C++ laptime_sim path used
% for controller comparison. Nonzero laptime_sim exit codes are accepted when a
% JSON payload is present, so incomplete laps are still recorded and ranked.

ensure_rl_driver_paths();

if nargin < 1 || isempty(config)
    config = struct();
end
if ~isstruct(config)
    error('benchmarkRLCheckpoints:InvalidConfig', 'config must be a struct.');
end

config = normalize_config(config);
ensure_dir(config.output_dir);
ensure_dir(config.actor_export_root);
ensure_dir(config.trace_output_dir);

if config.rebuild_laptime
    build_laptime(config);
end

checkpoints = resolve_checkpoints(config);
vehicle_params = resolve_export_vehicle_params(config);

manifest = struct();
manifest.schema_version = 'rl_checkpoint_benchmark/v1';
manifest.generated_by = 'benchmarkRLCheckpoints';
manifest.generated_at_utc = utc_now();
manifest.project_root = config.project_root;
manifest.checkpoint_dir = config.checkpoint_dir;
manifest.output_dir = config.output_dir;
manifest.actor_export_root = config.actor_export_root;
manifest.trace_output_dir = config.trace_output_dir;
manifest.samples_csv_path = fullfile(config.output_dir, 'checkpoint_benchmark.csv');
manifest.best_json_path = fullfile(config.output_dir, 'best_checkpoint.json');
manifest.params_file = config.params_file;
manifest.track = config.track;
manifest.sim_time = config.sim_time;
manifest.driver_torque_max = config.driver_torque_max;
manifest.driver_torque_rate_max = config.driver_torque_rate_max;
manifest.rl_steer_torque_rate_limit = config.rl_steer_torque_rate_limit;
manifest.checkpoints = checkpoints;
write_json(fullfile(config.output_dir, 'manifest.json'), manifest);

rows = repmat(empty_row(), 0, 1);
for i = 1:numel(checkpoints)
    checkpoint = checkpoints(i);
    actor_export_dir = fullfile(config.actor_export_root, sprintf('Agent%d', checkpoint.episode));
    trace_csv_path = fullfile(config.trace_output_dir, sprintf('Agent%d_%s.csv', ...
        checkpoint.episode, config.track));

    started = tic;
    try
        if isempty(config.execute_checkpoint_fn)
            export_checkpoint_actor(checkpoint.path, actor_export_dir, vehicle_params);
            [result, success, command_text, command_status] = run_laptime_checkpoint( ...
                config, actor_export_dir, trace_csv_path);
        else
            [result, success, command_text, command_status] = config.execute_checkpoint_fn( ...
                checkpoint, actor_export_dir, trace_csv_path, config);
        end
    catch ME
        result = struct( ...
            'status', 'failed', ...
            'failure_code', 'benchmark_exception', ...
            'failure_message', ME.message, ...
            'failure_time', NaN, ...
            'lap_completed', false, ...
            'lap_time', NaN);
        success = false;
        command_text = 'exception';
        command_status = -1;
    end
    runtime_sec = toc(started);

    row = build_row(config, checkpoint, actor_export_dir, trace_csv_path, ...
        result, success, command_text, command_status, runtime_sec);
    rows(end + 1) = row; %#ok<AGROW>
end

samples_table = struct2table(rows, 'AsArray', true);
writetable(samples_table, manifest.samples_csv_path);

best_row = select_best_checkpoint(samples_table);
write_json(manifest.best_json_path, best_row);

analysis = struct();
analysis.samples_csv_path = manifest.samples_csv_path;
analysis.best_json_path = manifest.best_json_path;
analysis.manifest_path = fullfile(config.output_dir, 'manifest.json');
analysis.output_dir = config.output_dir;
analysis.actor_export_root = config.actor_export_root;
analysis.trace_output_dir = config.trace_output_dir;
analysis.samples = samples_table;
analysis.best_checkpoint = best_row;
end

function config = normalize_config(config)
if ~isfield(config, 'project_root') || isempty(config.project_root)
    this_dir = fileparts(mfilename('fullpath'));
    config.project_root = fileparts(fileparts(this_dir));
else
    config.project_root = char(string(config.project_root));
end
config.project_root = char(java.io.File(config.project_root).getCanonicalPath());

if ~isfield(config, 'checkpoint_dir') || isempty(config.checkpoint_dir)
    config.checkpoint_dir = fullfile(config.project_root, 'rl_results', ...
        'autocross_winged_flexible_ft_v6_40Nm');
else
    config.checkpoint_dir = resolve_project_path(config.project_root, config.checkpoint_dir);
end

if ~isfield(config, 'run_id') || isempty(config.run_id)
    config.run_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~isfield(config, 'output_dir') || isempty(config.output_dir)
    config.output_dir = fullfile(config.project_root, 'output', 'runtime', ...
        'rl_checkpoint_benchmark', config.run_id);
else
    config.output_dir = resolve_project_path(config.project_root, config.output_dir);
end
if ~isfield(config, 'actor_export_root') || isempty(config.actor_export_root)
    config.actor_export_root = fullfile(config.output_dir, 'actors');
else
    config.actor_export_root = resolve_project_path(config.project_root, config.actor_export_root);
end
if ~isfield(config, 'trace_output_dir') || isempty(config.trace_output_dir)
    config.trace_output_dir = fullfile(config.output_dir, 'traces');
else
    config.trace_output_dir = resolve_project_path(config.project_root, config.trace_output_dir);
end
if ~isfield(config, 'params_file') || isempty(config.params_file)
    config.params_file = fullfile(config.project_root, 'matlab', 'parameter_presets', ...
        'params_winged_flexible.json');
else
    config.params_file = resolve_project_path(config.project_root, config.params_file);
end
if ~isfield(config, 'track') || isempty(config.track)
    config.track = 'autocross';
else
    config.track = char(string(config.track));
end
if ~isfield(config, 'sim_time') || isempty(config.sim_time)
    config.sim_time = 180.0;
end
if ~isfield(config, 'initial_speed') || isempty(config.initial_speed)
    config.initial_speed = -1.0;
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
if ~isfield(config, 'extra_cli_args') || isempty(config.extra_cli_args)
    config.extra_cli_args = '';
end
if ~isfield(config, 'episodes') || isempty(config.episodes)
    config.episodes = [];
else
    config.episodes = double(config.episodes(:)');
end
if ~isfield(config, 'checkpoint_paths') || isempty(config.checkpoint_paths)
    config.checkpoint_paths = {};
elseif ischar(config.checkpoint_paths) || isstring(config.checkpoint_paths)
    config.checkpoint_paths = cellstr(string(config.checkpoint_paths));
end
if ~isfield(config, 'max_checkpoints') || isempty(config.max_checkpoints)
    config.max_checkpoints = Inf;
end
if ~isfield(config, 'rebuild_laptime') || isempty(config.rebuild_laptime)
    config.rebuild_laptime = false;
end
if ~isfield(config, 'execute_checkpoint_fn')
    config.execute_checkpoint_fn = [];
end
end

function checkpoints = resolve_checkpoints(config)
if ~isempty(config.checkpoint_paths)
    paths = cellfun(@(p) resolve_project_path(config.project_root, p), ...
        config.checkpoint_paths, 'UniformOutput', false);
else
    if ~isfolder(config.checkpoint_dir)
        error('benchmarkRLCheckpoints:MissingCheckpointDir', ...
            'Checkpoint directory does not exist: %s', config.checkpoint_dir);
    end
    files = dir(fullfile(config.checkpoint_dir, 'Agent*.mat'));
    paths = fullfile({files.folder}, {files.name});
end

episodes = zeros(1, numel(paths));
keep = true(1, numel(paths));
for i = 1:numel(paths)
    if ~isfile(paths{i})
        error('benchmarkRLCheckpoints:MissingCheckpoint', ...
            'Checkpoint file does not exist: %s', paths{i});
    end
    episodes(i) = extract_episode(paths{i});
    if ~isempty(config.episodes)
        keep(i) = any(episodes(i) == config.episodes);
    end
end

paths = paths(keep);
episodes = episodes(keep);
if isempty(paths)
    error('benchmarkRLCheckpoints:NoCheckpoints', 'No checkpoints matched the request.');
end

[episodes, order] = sort(episodes);
paths = paths(order);
if isfinite(config.max_checkpoints) && numel(paths) > config.max_checkpoints
    start_idx = numel(paths) - config.max_checkpoints + 1;
    paths = paths(start_idx:end);
    episodes = episodes(start_idx:end);
end

checkpoints = repmat(struct('path', '', 'episode', 0), 1, numel(paths));
for i = 1:numel(paths)
    checkpoints(i).path = paths{i};
    checkpoints(i).episode = episodes(i);
end
end

function episode = extract_episode(path_in)
[~, name] = fileparts(path_in);
token = regexp(name, '^Agent(\d+)$', 'tokens', 'once');
if isempty(token)
    error('benchmarkRLCheckpoints:InvalidCheckpointName', ...
        'Checkpoint name must match Agent<N>.mat: %s', path_in);
end
episode = str2double(token{1});
end

function vehicle_params = resolve_export_vehicle_params(config)
raw = jsondecode(fileread(config.params_file));
if isfield(raw, 'params')
    assert_valid_vehicle_parameter_artifact(raw);
    vehicle_params = raw.params;
else
    vehicle_params = raw;
end
if ~isfield(vehicle_params, 'steering')
    vehicle_params.steering = struct();
end
vehicle_params.steering.torque_mode = true;
vehicle_params.steering.T_driver_max = double(config.driver_torque_max);
vehicle_params.steering.T_driver_rate_max = double(config.driver_torque_rate_max);
end

function export_checkpoint_actor(checkpoint_path, actor_export_dir, vehicle_params)
[agent, ~, ~] = loadRLAgentFromFile(checkpoint_path);
if ~isfolder(actor_export_dir)
    mkdir(actor_export_dir);
end
exportActorForCpp(agent, actor_export_dir, vehicle_params);
end

function [result, success, command_text, status] = run_laptime_checkpoint( ...
    config, actor_export_dir, trace_csv_path)
exe = fullfile(config.project_root, 'build', 'cpp', 'laptime_sim');
if ispc
    exe = [exe, '.exe'];
end
if ~isfile(exe)
    build_laptime(config);
end

command_text = sprintf(['%s --track %s --time %.6g --params_file %s --driver rl ' ...
    '--rl-actor-dir %s --driver-torque-max %.6g --driver-torque-rate-max %.6g ' ...
    '--output_json --output %s'], ...
    shell_quote(exe), shell_quote(config.track), config.sim_time, ...
    shell_quote(config.params_file), shell_quote(actor_export_dir), ...
    config.driver_torque_max, config.driver_torque_rate_max, shell_quote(trace_csv_path));
if config.initial_speed >= 0
    command_text = sprintf('%s --initial-speed %.6g', command_text, config.initial_speed);
end
if config.rl_steer_torque_rate_limit >= 0.0
    command_text = sprintf('%s --rl-steer-torque-rate-limit %.6g', ...
        command_text, config.rl_steer_torque_rate_limit);
end
command_text = strtrim(strjoin({command_text, config.extra_cli_args}, ' '));

previous_dir = pwd;
cleanup = onCleanup(@() cd(previous_dir));
cd(config.project_root);
[status, out] = system(command_text);
result = parse_laptime_result(out);
success = (status == 0) && isfield(result, 'lap_completed') && logical(result.lap_completed);
end

function result = parse_laptime_result(out)
start_token = '%%JSON_BEGIN%%';
end_token = '%%JSON_END%%';
start_idx = strfind(out, start_token);
end_idx = strfind(out, end_token);
if isempty(start_idx) || isempty(end_idx)
    error('benchmarkRLCheckpoints:MissingJson', ...
        'No JSON result markers found in laptime output:\n%s', out);
end
json_text = out(start_idx(1) + strlength(start_token):end_idx(end) - 1);
result = jsondecode(strtrim(json_text));
end

function row = build_row(config, checkpoint, actor_export_dir, trace_csv_path, ...
    result, success, command_text, command_status, runtime_sec)
row = empty_row();
row.timestamp_utc = utc_now();
row.run_id = config.run_id;
row.episode = checkpoint.episode;
row.checkpoint_path = checkpoint.path;
row.actor_export_dir = actor_export_dir;
row.trace_csv_path = trace_csv_path;
row.success = logical(success);
row.command_status = double(command_status);
row.status = char(string(field_or(result, 'status', 'failed')));
row.failure_code = char(string(field_or(result, 'failure_code', '')));
row.failure_message = char(string(field_or(result, 'failure_message', '')));
row.failure_time = double(field_or(result, 'failure_time', NaN));
row.lap_completed = logical(field_or(result, 'lap_completed', false));
row.lap_time = double(field_or(result, 'lap_time', NaN));
row.distance_traveled = double(field_or(result, 'distance_traveled', NaN));
row.max_lateral_error = double(field_or(result, 'max_lateral_error', NaN));
row.max_track_limit_excess = double(field_or(result, 'max_track_limit_excess', NaN));
row.max_speed = double(field_or(result, 'max_speed', NaN));
row.avg_speed = double(field_or(result, 'avg_speed', NaN));
row.rms_heading_error = double(field_or(result, 'rms_heading_error', NaN));
row.steering_smoothness = double(field_or(result, 'steering_smoothness', NaN));
row.throttle_smoothness = double(field_or(result, 'throttle_smoothness', NaN));
row.brake_smoothness = double(field_or(result, 'brake_smoothness', NaN));
row.steering_saturation_ratio = double(field_or(result, 'steering_saturation_ratio', NaN));
row.mean_steering_lag_ratio = double(field_or(result, 'mean_steering_lag_ratio', NaN));
row.max_steering_lag_ratio = double(field_or(result, 'max_steering_lag_ratio', NaN));
row.dominant_loss = char(string(field_or(result, 'dominant_driver_loss', ...
    field_or(result, 'dominant_loss', ''))));
row.runtime_sec = runtime_sec;
row.command = command_text;
row.selection_score = compute_selection_score(row);
end

function row = empty_row()
row = struct( ...
    'timestamp_utc', '', ...
    'run_id', '', ...
    'episode', 0, ...
    'checkpoint_path', '', ...
    'actor_export_dir', '', ...
    'trace_csv_path', '', ...
    'success', false, ...
    'command_status', NaN, ...
    'status', '', ...
    'failure_code', '', ...
    'failure_message', '', ...
    'failure_time', NaN, ...
    'lap_completed', false, ...
    'lap_time', NaN, ...
    'distance_traveled', NaN, ...
    'max_lateral_error', NaN, ...
    'max_track_limit_excess', NaN, ...
    'max_speed', NaN, ...
    'avg_speed', NaN, ...
    'rms_heading_error', NaN, ...
    'steering_smoothness', NaN, ...
    'throttle_smoothness', NaN, ...
    'brake_smoothness', NaN, ...
    'steering_saturation_ratio', NaN, ...
    'mean_steering_lag_ratio', NaN, ...
    'max_steering_lag_ratio', NaN, ...
    'dominant_loss', '', ...
    'runtime_sec', NaN, ...
    'command', '', ...
    'selection_score', -Inf);
end

function score = compute_selection_score(row)
if row.lap_completed && isfinite(row.lap_time)
    score = 1.0e9 - row.lap_time;
    return;
end
distance = nan_to_zero(row.distance_traveled);
failure_time = nan_to_zero(row.failure_time);
track_excess = max(nan_to_zero(row.max_track_limit_excess), 0.0);
smoothness_penalty = 0.01 * nan_to_zero(row.steering_smoothness);
score = distance + 0.1 * failure_time - 100.0 * track_excess - smoothness_penalty;
end

function best_row = select_best_checkpoint(tbl)
if isempty(tbl)
    best_row = struct();
    return;
end
[~, idx] = max(tbl.selection_score);
best_row = table2struct(tbl(idx, :));
end

function build_laptime(config)
build_cmd = sprintf('cmake --build %s --target laptime_sim', ...
    shell_quote(fullfile(config.project_root, 'build')));
previous_dir = pwd;
cleanup = onCleanup(@() cd(previous_dir));
cd(config.project_root);
[status, out] = system(build_cmd);
if status ~= 0
    error('benchmarkRLCheckpoints:BuildFailed', ...
        'Failed to build laptime_sim:\n%s', out);
end
end

function value = field_or(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default_value;
end
end

function value = nan_to_zero(value)
if isnan(value)
    value = 0.0;
end
end

function ensure_dir(path_in)
if ~exist(path_in, 'dir')
    mkdir(path_in);
end
end

function write_json(path_in, data)
parent = fileparts(path_in);
if ~isempty(parent) && ~exist(parent, 'dir')
    mkdir(parent);
end
fid = fopen(path_in, 'w');
if fid == -1
    error('benchmarkRLCheckpoints:WriteFailed', 'Failed to open %s.', path_in);
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
