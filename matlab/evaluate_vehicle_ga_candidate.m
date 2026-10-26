function record = evaluate_vehicle_ga_candidate(full_design_vector, context, candidate_id, batch_dir, trace_path)
% EVALUATE_VEHICLE_GA_CANDIDATE Materialize and evaluate one LPV vehicle.

started = tic;
if nargin < 5
    trace_path = '';
end
params_path = fullfile(batch_dir, [candidate_id, '.params.json']);
try
params = apply_full_design(context.base_params, full_design_vector, context.design_context);
if exist('compute_vehicle_kc', 'file') ~= 2
    addpath(fullfile(context.config.project_root, 'suspension_kinematics_design'));
end
% K&C follows the candidate's own track/wheelbase/alignment every time.
params = compute_vehicle_kc(params);
params.drs.enabled = false;
params.tire.model_dir = fullfile('output', 'tire_models', context.config.tire_model_name);
params.tire.model_manifest = fullfile(params.tire.model_dir, 'manifest.json');
if ~isempty(context.config.tire_lut_file)
    params.tire.pinn_lut_file = context.config.tire_lut_file;
end

write_json_atomic(params_path, params);

trace_args = '';
if ~isempty(trace_path)
    trace_args = sprintf(' --output %s --output-rate 0.01', shell_quote(trace_path));
end
if context.config.use_lpv_ggv
    % Per-vehicle ay_max(v) for the LPV cap; without it the cap is a fixed
    % 0.86*mu_y_max*g and chassis changes barely move the lap time.
    trace_args = [trace_args, ' --lpv-ggv ', ...
        shell_quote(vehicle_lateral_ggv(params_path, params, context.config))];
end
command = sprintf([ ...
    'cd %s && python3 %s --timeout %.12g -- %s ', ...
    '--track %s --radius %.12g --time %.12g ', ...
    '--driver lpv_lqr --lpv-model steering6 --initial-speed %.12g ', ...
    '--params_file %s --output_json%s'], ...
    shell_quote(context.config.project_root), ...
    shell_quote(fullfile(context.config.project_root, 'scripts', 'run_process_with_timeout.py')), ...
    context.config.candidate_timeout_sec, ...
    shell_quote(context.config.binary_path), ...
    shell_quote(context.config.track_name), ...
    context.config.track_radius, ...
    context.config.simulation_time_sec, ...
    context.config.initial_speed_mps, ...
    shell_quote(params_path), ...
    trace_args);

[return_code, command_output] = system(command);
result = parse_json_result(command_output);
% Parallel search can be descheduled by the OS, so wall-clock maxima are only
% enforced during the final, isolated trace evaluation. The mean remains gated.
enforce_max_runtime = ~isempty(trace_path);
[cost, gate] = score_vehicle_ga_result( ...
    result, return_code, context.target_distance, enforce_max_runtime);

record = struct( ...
    'schema_version', 'vehicle_ga_evaluation/v1', ...
    'candidate_id', candidate_id, ...
    'design_vector', full_design_vector, ...
    'cost', cost, ...
    'gate', gate, ...
    'return_code', return_code, ...
    'duration_sec', toc(started), ...
    'params_file', params_path, ...
    'result', result);
write_json_atomic(fullfile(batch_dir, [candidate_id, '.result.json']), record);
catch evaluation_error
    record = struct( ...
        'schema_version', 'vehicle_ga_evaluation/v1', ...
        'candidate_id', candidate_id, ...
        'design_vector', full_design_vector, ...
        'cost', 1e9, ...
        'gate', struct('passed', false, 'violations', {{'evaluation_exception'}}), ...
        'return_code', -1, ...
        'duration_sec', toc(started), ...
        'params_file', params_path, ...
        'error_identifier', evaluation_error.identifier, ...
        'error_message', evaluation_error.message, ...
        'result', struct());
    write_json_atomic(fullfile(batch_dir, [candidate_id, '.result.json']), record);
end
end

function result = parse_json_result(output)
result = struct();
begin_marker = '%%JSON_BEGIN%%';
end_marker = '%%JSON_END%%';
begin_index = find(contains(string(splitlines(output)), begin_marker), 1, 'last');
lines = splitlines(string(output));
end_index = find(contains(lines, end_marker), 1, 'last');
if isempty(begin_index) || isempty(end_index) || end_index <= begin_index
    return;
end
payload = strjoin(lines(begin_index + 1:end_index - 1), newline);
try
    result = jsondecode(char(payload));
catch
    result = struct();
end
end

function quoted = shell_quote(value)
value = char(value);
if contains(value, '''')
    error('evaluate_vehicle_ga_candidate:unsupportedPath', ...
        'Paths containing a single quote are not supported.');
end
quoted = ['''', value, ''''];
end

function write_json_atomic(path, value)
staging = [path, '.writing'];
fid = fopen(staging, 'w');
if fid == -1
    error('evaluate_vehicle_ga_candidate:writeFailed', 'Cannot write %s.', staging);
end
cleanup = onCleanup(@() close_if_open(fid));
fwrite(fid, jsonencode(value), 'char');
fclose(fid);
clear cleanup;
movefile(staging, path, 'f');
end

function close_if_open(fid)
if fid >= 0
    try
        fclose(fid);
    catch
    end
end
end
