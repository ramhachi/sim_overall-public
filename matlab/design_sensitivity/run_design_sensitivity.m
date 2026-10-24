function result = run_design_sensitivity(opts)
% RUN_DESIGN_SENSITIVITY Minimal design-vector OAT harness for GA screening.
%
% The baseline is mandatory: if it cannot complete the selected event, the
% sweep aborts instead of producing a NaN performance summary.

if nargin < 1 || isempty(opts)
    opts = struct();
end

project_root = opt_or(opts, 'project_root', fileparts(fileparts(fileparts(mfilename('fullpath')))));
baseline_artifact = opt_or(opts, 'baseline_artifact', '');
if isempty(baseline_artifact)
    baseline_artifact = fullfile(project_root, 'matlab', 'parameter_presets', 'params_flexible_torque.json');
end
step_fraction = opt_or(opts, 'step_fraction', 0.05);
min_step_fraction = opt_or(opts, 'min_step_fraction_of_range', 0.02);
max_variables = opt_or(opts, 'max_variables', 0);
track_name = opt_or(opts, 'track_name', 'autocross');
track_param = opt_or(opts, 'track_param', 0.0);
sim_time = opt_or(opts, 'sim_time', 180.0);
initial_speed = opt_or(opts, 'initial_speed', 6.0);
driver_mode = opt_or(opts, 'driver_mode', 'pid');
timeout_sec = opt_or(opts, 'timeout_sec', 300);
freeze_below_pct = opt_or(opts, 'freeze_below_pct', 0.03);

artifact = load_vehicle_artifact(baseline_artifact);
base_params = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
[base_x, schema, design_context] = build_design_vector_from_params(base_params);

names = schema.variable_names;
if isfield(opts, 'variable_names') && ~isempty(opts.variable_names)
    requested = cellstr(string(opts.variable_names));
    [present, loc] = ismember(requested, names);
    if any(~present)
        error('run_design_sensitivity:unknownVariable', ...
            'Unknown variable(s): %s', strjoin(requested(~present), ', '));
    end
    names = names(loc);
end
if max_variables > 0
    names = names(1:min(max_variables, numel(names)));
end

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if isfield(opts, 'out_dir') && ~isempty(opts.out_dir)
    out_dir = char(opts.out_dir);
else
    out_dir = fullfile(project_root, 'output', 'runtime', 'design_sensitivity', stamp);
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

runner = VehicleSimRunner(project_root);
baseline = evaluate_design(runner, base_params, design_context, base_x, ...
    track_name, track_param, sim_time, initial_speed, driver_mode, timeout_sec);
if ~baseline.feasible
    error('run_design_sensitivity:baselineNotFeasible', ...
        'Baseline did not complete %s: %s (%s)', ...
        track_name, baseline.failure_code, baseline.failure_message);
end

rows = {make_row('baseline', 0.0, 0.0, 'baseline', baseline)};
for i = 1:numel(names)
    name = names{i};
    idx = find(strcmp(schema.variable_names, name), 1, 'first');
    base_value = base_x(idx);
    range = schema.variable_bounds(idx, 2) - schema.variable_bounds(idx, 1);
    step = max(abs(base_value) * step_fraction, range * min_step_fraction);
    values = [max(schema.variable_bounds(idx, 1), base_value - step), ...
              min(schema.variable_bounds(idx, 2), base_value + step)];
    directions = {'minus', 'plus'};
    for k = 1:2
        if values(k) == base_value
            continue;
        end
        x = base_x;
        x(idx) = values(k);
        sample = evaluate_design(runner, base_params, design_context, x, ...
            track_name, track_param, sim_time, initial_speed, driver_mode, timeout_sec);
        rows{end + 1, 1} = make_row(name, base_value, values(k), directions{k}, sample); %#ok<AGROW>
    end
end

samples = struct2table(vertcat(rows{:}));
samples_path = fullfile(out_dir, 'samples.csv');
writetable(samples, samples_path);

summary = summarize_oats(samples, baseline.lap_time, freeze_below_pct);
summary_path = fullfile(out_dir, 'summary.csv');
writetable(summary, summary_path);
freeze_candidates = summary.variable(summary.freeze_candidate);
freeze_path = fullfile(out_dir, 'freeze_candidates.txt');
fid = fopen(freeze_path, 'w');
for i = 1:numel(freeze_candidates)
    fprintf(fid, '%s\n', char(freeze_candidates(i)));
end
fclose(fid);

manifest = struct();
manifest.schema_version = 'design_sensitivity/v1';
manifest.created_at = char(datetime('now', 'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
manifest.baseline_artifact = baseline_artifact;
manifest.track_name = track_name;
manifest.track_param = track_param;
manifest.sim_time = sim_time;
manifest.initial_speed = initial_speed;
manifest.driver_mode = driver_mode;
manifest.step_fraction = step_fraction;
manifest.min_step_fraction_of_range = min_step_fraction;
manifest.freeze_below_pct = freeze_below_pct;
manifest.min_step_fraction_of_range = min_step_fraction;
manifest.design_vector = base_x;
manifest.variable_names = names;
manifest.variable_bounds = schema.variable_bounds;
manifest.baseline_lap_time = baseline.lap_time;
manifest.baseline_feasible = baseline.feasible;
manifest.samples_path = samples_path;
manifest.summary_path = summary_path;
manifest.freeze_candidates_path = freeze_path;
manifest.freeze_candidates = freeze_candidates;
manifest_path = fullfile(out_dir, 'manifest.json');
fid = fopen(manifest_path, 'w');
fprintf(fid, '%s', jsonencode(manifest));
fclose(fid);

result = struct();
result.out_dir = out_dir;
result.samples = samples;
result.summary = summary;
result.manifest_path = manifest_path;
result.baseline = baseline;
fprintf('baseline lap: %.3f s, %d/%d feasible samples\n', ...
    baseline.lap_time, sum(samples.feasible), height(samples));
fprintf('samples: %s\nsummary: %s\nfreeze candidates: %s\n', samples_path, summary_path, freeze_path);
end

function sample = evaluate_design(runner, base_params, design_context, x, ...
    track_name, track_param, sim_time, initial_speed, driver_mode, timeout_sec)
params = apply_full_design(base_params, x, design_context);
runner_driver = driver_mode;
extra_args = '';
if strcmpi(driver_mode, 'lpv_lqr')
    % VehicleSimRunner exposes PID/RL directly; the LPV driver is selected
    % through the underlying CLI. Later --driver arguments take precedence.
    runner_driver = 'pid';
    extra_args = '--driver lpv_lqr';
end
[res, success] = runner.run(params, track_name, track_param, sim_time, ...
    'driver_mode', runner_driver, ...
    'initial_speed', initial_speed, ...
    'timeout_sec', timeout_sec, ...
    'extra_cli_args', extra_args);
sample = struct();
sample.success = logical(success);
sample.lap_completed = false;
sample.lap_time = NaN;
sample.max_lateral_error = NaN;
sample.max_track_limit_excess = NaN;
sample.failure_code = '';
sample.failure_message = '';
if isstruct(res) && ~isempty(res)
    if isfield(res, 'lap_completed'), sample.lap_completed = logical(res.lap_completed); end
    if isfield(res, 'lap_time'), sample.lap_time = double(res.lap_time); end
    if isfield(res, 'max_lateral_error'), sample.max_lateral_error = double(res.max_lateral_error); end
    if isfield(res, 'max_track_limit_excess'), sample.max_track_limit_excess = double(res.max_track_limit_excess); end
    if isfield(res, 'failure_code'), sample.failure_code = char(string(res.failure_code)); end
    if isfield(res, 'failure_message'), sample.failure_message = char(string(res.failure_message)); end
end
sample.feasible = sample.success && sample.lap_completed ...
    && isfinite(sample.lap_time) && sample.lap_time > 0.0;
end

function row = make_row(variable, base_value, applied_value, direction, sample)
row = struct();
row.variable = string(variable);
row.base_value = base_value;
row.applied_value = applied_value;
row.direction = direction;
row.success = sample.success;
row.lap_completed = sample.lap_completed;
row.feasible = sample.feasible;
row.lap_time = sample.lap_time;
row.max_lateral_error = sample.max_lateral_error;
row.max_track_limit_excess = sample.max_track_limit_excess;
row.failure_code = sample.failure_code;
row.failure_message = sample.failure_message;
end

function summary = summarize_oats(samples, baseline_lap, freeze_below_pct)
names = unique(samples.variable(samples.variable ~= "baseline"), 'stable');
rows = cell(0, 1);
for i = 1:numel(names)
    name = names{i};
    sub = samples(samples.variable == name, :);
    feasible = sub(sub.feasible, :);
    if isempty(feasible)
        max_abs_pct = NaN;
        best_direction = "";
    else
        delta_pct = (feasible.lap_time - baseline_lap) / baseline_lap * 100.0;
        max_abs_pct = max(abs(delta_pct));
        [~, best] = min(feasible.lap_time);
        best_direction = string(feasible.direction(best));
    end
    row = struct();
    row.variable = name;
    row.base_value = sub.base_value(1);
    row.samples = height(sub);
    row.feasible = sum(sub.feasible);
    row.failure_rate = 1.0 - sum(sub.feasible) / max(height(sub), 1);
    row.max_abs_delta_pct = max_abs_pct;
    row.best_direction = best_direction;
    row.freeze_candidate = isfinite(max_abs_pct) && max_abs_pct < freeze_below_pct;
    rows{end + 1, 1} = row; %#ok<AGROW>
end
summary = struct2table(vertcat(rows{:}));
score = summary.max_abs_delta_pct;
score(isnan(score)) = -inf;
[~, order] = sort(score, 'descend');
summary = summary(order, :);
end

function value = opt_or(opts, name, fallback)
if isfield(opts, name) && ~isempty(opts.(name))
    value = opts.(name);
else
    value = fallback;
end
end
