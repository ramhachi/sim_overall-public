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

% Same evaluator as the GA (LPV only, per-vehicle GGV cap), so screening and
% optimisation rank designs with one objective.
config = merge_nested_structs(vehicle_ga_default_config(project_root), struct( ...
    'track_name', track_name, 'track_radius', track_param, ...
    'simulation_time_sec', sim_time, 'initial_speed_mps', initial_speed, ...
    'candidate_timeout_sec', timeout_sec));
config = merge_nested_structs(config, opt_or(opts, 'config', struct()));
config.tire_lut_file = vehicle_tire_lut(config);
context = struct('config', config, 'base_params', base_params, ...
    'design_context', design_context, 'target_distance', 1006.2611818354848);
batch_dir = fullfile(out_dir, 'candidates');
if ~exist(batch_dir, 'dir')
    mkdir(batch_dir);
end

baseline = evaluate_design(context, base_x, 'baseline', batch_dir);
if ~baseline.feasible
    error('run_design_sensitivity:baselineNotFeasible', ...
        'Baseline did not complete %s: %s (%s)', ...
        track_name, baseline.failure_code, baseline.failure_message);
end

points = struct('variable', {}, 'base_value', {}, 'value', {}, 'direction', {}, 'x', {});
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
        points(end + 1) = struct('variable', name, 'base_value', base_value, ... %#ok<AGROW>
            'value', values(k), 'direction', directions{k}, 'x', x);
    end
end
point_samples = cell(numel(points), 1);
parfor i = 1:numel(points)
    point_samples{i} = evaluate_design(context, points(i).x, ...
        sprintf('%s_%s', points(i).variable, points(i).direction), batch_dir);
end
rows = {make_row('baseline', 0.0, 0.0, 'baseline', baseline)};
for i = 1:numel(points)
    rows{end + 1, 1} = make_row(points(i).variable, points(i).base_value, ... %#ok<AGROW>
        points(i).value, points(i).direction, point_samples{i});
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
manifest.driver_mode = config.driver_mode;
manifest.use_lpv_ggv = config.use_lpv_ggv;
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

function sample = evaluate_design(context, x, candidate_id, batch_dir)
record = evaluate_vehicle_ga_candidate(x, context, candidate_id, batch_dir);
res = record.result;
sample = struct();
sample.success = record.return_code == 0;
sample.lap_completed = false;
sample.lap_time = NaN;
sample.max_lateral_error = NaN;
sample.max_track_limit_excess = NaN;
sample.failure_code = '';
sample.failure_message = '';
if isfield(record, 'error_message')
    sample.failure_code = 'evaluation_exception';
    sample.failure_message = record.error_message;
end
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
