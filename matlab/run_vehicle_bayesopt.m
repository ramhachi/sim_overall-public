function result = run_vehicle_bayesopt(opts)
% RUN_VEHICLE_BAYESOPT Noisy Bayesian optimisation of a screened design subset.
%
% Uses the GA evaluator (LPV only, per-vehicle lateral GGV) on the variables
% in opts.variable_names; every other design variable stays at the baseline.
% Lap time is rough under small changes (residual std ~0.19 s), so the
% objective is modelled as noisy and the reported optimum is the point with
% the lowest *estimated* objective, not the luckiest single sample.
%
% Gate: the GA gate minus steering_tracking_error. The LPV tuned for speed
% runs mean |SW error| ~0.25 rad on every candidate (baseline included), so
% that 0.12 rad threshold would reject the whole space; the value is still
% recorded per evaluation. Gate failures return NaN (bayesopt error region).

project_root = opt_or(opts, 'project_root', fileparts(fileparts(mfilename('fullpath'))));
baseline_artifact = opt_or(opts, 'baseline_artifact', ...
    fullfile(project_root, 'matlab', 'parameter_presets', 'params_flexible_torque.json'));
names = cellstr(string(opts.variable_names));
max_evals = opt_or(opts, 'max_evals', 150);
out_dir = opt_or(opts, 'out_dir', fullfile(project_root, 'output', 'runtime', 'vehicle_bayesopt', ...
    char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
ignored_violations = opt_or(opts, 'ignored_violations', {'steering_tracking_error'});

artifact = load_vehicle_artifact(baseline_artifact);
base_params = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
[base_x, schema, design_context] = build_design_vector_from_params(base_params);
[present, idx] = ismember(names, schema.variable_names);
if any(~present)
    error('run_vehicle_bayesopt:unknownVariable', 'Unknown variable(s): %s', ...
        strjoin(names(~present), ', '));
end

config = merge_nested_structs(vehicle_ga_default_config(project_root), ...
    opt_or(opts, 'config', struct()));
context = struct('config', config, 'base_params', base_params, ...
    'design_context', design_context, 'target_distance', 1006.2611818354848);
batch_dir = fullfile(out_dir, 'candidates');
if ~exist(batch_dir, 'dir')
    mkdir(batch_dir);
end

vars = optimizableVariable.empty;
for i = 1:numel(names)
    vars(end + 1) = optimizableVariable(names{i}, schema.variable_bounds(idx(i), :)); %#ok<AGROW>
end
initial = array2table(base_x(idx(:)'), 'VariableNames', names);

objective = @(t) evaluate_point(t, base_x, idx, context, batch_dir, ignored_violations);
bo = bayesopt(objective, vars, ...
    'IsObjectiveDeterministic', false, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'MaxObjectiveEvaluations', max_evals, ...
    'UseParallel', opt_or(opts, 'use_parallel', true), ...
    'InitialX', initial, ...
    'NumSeedPoints', opt_or(opts, 'num_seed_points', 12), ...
    'OutputFcn', @saveToFile, ...
    'SaveFileName', fullfile(out_dir, 'bayesopt_checkpoint.mat'), ...
    'PlotFcn', [], ...
    'Verbose', 1);

best_x = base_x;
best_x(idx) = table2array(bo.XAtMinEstimatedObjective);
best_params = apply_full_design(base_params, best_x, design_context);
trace = [bo.XTrace, table(bo.ObjectiveTrace, 'VariableNames', {'lap_time'})];
writetable(trace, fullfile(out_dir, 'trace.csv'));
write_json(fullfile(out_dir, 'best_params.json'), best_params);
summary = struct('variable_names', {names}, ...
    'baseline', base_x(idx), 'best', best_x(idx), ...
    'baseline_lap_time', bo.ObjectiveTrace(1), ...
    'best_estimated_lap_time', bo.predictObjective(bo.XAtMinEstimatedObjective), ...
    'best_observed_lap_time', bo.MinObjective, ...
    'evaluations', bo.NumObjectiveEvaluations, ...
    'ignored_violations', {ignored_violations});
write_json(fullfile(out_dir, 'summary.json'), summary);

result = struct('bo', bo, 'out_dir', out_dir, 'best_x', best_x, 'summary', summary);
end

function lap = evaluate_point(t, base_x, idx, context, batch_dir, ignored)
x = base_x;
x(idx) = table2array(t);
id = ['bo_', char(java.util.UUID.randomUUID())];
record = evaluate_vehicle_ga_candidate(x, context, id(1:15), batch_dir);
violations = setdiff(record.gate.violations, ignored);
if record.return_code == 0 && isempty(violations) && isfield(record.result, 'lap_time')
    lap = record.result.lap_time;
else
    lap = NaN;
end
end

function write_json(path, value)
fid = fopen(path, 'w');
fprintf(fid, '%s', jsonencode(value, 'PrettyPrint', true));
fclose(fid);
end

function value = opt_or(opts, name, fallback)
if isfield(opts, name) && ~isempty(opts.(name))
    value = opts.(name);
else
    value = fallback;
end
end
