function analysis = run_hybrid_driver_comparison(config)
% RUN_HYBRID_DRIVER_COMPARISON Compare driver variants on the same sweep.
%
% This is the evaluation entry point for driver-stack experiments. It runs
% run_driver_generalization_sweep once per driver variant and aggregates
% success rate, lap time, off-track behavior, and steering health metrics.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_project_paths(config.project_root);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);

variants = normalize_variants(config);
rows = [];
variant_results = struct([]);

for i = 1:numel(variants)
    variant = variants(i);
    sweep_config = build_sweep_config(config, variant);
    result = run_driver_generalization_sweep(sweep_config);

    variant_results = append_struct(variant_results, struct( ...
        'name', variant.name, ...
        'driver_mode', variant.driver_mode, ...
        'extra_cli_args', variant.extra_cli_args, ...
        'manifest_path', result.manifest_path, ...
        'samples_csv_path', result.samples_csv_path, ...
        'summary_csv_path', result.summary_csv_path, ...
        'report_path', result.report_path));

    samples = readtable(result.samples_csv_path, 'TextType', 'string');
    rows = append_struct(rows, summarize_variant(variant, samples));
end

comparison_table = struct2table(rows);
comparison_csv_path = fullfile(config.runtime_output_dir, 'hybrid_driver_comparison.csv');
writetable(comparison_table, comparison_csv_path);

manifest = struct();
manifest.schema_version = 'hybrid_driver_comparison/v1';
manifest.generated_by = 'run_hybrid_driver_comparison';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.runtime_output_dir = config.runtime_output_dir;
manifest.comparison_csv_path = comparison_csv_path;
manifest.variants = variant_results;
manifest_path = fullfile(config.runtime_output_dir, 'manifest.json');
write_json(manifest_path, manifest);

report_path = fullfile(config.report_output_dir, sprintf('hybrid_driver_comparison_%s.md', config.run_id));
write_report(report_path, manifest, comparison_table);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = manifest_path;
analysis.comparison_csv_path = comparison_csv_path;
analysis.report_path = report_path;
analysis.variants = variant_results;
analysis.summary = comparison_table;
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
        'driver_generalization', 'hybrid_comparison', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end
if ~isfield(config, 'extra_cli_args') || isempty(config.extra_cli_args)
    config.extra_cli_args = '';
end
if ~isfield(config, 'sweep')
    config.sweep = struct();
end
end

function variants = normalize_variants(config)
if isfield(config, 'variants') && ~isempty(config.variants)
    variants = config.variants;
else
    variants = struct( ...
        'name', {'pid_centerline', 'pid_line_planner', 'mpc_preview'}, ...
        'driver_mode', {'pid', 'pid', 'mpc'}, ...
        'extra_cli_args', {'', '--line-planner', ''});
end

for i = 1:numel(variants)
    variants(i).name = char(string(field_or(variants(i), 'name', sprintf('variant_%d', i))));
    variants(i).driver_mode = char(string(field_or(variants(i), 'driver_mode', 'pid')));
    variants(i).extra_cli_args = char(string(field_or(variants(i), 'extra_cli_args', '')));
end
end

function sweep_config = build_sweep_config(config, variant)
sweep_config = config.sweep;
sweep_config.project_root = config.project_root;
sweep_config.run_id = sprintf('%s_%s', config.run_id, variant.name);
sweep_config.runtime_output_dir = fullfile(config.runtime_output_dir, variant.name);
sweep_config.report_output_dir = config.report_output_dir;
sweep_config.params_output_dir = fullfile(sweep_config.runtime_output_dir, 'params');
sweep_config.driver_mode = variant.driver_mode;
sweep_config.extra_cli_args = strtrim(strjoin({config.extra_cli_args, variant.extra_cli_args}, ' '));
sweep_config.driver_variant_name = variant.name;
end

function row = summarize_variant(variant, samples)
row = struct();
row.variant = variant.name;
row.driver_mode = variant.driver_mode;
row.extra_cli_args = variant.extra_cli_args;
row.sample_count = height(samples);
success = logical(samples.success);
row.success_rate = mean(double(success));
row.failure_count = sum(~success);
row.mean_lap_time = mean_or_nan(samples.lap_time(success));
row.best_lap_time = min_or_nan(samples.lap_time(success));
row.baseline_lap_time = mean_or_nan(samples.lap_time(samples.sample_role == "baseline" & success));
row.max_track_limit_excess = max(samples.max_track_limit_excess);
row.max_lateral_error = max(samples.max_lateral_error);
row.max_steering_saturation_ratio = max(samples.steering_saturation_ratio);
row.mean_steering_lag_ratio = mean(samples.mean_steering_lag_ratio, 'omitnan');
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

function list = append_struct(list, item)
if isempty(list)
    list = item;
else
    list(end + 1) = item;
end
end

function write_report(report_path, manifest, comparison_table)
fid = fopen(report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Hybrid Driver Comparison\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- comparison_csv: `%s`\n\n', manifest.comparison_csv_path);

fprintf(fid, '| variant | driver | success_rate | baseline_lap | mean_lap | best_lap | max_offtrack | max_lat_err | max_sat | mean_lag |\n');
fprintf(fid, '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(comparison_table)
    row = comparison_table(i, :);
    fprintf(fid, '| %s | %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |\n', ...
        char(row.variant), char(row.driver_mode), row.success_rate, ...
        row.baseline_lap_time, row.mean_lap_time, row.best_lap_time, ...
        row.max_track_limit_excess, row.max_lateral_error, ...
        row.max_steering_saturation_ratio, row.mean_steering_lag_ratio);
end
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
