function analysis = analyze_lap_speed_loss(config)
% ANALYZE_LAP_SPEED_LOSS Summarize where torque-mode PID loses lap speed.
%
% The input is a CSV trace from cpp/laptime_sim --output. The report separates
% planner/preview speed loss from controller-side emergency loss so that
% steering lag and saturation fixes can be targeted at the right track sectors.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);

trace = read_trace(config.trace_csv_path);
segments = build_segment_table(trace, config.segment_length_m);
summary = build_summary_table(trace, segments, config);
top_segments = top_loss_segments(segments, config.top_segment_count);

summary_path = fullfile(config.runtime_output_dir, 'lap_speed_loss_summary.csv');
segments_path = fullfile(config.runtime_output_dir, 'lap_speed_loss_segments.csv');
manifest_path = fullfile(config.runtime_output_dir, 'lap_speed_loss_manifest.json');
report_path = fullfile(config.report_output_dir, sprintf('lap_speed_loss_%s.md', config.run_id));

writetable(summary, summary_path);
writetable(segments, segments_path);

manifest = struct();
manifest.schema_version = 'lap_speed_loss/v1';
manifest.generated_by = 'analyze_lap_speed_loss';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.trace_csv_path = config.trace_csv_path;
manifest.summary_csv_path = summary_path;
manifest.segments_csv_path = segments_path;
manifest.report_path = report_path;
manifest.segment_length_m = config.segment_length_m;
write_json(manifest_path, manifest);
write_report(report_path, manifest, summary, top_segments, config);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = manifest_path;
analysis.summary_csv_path = summary_path;
analysis.segments_csv_path = segments_path;
analysis.report_path = report_path;
analysis.summary = summary;
analysis.segments = segments;
analysis.top_segments = top_segments;
end

function config = normalize_config(config)
if ~isfield(config, 'project_root') || isempty(config.project_root)
    this_file = mfilename('fullpath');
    config.project_root = fileparts(fileparts(fileparts(this_file)));
end
if ~isfield(config, 'run_id') || isempty(config.run_id)
    config.run_id = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
end
if ~isfield(config, 'trace_csv_path') || isempty(config.trace_csv_path)
    error('config.trace_csv_path is required.');
end
config.trace_csv_path = char(string(config.trace_csv_path));
if ~isfield(config, 'runtime_output_dir') || isempty(config.runtime_output_dir)
    config.runtime_output_dir = fullfile(config.project_root, 'output', 'runtime', ...
        'driver_generalization', 'lap_speed_loss', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end
config.segment_length_m = double(field_or(config, 'segment_length_m', 25.0));
config.top_segment_count = double(field_or(config, 'top_segment_count', 12));
config.reference_lap_time_s = double(field_or(config, 'reference_lap_time_s', 66.0));
if config.segment_length_m <= 0.0
    error('segment_length_m must be positive.');
end
end

function trace = read_trace(path)
if ~isfile(path)
    error('Trace CSV does not exist: %s', path);
end

tbl = readtable(path, 'TextType', 'string');
required = [ ...
    "time", "s", "speed", ...
    "target_speed_base", "target_speed_preview", "target_speed_final", ...
    "torque_speed_multiplier", "steering_lag_ratio", "steering_saturation", ...
    "sw_error", "steering_actuator_speed_cap", "track_margin_speed_cap", ...
    "lateral_error", "heading_error_deg", "kappa"];
missing = required(~ismember(required, string(tbl.Properties.VariableNames)));
if ~isempty(missing)
    error('Trace CSV is missing required columns: %s', strjoin(missing, ', '));
end

time = numeric_column(tbl, 'time');
s = numeric_column(tbl, 's');
speed = numeric_column(tbl, 'speed');
valid = isfinite(time) & isfinite(s) & isfinite(speed);
tbl = tbl(valid, :);
if isempty(tbl)
    error('Trace CSV has no finite time/s/speed rows: %s', path);
end

trace = struct();
trace.table = tbl;
trace.time = numeric_column(tbl, 'time');
trace.s = numeric_column(tbl, 's');
trace.speed = numeric_column(tbl, 'speed');
trace.target_speed_base = numeric_column(tbl, 'target_speed_base');
trace.target_speed_preview = numeric_column(tbl, 'target_speed_preview');
trace.target_speed_final = numeric_column(tbl, 'target_speed_final');
trace.torque_speed_multiplier = numeric_column(tbl, 'torque_speed_multiplier');
trace.steering_lag_ratio = numeric_column(tbl, 'steering_lag_ratio');
trace.steering_saturation = numeric_column(tbl, 'steering_saturation');
trace.sw_error = numeric_column(tbl, 'sw_error');
trace.effective_max_tire_steer_rate = optional_numeric_column(tbl, 'effective_max_tire_steer_rate');
trace.steering_actuator_speed_cap = numeric_column(tbl, 'steering_actuator_speed_cap');
trace.track_margin_speed_cap = numeric_column(tbl, 'track_margin_speed_cap');
trace.track_margin_effective_margin = optional_numeric_column(tbl, 'track_margin_effective_margin');
trace.track_margin_outward_rate = optional_numeric_column(tbl, 'track_margin_outward_rate');
trace.lateral_error = numeric_column(tbl, 'lateral_error');
trace.heading_error_deg = numeric_column(tbl, 'heading_error_deg');
trace.kappa = numeric_column(tbl, 'kappa');
trace.required_delta_preview = optional_numeric_column(tbl, 'required_delta_preview');
trace.actual_delta_tire = optional_numeric_column(tbl, 'actual_delta_tire');
trace.predicted_delta_tire = optional_numeric_column(tbl, 'predicted_delta_tire');
trace.preview_delta_gap = optional_numeric_column(tbl, 'preview_delta_gap');
trace.torque_headroom = optional_numeric_column(tbl, 'torque_headroom');
trace.time_to_reach_delta = optional_numeric_column(tbl, 'time_to_reach_delta');
trace.corner_throttle_steer_ratio = optional_numeric_column(tbl, 'corner_throttle_steer_ratio');
trace.corner_throttle_scale = optional_numeric_column(tbl, 'corner_throttle_scale');
end

function segments = build_segment_table(trace, segment_length_m)
min_s = floor(min(trace.s) / segment_length_m) * segment_length_m;
max_s = ceil(max(trace.s) / segment_length_m) * segment_length_m;
if max_s <= min_s
    max_s = min_s + segment_length_m;
end
edges = min_s:segment_length_m:max_s;
if edges(end) < max(trace.s)
    edges(end + 1) = edges(end) + segment_length_m;
end

rows = repmat(empty_segment_row(), 0, 1);
for i = 1:(numel(edges) - 1)
    if i == numel(edges) - 1
        mask = trace.s >= edges(i) & trace.s <= edges(i + 1);
    else
        mask = trace.s >= edges(i) & trace.s < edges(i + 1);
    end
    if ~any(mask)
        continue;
    end

    row = empty_segment_row();
    row.segment_id = numel(rows) + 1;
    row.s_start_m = edges(i);
    row.s_end_m = edges(i + 1);
    row.sample_count = sum(mask);
    row.duration_s = max(trace.time(mask)) - min(trace.time(mask));
    row.mean_speed_mps = finite_mean(trace.speed(mask));
    row.min_speed_mps = finite_min(trace.speed(mask));
    row.mean_target_base_mps = finite_mean(trace.target_speed_base(mask));
    row.mean_target_preview_mps = finite_mean(trace.target_speed_preview(mask));
    row.mean_target_final_mps = finite_mean(trace.target_speed_final(mask));
    row.mean_total_loss_mps = finite_mean(max(trace.target_speed_base(mask) - trace.target_speed_final(mask), 0.0));
    row.mean_preview_loss_mps = finite_mean(max(trace.target_speed_base(mask) - trace.target_speed_preview(mask), 0.0));
    row.mean_controller_loss_mps = finite_mean(max(trace.target_speed_preview(mask) - trace.target_speed_final(mask), 0.0));
    row.mean_torque_multiplier = finite_mean(trace.torque_speed_multiplier(mask));
    row.mean_effective_max_tire_steer_rate = finite_mean(trace.effective_max_tire_steer_rate(mask));
    row.min_steering_actuator_cap_mps = finite_min(trace.steering_actuator_speed_cap(mask));
    row.min_track_margin_cap_mps = finite_min(trace.track_margin_speed_cap(mask));
    row.min_track_margin_effective_margin_m = finite_min(trace.track_margin_effective_margin(mask));
    row.max_track_margin_outward_rate_mps = finite_max(trace.track_margin_outward_rate(mask));
    row.mean_steering_lag_ratio = finite_mean(trace.steering_lag_ratio(mask));
    row.max_steering_lag_ratio = finite_max(trace.steering_lag_ratio(mask));
    row.max_abs_sw_error_rad = finite_max(abs(trace.sw_error(mask)));
    row.mean_abs_preview_delta_gap_rad = finite_mean(abs(trace.preview_delta_gap(mask)));
    row.max_abs_preview_delta_gap_rad = finite_max(abs(trace.preview_delta_gap(mask)));
    row.mean_abs_required_delta_preview_rad = finite_mean(abs(trace.required_delta_preview(mask)));
    row.mean_abs_predicted_delta_tire_rad = finite_mean(abs(trace.predicted_delta_tire(mask)));
    row.mean_time_to_reach_delta_s = finite_mean(trace.time_to_reach_delta(mask));
    row.max_time_to_reach_delta_s = finite_max(trace.time_to_reach_delta(mask));
    row.mean_torque_headroom = finite_mean(trace.torque_headroom(mask));
    row.min_torque_headroom = finite_min(trace.torque_headroom(mask));
    row.mean_corner_throttle_steer_ratio = finite_mean(trace.corner_throttle_steer_ratio(mask));
    row.min_corner_throttle_scale = finite_min(trace.corner_throttle_scale(mask));
    row.mean_corner_throttle_loss = finite_mean(max(1.0 - trace.corner_throttle_scale(mask), 0.0));
    row.saturation_ratio = finite_mean(trace.steering_saturation(mask) > 0.5);
    row.max_lateral_error_m = finite_max(abs(trace.lateral_error(mask)));
    row.mean_abs_heading_error_deg = finite_mean(abs(trace.heading_error_deg(mask)));
    row.mean_abs_kappa_1pm = finite_mean(abs(trace.kappa(mask)));
    row.dominant_loss_source = classify_loss_source(row);
    row.dominant_controller_risk = classify_controller_risk(row);
    rows(end + 1) = row; %#ok<AGROW>
end

if isempty(rows)
    segments = struct2table(empty_segment_row(), 'AsArray', true);
    segments(1, :) = [];
else
    segments = struct2table(rows);
end
end

function row = empty_segment_row()
row = struct();
row.segment_id = 0;
row.s_start_m = NaN;
row.s_end_m = NaN;
row.sample_count = 0;
row.duration_s = NaN;
row.mean_speed_mps = NaN;
row.min_speed_mps = NaN;
row.mean_target_base_mps = NaN;
row.mean_target_preview_mps = NaN;
row.mean_target_final_mps = NaN;
row.mean_total_loss_mps = NaN;
row.mean_preview_loss_mps = NaN;
row.mean_controller_loss_mps = NaN;
row.mean_torque_multiplier = NaN;
row.mean_effective_max_tire_steer_rate = NaN;
row.min_steering_actuator_cap_mps = NaN;
row.min_track_margin_cap_mps = NaN;
row.min_track_margin_effective_margin_m = NaN;
row.max_track_margin_outward_rate_mps = NaN;
row.mean_steering_lag_ratio = NaN;
row.max_steering_lag_ratio = NaN;
row.max_abs_sw_error_rad = NaN;
row.mean_abs_preview_delta_gap_rad = NaN;
row.max_abs_preview_delta_gap_rad = NaN;
row.mean_abs_required_delta_preview_rad = NaN;
row.mean_abs_predicted_delta_tire_rad = NaN;
row.mean_time_to_reach_delta_s = NaN;
row.max_time_to_reach_delta_s = NaN;
row.mean_torque_headroom = NaN;
row.min_torque_headroom = NaN;
row.mean_corner_throttle_steer_ratio = NaN;
row.min_corner_throttle_scale = NaN;
row.mean_corner_throttle_loss = NaN;
row.saturation_ratio = NaN;
row.max_lateral_error_m = NaN;
row.mean_abs_heading_error_deg = NaN;
row.mean_abs_kappa_1pm = NaN;
row.dominant_loss_source = "";
row.dominant_controller_risk = "";
end

function summary = build_summary_table(trace, segments, config)
summary_row = struct();
summary_row.run_id = string(config.run_id);
summary_row.trace_csv_path = string(config.trace_csv_path);
summary_row.lap_time_estimate_s = max(trace.time) - min(trace.time);
summary_row.reference_lap_time_s = config.reference_lap_time_s;
summary_row.lap_time_delta_to_reference_s = summary_row.lap_time_estimate_s - config.reference_lap_time_s;
summary_row.s_start_m = min(trace.s);
summary_row.s_end_m = max(trace.s);
summary_row.segment_count = height(segments);
summary_row.mean_speed_mps = finite_mean(trace.speed);
summary_row.mean_target_base_mps = finite_mean(trace.target_speed_base);
summary_row.mean_target_preview_mps = finite_mean(trace.target_speed_preview);
summary_row.mean_target_final_mps = finite_mean(trace.target_speed_final);
summary_row.mean_total_loss_mps = finite_mean(max(trace.target_speed_base - trace.target_speed_final, 0.0));
summary_row.mean_preview_loss_mps = finite_mean(max(trace.target_speed_base - trace.target_speed_preview, 0.0));
summary_row.mean_controller_loss_mps = finite_mean(max(trace.target_speed_preview - trace.target_speed_final, 0.0));
summary_row.mean_torque_multiplier = finite_mean(trace.torque_speed_multiplier);
summary_row.mean_effective_max_tire_steer_rate = finite_mean(trace.effective_max_tire_steer_rate);
summary_row.min_steering_actuator_cap_mps = finite_min(trace.steering_actuator_speed_cap);
summary_row.min_track_margin_cap_mps = finite_min(trace.track_margin_speed_cap);
summary_row.min_track_margin_effective_margin_m = finite_min(trace.track_margin_effective_margin);
summary_row.max_track_margin_outward_rate_mps = finite_max(trace.track_margin_outward_rate);
summary_row.mean_steering_lag_ratio = finite_mean(trace.steering_lag_ratio);
summary_row.max_steering_lag_ratio = finite_max(trace.steering_lag_ratio);
summary_row.max_abs_sw_error_rad = finite_max(abs(trace.sw_error));
summary_row.mean_abs_preview_delta_gap_rad = finite_mean(abs(trace.preview_delta_gap));
summary_row.max_abs_preview_delta_gap_rad = finite_max(abs(trace.preview_delta_gap));
summary_row.mean_time_to_reach_delta_s = finite_mean(trace.time_to_reach_delta);
summary_row.max_time_to_reach_delta_s = finite_max(trace.time_to_reach_delta);
summary_row.mean_torque_headroom = finite_mean(trace.torque_headroom);
summary_row.min_torque_headroom = finite_min(trace.torque_headroom);
summary_row.mean_corner_throttle_steer_ratio = finite_mean(trace.corner_throttle_steer_ratio);
summary_row.min_corner_throttle_scale = finite_min(trace.corner_throttle_scale);
summary_row.mean_corner_throttle_loss = finite_mean(max(1.0 - trace.corner_throttle_scale, 0.0));
summary_row.saturation_ratio = finite_mean(trace.steering_saturation > 0.5);
summary_row.max_lateral_error_m = finite_max(abs(trace.lateral_error));
summary_row.mean_abs_heading_error_deg = finite_mean(abs(trace.heading_error_deg));
summary = struct2table(summary_row, 'AsArray', true);
end

function top_segments = top_loss_segments(segments, count)
if isempty(segments)
    top_segments = segments;
    return;
end
score = segments.mean_controller_loss_mps + 0.5 * segments.mean_total_loss_mps ...
    + 2.0 * segments.saturation_ratio + 0.5 * segments.max_steering_lag_ratio;
score(~isfinite(score)) = -Inf;
[~, idx] = sort(score, 'descend');
idx = idx(1:min(count, numel(idx)));
top_segments = segments(idx, :);
end

function source = classify_loss_source(row)
if ~isfinite(row.mean_total_loss_mps) || row.mean_total_loss_mps < 0.10
    source = "none";
elseif row.mean_controller_loss_mps >= 1.25 * row.mean_preview_loss_mps && row.mean_controller_loss_mps > 0.10
    source = "controller";
elseif row.mean_preview_loss_mps >= 1.25 * row.mean_controller_loss_mps && row.mean_preview_loss_mps > 0.10
    source = "planner_preview";
else
    source = "mixed";
end
end

function risk = classify_controller_risk(row)
if row.saturation_ratio > 0.10 && row.max_steering_lag_ratio > 0.50
    risk = "saturation_lag";
elseif isfinite(row.min_torque_headroom) && row.min_torque_headroom < 0.05
    risk = "low_torque_headroom";
elseif isfinite(row.max_time_to_reach_delta_s) && row.max_time_to_reach_delta_s > 0.12
    risk = "steering_reach_time";
elseif row.saturation_ratio > 0.10
    risk = "torque_saturation";
elseif row.max_steering_lag_ratio > 0.50
    risk = "steering_lag";
elseif isfinite(row.min_steering_actuator_cap_mps) && row.min_steering_actuator_cap_mps < row.mean_speed_mps
    risk = "actuator_cap";
elseif isfinite(row.min_track_margin_cap_mps) && row.min_track_margin_cap_mps < row.mean_speed_mps
    risk = "track_margin";
else
    risk = "stable";
end
end

function write_report(report_path, manifest, summary, top_segments, config)
fid = fopen(report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', report_path);
end
cleanup = onCleanup(@() fclose(fid));

s = summary(1, :);
fprintf(fid, '# Lap Speed Loss Diagnostic\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- trace_csv: `%s`\n', manifest.trace_csv_path);
fprintf(fid, '- summary_csv: `%s`\n', manifest.summary_csv_path);
fprintf(fid, '- segments_csv: `%s`\n\n', manifest.segments_csv_path);

fprintf(fid, '## Summary\n\n');
fprintf(fid, '- lap_time_estimate: %.3f s\n', s.lap_time_estimate_s);
fprintf(fid, '- reference_lap_time: %.3f s\n', config.reference_lap_time_s);
fprintf(fid, '- delta_to_reference: %.3f s\n', s.lap_time_delta_to_reference_s);
fprintf(fid, '- mean_speed: %.3f m/s\n', s.mean_speed_mps);
fprintf(fid, '- mean_total_loss: %.3f m/s\n', s.mean_total_loss_mps);
fprintf(fid, '- mean_preview_loss: %.3f m/s\n', s.mean_preview_loss_mps);
fprintf(fid, '- mean_controller_loss: %.3f m/s\n', s.mean_controller_loss_mps);
fprintf(fid, '- mean_torque_multiplier: %.3f\n', s.mean_torque_multiplier);
fprintf(fid, '- mean_effective_max_tire_steer_rate: %.3f rad/s\n', s.mean_effective_max_tire_steer_rate);
fprintf(fid, '- saturation_ratio: %.3f\n', s.saturation_ratio);
fprintf(fid, '- max_steering_lag_ratio: %.3f\n\n', s.max_steering_lag_ratio);
fprintf(fid, '- max_preview_delta_gap: %.3f rad\n', s.max_abs_preview_delta_gap_rad);
fprintf(fid, '- max_time_to_reach_delta: %.3f s\n', s.max_time_to_reach_delta_s);
fprintf(fid, '- min_torque_headroom: %.3f\n\n', s.min_torque_headroom);
fprintf(fid, '- min_track_margin_effective_margin: %.3f m\n', s.min_track_margin_effective_margin_m);
fprintf(fid, '- max_track_margin_outward_rate: %.3f m/s\n\n', s.max_track_margin_outward_rate_mps);
fprintf(fid, '- mean_corner_throttle_loss: %.3f\n', s.mean_corner_throttle_loss);
fprintf(fid, '- min_corner_throttle_scale: %.3f\n\n', s.min_corner_throttle_scale);

fprintf(fid, '## Worst Segments\n\n');
if isempty(top_segments)
    fprintf(fid, 'No segment rows were produced.\n');
    return;
end

fprintf(fid, '| s_range_m | total_loss | controller_loss | preview_loss | mean_speed | torque_mult | steer_rate | margin_m | out_rate | corner_scale | sat_ratio | max_lag | gap_rad | reach_s | min_headroom | source | risk |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|\n');
for i = 1:height(top_segments)
    r = top_segments(i, :);
    fprintf(fid, '| %.1f-%.1f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f | %s | %s |\n', ...
        r.s_start_m, r.s_end_m, r.mean_total_loss_mps, ...
        r.mean_controller_loss_mps, r.mean_preview_loss_mps, ...
        r.mean_speed_mps, r.mean_torque_multiplier, r.mean_effective_max_tire_steer_rate, ...
        r.min_track_margin_effective_margin_m, r.max_track_margin_outward_rate_mps, ...
        r.min_corner_throttle_scale, ...
        r.saturation_ratio, r.max_steering_lag_ratio, r.max_abs_preview_delta_gap_rad, ...
        r.max_time_to_reach_delta_s, r.min_torque_headroom, ...
        table_text(r, 'dominant_loss_source'), table_text(r, 'dominant_controller_risk'));
end
end

function value = numeric_column(tbl, name)
value = tbl.(name);
if iscell(value) || isstring(value)
    value = str2double(string(value));
end
value = double(value);
end

function value = optional_numeric_column(tbl, name)
if ~ismember(string(name), string(tbl.Properties.VariableNames))
    value = NaN(height(tbl), 1);
    return;
end
value = numeric_column(tbl, name);
end

function value = finite_mean(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = mean(double(x));
end
end

function value = finite_min(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = min(double(x));
end
end

function value = finite_max(x)
x = x(isfinite(x));
if isempty(x)
    value = NaN;
else
    value = max(double(x));
end
end

function text = table_text(row, name)
value = row.(name);
if iscell(value)
    text = char(value{1});
else
    text = char(string(value));
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
