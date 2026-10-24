function analysis = run_steering_inner_loop_benchmark(config)
% RUN_STEERING_INNER_LOOP_BENCHMARK Benchmark steering torque inner-loop response.
%
% The benchmark isolates the steering wheel torque plant:
%   I*d2(delta_sw) = T_driver - T_sat - T_friction - T_damping - T_return
% and applies the same PID-style driver torque control used by the C++ PID
% driver. It produces response metrics that can be used to tune actuator-aware
% speed envelopes without running a full vehicle lap.

if nargin < 1 || isempty(config)
    config = struct();
end

config = normalize_config(config);
ensure_dir(config.runtime_output_dir);
ensure_dir(config.report_output_dir);

scenarios = default_scenarios(config);
rows = [];
trace_paths = strings(0, 1);

for i = 1:numel(scenarios)
    scenario = scenarios(i);
    trace = simulate_scenario(config, scenario);
    metrics = compute_metrics(config, scenario, trace);
    if isempty(rows)
        rows = metrics;
    else
        rows(end + 1) = metrics; %#ok<AGROW>
    end

    trace_path = fullfile(config.runtime_output_dir, sprintf('%s_trace.csv', scenario.name));
    writetable(trace_to_table(trace), trace_path);
    trace_paths(end + 1, 1) = string(trace_path); %#ok<AGROW>
end

summary_table = struct2table(rows);
summary_path = fullfile(config.runtime_output_dir, 'steering_inner_loop_summary.csv');
writetable(summary_table, summary_path);

manifest = struct();
manifest.schema_version = 'steering_inner_loop_benchmark/v1';
manifest.generated_by = 'run_steering_inner_loop_benchmark';
manifest.generated_at_utc = utc_now();
manifest.run_id = config.run_id;
manifest.runtime_output_dir = config.runtime_output_dir;
manifest.summary_csv_path = summary_path;
manifest.trace_paths = cellstr(trace_paths);
manifest.config = config;
manifest_path = fullfile(config.runtime_output_dir, 'manifest.json');
write_json(manifest_path, manifest);

report_path = fullfile(config.report_output_dir, sprintf('steering_inner_loop_%s.md', config.run_id));
write_report(report_path, manifest, summary_table);

analysis = struct();
analysis.run_id = config.run_id;
analysis.manifest_path = manifest_path;
analysis.summary_csv_path = summary_path;
analysis.report_path = report_path;
analysis.trace_paths = trace_paths;
analysis.summary = summary_table;
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
        'driver_generalization', 'steering_inner_loop', config.run_id);
end
if ~isfield(config, 'report_output_dir') || isempty(config.report_output_dir)
    config.report_output_dir = fullfile(config.project_root, 'output', 'reports');
end

config.dt = field_or(config, 'dt', 0.001);
config.duration = field_or(config, 'duration', 2.0);
config.R_steer = field_or(config, 'R_steer', 4.5);
config.I_steer = field_or(config, 'I_steer', 0.02);
config.T_fric = field_or(config, 'T_fric', 0.3);
config.C_steer_torque = field_or(config, 'C_steer_torque', 0.3);
config.K_return = field_or(config, 'K_return', 0.0);
config.T_driver_max = field_or(config, 'T_driver_max', 25.0);
config.inner_integral_limit = field_or(config, 'inner_integral_limit', 0.5);
% Inner-loop benchmark should measure steering actuator bandwidth, not the
% outer-loop target-rate limiter used during path following.
config.max_tire_steer_rate = field_or(config, 'max_tire_steer_rate', 100.0);
config.sat_stiffness_nm_per_rad_tire = field_or(config, 'sat_stiffness_nm_per_rad_tire', 18.0);

% A-3: omega_n/zeta ベース自動ゲイン正規化 (C++ sync_steering_contract() と同一ロジック)
% inner_loop_auto_tune=false の場合は Kp/Kd/Ki を手動値で指定できる。
config.inner_loop_auto_tune = field_or(config, 'inner_loop_auto_tune', true);
config.inner_loop_omega_n   = field_or(config, 'inner_loop_omega_n', 30.0);
config.inner_loop_zeta      = field_or(config, 'inner_loop_zeta', 0.70);
config.inner_loop_ki_override = field_or(config, 'inner_loop_ki_override', -1.0);
if config.inner_loop_auto_tune
    I = max(config.I_steer, 1e-6);
    C = max(config.C_steer_torque, 0.0);
    omega_n = config.inner_loop_omega_n;
    zeta    = config.inner_loop_zeta;
    config.Kp_inner = omega_n^2 * I;
    config.Kd_inner = max(2.0 * zeta * omega_n * I - C, 0.0);
    if config.inner_loop_ki_override >= 0.0
        config.Ki_inner = config.inner_loop_ki_override;
    else
        config.Ki_inner = 0.10 * config.Kp_inner;
    end
else
    config.Kp_inner = field_or(config, 'Kp_inner', 50.0);
    config.Kd_inner = field_or(config, 'Kd_inner', 1.5);
    config.Ki_inner = field_or(config, 'Ki_inner', 5.0);
end
end

function scenarios = default_scenarios(config)
if isfield(config, 'scenarios') && ~isempty(config.scenarios)
    scenarios = config.scenarios;
    return;
end

% 基本シナリオ: step / sine / reversal (既存)
scenarios = struct( ...
    'name', {'step_0p20', 'sine_1hz', 'reversal'}, ...
    'kind', {'step', 'sine', 'reversal'}, ...
    'amplitude_tire_rad', {0.20, 0.14, 0.18}, ...
    'frequency_hz', {0.0, 1.0, 0.0}, ...
    'start_time', {0.10, 0.0, 0.20}, ...
    'override_I_steer', {NaN, NaN, NaN});

% A-3: I_steer sweep シナリオ (4点 × step 入力)
% 目標: 全点で rise_time < 120 ms、saturation_rate < 0.05
I_sweep = [0.01, 0.02, 0.05, 0.12];
for k = 1:numel(I_sweep)
    s = struct();
    s.name = sprintf('step_I%.4g', I_sweep(k));
    s.kind = 'step';
    s.amplitude_tire_rad = 0.20;
    s.frequency_hz = 0.0;
    s.start_time = 0.10;
    s.override_I_steer = I_sweep(k);  % このシナリオのみ I_steer を上書き
    scenarios(end + 1) = s; %#ok<AGROW>
end
end

function trace = simulate_scenario(config, scenario)
% A-3: I_steer sweep 対応 -- シナリオに override_I_steer があれば
%      そのシナリオ用のゲインも omega_n/zeta で再計算する
I_steer = config.I_steer;
Kp = config.Kp_inner;
Kd = config.Kd_inner;
Ki = config.Ki_inner;
if isfield(scenario, 'override_I_steer') && ~isnan(scenario.override_I_steer)
    I_steer = scenario.override_I_steer;
    if config.inner_loop_auto_tune
        I = max(I_steer, 1e-6);
        C = max(config.C_steer_torque, 0.0);
        Kp = config.inner_loop_omega_n^2 * I;
        Kd = max(2.0 * config.inner_loop_zeta * config.inner_loop_omega_n * I - C, 0.0);
        if config.inner_loop_ki_override >= 0.0
            Ki = config.inner_loop_ki_override;
        else
            Ki = 0.10 * Kp;
        end
    end
end

n = floor(config.duration / config.dt) + 1;
time = zeros(n, 1);
target_tire = zeros(n, 1);
target_sw = zeros(n, 1);
delta_sw = zeros(n, 1);
ddelta_sw = zeros(n, 1);
driver_torque_raw = zeros(n, 1);
driver_torque = zeros(n, 1);
sat_torque = zeros(n, 1);
friction_torque = zeros(n, 1);
steering_error = zeros(n, 1);

integral = 0.0;
target_sw_limited = 0.0;

for i = 2:n
    t = (i - 1) * config.dt;
    time(i) = t;
    raw_target_tire = scenario_target_tire(scenario, t);
    raw_target_sw = raw_target_tire * config.R_steer;

    max_sw_change = config.max_tire_steer_rate * config.R_steer * config.dt;
    target_sw_limited = target_sw_limited + clamp(raw_target_sw - target_sw_limited, ...
        -max_sw_change, max_sw_change);

    target_tire(i) = target_sw_limited / config.R_steer;
    target_sw(i) = target_sw_limited;

    steering_error(i) = target_sw(i) - delta_sw(i - 1);
    integral = clamp(integral + steering_error(i) * config.dt, ...
        -config.inner_integral_limit, config.inner_integral_limit);

    raw_torque = Kp * steering_error(i) ...
        + Ki * integral ...
        - Kd * ddelta_sw(i - 1);
    torque = clamp(raw_torque, -config.T_driver_max, config.T_driver_max);

    delta_tire = delta_sw(i - 1) / config.R_steer;
    sat = config.sat_stiffness_nm_per_rad_tire * delta_tire / config.R_steer;
    friction = config.T_fric * sign_with_zero(ddelta_sw(i - 1));
    damping = config.C_steer_torque * ddelta_sw(i - 1);
    returning = config.K_return * delta_sw(i - 1);

    accel = (torque - sat - friction - damping - returning) / I_steer;
    ddelta_sw(i) = ddelta_sw(i - 1) + accel * config.dt;
    delta_sw(i) = delta_sw(i - 1) + ddelta_sw(i) * config.dt;

    driver_torque_raw(i) = raw_torque;
    driver_torque(i) = torque;
    sat_torque(i) = sat;
    friction_torque(i) = friction;
end

trace = struct();
trace.time = time;
trace.target_tire_rad = target_tire;
trace.target_sw_rad = target_sw;
trace.delta_sw_rad = delta_sw;
trace.ddelta_sw_radps = ddelta_sw;
trace.delta_tire_rad = delta_sw / config.R_steer;
trace.steering_error_rad = steering_error;
trace.driver_torque_raw_nm = driver_torque_raw;
trace.driver_torque_nm = driver_torque;
trace.sat_torque_nm = sat_torque;
trace.friction_torque_nm = friction_torque;
trace.I_steer_used = I_steer;
trace.Kp_inner_used = Kp;
trace.Kd_inner_used = Kd;
trace.Ki_inner_used = Ki;
end

function tbl = trace_to_table(trace)
row_count = numel(trace.time);
names = fieldnames(trace);
tbl = table();

for i = 1:numel(names)
    name = names{i};
    value = trace.(name);
    if isscalar(value)
        value = repmat(value, row_count, 1);
    end
    if numel(value) == row_count
        tbl.(name) = value(:);
    end
end
end

function target = scenario_target_tire(scenario, t)
if strcmp(scenario.kind, 'step')
    target = double(t >= scenario.start_time) * scenario.amplitude_tire_rad;
elseif strcmp(scenario.kind, 'sine')
    target = scenario.amplitude_tire_rad * sin(2.0 * pi * scenario.frequency_hz * t);
elseif strcmp(scenario.kind, 'reversal')
    if t < scenario.start_time
        target = 0.0;
    elseif t < scenario.start_time + 0.5
        target = scenario.amplitude_tire_rad;
    else
        target = -scenario.amplitude_tire_rad;
    end
else
    error('Unknown steering benchmark scenario kind: %s', scenario.kind);
end
end

function metrics = compute_metrics(config, scenario, trace)
abs_error = abs(trace.steering_error_rad);
abs_target = abs(trace.target_sw_rad);
saturation = abs(trace.driver_torque_raw_nm - trace.driver_torque_nm) > 1e-9;

metrics = struct();
metrics.scenario = char(scenario.name);
metrics.kind = char(scenario.kind);
metrics.I_steer_used = trace.I_steer_used;
metrics.Kp_inner_used = trace.Kp_inner_used;
metrics.Kd_inner_used = trace.Kd_inner_used;
metrics.Ki_inner_used = trace.Ki_inner_used;
metrics.rms_error_sw_rad = rms(abs_error);
metrics.max_abs_error_sw_rad = max(abs_error);
metrics.saturation_ratio = mean(double(saturation));
metrics.max_tire_rate_radps = max(abs(trace.ddelta_sw_radps)) / config.R_steer;
metrics.safe_tire_rate_radps = 0.8 * percentile(abs(trace.ddelta_sw_radps) / config.R_steer, 95.0);
metrics.final_target_sw_rad = trace.target_sw_rad(end);
metrics.final_delta_sw_rad = trace.delta_sw_rad(end);
metrics.rise_time_s = NaN;
metrics.settling_time_s = NaN;
metrics.overshoot_ratio = NaN;
metrics.normalized_rms_error = NaN;
metrics.pass_rise_time = false;
metrics.pass_saturation = false;

if strcmp(scenario.kind, 'step')
    final_target = scenario.amplitude_tire_rad * config.R_steer;
    response = trace.delta_sw_rad;
    post_step = trace.time >= scenario.start_time;
    rise_idx = find(post_step & response >= 0.9 * final_target, 1, 'first');
    if ~isempty(rise_idx)
        metrics.rise_time_s = trace.time(rise_idx) - scenario.start_time;
    end
    tolerance = max(0.05 * abs(final_target), 1e-4);
    settling_idx = find_settling_index(trace.time, response, final_target, tolerance, scenario.start_time);
    if ~isnan(settling_idx)
        metrics.settling_time_s = trace.time(settling_idx) - scenario.start_time;
    end
    metrics.overshoot_ratio = max(0.0, (max(response(post_step)) - final_target) / max(abs(final_target), 1e-9));
    % A-3: 合否判定 (rise_time < 120ms, saturation_rate < 0.05)
    metrics.pass_rise_time   = ~isnan(metrics.rise_time_s) && metrics.rise_time_s < 0.120;
    metrics.pass_saturation  = metrics.saturation_ratio < 0.05;
elseif strcmp(scenario.kind, 'sine')
    nonzero_target = abs_target > 1e-4;
    if any(nonzero_target)
        metrics.normalized_rms_error = rms(abs_error(nonzero_target) ./ abs_target(nonzero_target));
    else
        metrics.normalized_rms_error = NaN;
    end
end
end

function idx = find_settling_index(time, response, target, tolerance, start_time)
idx = NaN;
candidate_indices = find(time >= start_time);
for k = reshape(candidate_indices, 1, [])
    if all(abs(response(k:end) - target) <= tolerance)
        idx = k;
        return;
    end
end
end

function write_report(report_path, manifest, summary_table)
fid = fopen(report_path, 'w');
if fid == -1
    error('Failed to open report path: %s', report_path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Steering Inner Loop Benchmark\n\n');
fprintf(fid, '- run_id: `%s`\n', manifest.run_id);
fprintf(fid, '- summary_csv: `%s`\n\n', manifest.summary_csv_path);

fprintf(fid, '## 基本シナリオ (step / sine / reversal)\n\n');
fprintf(fid, '| scenario | rms_error_sw | max_error_sw | saturation | max_tire_rate | safe_tire_rate | rise_time | settling_time |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:height(summary_table)
    row = summary_table(i, :);
    if ~startsWith(char(row.scenario), 'step_I')
        fprintf(fid, '| %s | %.4f | %.4f | %.3f | %.3f | %.3f | %.4f | %.4f |\n', ...
            char(row.scenario), row.rms_error_sw_rad, row.max_abs_error_sw_rad, ...
            row.saturation_ratio, row.max_tire_rate_radps, row.safe_tire_rate_radps, ...
            row.rise_time_s, row.settling_time_s);
    end
end

fprintf(fid, '\n## I_steer Sweep (A-3: omega_n=%.0f rad/s, zeta=%.2f)\n\n', ...
    manifest.config.inner_loop_omega_n, manifest.config.inner_loop_zeta);
fprintf(fid, '目標: rise_time < 120 ms, saturation_rate < 0.05 (全点)\n\n');
fprintf(fid, '| scenario | I_steer | Kp | Kd | Ki | rise_time_ms | saturation | pass_rise | pass_sat |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|:---:|:---:|\n');
for i = 1:height(summary_table)
    row = summary_table(i, :);
    if startsWith(char(row.scenario), 'step_I')
        fprintf(fid, '| %s | %.4f | %.2f | %.3f | %.3f | %.1f | %.3f | %s | %s |\n', ...
            char(row.scenario), row.I_steer_used, row.Kp_inner_used, row.Kd_inner_used, ...
            row.Ki_inner_used, row.rise_time_s * 1000, row.saturation_ratio, ...
            tf_str(row.pass_rise_time), tf_str(row.pass_saturation));
    end
end
end

function s = tf_str(v)
if v
    s = 'PASS';
else
    s = 'FAIL';
end
end

function value = field_or(s, name, default_value)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = default_value;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function s = sign_with_zero(x)
if x > 0.0
    s = 1.0;
elseif x < 0.0
    s = -1.0;
else
    s = 0.0;
end
end

function y = percentile(x, pct)
x = sort(x(:));
if isempty(x)
    y = NaN;
    return;
end
rank = 1.0 + (numel(x) - 1.0) * pct / 100.0;
lo = floor(rank);
hi = ceil(rank);
if lo == hi
    y = x(lo);
else
    alpha = rank - lo;
    y = (1.0 - alpha) * x(lo) + alpha * x(hi);
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
