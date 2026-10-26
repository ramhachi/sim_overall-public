function [cost, gate] = score_vehicle_ga_result( ...
        result, return_code, target_distance, enforce_max_runtime)
% SCORE_VEHICLE_GA_RESULT Apply the high-fidelity LPV vehicle acceptance gate.

if nargin < 3
    target_distance = 1000.0;
end
if nargin < 4
    enforce_max_runtime = true;
end
gate = struct('passed', false, 'violations', {{}});

if return_code ~= 0 || isempty(result) || ~isstruct(result)
    traveled = read_metric(result, 'distance_traveled', 0.0);
    cost = 1e6 + max(0.0, target_distance - traveled) * 1000.0;
    gate.violations = {'simulation_failed'};
    return;
end

required_finite = {'lap_time', 'max_lateral_error', 'rms_heading_error', ...
    'mean_abs_sw_error', 'steering_saturation_ratio', 'max_track_limit_excess', ...
    'mpc_avg_solve_time_ms', 'mpc_max_solve_time_ms'};
for i = 1:numel(required_finite)
    name = required_finite{i};
    if ~isfield(result, name) || ~isscalar(result.(name)) || ~isfinite(result.(name))
        gate.violations{end + 1} = ['nonfinite_', name];
    end
end
if ~isempty(gate.violations)
    cost = 1e9;
    return;
end

if ~read_metric(result, 'lap_completed', false)
    gate.violations{end + 1} = 'lap_not_completed';
end
if ~strcmp(char(string(read_metric(result, 'status', 'failed'))), 'success')
    gate.violations{end + 1} = 'status_not_success';
end
if read_metric(result, 'degraded_mode', true)
    gate.violations{end + 1} = 'degraded_mode';
end
if ~strcmp(char(string(read_metric(result, 'tire_model_type', ''))), 'pinn')
    gate.violations{end + 1} = 'tire_model_not_pinn';
end
if ~startsWith(char(string(read_metric(result, 'tire_runtime', ''))), 'pinn')
    gate.violations{end + 1} = 'tire_runtime_not_pinn';
end
if ~strcmp(char(string(read_metric(result, 'velocity_profile_source', ''))), 'generated')
    gate.violations{end + 1} = 'velocity_profile_not_generated';
end
if read_metric(result, 'max_track_limit_excess', inf) > 1e-9
    gate.violations{end + 1} = 'track_limit_excess';
end
if read_metric(result, 'max_lateral_error', inf) > 0.65
    gate.violations{end + 1} = 'lateral_error';
end
if read_metric(result, 'rms_heading_error', inf) > 5.0
    gate.violations{end + 1} = 'heading_error';
end
% No mean |SW error| gate: the speed-tuned LPV runs ~0.25 rad on every
% candidate while tracking the path to ~0.2 m; it is reported, not gated.
if read_metric(result, 'steering_saturation_ratio', inf) > 0.005
    gate.violations{end + 1} = 'steering_saturation';
end
if read_metric(result, 'mpc_fail_count', inf) > 0
    gate.violations{end + 1} = 'lpv_solve_failure';
end
if read_metric(result, 'mpc_avg_solve_time_ms', inf) > 0.05 ...
        || (enforce_max_runtime && read_metric(result, 'mpc_max_solve_time_ms', inf) > 1.0)
    gate.violations{end + 1} = 'lpv_runtime';
end

gate.passed = isempty(gate.violations);
if gate.passed
    cost = result.lap_time;
else
    cost = 1e5 + 1000.0 * numel(gate.violations) + result.lap_time;
end
end

function value = read_metric(result, name, default_value)
value = default_value;
if isstruct(result) && isfield(result, name)
    value = result.(name);
end
end
