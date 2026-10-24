function params = resolve_rl_vehicle_params(vehicleParams)
% RESOLVE_RL_VEHICLE_PARAMS Merge optional RL vehicle overrides onto the baseline template.

ensure_rl_driver_paths();

% Default: Try to load params_winged_flexible.json if it exists, otherwise template
this_dir = fileparts(mfilename('fullpath'));
matlab_dir = fileparts(this_dir);
winged_json = fullfile(matlab_dir, 'parameter_presets', 'params_winged_flexible.json');

if isfile(winged_json)
    data = jsondecode(fileread(winged_json));
    assert_valid_vehicle_parameter_artifact(data);
    params = data.params;
    % Ensure torque_mode is true for RL
    params.steering.torque_mode = true;
else
    params = create_vehicle_baseline_template();
    params.steering.torque_mode = true;
    params.suspension.K_chassis = 2500.0;
end

if nargin < 1 || isempty(vehicleParams)
    return;
end

params = merge_nested_structs(params, vehicleParams);
end
