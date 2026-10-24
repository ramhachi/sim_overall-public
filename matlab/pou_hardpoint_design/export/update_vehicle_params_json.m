function update_vehicle_params_json(results, geom, params_json_path)
% UPDATE_VEHICLE_PARAMS_JSON - Update existing vehicle params JSON with POU hardpoints
%
% Reads an existing optimized vehicle parameters JSON file and adds/updates
% the POU hardpoint coordinates section. This allows the C++ simulation to
% access both polynomial coefficients AND physical hardpoint data.
%
% INPUTS:
%   results         - Results struct from main_pou_hardpoint_optimizer
%   geom            - Suspension geometry struct
%   params_json_path - Path to existing vehicle params JSON
%
% MODIFIES:
%   The JSON file at params_json_path (adds pou_hardpoints section)

    if ~exist(params_json_path, 'file')
        error('Vehicle params file not found: %s', params_json_path);
    end

    helper_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(helper_dir);
    end

    data = load_vehicle_artifact(params_json_path);

    wheel_names = {'FL', 'FR', 'RL', 'RR'};
    wn = wheel_names{geom.wheel_index + 1};

    target = data.params;

    % Add pou_hardpoints section
    if ~isfield(target, 'pou_hardpoints')
        target.pou_hardpoints = struct();
    end

    % Per-wheel hardpoint data
    hp = struct();
    hp.M_pivot_chassis = round(results.M_pivot_chassis(:)', 6);
    hp.M_pivot_upright = round(results.M_pivot_upright(:)', 6);
    hp.bellcrank_pivot = round(results.bellcrank_pivot(:)', 6);
    hp.pushrod_length_m = round(results.pushrod_length_nominal_m, 4);
    hp.installation_ratio = round(results.installation_ratio, 4);
    if isfield(results, 'bellcrank')
        bc = results.bellcrank;
        hp.bellcrank_frame_pivot = round(bc.frame_pivot(:)', 6);
        hp.bellcrank_rotation_axis = round(bc.rotation_axis(:)', 6);
        hp.bellcrank_damper_pin = round(bc.damper_pin(:)', 6);
        hp.damper_chassis_mount = round(bc.damper_chassis_mount(:)', 6);
    end
    hp.upper_arm_front_chassis = round(geom.upper_arm_front_chassis(:)', 6);
    hp.upper_arm_rear_chassis = round(geom.upper_arm_rear_chassis(:)', 6);
    hp.upper_arm_upright = round(geom.upper_arm_upright(:)', 6);
    hp.lower_arm_front_chassis = round(geom.lower_arm_front_chassis(:)', 6);
    hp.lower_arm_rear_chassis = round(geom.lower_arm_rear_chassis(:)', 6);
    hp.lower_arm_upright = round(geom.lower_arm_upright(:)', 6);
    hp.tie_rod_chassis = round(geom.tie_rod_chassis(:)', 6);
    hp.tie_rod_upright = round(geom.tie_rod_upright(:)', 6);
    hp.kingpin_origin = round(geom.kingpin_axis.origin(:)', 6);
    hp.kingpin_direction = round(geom.kingpin_axis.direction(:)', 6);

    target.pou_hardpoints.(wn) = hp;
    if isfield(results, 'pou_lut')
        if ~isfield(target.suspension, 'pou_lut')
            target.suspension.pou_lut = struct();
        end
        target.suspension.pou_lut.(wn) = results.pou_lut;
    end

    if isfield(data, 'event_params') && isstruct(data.event_params)
        event_names = fieldnames(data.event_params);
        for i = 1:numel(event_names)
            event_name = event_names{i};
            if isstruct(data.event_params.(event_name))
                if ~isfield(data.event_params.(event_name), 'pou_hardpoints')
                    data.event_params.(event_name).pou_hardpoints = struct();
                end
                data.event_params.(event_name).pou_hardpoints.(wn) = hp;
            end
        end
    end

    stage_result = struct( ...
        'installation_ratio', results.installation_ratio, ...
        'pou_lut_converged', isfield(results, 'pou_lut') && results.pou_lut.converged, ...
        'polynomial_error_mm', results.polynomial_error_mm, ...
        'max_jack_error_mm', results.max_jack_error_mm, ...
        'max_camber_error_deg', results.max_camber_error_deg, ...
        'packaging_valid', results.packaging_valid, ...
        'optimization_exitflag', results.optimization_exitflag, ...
        'optimization_time_s', results.optimization_time_s);
    if isfield(results, 'violations')
        stage_result.violations = results.violations;
    end

    stage_entry = struct();
    stage_entry.name = 'update_pou_hardpoints';
    stage_entry.optimizer = 'inverse_kinematics_postprocess';
    stage_entry.wheel = wn;
    stage_entry.params = target;
    stage_entry.result = stage_result;

    data = build_vehicle_artifact(target, 'base_artifact', data, 'stage_entry', stage_entry);
    save_vehicle_artifact(params_json_path, data);

    fprintf('Vehicle params updated: %s (wheel: %s)\n', params_json_path, wn);
end
