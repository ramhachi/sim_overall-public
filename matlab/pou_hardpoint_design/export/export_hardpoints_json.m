function json_path = export_hardpoints_json(results, geom, output_path)
% EXPORT_HARDPOINTS_JSON - Export POU results to JSON for C++ simulation
%
% Updates or creates a vehicle parameters JSON file with POU hardpoint data.
% The exported data includes both polynomial coefficients (used by C++ sim)
% and physical hardpoint coordinates (for reference/debugging).
%
% INPUTS:
%   results     - Results struct from main_pou_hardpoint_optimizer
%   geom        - Suspension geometry struct from define_suspension_geometry
%   output_path - Path to output JSON file
%
% OUTPUT:
%   json_path - Path to written JSON file
%
% JSON STRUCTURE:
%   { "pou_hardpoints": {
%       "FL": { "M_pivot_chassis": [...], "bellcrank_pivot": [...], ... },
%       ...
%     }
%   }

    if nargin < 3 || isempty(output_path)
        output_path = 'pou_hardpoints.json';
    end

    wheel_names = {'FL', 'FR', 'RL', 'RR'};
    wn = wheel_names{geom.wheel_index + 1};

    % Build per-wheel data structure
    wheel_data = struct();
    wheel_data.M_pivot_chassis = round(results.M_pivot_chassis(:)', 6);
    wheel_data.M_pivot_upright = round(results.M_pivot_upright(:)', 6);
    wheel_data.bellcrank_pivot = round(results.bellcrank_pivot(:)', 6);
    wheel_data.pushrod_length_m = round(results.pushrod_length_nominal_m, 4);
    wheel_data.installation_ratio = round(results.installation_ratio, 4);
    if isfield(results, 'bellcrank')
        bc = results.bellcrank;
        wheel_data.bellcrank_frame_pivot = round(bc.frame_pivot(:)', 6);
        wheel_data.bellcrank_rotation_axis = round(bc.rotation_axis(:)', 6);
        wheel_data.bellcrank_damper_pin = round(bc.damper_pin(:)', 6);
        wheel_data.damper_chassis_mount = round(bc.damper_chassis_mount(:)', 6);
    end
    if isfield(results, 'pou_lut')
        wheel_data.pou_lut = results.pou_lut;
    end
    wheel_data.max_rod_angle_deg = round(results.max_rod_angle_deg, 2);
    wheel_data.polynomial_error_mm = round(results.polynomial_error_mm, 3);
    wheel_data.packaging_valid = results.packaging_valid;
    wheel_data.exitflag = results.optimization_exitflag;

    % A-arm hardpoints (from geometry - full DW model)
    wheel_data.upper_arm_front_chassis = round(geom.upper_arm_front_chassis(:)', 6);
    wheel_data.upper_arm_rear_chassis = round(geom.upper_arm_rear_chassis(:)', 6);
    wheel_data.upper_arm_upright = round(geom.upper_arm_upright(:)', 6);
    wheel_data.lower_arm_front_chassis = round(geom.lower_arm_front_chassis(:)', 6);
    wheel_data.lower_arm_rear_chassis = round(geom.lower_arm_rear_chassis(:)', 6);
    wheel_data.lower_arm_upright = round(geom.lower_arm_upright(:)', 6);
    wheel_data.tie_rod_chassis = round(geom.tie_rod_chassis(:)', 6);
    wheel_data.tie_rod_upright = round(geom.tie_rod_upright(:)', 6);
    wheel_data.kingpin_origin = round(geom.kingpin_axis.origin(:)', 6);
    wheel_data.kingpin_direction = round(geom.kingpin_axis.direction(:)', 6);

    % Load existing file if present, or create new
    if exist(output_path, 'file')
        raw = fileread(output_path);
        data = jsondecode(raw);
    else
        data = struct();
    end

    % Ensure pou_hardpoints field exists
    if ~isfield(data, 'pou_hardpoints')
        data.pou_hardpoints = struct();
    end

    % Update the specific wheel
    data.pou_hardpoints.(wn) = wheel_data;

    % Write JSON
    json_text = jsonencode(data);

    % Pretty-print (basic indentation)
    json_text = strrep(json_text, ',"', sprintf(',\n  "'));
    json_text = strrep(json_text, '{', sprintf('{\n  '));
    json_text = strrep(json_text, '}', sprintf('\n}'));

    fid = fopen(output_path, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', output_path);
    end
    fwrite(fid, json_text, 'char');
    fclose(fid);

    json_path = output_path;
    fprintf('JSON exported: %s (wheel: %s)\n', json_path, wn);
end
