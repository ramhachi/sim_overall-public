function [rear_mount_nodes, rear_upright_nodes, success] = load_rear_suspension_hardpoints(json_path, wheelbase_m, verbose)
% LOAD_REAR_SUSPENSION_HARDPOINTS Parse rear suspension hardpoints from vehicle artifact
%
% Parses the authoritative vehicle artifact and extracts the 8 physical chassis
% mount points for the rear suspension (RL/RR, Lower/Upper, Front/Rear).
% Coordinates are converted to frame coordinate system with wheelbase offset.
%
% Coordinate mapping:
%   Suspension JSON: X forward, Y left, Z downward (origin at rear wheel center)
%   Frame CS:        X rearward, Y rightward, Z upward (origin at front wheel center)
%   Mapping: frame = [-susp_x + wheelbase, susp_y, -susp_z]
%
% Inputs:
%   json_path    - Path to the vehicle artifact JSON with suspension hardpoints
%   wheelbase_m  - Wheelbase in meters (for X offset from front to rear axle)
%   verbose      - Optional flag to print load diagnostics (default: true)
%
% Outputs:
%   rear_mount_nodes   - 8x3 matrix [X, Y, Z] rear chassis mount coordinates
%                        Order: [RL_LF, RR_LF, RL_UF, RR_UF,
%                                RL_LR, RR_LR, RL_UR, RR_UR]
%   rear_upright_nodes - 6x3 matrix [X, Y, Z] upright coordinates (for viz)
%                        Order: [RL_Lower, RR_Lower, RL_Upper, RR_Upper,
%                                RL_WC, RR_WC]
%   success            - Boolean indicating success

    persistent cached_json_path cached_rl

    if nargin < 3 || isempty(verbose)
        verbose = true;
    end

    rear_mount_nodes = zeros(8, 3);
    rear_upright_nodes = zeros(6, 3);
    success = false;

    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'matlab'));
    end

    if ~isfile(json_path)
        warning('load_rear_suspension_hardpoints:FileNotFound', ...
                'Vehicle artifact file %s not found.', json_path);
        return;
    end

    try
        if isempty(cached_json_path) || ~strcmp(cached_json_path, json_path)
            artifact = load_vehicle_artifact(json_path);
            if ~isfield(artifact.params, 'suspension') || ~isfield(artifact.params.suspension, 'kc') ...
                    || ~isfield(artifact.params.suspension.kc, 'corners') || ~isfield(artifact.params.suspension.kc.corners, 'RL') ...
                    || ~isfield(artifact.params.suspension.kc.corners.RL, 'hardpoints')
                warning('load_rear_suspension_hardpoints:MissingRL', ...
                        'RL (Rear Left) data not found in artifact. Run EasyStart [8] first: %s', json_path);
                return;
            end
            cached_rl = artifact.params.suspension.kc.corners.RL.hardpoints;
            cached_json_path = json_path;
        end
        rl = cached_rl;

        % Verify required fields
        req_fields = {'lower_arm_chassis_fore', 'lower_arm_chassis_aft', ...
                      'upper_arm_chassis_fore', 'upper_arm_chassis_aft', ...
                      'lower_ball_joint', 'upper_ball_joint', 'wheel_center'};
        for i = 1:length(req_fields)
            if ~isfield(rl, req_fields{i})
                warning('load_rear_suspension_hardpoints:MissingField', ...
                        'Missing required field: %s', req_fields{i});
                return;
            end
        end

        % Coordinate mapping: susp -> frame CS with wheelbase offset
        % Susp: X fwd, Y left, Z down | Frame: X rear, Y right, Z up
        map_coord = @(pt) [-pt(1) + wheelbase_m, pt(2), -pt(3)];

        lowerF = rl.lower_arm_chassis_fore;
        upperF = rl.upper_arm_chassis_fore;
        lowerR = rl.lower_arm_chassis_aft;
        upperR = rl.upper_arm_chassis_aft;
        lowerU = rl.lower_ball_joint;
        upperU = rl.upper_ball_joint;

        % --- Chassis Mounts (8 points, mirrored L/R) ---
        % Lower Front
        pt = map_coord(lowerF);
        rear_mount_nodes(1, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL (left)
        rear_mount_nodes(2, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR (right)

        % Upper Front
        pt = map_coord(upperF);
        rear_mount_nodes(3, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL
        rear_mount_nodes(4, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR

        % Lower Rear
        pt = map_coord(lowerR);
        rear_mount_nodes(5, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL
        rear_mount_nodes(6, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR

        % Upper Rear
        pt = map_coord(upperR);
        rear_mount_nodes(7, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL
        rear_mount_nodes(8, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR

        % --- Upright Nodes (for visualization) ---
        pt = map_coord(lowerU);
        rear_upright_nodes(1, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL Lower
        rear_upright_nodes(2, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR Lower

        pt = map_coord(upperU);
        rear_upright_nodes(3, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL Upper
        rear_upright_nodes(4, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR Upper

        pt = map_coord(rl.wheel_center);
        rear_upright_nodes(5, :) = [pt(1), -abs(pt(2)), pt(3)]; % RL WC
        rear_upright_nodes(6, :) = [pt(1),  abs(pt(2)), pt(3)]; % RR WC

        success = true;
        if verbose
            fprintf('  Loaded 8 rear chassis mounts and 6 rear upright points.\n');
            fprintf('    Rear mount X range: %.3f to %.3f m\n', ...
                    min(rear_mount_nodes(:,1)), max(rear_mount_nodes(:,1)));
            fprintf('    Rear mount |Y| range: %.3f to %.3f m\n', ...
                    min(abs(rear_mount_nodes(:,2))), max(abs(rear_mount_nodes(:,2))));
        end

    catch ME
        warning('load_rear_suspension_hardpoints:Error', ...
                'Error parsing JSON: %s', ME.message);
    end
end
