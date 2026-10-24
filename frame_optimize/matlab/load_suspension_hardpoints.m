function [fbh_nodes, front_mount_nodes, upright_nodes, success] = load_suspension_hardpoints(json_path, offset_x_m, offset_z_m, fbh_width_m, fbh_height_m, verbose)
% LOAD_SUSPENSION_HARDPOINTS Parse front suspension hardpoints from vehicle artifact and place FBH
%
% Parses the authoritative vehicle artifact and extracts all 8 physical chassis
% mount points for the front suspension (FL/FR, Lower/Upper, Front/Rear).
% Also positions the Front Bulkhead (FBH) relative to the front wheel center line.
% Optionally accepts FBH width and height parameters.
%
% Geometry conventions:
%   Suspension json puts coordinate origin [0,0] at the midpoint of the front
%   wheels' center line. 
%
% Frame coordinate mapping convention here:
%   We will translate everything so the Front Hoop is still at X=0, but the 
%   user specifies offsets from the suspension origin.
%   By default, let's keep the suspension origin as X=0 in THIS scope's output,
%   then the main frame script just offsets everything by the required distance.
%   Actually, let's generate FBH and Mounts in the SUSPENSION's coordinate system first
%   where Wheel Center Line is X=0 and Z=0.
%
% Inputs:
%   json_path - Path to the vehicle artifact JSON with suspension hardpoints
%   offset_x_m - Distance of the FBH from the front wheel center line (e.g. 0.5m forward)
%                (Positive value means forward of wheel center line)
%   offset_z_m - Distance of the FBH bottom from the front wheel center Z height
%   fbh_width_m - (Optional) Full width of the FBH (default: 0.300)
%   fbh_height_m - (Optional) Total height of the FBH (default: 0.250)
%
% Outputs:
%   fbh_nodes - 4x3 matrix of [X, Y, Z] FBH coordinates (Lower L, Lower R, Upper L, Upper R)
%   front_mount_nodes - 8x3 matrix of [X, Y, Z] Suspension mount coordinates
%   upright_nodes - 6x3 matrix of [X,Y,Z] Upright coordinates for visualization
%                   (FL Lower/Upper, FR Lower/Upper, FL/FR Wheel Center)
%   success - Boolean indicating if the load was successful

    persistent cached_json_path cached_fl

    if nargin < 2
        offset_x_m = 0.500; % Default 500mm forward of wheel center
    end
    if nargin < 3
        offset_z_m = 0.0; % Default: FBH bottom is at wheel center height
    end
    if nargin < 4 || isempty(fbh_width_m)
        fbh_width_m = 0.300; % Default 300mm width
    end
    if nargin < 5 || isempty(fbh_height_m)
        fbh_height_m = 0.250; % Default 250mm height
    end
    if nargin < 6 || isempty(verbose)
        verbose = true;
    end

    fbh_nodes = zeros(4, 3);
    front_mount_nodes = zeros(8, 3);
    upright_nodes = zeros(6, 3);
    success = false;

    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'matlab'));
    end

    if ~isfile(json_path)
        warning('load_suspension_hardpoints:FileNotFound', 'Suspension geometry file %s not found.', json_path);
        return;
    end

    try
        if isempty(cached_json_path) || ~strcmp(cached_json_path, json_path)
            artifact = load_vehicle_artifact(json_path);
            if isfield(artifact.params, 'suspension') && isfield(artifact.params.suspension, 'kc') ...
                && isfield(artifact.params.suspension.kc, 'corners') && isfield(artifact.params.suspension.kc.corners, 'FL') ...
                && isfield(artifact.params.suspension.kc.corners.FL, 'hardpoints')
                cached_fl = artifact.params.suspension.kc.corners.FL.hardpoints;
                cached_json_path = json_path;
            else
                warning('load_suspension_hardpoints:MissingFL', ...
                    'FL hardpoints not found in artifact. Run EasyStart [8] first: %s', json_path);
                return;
            end
        end
        fl = cached_fl;

        % FBH Dimensions (Configurable)
        W_fbh = fbh_width_m / 2; % Half width
        H_fbh = fbh_height_m;    % Full height
        
        % Frame Coordinate System: X is rearward, Z is upward.
        % Suspension JSON: X forward, Z downward.
        % The front wheel center line in Frame CS is at X=0.
        
        % Find wheel center Z in Frame CS (Upward)
        if isfield(fl, 'wheel_center')
            z_cv = -fl.wheel_center(3);
        else
            z_cv = 0.228; % Default
        end

        % Suspension origin is front wheel center line (X=0).
        % FBH is placed offset_x_m forward. Since +X is rearward in Frame CS, FBH is at -offset_x_m.
        fbh_x = -offset_x_m;
        fbh_z = z_cv + offset_z_m;
        
        fbh_nodes = [
            fbh_x, -W_fbh, fbh_z;            % 25: FBH lower left
            fbh_x,  W_fbh, fbh_z;            % 26: FBH lower right
            fbh_x, -W_fbh, fbh_z + H_fbh;    % 27: FBH upper left
            fbh_x,  W_fbh, fbh_z + H_fbh;    % 28: FBH upper right
        ];
        
        % Helper to map coordinate (FL gives left side)
        % Suspension JSON: X forward, Y left, Z DOWNWARD.
        % Frame assumes: Right-handed standard where Y is width, +X is REARWARD, +Z is UPWARD.
        % We invert X and Z.
        map_coord = @(pt) [-pt(1), pt(2), -pt(3)];
        
        lowerF = fl.lower_arm_chassis_fore;
        upperF = fl.upper_arm_chassis_fore;
        lowerR = fl.lower_arm_chassis_aft;
        upperR = fl.upper_arm_chassis_aft;
        lowerU = fl.lower_ball_joint;
        upperU = fl.upper_ball_joint;
        
        % --- Chassis Mounts ---
        pt = map_coord(lowerF);
        front_mount_nodes(1, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL
        front_mount_nodes(2, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR
        
        pt = map_coord(upperF);
        front_mount_nodes(3, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL
        front_mount_nodes(4, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR
        
        pt = map_coord(lowerR);
        front_mount_nodes(5, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL
        front_mount_nodes(6, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR
        
        pt = map_coord(upperR);
        front_mount_nodes(7, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL
        front_mount_nodes(8, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR
        
        % --- Upright Nodes ---
        pt = map_coord(lowerU);
        upright_nodes(1, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL Lower
        upright_nodes(2, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR Lower
        
        pt = map_coord(upperU);
        upright_nodes(3, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL Upper
        upright_nodes(4, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR Upper
        
        pt = map_coord(fl.wheel_center);
        upright_nodes(5, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL Wheel Center
        upright_nodes(6, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR Wheel Center
        
        success = true;
        if verbose
            fprintf('  Loaded FBH points, 8 chassis mounts, and 6 upright mounts based on wheel center.\n');
        end
        
    catch ME
        warning('load_suspension_hardpoints:Error', 'Error parsing artifact: %s', ME.message);
    end
end
