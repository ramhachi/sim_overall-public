function [nodes, fbh_nodes, suspension_mount_nodes, inner_bracket_nodes, X_fh, X_mh, fbh_idx, susp_idx, rear_susp_idx] = generate_parametric_nodes(geom, suspension_file, shape_params, node_toggles, node_ratios)
% GENERATE_PARAMETRIC_NODES Creates a variable-node FSAE frame based on continuous geometric constraints.
% 
% Inputs:
%   geom            - Structure enclosing wheelbase_m, etc.
%   suspension_file - JSON filepath to hardpoints
%   shape_params    - Array of continuous variables [H_fh, W_si, H_si_upper, X_si_ratio]
%   node_toggles    - Array of boolean [Has_Front_Mid, Has_Rear_Mid]
%
% Outputs:
%   nodes           - Nx3 matrix [x, y, z] of the frame
%   ...             - Helper outputs used for rendering or constraints

    %% 1. Unpack Fixed Geometric Base Params
    L = geom.dimensions.wheelbase_m;
    Z_floor = 0.050; % 50mm ground clearance
    
    FBH_width = 0.400;  
    FBH_height = 0.400; 
    FBH_z_offset_from_cv = -0.050; 
    FBH_x_shift = 0.030; 
    
    %% 2. Unpack Optimized Shape Parameters
    H_fh       = shape_params(1);
    W_si       = shape_params(2);
    H_si_upper = shape_params(3);
    X_si_ratio = shape_params(4);
    
    % Derived/Fixed width and heights to keep problem stable
    % W_fh pushed to 320mm to confidently clear the 250mm front_hoop_sweep + pipe radius
    W_fh  = 0.320;
    W_mh  = 0.300;   
    
    % Main Hoop Brace connection width constraint (Variable, target 400mm total width)
    % The user explicitly requested the distance between brace mounting nodes to be fixed.
    W_mh_brace_target = 0.400; 
    W_mh_mid = W_mh + 0.100; % MH mid-wide: ±400mm
    W_mh_top = 0.100;  % MH shoulder: ±100mm
    
    H_si_lower = 0.05 + Z_floor;
    H_mh  = 1.35 + Z_floor; 
    H_sh  = 1.15 + Z_floor; 
    
    X_mh  = L;

    %% 3. Generate Fixed Front End (FBH and Suspension)
    base_offset_x = 0.500; 
    offset_x = base_offset_x - FBH_x_shift; 
    offset_z = FBH_z_offset_from_cv; 

    [fbh_nodes_susp, suspension_mount_nodes_susp, upright_nodes_susp, has_suspension] = load_suspension_hardpoints(suspension_file, offset_x, offset_z, FBH_width, FBH_height, false);
    
    % Load rear suspension hardpoints
    [rear_mount_nodes_raw, rear_upright_nodes, has_rear_suspension] = load_rear_suspension_hardpoints(suspension_file, L, false);

    FH_z_drop_from_fbh = 0.100;
    fbh_min_z = min(fbh_nodes_susp(:, 3));
    Z_FH_lower = max(Z_floor, fbh_min_z - FH_z_drop_from_fbh);
    FH_min_dist_from_susp = 0.150;
    inner_bracket_nodes = zeros(8, 3);
    
    if has_suspension
        fbh_nodes = fbh_nodes_susp;
        suspension_mount_nodes = suspension_mount_nodes_susp;
        
        max_susp_x = max(suspension_mount_nodes(:, 1));
        X_fh = max_susp_x + FH_min_dist_from_susp;
        
        % Bracket extension length (50mm)
        L_bracket = 0.050; 
        
        % Mapping Mounts (8) to Uprights (4)
        % Mounts: 1:FL_LF, 2:FR_LF, 3:FL_UF, 4:FR_UF, 5:FL_LR, 6:FR_LR, 7:FL_UR, 8:FR_UR
        % Uprights: 1:FL_Lower, 2:FR_Lower, 3:FL_Upper, 4:FR_Upper
        upright_map = [1, 2, 3, 4, 1, 2, 3, 4];
        
        for i = 1:8
            mount_pt = suspension_mount_nodes(i, :);
            upright_pt = upright_nodes_susp(upright_map(i), :);
            
            % Vector from Upright to Mount (pointing into the chassis)
            arm_vec = mount_pt - upright_pt;
            unit_vec = arm_vec / norm(arm_vec);
            
            % Place inner bracket node 50mm along this axis from the mount
            inner_bracket_nodes(i, :) = mount_pt + unit_vec * L_bracket;
        end
    else
        error('Suspension data must be present for optimization.');
    end
    
    X_mh = X_fh + 0.60; % T.1.1.1 cockpit template dictates minimum distance here
    
    % Use ratio to place the SI structure between FH and MH
    X_si = X_fh + (X_mh - X_fh) * X_si_ratio;

    % Rear section box positions from rear suspension
    if has_rear_suspension
        rear_f_mount_x = min(rear_mount_nodes_raw(1:4, 1));
        rear_r_mount_x = max(rear_mount_nodes_raw(5:8, 1));
        rear_mount_y_max = max(abs(rear_mount_nodes_raw(:, 2)));
        rear_mount_z_max = max(rear_mount_nodes_raw(:, 3));
        
        X_rear_f = rear_f_mount_x - 0.05;
        X_rear_r = rear_r_mount_x + 0.05;
        W_rear = W_mh;  % Same width as MH — mounts protrude outward
        H_rear_upper = rear_mount_z_max + 0.10;
    else
        X_rear_f = X_mh + 0.30;
        X_rear_r = X_mh + 0.60;
        W_rear = 0.250;
        H_rear_upper = 0.35;
    end

    H_mh_mid = H_fh - 0.200;

    %% 4. Construct Base Node Matrix
    base_nodes = [
        %% 1-4: Front Hoop
        X_fh, -W_fh, Z_FH_lower; 
        X_fh,  W_fh, Z_FH_lower; 
        X_fh, -W_fh, H_fh;       
        X_fh,  W_fh, H_fh;       

        %% 5-8: Side Impact Structure
        X_si, -W_si, H_si_lower;  
        X_si,  W_si, H_si_lower;  
        X_si, -W_si, H_si_upper;  
        X_si,  W_si, H_si_upper;  

        %% 9-15: Main Hoop Heptagon
        X_mh, -W_mh,     Z_floor;    % 9
        X_mh,  W_mh,     Z_floor;    % 10
        X_mh, -W_mh_mid, H_mh_mid;   % 11
        X_mh,  W_mh_mid, H_mh_mid;   % 12
        X_mh, -W_mh_top, H_sh;       % 13
        X_mh,  W_mh_top, H_sh;       % 14
        X_mh,  0.0,      H_mh;       % 15

        %% 16-19: Rear Section Front Ring
        X_rear_f, -W_rear, H_rear_upper;  % 16
        X_rear_f,  W_rear, H_rear_upper;  % 17
        X_rear_f, -W_rear, Z_floor;       % 18
        X_rear_f,  W_rear, Z_floor;       % 19

        %% 20-23: Rear Section Rear Ring
        X_rear_r, -W_rear, H_rear_upper;  % 20
        X_rear_r,  W_rear, H_rear_upper;  % 21
        X_rear_r, -W_rear, Z_floor;       % 22
        X_rear_r,  W_rear, Z_floor;       % 23
    ];
    
    nodes = base_nodes;
    
    %% 5. Conditional Dynamic Nodes (Mid-Planes)
    % Note: Front mid-planes removed — they fall inside the cockpit exclusion zone.
    has_rear_mid  = node_toggles(2);
    
    rear_x_ratio  = node_ratios(3);
    rear_z_ratio  = node_ratios(4);
    
    if has_rear_mid && (X_rear_r - X_mh > 0.3) % Need > 300mm gap
        % Position X continuously between MH and Rear
        X_rear_mid = X_mh + (X_rear_r - X_mh) * rear_x_ratio;
        W_rear_mid = (W_mh + W_rear) / 2;
        
        % Position Z continuously
        rear_z_lower = Z_floor;
        rear_z_upper = H_rear_upper;
        
        Z_rear_mid = rear_z_lower + (rear_z_upper - rear_z_lower) * rear_z_ratio;
        H_rear_mid = Z_rear_mid + 0.30; % 300mm box
        
        % Safety check
        H_rear_mid = min(H_rear_mid, H_rear_upper * 1.1);
        
        rear_mid_nodes = [
            X_rear_mid, -W_rear_mid, Z_rear_mid;
            X_rear_mid,  W_rear_mid, Z_rear_mid;
            X_rear_mid, -W_rear_mid, H_rear_mid;
            X_rear_mid,  W_rear_mid, H_rear_mid;
        ];
        nodes = [nodes; rear_mid_nodes];
    end
    
    %% 6. Append Fixed Structure and Track Indices
    
    % Track starting index for FBH nodes
    fbh_start = size(nodes, 1) + 1;
    nodes = [nodes; fbh_nodes];
    fbh_end = size(nodes, 1);
    fbh_idx = fbh_start:fbh_end;
    
    % Track starting index for Front Suspension Mount Nodes
    susp_start = size(nodes, 1) + 1;
    nodes = [nodes; suspension_mount_nodes];
    susp_end = size(nodes, 1);
    susp_idx = susp_start:susp_end;
    
    % Front Inner Brackets
    nodes = [nodes; inner_bracket_nodes];
    
    % Rear Suspension Mount Nodes and Inner Brackets
    rear_susp_idx = [];
    if has_rear_suspension
        rear_susp_start = size(nodes, 1) + 1;
        nodes = [nodes; rear_mount_nodes_raw];
        rear_susp_end = size(nodes, 1);
        rear_susp_idx = rear_susp_start:rear_susp_end;
        
        % Generate rear inner brackets (50mm axial extension)
        L_bracket_rear = 0.050;
        rear_inner_bracket_nodes = zeros(8, 3);
        rear_upright_map = [1, 2, 3, 4, 1, 2, 3, 4];
        for i = 1:8
            mount_pt = rear_mount_nodes_raw(i, :);
            upright_pt = rear_upright_nodes(rear_upright_map(i), :);
            arm_vec = mount_pt - upright_pt;
            unit_vec = arm_vec / norm(arm_vec);
            rear_inner_bracket_nodes(i, :) = mount_pt + unit_vec * L_bracket_rear;
        end
        nodes = [nodes; rear_inner_bracket_nodes];
    end
end
