function elements_geom = generate_adaptive_topology(nodes, fbh_nodes_idx, susp_nodes_idx, fh_x, mh_x, config, distance_thresholds, rear_susp_idx)
% GENERATE_ADAPTIVE_TOPOLOGY Creates dynamic frame connectivity based on distance
%
% Inputs:
%   nodes               - Nx3 matrix of all nodes
%   fbh_nodes_idx       - Vector of indices for FBH nodes (24-27)
%   susp_nodes_idx      - Vector of indices for front suspension nodes (28-35)
%   fh_x, mh_x          - X-coordinates of FH and MH for logical zoning
%   config              - Struct containing exclusion zones for collision filtering
%   distance_thresholds - [R_front, R_side, R_rear] Max connect distances (m)
%   rear_susp_idx       - (optional) Vector of indices for rear suspension nodes (44-51)
%
% Outputs:
%   elements_geom - Struct matching the required FEM solver format

    if nargin < 8
        rear_susp_idx = [];
    end

    % 1. Strategy Parameters
    R_front = distance_thresholds(1);
    R_side = distance_thresholds(2);
    R_rear = distance_thresholds(3);
    
    % Default pipe properties (Standard Steel 25x2.0 A)
    % NOTE: E and G must be in GPa to match FEM solver expectations
    D = 25.0; t = 2.0; E = 210; G = 80; rho = 7850; shape = 1; jf = 0.85;

    n_nodes = size(nodes, 1);
    
    % Track active edges to prevent duplicates (Adjacency Matrix)
    adj_matrix = false(n_nodes, n_nodes);
    % Track number of connections per node to enforce valency rules
    node_degrees = zeros(n_nodes, 1);
    elements_raw = [];
    
    % 2. Establish Fixed/Required Infrastructure (Base hoops)
    % Extract FBH nodes
    f1 = fbh_nodes_idx(1); f2 = fbh_nodes_idx(2); 
    f3 = fbh_nodes_idx(3); f4 = fbh_nodes_idx(4);

    % MH Heptagon: 9(base L), 10(base R), 11(mid L), 12(mid R),
    %              13(shoulder L), 14(shoulder R), 15(top)
    % RS Front Ring: 16(UL), 17(UR), 18(LL), 19(LR)
    % RS Rear Ring:  20(UL), 21(UR), 22(LL), 23(LR)
    mandatory_edges = [
        1,  2, 1;  1,  3, 1;  2,  4, 1;  3,  4, 1; % Front Hoop
        % MH Heptagon (Size A)
         9, 10, 2; % MH bottom
        10, 12, 2; 12, 14, 2; 14, 15, 2; % MH right: base→mid→shoulder→top
        15, 13, 2; 13, 11, 2; 11,  9, 2; % MH left: top→shoulder→mid→base
        13, 14, 5; % Shoulder Harness Bar (Group 5)
        % Front Bulkhead (Dynamic)
        f1, f2, 4; f1, f3, 4; f2, f4, 4; f3, f4, 4;
        f1, 1, 4; f2, 2, 4; % FBH Support Lower
        f3, 3, 4; f4, 4, 4; % FBH Support Upper
        % Side Impact Structure
        1,  5, 3; 2,  6, 3; % SI Lower (Front Half)
        5,  9, 3; 6, 10, 3; % SI Lower (Rear Half)
        3,  7, 3; 4,  8, 3; % SI Upper (Front Half)
        7, 11, 3; 8, 12, 3; % SI Upper (Rear Half) → MH mid-wide
        % MH → RS Front Ring
        13, 16, 4; 14, 17, 4; % MH shoulder → RS front upper
         9, 18, 4; 10, 19, 4; % MH base → RS front lower
        % RS Front Ring
        16, 17, 4; 18, 19, 4; 16, 18, 4; 17, 19, 4;
        % RS Rear Ring
        20, 21, 4; 22, 23, 4; 20, 22, 4; 21, 23, 4;
        % RS Rails (front → rear)
        16, 20, 4; 17, 21, 4; 18, 22, 4; 19, 23, 4;
        % Diagonals for rigidity
        f3, 1, 4; f4, 2, 4; % FBH→FH cross
        1,  7, 4; 2,  8, 4; % SI front diagonals
        5, 11, 4; 6, 12, 4; % SI rear diagonals → MH mid
        9, 16, 4; 10, 17, 4; % MH base → RS front upper (cross)
        16, 22, 4; 17, 23, 4; % RS left/right X-brace
    ];
    
    % All suspension nodes to isolate (front + rear)
    all_susp_nodes = [susp_nodes_idx(:)', rear_susp_idx(:)'];
    
    % Add front Suspension to Inner Bracket mandatory links
    inner_brackets_idx = susp_nodes_idx(end) + 1 : susp_nodes_idx(end) + length(susp_nodes_idx);
    for k = 1:length(susp_nodes_idx)
        mandatory_edges = [mandatory_edges; susp_nodes_idx(k), inner_brackets_idx(k), 4];
    end
    
    % Front inner brackets → FBH and FH (complete the load path!)
    % F-brackets (1-4: LF_L, LF_R, UF_L, UF_R) → FBH (f1-f4)
    fbh_targets = [f1, f2, f3, f4]; % lower L/R, upper L/R
    for k = 1:min(4, length(inner_brackets_idx))
        mandatory_edges = [mandatory_edges; inner_brackets_idx(k), fbh_targets(k), 4];
    end
    % R-brackets (5-8: LR_L, LR_R, UR_L, UR_R) → FH (1-4)
    fh_targets = [1, 2, 3, 4]; % lower L/R, upper L/R
    for k = 5:min(8, length(inner_brackets_idx))
        mandatory_edges = [mandatory_edges; inner_brackets_idx(k), fh_targets(k-4), 4];
    end
    
    % Add rear Suspension to Inner Bracket mandatory links + RS connections
    if ~isempty(rear_susp_idx)
        rear_inner_idx = rear_susp_idx(end) + 1 : rear_susp_idx(end) + length(rear_susp_idx);
        for k = 1:length(rear_susp_idx)
            mandatory_edges = [mandatory_edges; rear_susp_idx(k), rear_inner_idx(k), 4];
        end
        % Rear inner F brackets (1-4) → RS front ring
        rear_f_targets = [18, 19, 16, 17]; % lower L/R, upper L/R
        for k = 1:4
            if rear_inner_idx(k) <= n_nodes
                mandatory_edges = [mandatory_edges; rear_inner_idx(k), rear_f_targets(k), 4];
            end
        end
        % Rear inner R brackets (5-8) → RS rear ring
        rear_r_targets = [22, 23, 20, 21];
        for k = 5:8
            if k <= length(rear_inner_idx) && rear_inner_idx(k) <= n_nodes
                mandatory_edges = [mandatory_edges; rear_inner_idx(k), rear_r_targets(k-4), 4];
            end
        end
    end

    for r = 1:size(mandatory_edges, 1)
        i = mandatory_edges(r, 1);
        j = mandatory_edges(r, 2);
        grp = mandatory_edges(r, 3);
        adj_matrix(i, j) = true;
        adj_matrix(j, i) = true;
        node_degrees(i) = node_degrees(i) + 1;
        node_degrees(j) = node_degrees(j) + 1;
        elements_raw = [elements_raw; i, j, D, t, E, G, rho, shape, jf, grp];
    end
    
    % 3. Graph-Theoretic Distance Connection (Adaptive Truss)
    for i = 1:n_nodes
        for j = i+1:n_nodes
            if adj_matrix(i, j)
                continue; % Already connected
            end
            % ISOLATION RULE: Suspension mounts (e.g., Nodes 22-29) MUST NOT connect to ANY
            % node other than their designated inner bracket (which was handled in mandatory_edges).
            if ismember(i, all_susp_nodes) || ismember(j, all_susp_nodes)
                continue;
            end
            
            % Calculate spatial distance
            n1 = nodes(i, :);
            n2 = nodes(j, :);
            dist = norm(n1 - n2);
            
            % Avoid micro-pipes
            if dist < 0.15 
                continue; 
            end
            
            % Determine regional distance threshold
            mid_x = (n1(1) + n2(1)) / 2;
            if mid_x < fh_x
                R_target = R_front;
                group = 4; % Structural (Front)
            elseif mid_x < mh_x
                R_target = R_side;
                group = 3; % Side Impact
            else
                R_target = R_rear;
                group = 4; % Structural (Rear)
            end
            
            % Max degree constraint: do not exceed 4 connections per node
            if node_degrees(i) >= 4 || node_degrees(j) >= 4
                continue;
            end
            
            % If nodes are close enough, attempt connection
            if dist <= R_target
                % Pre-filter: Check collision with exclusion zones BEFORE creating edge
                temp_elem = [1, 2]; % dummy indices for row
                temp_nodes = [n1; n2]; % provide exactly 2 nodes
                penalties = checkExclusionZones(temp_nodes, temp_elem, config);
                
                if penalties(1) == 0 % Physically valid pipe
                    adj_matrix(i, j) = true;
                    adj_matrix(j, i) = true;
                    node_degrees(i) = node_degrees(i) + 1;
                    node_degrees(j) = node_degrees(j) + 1;
                    elements_raw = [elements_raw; i, j, D, t, E, G, rho, shape, jf, group];
                end
            end
        end
    end
    
    % 4. Minimum Degree Enforcement (Post-pass)
    % Find nodes that are dead-ends or isolated (degree < 2)
    % Only apply to frame nodes, exclude suspension brackets
    for i = 1:n_nodes
        if ismember(i, all_susp_nodes)
            continue;
        end
        
        while node_degrees(i) < 2
            % Find the closest legal neighbor we aren't already connected to
            % that also hasn't exceeded its max degree
            best_j = -1;
            min_dist = inf;
            n1 = nodes(i, :);
            
            for j = 1:n_nodes
                if i == j || adj_matrix(i, j) || node_degrees(j) >= 4
                    continue;
                end
                if ismember(j, all_susp_nodes)
                    continue; % Do not anchor directly to suspension mounts
                end
                
                n2 = nodes(j, :);
                d = norm(n1 - n2);
                
                % Must still be manufacturable (> 15cm)
                if d < 0.150
                    continue;
                end
                
                if d < min_dist
                    % Quick collision check as a candidate
                    temp_nodes = [n1; n2];
                    penalties = checkExclusionZones(temp_nodes, [1, 2], config);
                    if penalties(1) == 0
                        min_dist = d;
                        best_j = j;
                    end
                end
            end
            
            if best_j ~= -1
                % Force connection to rescue the node
                adj_matrix(i, best_j) = true;
                adj_matrix(best_j, i) = true;
                node_degrees(i) = node_degrees(i) + 1;
                node_degrees(best_j) = node_degrees(best_j) + 1;
                
                mid_x = (n1(1) + nodes(best_j, 1)) / 2;
                if mid_x < fh_x
                    group = 4;
                elseif mid_x < mh_x
                    group = 3;
                else
                    group = 4;
                end
                
                elements_raw = [elements_raw; i, best_j, D, t, E, G, rho, shape, jf, group];
            else
                % Could not rescue node (no valid paths)
                break;
            end
        end
    end
    
    % 4. Format Output Structure
    elements_geom = struct();
    if isempty(elements_raw)
        % Failsafe
        elements_geom.base_elements = [1, 2, D, t, E, G, rho, shape, jf, 1];
        elements_geom.connectivity = [1, 2];
        elements_geom.group_indices = [1];
    else
        elements_geom.base_elements = elements_raw(:, 1:9);
        elements_geom.connectivity = elements_raw(:, 1:2);
        elements_geom.group_indices = elements_raw(:, 10);
    end
    
    elements_geom.group_names = {'front_hoop', 'main_hoop', 'side_impact', 'structural', 'shoulder'};
end
