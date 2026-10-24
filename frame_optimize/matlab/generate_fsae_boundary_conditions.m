function [bcs, loads, fixture] = generate_fsae_boundary_conditions(nodes, load_case_name, config)
% GENERATE_FSAE_BOUNDARY_CONDITIONS Generate physically correct BCs for FSAE frame
%
% Generates boundary conditions matching real FSAE torsional rigidity test setup:
%   - Torsion: Rear suspension mounts constrained, front couple force
%   - Lateral: Appropriate cornering constraints
%   - Longitudinal: Braking constraints
%   - Rollover: Rollover proxy constraints
%   - Diff reaction: differential mount reaction constraints
%
% Inputs:
%   nodes         - Nx3 node coordinates [m]
%   load_case_name - 'torsion', 'lateral', 'longitudinal', 'rollover', 'diff_reaction'
%   config        - Struct with fields:
%                   .rear_lower_nodes  - [left, right] rear lower node IDs (suspension mounts)
%                   .front_lower_nodes - [left, right] front lower node IDs
%                   .rear_upper_nodes  - [left, right] rear upper node IDs (optional)
%                   .front_upper_nodes - [left, right] front upper node IDs
%                   .torque_Nm         - Torque for torsion test [Nm] (default: 1000)
%                   .total_mass_kg     - Vehicle mass for inertial loads (default: 230)
%                   .lateral_g         - Lateral acceleration [G] (default: 1.5)
%                   .braking_g         - Braking deceleration [G] (default: 1.0)
%                   .main_hoop_top_nodes - main-hoop top node(s) (rollover)
%                   .rollover_force_N  - [Fx Fy Fz] at the MH top (default rule 6/5/-9 kN)
%                   .torsion_fixture   - 'pinned' (default: all rear mounts pinned) or
%                                        '321' (rigid per-corner hubs, 3-2-1 rear support,
%                                        front couple at the hubs, #51)
%
% Outputs:
%   bcs     - Bx3 [node_id, dof, value]
%   loads   - Lx3 [node_id, dof, value]
%   fixture - struct: extra_nodes (Kx3, appended after nodes), rigid_links (Rx2)
%
% DOF convention: 1=ux, 2=uy, 3=uz, 4=rx, 5=ry, 6=rz
%
% FSAE Torsion Test Setup:
%   Real test: Rear axle fixed (4 points), front axle free with torque input
%   FEM equivalent:
%     - Rear lower L/R: constrain ux, uy, uz (simulate suspension mounts)
%     - Front lower L: constrain uz only (vertical support)
%     - Front lower R: constrain uy, uz (prevent lateral slide + vertical support)
%     - Torsion: Couple force at front lower L/R in Z direction
%   This gives exactly 6+2=8 constraints (6 rigid body + 2 for stability)
%   while allowing the frame to twist naturally.

    narginchk(3, 3);
    fixture = struct('extra_nodes', zeros(0, 3), 'rigid_links', zeros(0, 2));
    
    rear_L = config.rear_lower_nodes(1);
    rear_R = config.rear_lower_nodes(2);
    
    if isfield(config, 'front_susp_left_nodes') && isfield(config, 'front_susp_right_nodes')
        front_left_nodes = config.front_susp_left_nodes;
        front_right_nodes = config.front_susp_right_nodes;
    else
        front_left_nodes = [config.front_lower_nodes(1)];
        front_right_nodes = [config.front_lower_nodes(2)];
    end
    
    % Rear suspension mount nodes (if available, used for distributed BC)
    has_rear_susp = isfield(config, 'rear_susp_left_nodes') && isfield(config, 'rear_susp_right_nodes');
    if has_rear_susp
        rear_left_nodes = config.rear_susp_left_nodes;
        rear_right_nodes = config.rear_susp_right_nodes;
    end
    
    has_rear_upper = isfield(config, 'rear_upper_nodes');
    if has_rear_upper
        rear_upper_L = config.rear_upper_nodes(1);
        rear_upper_R = config.rear_upper_nodes(2);
    end
    
    torque = 1000;
    if isfield(config, 'torque_Nm'), torque = config.torque_Nm; end
    
    total_mass = 230;
    if isfield(config, 'total_mass_kg'), total_mass = config.total_mass_kg; end
    
    lateral_g = 1.5;
    if isfield(config, 'lateral_g'), lateral_g = config.lateral_g; end
    
    braking_g = 1.0;
    if isfield(config, 'braking_g'), braking_g = config.braking_g; end

    g = 9.81;
    
    switch lower(load_case_name)
        case 'torsion'
            if isfield(config, 'torsion_fixture') && strcmpi(config.torsion_fixture, '321')
                % Rigid hub per corner (dummy upright/A-arms) tied to its 4 mounts. Rear:
                % 3-2-1 statically determinate support (RL: x,y,z + pitch; RR: x,z), so
                % the rear is not artificially stiffened. Front: +-F at the hubs, i.e. a
                % jig beam with spherical ends; torque = F * hub track exactly.
                if ~has_rear_susp
                    error('generate_fsae_boundary_conditions:MissingRearMounts', ...
                        'torsion_fixture=321 needs rear_susp_left_nodes/right_nodes.');
                end
                groups = {front_left_nodes, front_right_nodes, rear_left_nodes, rear_right_nodes};
                n0 = size(nodes, 1);
                for h = 1:4
                    fixture.extra_nodes(h, :) = mean(nodes(groups{h}, :), 1);
                    g = reshape(groups{h}, [], 1);
                    fixture.rigid_links = [fixture.rigid_links; repmat(n0 + h, numel(g), 1), g];
                end
                hub = n0 + (1:4);  % FL FR RL RR
                bcs = [hub(3) 1 0; hub(3) 2 0; hub(3) 3 0; hub(3) 5 0; hub(4) 1 0; hub(4) 3 0];
                arm = abs(fixture.extra_nodes(1, 2) - fixture.extra_nodes(2, 2));
                loads = [hub(1) 3 torque / arm; hub(2) 3 -torque / arm];
                return;
            end
            %% Torsion Test - FSAE Standard Setup
            % Fix all rear suspension mount points (simulate rigid rear axle)
            bcs = [];
            if has_rear_susp
                % Constrain ALL rear susp mount nodes (all 8 points)
                for n = [rear_left_nodes, rear_right_nodes]
                    bcs = [bcs; n, 1, 0; n, 2, 0; n, 3, 0];
                end
            else
                % Fallback: use rear_lower + rear_upper
                bcs = [
                    rear_L, 1, 0;  rear_L, 2, 0;  rear_L, 3, 0;
                    rear_R, 1, 0;  rear_R, 2, 0;  rear_R, 3, 0;
                ];
                if has_rear_upper
                    bcs = [bcs;
                        rear_upper_L, 1, 0; rear_upper_L, 2, 0; rear_upper_L, 3, 0;
                        rear_upper_R, 1, 0; rear_upper_R, 2, 0; rear_upper_R, 3, 0;
                    ];
                end
            end
            
            % Apply torque as couple force at front nodes. Equal forces per node, so the
            % applied moment is F_couple times the L/R centroid distance (#51).
            arm = abs(mean(nodes(front_left_nodes, 2)) - mean(nodes(front_right_nodes, 2)));
            
            loads = [];
            if arm > 1e-4
                F_couple = torque / arm;
                F_per_node = F_couple / length(front_left_nodes);
                
                for n = front_left_nodes
                    loads = [loads; n, 3, F_per_node];
                end
                for n = front_right_nodes
                    loads = [loads; n, 3, -F_per_node];
                end
            else
                % Fallback: apply moment directly
                for n = front_left_nodes
                    loads = [loads; n, 4, torque / (2 * length(front_left_nodes))];
                end
                for n = front_right_nodes
                    loads = [loads; n, 4, torque / (2 * length(front_right_nodes))];
                end
            end
            
        case 'lateral'
            %% Lateral (Cornering) Load Case
            % Rear susp mounts fixed, lateral forces at front susp mounts
            
            bcs = [];
            if has_rear_susp
                for n = [rear_left_nodes, rear_right_nodes]
                    bcs = [bcs; n, 1, 0; n, 2, 0; n, 3, 0];
                end
            else
                bcs = [
                    rear_L, 1, 0;  rear_L, 2, 0;  rear_L, 3, 0;
                    rear_R, 1, 0;  rear_R, 2, 0;  rear_R, 3, 0;
                ];
                if has_rear_upper
                    bcs = [bcs;
                        rear_upper_L, 1, 0; rear_upper_L, 2, 0; rear_upper_L, 3, 0;
                        rear_upper_R, 1, 0; rear_upper_R, 2, 0; rear_upper_R, 3, 0;
                    ];
                end
            end
            
            % Total inertial force at the front mounts (rear is fixed), not
            % a per-axle half-vehicle share.
            F_lat = total_mass * lateral_g * g;
            F_per_node = F_lat / (length(front_left_nodes) + length(front_right_nodes));
            
            loads = [];
            for n = [front_left_nodes, front_right_nodes]
                loads = [loads; n, 2, F_per_node];
            end
            
        case 'longitudinal'
            %% Longitudinal (Braking) Load Case
            % Rear susp mounts fixed, braking forces at front susp mounts
            
            bcs = [];
            if has_rear_susp
                for n = [rear_left_nodes, rear_right_nodes]
                    bcs = [bcs; n, 1, 0; n, 2, 0; n, 3, 0];
                end
            else
                bcs = [
                    rear_L, 1, 0;  rear_L, 2, 0;  rear_L, 3, 0;
                    rear_R, 1, 0;  rear_R, 2, 0;  rear_R, 3, 0;
                ];
                if has_rear_upper
                    bcs = [bcs;
                        rear_upper_L, 1, 0; rear_upper_L, 2, 0; rear_upper_L, 3, 0;
                        rear_upper_R, 1, 0; rear_upper_R, 2, 0; rear_upper_R, 3, 0;
                    ];
                end
            end
            
            % Total braking inertial force at the front mounts, not a
            % per-axle half-vehicle share.
            F_brake = total_mass * braking_g * g;
            F_per_node = F_brake / (length(front_left_nodes) + length(front_right_nodes));
            
            loads = [];
            for n = [front_left_nodes, front_right_nodes]
                loads = [loads; n, 1, -F_per_node];
            end
            
        case 'rollover'
            %% Rollover Load Case
            % All lower nodes fixed, lateral force at main hoop top
            
            bcs = [
                rear_L, 1, 0;  rear_L, 2, 0;  rear_L, 3, 0;
                rear_R, 1, 0;  rear_R, 2, 0;  rear_R, 3, 0;
            ];
            for n = [front_left_nodes, front_right_nodes]
                bcs = [bcs; n, 1, 0; n, 2, 0; n, 3, 0];
            end
            
            % Rule main-hoop load case at the main-hoop top (#49): FSAE alternative-frame
            % "Main Hoop" case Fx = 6.0 kN, Fy = 5.0 kN, Fz = -9.0 kN.
            % TODO(rules): verify magnitudes/directions against the current rulebook.
            % Frame CS: +X rearward, +Y right, +Z up -> rearward, lateral, downward.
            if ~isfield(config, 'main_hoop_top_nodes') || isempty(config.main_hoop_top_nodes)
                error('generate_fsae_boundary_conditions:MissingMainHoopTop', ...
                    'rollover requires config.main_hoop_top_nodes.');
            end
            roll_F = [6000, 5000, -9000];
            if isfield(config, 'rollover_force_N'), roll_F = config.rollover_force_N; end
            top = reshape(config.main_hoop_top_nodes, [], 1);
            loads = [];
            for d = 1:3
                loads = [loads; top, repmat([d, roll_F(d) / numel(top)], numel(top), 1)]; %#ok<AGROW>
            end

        case 'diff_reaction'
            %% Differential Reaction Load Case
            % Rear suspension mounts fixed, drivetrain reaction at diff mounts.
            if ~isfield(config, 'diff_mount_nodes') || isempty(config.diff_mount_nodes)
                error('generate_fsae_boundary_conditions:MissingDiffMountNodes', ...
                    'diff_reaction requires config.diff_mount_nodes.');
            end

            bcs = [];
            if has_rear_susp
                for n = [rear_left_nodes, rear_right_nodes]
                    bcs = [bcs; n, 1, 0; n, 2, 0; n, 3, 0];
                end
            else
                bcs = [
                    rear_L, 1, 0;  rear_L, 2, 0;  rear_L, 3, 0;
                    rear_R, 1, 0;  rear_R, 2, 0;  rear_R, 3, 0;
                ];
                if has_rear_upper
                    bcs = [bcs;
                        rear_upper_L, 1, 0; rear_upper_L, 2, 0; rear_upper_L, 3, 0;
                        rear_upper_R, 1, 0; rear_upper_R, 2, 0; rear_upper_R, 3, 0;
                    ];
                end
            end

            diff_nodes = reshape(config.diff_mount_nodes, 1, []);
            reaction_force = 1500;
            if isfield(config, 'diff_reaction_force_N')
                reaction_force = config.diff_reaction_force_N;
            end
            reaction_torque = 500;
            if isfield(config, 'diff_reaction_torque_Nm')
                reaction_torque = config.diff_reaction_torque_Nm;
            end

            loads = [];
            for n = diff_nodes
                loads = [loads; n, 1, reaction_force / numel(diff_nodes)];
            end

            y_values = nodes(diff_nodes, 2);
            arm = max(y_values) - min(y_values);
            if arm > 1e-4
                F_couple = reaction_torque / arm;
                left_nodes = diff_nodes(y_values < mean(y_values));
                right_nodes = diff_nodes(y_values >= mean(y_values));
                if isempty(left_nodes) || isempty(right_nodes)
                    for n = diff_nodes
                        loads = [loads; n, 4, reaction_torque / numel(diff_nodes)];
                    end
                else
                    for n = left_nodes
                        loads = [loads; n, 3, F_couple / numel(left_nodes)];
                    end
                    for n = right_nodes
                        loads = [loads; n, 3, -F_couple / numel(right_nodes)];
                    end
                end
            else
                for n = diff_nodes
                    loads = [loads; n, 4, reaction_torque / numel(diff_nodes)];
                end
            end
            
        otherwise
            error('generate_fsae_boundary_conditions:UnknownCase', ...
                  'Unknown load case: %s. Use torsion/lateral/longitudinal/rollover/diff_reaction', ...
                  load_case_name);
    end
end
