function visualize_3d_suspension(all_results, vehicle_json_path)
% VISUALIZE_3D_SUSPENSION - Plots the optimized 3D suspension geometry
% Extracts A-arm, tie rod, and POU hardpoints for each optimized wheel
% and displays them in a 3D interactive plot.

    helper_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(helper_dir);
    end
    if nargin < 2 || isempty(vehicle_json_path)
        vehicle_json_path = resolve_default_vehicle_json(fullfile(helper_dir, '..'), ...
            'RequireExportReady', true, ...
            'RequireSuspensionHardpoints', true);
    end
    if isempty(vehicle_json_path) || ~isfile(vehicle_json_path)
        error('No vehicle artifact with suspension hardpoints found for visualization.');
    end

    f = figure('Name', 'Optimized Suspension 3D Geometry', 'Color', 'w', ...
           'Position', [100, 100, 1000, 800]);
    hold on; grid on; axis equal;
    view(-35, 30);
    xlabel('X (Forward) [m]'); ylabel('Y (Right) [m]'); zlabel('Z (Down/Up) [m]');
    
    % MATLAB convention: SAE is X-fwd, Y-right, Z-down.
    % We will invert the Z axis so the car points the correct way up visually.
    set(gca, 'ZDir', 'reverse');
    
    title('Optimized Suspension 3D Geometry (A-Arm + POU Linkage)');
    
    % ======================================================
    % Load ALL wheel geometry directly from JSON
    % ======================================================
    % All A-arm optimizers use axle-local coordinates (X=0 at each axle).
    % We apply global X offsets here for correct vehicle-level positioning:
    %   Front axle: X = +0.60 m (default, from CG)
    %   Rear  axle: X = +0.60 - wheelbase
    % This approach bypasses define_suspension_geometry.m entirely,
    % ensuring consistent coordinate handling for visualization.
    % POU data (M_pivot, bellcrank) is kept from all_results for front wheels.
    % ======================================================
    wheelbase = 1.535;
    artifact = load_vehicle_artifact(vehicle_json_path);
    if isfield(artifact, 'params') && isfield(artifact.params, 'mass') && isfield(artifact.params.mass, 'wheelbase')
        wheelbase = artifact.params.mass.wheelbase;
    end

    if ~isfield(artifact.params, 'suspension') || ~isfield(artifact.params.suspension, 'kc') ...
            || ~isfield(artifact.params.suspension.kc, 'corners')
        error('Artifact does not contain suspension hardpoints: %s', vehicle_json_path);
    end

    corners = artifact.params.suspension.kc.corners;
    cfgs = { ...
        'FL', 'FL', false; ...
        'FR', 'FR', false; ...
        'RL', 'RL', true;  ...
        'RR', 'RR', true;  ...
    };

    for ci = 1:size(cfgs, 1)
        tag = cfgs{ci, 1};
        cornerName = cfgs{ci, 2};
        is_rear = cfgs{ci, 3};

        if ~isfield(corners, cornerName) || ~isfield(corners.(cornerName), 'hardpoints')
            continue;
        end

        hp = corners.(cornerName).hardpoints;
        g = struct();
        g.upper_arm_front_chassis = hp.upper_arm_chassis_fore(:)';
        g.upper_arm_rear_chassis  = hp.upper_arm_chassis_aft(:)';
        g.lower_arm_front_chassis = hp.lower_arm_chassis_fore(:)';
        g.lower_arm_rear_chassis  = hp.lower_arm_chassis_aft(:)';
        g.upper_arm_upright       = hp.upper_ball_joint(:)';
        g.lower_arm_upright       = hp.lower_ball_joint(:)';
        g.x_wheel = hp.wheel_center(1);
        g.y_wheel = hp.wheel_center(2);
        if isfield(hp, 'contact_patch')
            g.tire_radius = norm(hp.wheel_center(:)' - hp.contact_patch(:)');
        else
            g.tire_radius = abs(hp.wheel_center(3));
        end
        g.is_rear = is_rear;

        if ~is_rear && isfield(hp, 'tie_rod_inner') && isfield(hp, 'tie_rod_outer') ...
                && any(hp.tie_rod_inner(:) ~= 0) && any(hp.tie_rod_outer(:) ~= 0)
            g.tie_rod_chassis = hp.tie_rod_inner(:)';
            g.tie_rod_upright = hp.tie_rod_outer(:)';
        else
            g.tie_rod_chassis = g.upper_arm_front_chassis;
            g.tie_rod_upright = g.upper_arm_upright;
        end

        if isfield(all_results, tag)
            all_results.(tag).geom = g;
        else
            all_results.(tag) = struct('geom', g);
        end
    end

    wheels = fieldnames(all_results);
    
    % Colors for different components
    a_arm_color = [0.2 0.2 0.2];
    upright_color = [0.8 0.4 0.1];
    tie_rod_color = [0.2 0.8 0.2];
    pushrod_color = [0.8 0.2 0.2];
    bellcrank_color = [0.2 0.4 0.8];
    
    % Handle storage for legend (only plot legends once)
    h_a_arm = [];
    h_upright = [];
    h_kp = [];
    h_tie = [];
    h_push = [];
    h_bell = [];
    h_damper = [];
    h_tire = [];

    for i = 1:length(wheels)
        wn = wheels{i};
        res = all_results.(wn);
        
        if isfield(res, 'error') || ~isfield(res, 'geom')
            continue;
        end
        
        geom = res.geom;
        
        % Plot Upper A-arm
        p1 = plot3([geom.upper_arm_front_chassis(1) geom.upper_arm_upright(1) geom.upper_arm_rear_chassis(1) geom.upper_arm_front_chassis(1)], ...
              [geom.upper_arm_front_chassis(2) geom.upper_arm_upright(2) geom.upper_arm_rear_chassis(2) geom.upper_arm_front_chassis(2)], ...
              [geom.upper_arm_front_chassis(3) geom.upper_arm_upright(3) geom.upper_arm_rear_chassis(3) geom.upper_arm_front_chassis(3)], ...
              '-o', 'Color', a_arm_color, 'LineWidth', 2, 'MarkerFaceColor', a_arm_color);
        if isempty(h_a_arm), h_a_arm = p1(1); end
          
        % Plot Lower A-arm
        plot3([geom.lower_arm_front_chassis(1) geom.lower_arm_upright(1) geom.lower_arm_rear_chassis(1) geom.lower_arm_front_chassis(1)], ...
              [geom.lower_arm_front_chassis(2) geom.lower_arm_upright(2) geom.lower_arm_rear_chassis(2) geom.lower_arm_front_chassis(2)], ...
              [geom.lower_arm_front_chassis(3) geom.lower_arm_upright(3) geom.lower_arm_rear_chassis(3) geom.lower_arm_front_chassis(3)], ...
              '-o', 'Color', a_arm_color, 'LineWidth', 2, 'MarkerFaceColor', a_arm_color);
          
        % Plot Upright Centerline (Kingpin)
        p2 = plot3([geom.upper_arm_upright(1) geom.lower_arm_upright(1)], ...
              [geom.upper_arm_upright(2) geom.lower_arm_upright(2)], ...
              [geom.upper_arm_upright(3) geom.lower_arm_upright(3)], ...
              '-', 'Color', 'k', 'LineWidth', 3);
        if isempty(h_kp), h_kp = p2(1); end

        is_rear_wheel = isfield(geom, 'is_rear') && geom.is_rear;
          
        % Plot Upright Steering Arm & Tie Rod (front only)
        if ~is_rear_wheel
            p3 = plot3([geom.tie_rod_upright(1) geom.lower_arm_upright(1)], ...
                  [geom.tie_rod_upright(2) geom.lower_arm_upright(2)], ...
                  [geom.tie_rod_upright(3) geom.lower_arm_upright(3)], ...
                  '-', 'Color', upright_color, 'LineWidth', 2);
            if isempty(h_upright), h_upright = p3(1); end
              
            p4 = plot3([geom.tie_rod_chassis(1) geom.tie_rod_upright(1)], ...
                  [geom.tie_rod_chassis(2) geom.tie_rod_upright(2)], ...
                  [geom.tie_rod_chassis(3) geom.tie_rod_upright(3)], ...
                  '-o', 'Color', tie_rod_color, 'LineWidth', 2, 'MarkerFaceColor', tie_rod_color);
            if isempty(h_tie), h_tie = p4(1); end
        end
          
        % If optimization succeeded, plot POU components
        if isfield(res, 'M_pivot_chassis')
            M = res.M_pivot_chassis;
            B = res.bellcrank_pivot;
            
            % Plot Upright to M-pivot link
            plot3([geom.lower_arm_upright(1) M(1)], ...
                  [geom.lower_arm_upright(2) M(2)], ...
                  [geom.lower_arm_upright(3) M(3)], ...
                  '-', 'Color', upright_color, 'LineWidth', 2);
                  
            % --- Physical Bellcrank Mapping ---
            % Obtain the true physical rocker geometry (Pivot, Pushrod Pin, Damper Pin)
            pushrod_vec_nominal = B - M; 
            phys = generate_physical_bellcrank(B, pushrod_vec_nominal, geom);
            
            P_pin = phys.pushrod_pin;
            F_piv = phys.frame_pivot;
            D_pin = phys.damper_pin;
            D_mnt = phys.damper_chassis_mount;
              
            % Plot Pushrod (Upright M-pivot to Bellcrank Pushrod Pin)
            p5 = plot3([M(1) P_pin(1)], ...
                  [M(2) P_pin(2)], ...
                  [M(3) P_pin(3)], ...
                  '-^', 'Color', pushrod_color, 'LineWidth', 2.5, 'MarkerFaceColor', pushrod_color, 'MarkerSize', 6);
            if isempty(h_push), h_push = p5(1); end
              
            % Plot Physical Bellcrank Triangle (Frame Pivot, Pushrod Pin, Damper Pin)
            p6 = fill3([F_piv(1) P_pin(1) D_pin(1) F_piv(1)], ...
                       [F_piv(2) P_pin(2) D_pin(2) F_piv(2)], ...
                       [F_piv(3) P_pin(3) D_pin(3) F_piv(3)], ...
                       bellcrank_color, 'FaceAlpha', 0.8, 'EdgeColor', 'k');
            if isempty(h_bell), h_bell = p6(1); end
            
            % Plot Damper (Bellcrank Damper Pin to Chassis Frame Mount)
            p_damp = plot3([D_pin(1) D_mnt(1)], ...
                           [D_pin(2) D_mnt(2)], ...
                           [D_pin(3) D_mnt(3)], ...
                           '-o', 'Color', [0.8 0.6 0.1], 'LineWidth', 3, 'MarkerFaceColor', [0.8 0.6 0.1], 'MarkerSize', 5);
            if isempty(h_damper), h_damper = p_damp(1); end
            
            % Add text labels
            text(F_piv(1), F_piv(2), F_piv(3)-0.05, sprintf('%s Rocker', wn), 'FontSize', 9, 'HorizontalAlignment', 'center');
            text(M(1), M(2), M(3)+0.02, sprintf('%s POU', wn), 'FontSize', 9, 'HorizontalAlignment', 'center');
        end
        
        % Plot Wheel Center as reference
        if isfield(geom, 'tire_radius'), R_draw = geom.tire_radius; else, R_draw = 0.228; end
        wc_z = -R_draw;
        plot3(geom.x_wheel, geom.y_wheel, wc_z, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        
        % Draw a mock tire (circle in XZ plane at wheel Y, centered at wheel center Z)
        theta = linspace(0, 2*pi, 40);
        X_tire = geom.x_wheel + R_draw * cos(theta);
        Y_tire = geom.y_wheel * ones(size(theta));
        Z_tire = wc_z + R_draw * sin(theta);  % centered at wheel hub, not ground
        p7 = plot3(X_tire, Y_tire, Z_tire, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        if isempty(h_tire), h_tire = p7(1); end
    end
    
    % Draw chassis centerline reference
    plot3([-1.2, 0.5], [0, 0], [0, 0], 'k-.', 'LineWidth', 1);
    
    % Ground plane representation
    [XG, YG] = meshgrid(linspace(-1.5, 1.0, 5), linspace(-0.8, 0.8, 5));
    ZG = 0 * ones(size(XG)); % Ground is Z=0 in this SAE coordinate system.
    surf(XG, YG, ZG, 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    % Assemble Legend
    handles = [h_a_arm, h_kp, h_upright, h_tie, h_push, h_bell, h_damper, h_tire];
    labels = {'A-Arms', 'Kingpin Axis', 'Upright Body', 'Tie Rod', 'Pushrod', 'Bellcrank/Rocker', 'Damper / Spring', 'Tire Outline'};
    legend(handles, labels, 'Location', 'northeast', 'FontSize', 11);
    
    rotate3d on;
    fprintf('3D Geometry Plot Generated.\n');
end
