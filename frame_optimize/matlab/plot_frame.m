function plot_frame(nodes, elements_geom, pipe_assignments, catalog, show_nodes)
% PLOT_FRAME Renders the frame topology using actual 3D cylinders sized to the pipe dimensions.
%
% Inputs:
%   nodes            - Nx3 matrix of node coordinates [x, y, z] in meters
%   elements_geom    - Struct containing:
%                      .connectivity (Mx2) node indices
%                      .group_indices (Mx1) group indices per element
%   pipe_assignments - Kx1 vector of pipe catalog indices chosen by optimizer (where K = max group index)
%   catalog          - Struct array of the pipe catalog
%   show_nodes       - Boolean to determine if nodes should be drawn as spheres (default false)

    if nargin < 5
        show_nodes = false;
    end
    
    num_elems = size(elements_geom.connectivity, 1);
    
    % Color mapping for size classes (FSAE minimums)
    % A (Red), B (Orange), C (Green), D (Blue), Other (Grey)
    color_map = containers.Map({'A', 'B', 'C', 'D', 'None'}, ...
                               {[0.8, 0.2, 0.2], [0.9, 0.5, 0.1], [0.2, 0.7, 0.3], [0.2, 0.4, 0.8], [0.5, 0.5, 0.5]});
    
    hold on;
    grid on;
    axis equal;
    
    % Use a cylinder patch component
    [Z_base, Y_base, X_base] = cylinder(1, 12); % 12 facets is enough for display
    
    for i = 1:num_elems
        n1_idx = elements_geom.connectivity(i, 1);
        n2_idx = elements_geom.connectivity(i, 2);
        
        p1 = nodes(n1_idx, :);
        p2 = nodes(n2_idx, :);
        
        group_id = elements_geom.group_indices(i);
        
        if group_id == 0
            % Fixed element: Use a default pipe index (e.g. 1) or a specific property
            % if predefined in elements_geom.
            if isfield(elements_geom, 'base_elements')
                pipe = struct('D_outer_mm', elements_geom.base_elements(i, 3), 't_mm', elements_geom.base_elements(i, 4), 'shape', 'round', 'size_class', 'None');
            else
                pipe = catalog(1);
            end
        else
            pipe_idx = round(pipe_assignments(group_id));
            pipe_idx = max(1, min(length(catalog), pipe_idx)); % Bounds safety
            pipe = catalog(pipe_idx);
        end
        
        % Vector pointing from p1 to p2
        v = p2 - p1;
        L = norm(v);
        
        % Avoid drawing zero-length elements
        if L < 1e-4
            continue;
        end
        
        % Determine pipe radius in meters (from catalog given in mm)
        R_m = (pipe.D_outer_mm / 2) / 1000;
        
        % Calculate rotation matrix to align cylinder 'Z' axis with vector 'v'
        z_dir = v / L;
        
        % Initial orientation is [0 0 1], we want z_dir
        if abs(z_dir(3) - 1) < 1e-6
            R = eye(3);
        elseif abs(z_dir(3) + 1) < 1e-6
            R = diag([1 -1 -1]);
        else
            cross_prod = cross([0 0 1], z_dir);
            sin_theta = norm(cross_prod);
            v_skew = [0, -cross_prod(3), cross_prod(2);
                      cross_prod(3), 0, -cross_prod(1);
                      -cross_prod(2), cross_prod(1), 0];
            R = eye(3) + v_skew + (v_skew^2) * ((1 - z_dir(3)) / (sin_theta^2));
        end
        
        % Transform cylinder coordinates
        X = zeros(size(X_base));
        Y = zeros(size(Y_base));
        Z = zeros(size(Z_base));
        
        for row = 1:2
            for col = 1:size(X_base, 2)
                % Scale base cylinder
                pt = [X_base(row,col)*R_m, Y_base(row,col)*R_m, (row-1)*L];
                % Rotate and translate
                pt_rot = (R * pt')';
                pt_trans = pt_rot + p1;
                
                X(row,col) = pt_trans(1);
                Y(row,col) = pt_trans(2);
                Z(row,col) = pt_trans(3);
            end
        end
        
        % Get color classification
        if ~isfield(pipe, 'size_class')
            color = color_map('None');
        else
            if isKey(color_map, pipe.size_class)
                color = color_map(pipe.size_class);
            else
                color = color_map('None');
            end
        end
        
        % Format: Grey for steel, lighter colour for Aluminum?
        % Optional aesthetics (Keeping solid colors mapping to rules for now).
        
        surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none', ...
             'AmbientStrength', 0.6, 'DiffuseStrength', 0.8, 'SpecularStrength', 0.2);
    end
    
    % Draw nodes explicitly if requested
    if show_nodes
        scatter3(nodes(:,1), nodes(:,2), nodes(:,3), 20, 'k', 'filled');
    end
    
    % Light setup for metallic visualization
    camlight('headlight');
    lighting gouraud;
    material shiny;
    
    xlabel('X [m]', 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontWeight', 'bold');
    
    view(3);
    set(gca, 'Color', [0.95, 0.95, 0.95]); % Soft background
end
