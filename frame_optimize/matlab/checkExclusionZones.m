function [penalties, collision_names] = checkExclusionZones(nodes, elements, config, group_indices, group_names)
% CHECKEXCLUSIONZONES Checks if any element intersects with exclusion zones.
%
% Inputs:
%   nodes          - Nx3 matrix of node coordinates [x, y, z]
%   elements       - Mx2 matrix of element connectivities [node1, node2]
%   config         - Struct containing exclusion_zones definition
%   group_indices  - (Optional) Mx1 vector of element group IDs
%   group_names    - (Optional) Cell array mapping group ID -> group name.
%                    Exemptions are matched by name so V1 and V2 group order
%                    cannot silently change their meaning.
%
% Outputs:
%   penalties       - Array of penalties for each element (length M)
%   collision_names - Cell array of strings with the name of the zone hit
%                     (Empty string if no collision)

    if nargin < 4
        group_indices = [];
    end
    if nargin < 5
        group_names = {};
    end

    num_elements = size(elements, 1);
    penalties = zeros(num_elements, 1);
    collision_names = cell(num_elements, 1);
    for i=1:num_elements, collision_names{i} = ''; end
    
    if ~isfield(config, 'exclusion_zones') || isempty(config.exclusion_zones)
        return;
    end
    
    zones = config.exclusion_zones;
    penalty_factor = 1e6; % large penalty per mm of intersection
    
    % Build exemption map: {zone_name} -> exempt group names.
    % The physical mandatory structures define these zones, so they cannot
    % be penalized for intersecting their own boundary.
    exemptions = struct();
    exemptions.front_hoop_sweep = {'front_hoop', 'side_impact'};
    exemptions.driver_cockpit = {'main_hoop', 'mh_bracing', 'mh_brace_support'};

    % The driver cockpit starts behind the front hoop. The static zone in the
    % vehicle artifact may start farther forward; clip it to the actual hoop.
    if ~isempty(group_indices)
        if ~isempty(group_names)
            group_name_by_element = group_names(group_indices);
            front_idx = find(strcmp(group_name_by_element, 'front_hoop'));
        else
            front_idx = find(group_indices == 1);
        end
        if ~isempty(front_idx)
            front_nodes = unique(elements(front_idx, :));
            front_max_x = max(nodes(front_nodes, 1));
            zones = clipDriverCockpitZone(zones, front_max_x + 1e-6);
        end
    end

    % Pre-process zones to ensure min < max
    for j = 1:length(zones)
        if iscell(zones)
            z = zones{j};
        else
            z = zones(j);
        end
        if strcmp(z.type, 'AABB')
            actual_min = min(z.min_bounds, z.max_bounds);
            actual_max = max(z.min_bounds, z.max_bounds);
            if iscell(zones)
                zones{j}.min_bounds = actual_min;
                zones{j}.max_bounds = actual_max;
            else
                zones(j).min_bounds = actual_min;
                zones(j).max_bounds = actual_max;
            end
        end
    end

    % Handle both cell array and struct array
    is_cell = iscell(zones);

    for i = 1:num_elements
        n1 = nodes(elements(i, 1), :);
        n2 = nodes(elements(i, 2), :);

        L_elem = norm(n2 - n1);
        if L_elem == 0
            continue;
        end

        % Direction vector of the segment
        dir = (n2 - n1) / L_elem;

        total_intersection_len = 0;

        % Check each exclusion zone
        for j = 1:length(zones)
            if is_cell
                zone = zones{j};
            else
                zone = zones(j);
            end
            
            % --- Group-based exemption check ---
            % If we know this element's group name and it's exempt, skip.
            if ~isempty(group_indices) && ~isempty(group_names) ...
                    && group_indices(i) >= 1 && group_indices(i) <= numel(group_names)
                zone_name = getZoneName(zone, j);
                field_name = strrep(strrep(zone_name, '.', '_'), '-', '_');
                if isfield(exemptions, field_name) ...
                        && any(strcmp(exemptions.(field_name), group_names{group_indices(i)}))
                    continue;
                end
            end
            intersect_len = 0;
            
            if strcmp(zone.type, 'AABB')
                % Axis-Aligned Bounding Box (Slab method)
                tmin = 0;
                tmax = L_elem;
                
                for axis = 1:3
                    if abs(dir(axis)) < 1e-6
                        % Ray is parallel to slab
                        if n1(axis) < zone.min_bounds(axis) || n1(axis) > zone.max_bounds(axis)
                            tmin = inf; % No intersection
                            break;
                        end
                    else
                        t1 = (zone.min_bounds(axis) - n1(axis)) / dir(axis);
                        t2 = (zone.max_bounds(axis) - n1(axis)) / dir(axis);
                        
                        if t1 > t2
                            temp = t1; t1 = t2; t2 = temp;
                        end
                        
                        tmin = max(tmin, t1);
                        tmax = min(tmax, t2);
                        
                        if tmin > tmax
                            break; % No intersection
                        end
                    end
                end
                
                if tmin <= tmax
                    intersect_len = tmax - tmin;
                end
                
            elseif strcmp(zone.type, 'Cylinder')
                % Z-aligned Cylinder
                cx = zone.center(1);
                cy = zone.center(2);
                r = zone.radius;
                
                % Projection onto XY plane
                p1_xy = [n1(1) - cx, n1(2) - cy];
                dir_xy = [dir(1), dir(2)];
                
                a = dot(dir_xy, dir_xy);
                b = 2 * dot(p1_xy, dir_xy);
                c = dot(p1_xy, p1_xy) - r^2;
                
                if a > 1e-6
                    discriminant = b^2 - 4*a*c;
                    if discriminant >= 0
                        t1 = (-b - sqrt(discriminant)) / (2*a);
                        t2 = (-b + sqrt(discriminant)) / (2*a);
                        
                        % Clamp to [0, L_elem]
                        t1_clamp = max(0, min(L_elem, t1));
                        t2_clamp = max(0, min(L_elem, t2));
                        
                        if t1_clamp < t2_clamp
                            % Check Z bounds if defined
                            if isfield(zone, 'z_min') && isfield(zone, 'z_max')
                                z1 = n1(3) + t1_clamp * dir(3);
                                z2 = n1(3) + t2_clamp * dir(3);
                                z_min_act = min(z1, z2);
                                z_max_act = max(z1, z2);
                                
                                if z_max_act >= zone.z_min && z_min_act <= zone.z_max
                                    % Overlap in Z (simplified)
                                    intersect_len = t2_clamp - t1_clamp;
                                end
                            else
                                intersect_len = t2_clamp - t1_clamp;
                            end
                        end
                    end
                end
            else
                error('checkExclusionZones:UnsupportedZoneType', ...
                    'Unsupported exclusion zone type: %s. Supported: AABB, Cylinder.', ...
                    char(string(zone.type)));
            end
            
            total_intersection_len = total_intersection_len + intersect_len;
            if intersect_len > 0
                % Store the name of the first colliding zone for this element
                if isempty(collision_names{i})
                    collision_names{i} = getZoneName(zone, j);
                end
            end
        end
        
        penalties(i) = total_intersection_len * penalty_factor;
    end
end

function name = getZoneName(zone, index)
    if isfield(zone, 'name') && ~isempty(zone.name)
        name = char(zone.name);
    else
        name = sprintf('Zone_%d', index);
    end
end

function zones = clipDriverCockpitZone(zones, min_x)
    if iscell(zones)
        for j = 1:numel(zones)
            zones{j} = clipOneZone(zones{j}, j, min_x);
        end
    else
        for j = 1:numel(zones)
            zones(j) = clipOneZone(zones(j), j, min_x);
        end
    end
end

function zone = clipOneZone(zone, index, min_x)
    if strcmp(getZoneName(zone, index), 'driver_cockpit') ...
            && isfield(zone, 'type') && strcmp(zone.type, 'AABB') ...
            && isfield(zone, 'min_bounds') && zone.min_bounds(1) < min_x
        zone.min_bounds(1) = min_x;
    end
end
