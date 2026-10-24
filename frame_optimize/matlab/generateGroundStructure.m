function elements = generateGroundStructure(nodes, config)
% GENERATEGROUNDSTRUCTURE Generates all candidate pipe connections for a set of nodes.
% 
% This function creates a "Ground Structure" by connecting every node to every
% other node, and then filtering out invalid connections based on maximum length
% and intersection with exclusion zones. This is the search space for the
% topology optimization.
%
% Inputs:
%   nodes  - Nx3 matrix of node coordinates [x, y, z]
%   config - Configuration struct:
%            .max_length_mm    - Maximum allowed length for a single pipe
%            .exclusion_zones - Struct array of exclusion zones (optional)
%
% Outputs:
%   elements - Mx2 matrix of element connectivities [node1, node2]

    num_nodes = size(nodes, 1);
    num_possible_elements = num_nodes * (num_nodes - 1) / 2;
    
    % Preallocate (maximum possible size)
    all_elements = zeros(num_possible_elements, 2);
    
    % Configuration defaults
    max_length_mm = inf;
    if isfield(config, 'max_length_mm')
        max_length_mm = config.max_length_mm;
    end
    
    % Generate all unique node pairs
    elem_idx = 1;
    for i = 1:(num_nodes - 1)
        for j = (i + 1):num_nodes
            % Calculate length
            L = norm(nodes(i, :) - nodes(j, :));
            
            % 1. Filter by length
            if L <= max_length_mm && L > 1e-3 % Ignore 0-length
                all_elements(elem_idx, :) = [i, j];
                elem_idx = elem_idx + 1;
            end
        end
    end
    
    % Trim to actual size
    elements = all_elements(1:(elem_idx - 1), :);
    
    % 2. Filter by exclusion zones (if provided)
    if isfield(config, 'exclusion_zones') && ~isempty(config.exclusion_zones)
        % checkExclusionZones returns penalty proportionally to intersection
        penalties = checkExclusionZones(nodes, elements, config);
        
        % Keep only elements that do not intersect exclusion zones
        valid_idx = (penalties == 0);
        elements = elements(valid_idx, :);
    end
    
    fprintf('Ground Structure generated: %d candidate pipes (from %d nodes).\n', ...
            size(elements, 1), num_nodes);
end
