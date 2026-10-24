function elements_geom = generate_adaptive_topology_v2(nodes, meta, config, distance_thresholds)
% GENERATE_ADAPTIVE_TOPOLOGY_V2 Create regulation-named V2 frame connectivity.
%
% The rear free region is only populated aft of the mandatory main-hoop-brace
% trapezoid. Suspension mounts remain isolated to their bracket nodes.

    if nargin < 4 || isempty(distance_thresholds)
        distance_thresholds = [0.60, 0.65, 0.60];
    end
    if nargin < 3 || isempty(config)
        config = struct();
    end

    group_names = meta.group_names;
    n_nodes = size(nodes, 1);

    D = 25.0; t = 2.0; E = 210; G = 80; rho = 7850; shape = 1; jf = 0.85;
    elements_raw = [];
    adj_matrix = false(n_nodes, n_nodes);
    node_degrees = zeros(n_nodes, 1);

    all_susp_nodes = [meta.susp_idx(:)', meta.rear_susp_idx(:)'];

    % Front hoop / main hoop / side impact required structure.
    addEdges([1 2; 1 3; 2 4; 3 4], 'front_hoop');
    addEdges([ ...
        9 10; 9 11; 11 13; 13 16; 16 15; ...
        15 17; 17 14; 14 12; 12 10; 16 17], 'main_hoop');
    addEdges([13 14], 'shoulder_bar');
    addEdges([ ...
        1 5; 2 6; 5 9; 6 10; ...
        3 7; 4 8; 7 11; 8 12; ...
        5 7; 6 8; 1 7; 2 8; 5 11; 6 12], 'side_impact');

    % Front bulkhead and support tubes.
    f = meta.fbh_idx;
    if numel(f) >= 4
        addEdges([f(1) f(2); f(1) f(3); f(2) f(4); f(3) f(4)], 'front_bulkhead');
        addEdges([f(1) 1; f(2) 2; f(3) 3; f(4) 4; f(3) 1; f(4) 2], 'fbh_support');
    end

    % Mandatory main hoop brace trapezoid and rear support structure.
    addEdges([16 18; 17 19], 'mh_bracing');
    addEdges([18 19; 20 21; 18 20; 19 21; 13 18; 14 19; 9 20; 10 21], 'mh_brace_support');

    rear_edges = [ ...
        18 22; 19 23; 20 24; 21 25; ...
        22 23; 24 25; 22 24; 23 25];
    if strcmpi(getConfigDefault(config, 'rear_box_diagonal_mode', 'minimal'), 'full')
        rear_edges = [rear_edges; 18 24; 19 25; 20 22; 21 23];
    end
    addEdges(rear_edges, 'rear_box');

    if ~isempty(meta.rear_mid_idx)
        r = meta.rear_mid_idx;
        addEdges([18 r(1); 19 r(2); 20 r(3); 21 r(4); r(1) 22; r(2) 23; r(3) 24; r(4) 25], 'secondary_brace');
        addEdges([r(1) r(2); r(3) r(4); r(1) r(3); r(2) r(4)], 'secondary_brace');
    end

    addBracketLinks(meta.susp_idx, meta.inner_bracket_idx, [f(1), f(2), f(3), f(4), 1, 2, 3, 4], 'front_susp_mount');
    if ~isempty(meta.rear_susp_idx) && ~isempty(meta.rear_inner_bracket_idx)
        addBracketLinks(meta.rear_susp_idx, meta.rear_inner_bracket_idx, [20, 21, 18, 19, 24, 25, 22, 23], 'component_protection');
    end

    if ~isempty(meta.diff_mount_idx)
        addDiffMountLinks(meta.diff_mount_idx, 'diff_mount');
    end

    % Rear optional freedom: only behind the brace trapezoid.
    R_rear = distance_thresholds(min(3, numel(distance_thresholds)));
    max_pipe_len = getConfigDefault(config, 'max_rear_free_pipe_length_m', 1.20);
    rear_candidates = find(nodes(:, 1) >= meta.X_brace_end - 1e-6);
    rear_candidates = setdiff(rear_candidates(:)', all_susp_nodes);

    if getConfigDefault(config, 'enable_rear_optional_edges', false)
        max_optional_degree = getConfigDefault(config, 'rear_optional_max_node_degree', 4);
        for a = 1:numel(rear_candidates)
            i = rear_candidates(a);
            for b = a + 1:numel(rear_candidates)
                j = rear_candidates(b);
                if adj_matrix(i, j) || node_degrees(i) >= max_optional_degree || node_degrees(j) >= max_optional_degree
                    continue;
                end
                dist = norm(nodes(i, :) - nodes(j, :));
                if dist < 0.150 || dist > min(R_rear, max_pipe_len)
                    continue;
                end
                if crossesExclusion(nodes(i, :), nodes(j, :), config)
                    continue;
                end
                addEdge(i, j, 'secondary_brace');
            end
        end
    end

    rescueRearNodes(rear_candidates, all_susp_nodes, max_pipe_len, config);

    elements_geom = struct();
    if isempty(elements_raw)
        elements_geom.base_elements = [1, 2, D, t, E, G, rho, shape, jf];
        elements_geom.connectivity = [1, 2];
        elements_geom.group_indices = groupIndex('secondary_brace');
    else
        elements_geom.base_elements = elements_raw(:, 1:9);
        elements_geom.connectivity = elements_raw(:, 1:2);
        elements_geom.group_indices = elements_raw(:, 10);
    end
    elements_geom.group_names = group_names;

    function addEdges(edge_pairs, group_name)
        for edge_row = 1:size(edge_pairs, 1)
            addEdge(edge_pairs(edge_row, 1), edge_pairs(edge_row, 2), group_name);
        end
    end

    function addEdge(i, j, group_name)
        if i < 1 || j < 1 || i > n_nodes || j > n_nodes || i == j
            return;
        end
        if adj_matrix(i, j)
            return;
        end
        adj_matrix(i, j) = true;
        adj_matrix(j, i) = true;
        node_degrees(i) = node_degrees(i) + 1;
        node_degrees(j) = node_degrees(j) + 1;
        elements_raw = [elements_raw; i, j, D, t, E, G, rho, shape, jf, groupIndex(group_name)];
    end

    function idx = groupIndex(group_name)
        idx = find(strcmp(group_names, group_name), 1);
        if isempty(idx)
            error('generate_adaptive_topology_v2:unknownGroup', ...
                'Unknown V2 frame group: %s', group_name);
        end
    end

    function addBracketLinks(mount_idx, bracket_idx, targets, group_name)
        n = min([numel(mount_idx), numel(bracket_idx), numel(targets)]);
        for k = 1:n
            addEdge(mount_idx(k), bracket_idx(k), group_name);
            addEdge(bracket_idx(k), targets(k), group_name);
        end
    end

    function addDiffMountLinks(diff_idx, group_name)
        anchor_set = unique([18:25, meta.rear_mid_idx(:)']);
        for k = 1:numel(diff_idx)
            i = diff_idx(k);
            same_side = anchor_set(signOrZero(nodes(anchor_set, 2)) == signOrZero(nodes(i, 2)));
            if isempty(same_side)
                same_side = anchor_set;
            end
            [~, order] = sort(vecnorm(nodes(same_side, :) - nodes(i, :), 2, 2));
            for c = 1:min(2, numel(order))
                addEdge(i, same_side(order(c)), group_name);
            end
        end
    end

    function out = signOrZero(values)
        out = sign(values);
        out(abs(values) < 1e-9) = 0;
    end

    function blocked = crossesExclusion(p1, p2, cfg)
        blocked = false;
        if ~isfield(cfg, 'exclusion_zones') || isempty(cfg.exclusion_zones)
            return;
        end
        penalties = checkExclusionZones([p1; p2], [1, 2], cfg);
        blocked = penalties(1) > 0;
    end

    function rescueRearNodes(candidates, susp_nodes, max_len, cfg)
        for ii = candidates(:)'
            if ismember(ii, susp_nodes)
                continue;
            end
            while node_degrees(ii) < 2
                best_j = -1;
                best_d = inf;
                for jj = candidates(:)'
                    if ii == jj || adj_matrix(ii, jj) || node_degrees(jj) >= 5 || ismember(jj, susp_nodes)
                        continue;
                    end
                    d = norm(nodes(ii, :) - nodes(jj, :));
                    if d < 0.150 || d > max_len
                        continue;
                    end
                    if d < best_d && ~crossesExclusion(nodes(ii, :), nodes(jj, :), cfg)
                        best_d = d;
                        best_j = jj;
                    end
                end
                if best_j < 0
                    break;
                end
                addEdge(ii, best_j, 'secondary_brace');
            end
        end
    end
end

function value = getConfigDefault(config, field_name, default_value)
    value = default_value;
    if isstruct(config) && isfield(config, field_name) && ~isempty(config.(field_name))
        value = config.(field_name);
    end
end
