function [obj_val, results] = evaluateTopology(x, nodes, candidate_elements, ...
                                               is_mandatory, pipe, material, ...
                                               bcs, loads, config)
% EVALUATETOPOLOGY Evaluate a binary topology design variable
%
% Inputs:
%   x                 - Binary vector: per-group if config.symmetry_group exists,
%                       otherwise per-element (n_candidates × 1)
%   nodes             - Nx3 node coordinates
%   candidate_elements- Mx2 candidate element connectivity
%   is_mandatory      - Mx1 logical, true for mandatory elements
%   pipe              - Fixed pipe struct (D_outer, t, shape, etc.)
%   material          - Material struct (E, G, rho)
%   bcs               - Boundary conditions
%   loads             - Applied loads
%   config            - Configuration (exclusion_zones, weights, symmetry_group, etc.)
%
% Outputs:
%   obj_val - Objective function value (lower is better)
%   results - Detailed results struct

    %% 1. Expand symmetry groups to full element vector if needed
    if isfield(config, 'symmetry_group')
        % x has one variable per symmetry group -> expand to full element vector
        x_full = x(config.symmetry_group);
    else
        x_full = x;
    end

    %% 2. Enforce mandatory elements
    x_binary = (x_full > 0.5);  % Convert to binary
    x_binary(is_mandatory) = 1;  % Force mandatory elements to be included

    %% 3. Select active elements
    selected_elements = candidate_elements(x_binary, :);
    n_selected = size(selected_elements, 1);

    % Check minimum element count
    if n_selected < 8
        obj_val = 1e9;
        results = struct('mass', 0, 'K_torsion', 0, 'success', false, ...
                        'exclusion_penalty', 0, 'connectivity_penalty', 0);
        return;
    end

    %% 4. Connectivity check (before FEM to save computation)
    n_structural = 8;  % Structural nodes {1..8}
    if isfield(config, 'n_structural_nodes')
        n_structural = config.n_structural_nodes;
    end

    connectivity_penalty = computeConnectivityPenalty(selected_elements, n_structural);

    % If severely disconnected, skip FEM entirely
    if connectivity_penalty > 50
        obj_val = 1e6 + connectivity_penalty;
        results = struct('mass', 0, 'K_torsion', 0, 'success', false, ...
                        'exclusion_penalty', 0, 'connectivity_penalty', connectivity_penalty, ...
                        'n_pipes', n_selected);
        return;
    end

    %% 5. Build FEM element array
    elements_fem = zeros(n_selected, 9);

    for i = 1:n_selected
        elements_fem(i, 1) = selected_elements(i, 1);  % node_i
        elements_fem(i, 2) = selected_elements(i, 2);  % node_j
        elements_fem(i, 3) = pipe.D_outer_mm;
        elements_fem(i, 4) = pipe.t_mm;
        elements_fem(i, 5) = material.E_GPa;
        elements_fem(i, 6) = material.G_GPa;
        elements_fem(i, 7) = material.rho_kg_m3;

        if strcmp(pipe.shape, 'round')
            elements_fem(i, 8) = 1;
        else
            elements_fem(i, 8) = 2;
        end

        elements_fem(i, 9) = 0.75;  % Joint factor
    end

    %% 6. Run FEM solver
    try
        fem_result = beam_fem_solver(nodes, elements_fem, bcs, loads, struct());

        if ~fem_result.success
            obj_val = 1e9;
            results = fem_result;
            results.connectivity_penalty = connectivity_penalty;
            return;
        end
    catch
        obj_val = 1e9;
        results = struct('mass', 0, 'K_torsion', 0, 'success', false, ...
                        'exclusion_penalty', 0, 'connectivity_penalty', connectivity_penalty);
        return;
    end

    %% 7. Check exclusion zones
    excl_penalties = checkExclusionZones(nodes, selected_elements, config);
    total_excl_penalty = sum(excl_penalties);

    %% 8. Calculate objective
    % Configuration defaults
    W_mass = 1.0;
    W_stiff = 0.3;
    mass_ref = 15.0;
    K_min = 400;
    K_target = 1000;
    gamma_stiff = 20;

    if isfield(config, 'W_mass'), W_mass = config.W_mass; end
    if isfield(config, 'W_stiff'), W_stiff = config.W_stiff; end
    if isfield(config, 'mass_ref'), mass_ref = config.mass_ref; end
    if isfield(config, 'K_min'), K_min = config.K_min; end
    if isfield(config, 'K_target'), K_target = config.K_target; end
    if isfield(config, 'gamma_stiff'), gamma_stiff = config.gamma_stiff; end

    mass = fem_result.mass;
    K = fem_result.K_torsion;

    % Mass term (normalized)
    f_mass = mass / mass_ref;

    % Stiffness penalty/reward
    if K < K_min
        % Severe penalty for insufficient stiffness
        f_stiff = -gamma_stiff * ((K_min - K) / K_min)^2;
    elseif K < K_target
        % Small reward for meeting minimum
        f_stiff = (K - K_min) / (K_target - K_min);
    else
        % Full reward for exceeding target
        f_stiff = 1.0;
    end

    % Complexity penalty (target-range approach)
    n_candidates = size(candidate_elements, 1);
    ratio = n_selected / n_candidates;
    if ratio < 0.3
        % Too sparse — penalize to prevent structurally unsound designs
        complexity_penalty = 2.0 * (0.3 - ratio)^2;
    elseif ratio <= 0.7
        % Free zone — no complexity penalty
        complexity_penalty = 0.0;
    else
        % Slightly penalize very dense structures (mass term already covers this)
        complexity_penalty = 0.1 * (ratio - 0.7);
    end

    % Edge length penalty — prefer shorter pipes for cowl fitment
    W_edge_length = 0.02;  % default weight
    edge_length_threshold = 1000;  % mm, penalty starts above this
    if isfield(config, 'W_edge_length'), W_edge_length = config.W_edge_length; end
    if isfield(config, 'edge_length_threshold'), edge_length_threshold = config.edge_length_threshold; end

    edge_length_penalty = 0;
    for ei = 1:n_selected
        n1 = nodes(selected_elements(ei, 1), :);
        n2 = nodes(selected_elements(ei, 2), :);
        L_mm = norm(n2 - n1) * 1000;  % m to mm
        if L_mm > edge_length_threshold
            excess = (L_mm - edge_length_threshold) / edge_length_threshold;
            edge_length_penalty = edge_length_penalty + W_edge_length * excess^2;
        end
    end

    % Total objective (minimize)
    obj_val = W_mass * f_mass - W_stiff * f_stiff + ...
              total_excl_penalty / 1e6 + complexity_penalty + ...
              connectivity_penalty + edge_length_penalty;


    %% 9. Return detailed results
    results = fem_result;
    results.n_pipes = n_selected;
    results.f_mass = f_mass;
    results.f_stiff = f_stiff;
    results.exclusion_penalty = total_excl_penalty;
    results.complexity_penalty = complexity_penalty;
    results.connectivity_penalty = connectivity_penalty;
    results.edge_length_penalty = edge_length_penalty;
    results.obj_val = obj_val;
end


%% ========== Local Functions ==========

function penalty = computeConnectivityPenalty(elements, n_structural)
% COMPUTECONNECTIVITYPENALTY Check structural connectivity via BFS and max-flow
%
% Returns total penalty:
%   +100 per disconnected structural node
%   +5.0 per missing edge-disjoint path (target: 4 paths front->rear)

    penalty = 0.0;

    % --- BFS connectivity check ---
    % Build adjacency list for all nodes appearing in elements
    max_node = max(elements(:));
    adj = cell(max_node, 1);
    for i = 1:size(elements, 1)
        u = elements(i, 1);
        v = elements(i, 2);
        adj{u}(end+1) = v;
        adj{v}(end+1) = u;
    end

    % BFS from node 1
    visited = false(max_node, 1);
    queue = 1;
    visited(1) = true;
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        neighbors = adj{current};
        for k = 1:length(neighbors)
            nb = neighbors(k);
            if ~visited(nb)
                visited(nb) = true;
                queue(end+1) = nb; %#ok<AGROW>
            end
        end
    end

    % Count disconnected structural nodes
    for i = 1:n_structural
        if ~visited(i)
            penalty = penalty + 100;
        end
    end

    % --- Edge-disjoint path count (front {1,2,3,4} -> rear {5,6,7,8}) ---
    front_nodes = [1, 2, 3, 4];
    rear_nodes = [5, 6, 7, 8];
    n_paths = countEdgeDisjointPaths(elements, front_nodes, rear_nodes, max_node);

    target_paths = 4;
    if n_paths < target_paths
        penalty = penalty + 5.0 * (target_paths - n_paths);
    end
end


function n_paths = countEdgeDisjointPaths(elements, sources, sinks, max_node)
% COUNTEDGEDISJOINTPATHS Count edge-disjoint paths using iterative max-flow
%
% Uses Ford-Fulkerson with BFS (Edmonds-Karp) on a unit-capacity network.
% Adds super-source connected to all sources, super-sink connected to all sinks.

    n_edges = size(elements, 1);

    % Build directed edge list with unit capacity (each undirected edge -> 2 directed)
    % Nodes: 1..max_node, super_source = max_node+1, super_sink = max_node+2
    s = max_node + 1;
    t = max_node + 2;
    n_total = max_node + 2;

    % Edge list: [from, to, capacity, flow]
    % Each undirected edge becomes 2 directed edges + 2 reverse edges
    % Plus source/sink edges
    n_src = length(sources);
    n_snk = length(sinks);
    total_edges = 4 * n_edges + 2 * (n_src + n_snk);

    edge_from = zeros(total_edges, 1);
    edge_to = zeros(total_edges, 1);
    edge_cap = zeros(total_edges, 1);
    edge_flow = zeros(total_edges, 1);

    % Build adjacency: for each node, list of edge indices
    adj = cell(n_total, 1);
    eidx = 0;

    % Add structural edges (undirected -> 2 pairs of directed edges)
    for i = 1:n_edges
        u = elements(i, 1);
        v = elements(i, 2);
        % Forward edge u->v and its reverse
        eidx = eidx + 1; fwd1 = eidx;
        edge_from(eidx) = u; edge_to(eidx) = v; edge_cap(eidx) = 1;
        eidx = eidx + 1; rev1 = eidx;
        edge_from(eidx) = v; edge_to(eidx) = u; edge_cap(eidx) = 0;

        % Backward edge v->u and its reverse
        eidx = eidx + 1; fwd2 = eidx;
        edge_from(eidx) = v; edge_to(eidx) = u; edge_cap(eidx) = 1;
        eidx = eidx + 1; rev2 = eidx;
        edge_from(eidx) = u; edge_to(eidx) = v; edge_cap(eidx) = 0;

        adj{u}(end+1) = fwd1; adj{v}(end+1) = rev1;
        adj{v}(end+1) = fwd2; adj{u}(end+1) = rev2;
    end

    % Add super-source -> sources (high capacity)
    for i = 1:n_src
        eidx = eidx + 1; fwd = eidx;
        edge_from(eidx) = s; edge_to(eidx) = sources(i); edge_cap(eidx) = n_edges;
        eidx = eidx + 1; rev = eidx;
        edge_from(eidx) = sources(i); edge_to(eidx) = s; edge_cap(eidx) = 0;

        adj{s}(end+1) = fwd; adj{sources(i)}(end+1) = rev;
    end

    % Add sinks -> super-sink (high capacity)
    for i = 1:n_snk
        eidx = eidx + 1; fwd = eidx;
        edge_from(eidx) = sinks(i); edge_to(eidx) = t; edge_cap(eidx) = n_edges;
        eidx = eidx + 1; rev = eidx;
        edge_from(eidx) = t; edge_to(eidx) = sinks(i); edge_cap(eidx) = 0;

        adj{sinks(i)}(end+1) = fwd; adj{t}(end+1) = rev;
    end

    % Trim arrays
    edge_from = edge_from(1:eidx);
    edge_to = edge_to(1:eidx);
    edge_cap = edge_cap(1:eidx);
    edge_flow = edge_flow(1:eidx);

    % Build reverse edge index (each pair: i <-> i+1 if i is odd, i <-> i-1 if even)
    % The edges are added in pairs: fwd at odd index, rev at even index
    rev_idx = zeros(eidx, 1);
    for i = 1:2:eidx
        rev_idx(i) = i + 1;
        rev_idx(i+1) = i;
    end

    % Edmonds-Karp (BFS-based Ford-Fulkerson)
    n_paths = 0;
    while true
        % BFS to find augmenting path
        parent_edge = -ones(n_total, 1);
        visited = false(n_total, 1);
        visited(s) = true;
        queue = s;

        found = false;
        while ~isempty(queue) && ~found
            current = queue(1);
            queue(1) = [];

            for k = 1:length(adj{current})
                ei = adj{current}(k);
                next = edge_to(ei);
                residual = edge_cap(ei) - edge_flow(ei);

                if ~visited(next) && residual > 0
                    visited(next) = true;
                    parent_edge(next) = ei;
                    if next == t
                        found = true;
                        break;
                    end
                    queue(end+1) = next; %#ok<AGROW>
                end
            end
        end

        if ~found
            break;
        end

        % Trace path and find bottleneck
        bottleneck = Inf;
        node = t;
        while node ~= s
            ei = parent_edge(node);
            bottleneck = min(bottleneck, edge_cap(ei) - edge_flow(ei));
            node = edge_from(ei);
        end

        % Update flow
        node = t;
        while node ~= s
            ei = parent_edge(node);
            edge_flow(ei) = edge_flow(ei) + bottleneck;
            edge_flow(rev_idx(ei)) = edge_flow(rev_idx(ei)) - bottleneck;
            node = edge_from(ei);
        end

        n_paths = n_paths + bottleneck;
    end
end
