function report = experiment_frame_v2_mass_cap_stiffness(varargin)
% EXPERIMENT_FRAME_V2_MASS_CAP_STIFFNESS Evaluate stiffness under a mass cap.
%
% This helper is intentionally experimental. It can keep the saved V2
% topology fixed, or replace it with a regulation-minimum topology built on
% the saved V2 node set. It then assigns the lightest regulation-compliant
% pipe to each group and re-runs the MEX FEM cases.

    p = inputParser;
    addParameter(p, 'ArtifactPath', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MassCapKg', 28, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'TopologyMode', 'artifact', ...
        @(x) any(strcmpi(string(x), ["artifact", "regulation_minimal"])));
    addParameter(p, 'ConnectDiffMounts', true, ...
        @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
    parse(p, varargin{:});
    opts = p.Results;

    script_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(script_dir);
    project_root = fileparts(frame_root);
    addpath(fullfile(frame_root, 'matlab'));
    addpath(fullfile(project_root, 'matlab'));
    addpath(fullfile(frame_root, 'mex'), '-begin');

    artifact_path = char(string(opts.ArtifactPath));
    if isempty(artifact_path)
        artifact_path = fullfile(project_root, 'output', 'runtime', ...
            'frame_v2_ga_300_mex_final', 'frame_v2_ga_15000_vehicle_artifact.json');
    end
    output_dir = char(string(opts.OutputDir));
    if isempty(output_dir)
        output_dir = fullfile(project_root, 'output', 'runtime', 'frame_v2_mass_cap_experiment');
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    artifact = jsondecode(fileread(artifact_path));
    frame = artifact.params.frame;
    data_dir = fullfile(frame_root, 'data');
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
    reg_params = jsondecode(fileread(fullfile(data_dir, 'regulation_params.json')));

    nodes = numericMatrix(frame.geometry.nodes_m);
    connectivity = numericMatrix(frame.geometry.connectivity);
    group_indices = numericVector(frame.geometry.group_indices);
    group_names = toCellstr(frame.geometry.group_names);
    original_node_count = size(nodes, 1);
    original_to_active = (1:original_node_count)';
    topology_mode = lower(char(string(opts.TopologyMode)));
    topology_report = struct( ...
        'mode', topology_mode, ...
        'connect_diff_mounts', logical(opts.ConnectDiffMounts), ...
        'original_node_count', original_node_count, ...
        'active_node_count', original_node_count, ...
        'original_element_count', size(connectivity, 1), ...
        'active_element_count', size(connectivity, 1), ...
        'notes', {{'Using topology saved in artifact.'}});

    if strcmp(topology_mode, 'regulation_minimal')
        [nodes, connectivity, group_indices, topology_report, original_to_active] = ...
            buildRegulationMinimalTopologyFromFrame(frame, nodes, group_names, ...
            logical(opts.ConnectDiffMounts));
    end

    current_assignment = assignmentFromSelectedPipes(frame.selected_pipes, numel(group_names));
    lightest = chooseLightestCompliantPipes(group_names, catalog, materials, reg_params);

    total_mass_kg = getNestedDefault(artifact.params, {'mass', 'm_total'}, 250);
    node_config = inferV2NodeConfig(frame, original_node_count, total_mass_kg);
    if strcmp(topology_mode, 'regulation_minimal')
        node_config = remapNodeConfig(node_config, original_to_active);
    end
    load_cases = generate_load_cases(nodes, total_mass_kg, 9.81, node_config);

    current_eval = evaluateAssignment(nodes, connectivity, group_indices, current_assignment, ...
        group_names, catalog, materials, load_cases);
    lightest_eval = evaluateAssignment(nodes, connectivity, group_indices, lightest.assignment, ...
        group_names, catalog, materials, load_cases);

    report = struct();
    report.schema_version = 'frame_v2_mass_cap_stiffness_experiment/v1';
    report.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    report.artifact_path = artifact_path;
    report.output_dir = output_dir;
    report.mass_cap_kg = opts.MassCapKg;
    report.topology = topology_report;
    report.current_assignment = current_eval;
    report.lightest_compliant_assignment = lightest_eval;
    report.lightest_pipe_table = lightest.table;
    report.cap_feasible_in_fixed_topology = lightest_eval.mass_kg <= opts.MassCapKg;
    report.mass_gap_to_cap_kg = lightest_eval.mass_kg - opts.MassCapKg;

    json_path = fullfile(output_dir, 'frame_v2_mass_cap_stiffness_experiment.json');
    md_path = fullfile(output_dir, 'frame_v2_mass_cap_stiffness_experiment.md');
    writeTextFile(json_path, jsonencode(report, PrettyPrint=true));
    writeTextFile(md_path, renderMarkdown(report));
    report.json_path = json_path;
    report.markdown_path = md_path;

    fprintf('=== Frame V2 mass cap stiffness experiment ===\n');
    fprintf('Artifact: %s\n', artifact_path);
    fprintf('Topology: %s, nodes %d -> %d, elements %d -> %d\n', ...
        report.topology.mode, report.topology.original_node_count, ...
        report.topology.active_node_count, report.topology.original_element_count, ...
        report.topology.active_element_count);
    fprintf('Mass cap: %.2f kg\n', opts.MassCapKg);
    fprintf('Current:  mass %.2f kg, K %.1f Nm/deg\n', ...
        current_eval.mass_kg, current_eval.torsion.K_torsion_Nm_per_deg);
    fprintf('Lightest compliant fixed-topology: mass %.2f kg, K %.1f Nm/deg\n', ...
        lightest_eval.mass_kg, lightest_eval.torsion.K_torsion_Nm_per_deg);
    if report.cap_feasible_in_fixed_topology
        fprintf('Result: cap feasible in this fixed topology.\n');
    else
        fprintf('Result: cap infeasible in this fixed topology by %.2f kg.\n', ...
            report.mass_gap_to_cap_kg);
    end
    fprintf('Saved JSON: %s\n', json_path);
    fprintf('Saved MD: %s\n', md_path);
end

function lightest = chooseLightestCompliantPipes(group_names, catalog, materials, reg_params)
    assignment = zeros(1, numel(group_names));
    rows = struct('group_name', {}, 'required_size', {}, 'catalog_index', {}, ...
        'pipe_id', {}, 'size_class', {}, 'material_id', {}, 'mass_per_m_kg', {});

    for g = 1:numel(group_names)
        group_name = group_names{g};
        required_size = getRequiredSizeClass(group_name, reg_params);
        best_idx = NaN;
        best_mass_per_m = Inf;
        for i = 1:numel(catalog)
            pipe = catalog(i);
            if isfield(pipe, 'available') && ~pipe.available
                continue;
            end
            if ~pipeMeetsRequiredSizeLocal(pipe, required_size, reg_params)
                continue;
            end
            mat_key = matlab.lang.makeValidName(pipe.material_id);
            if ~isfield(materials, mat_key)
                continue;
            end
            mat = materials.(mat_key);
            [is_valid, ~] = checkMaterialRestriction(group_name, pipe, mat, reg_params);
            if ~is_valid
                continue;
            end
            mass_per_m = pipe.A_mm2 * 1e-6 * mat.rho_kg_m3;
            if mass_per_m < best_mass_per_m
                best_idx = i;
                best_mass_per_m = mass_per_m;
            end
        end
        if isnan(best_idx)
            error('experiment_frame_v2_mass_cap_stiffness:noCompliantPipe', ...
                'No compliant pipe found for group %s.', group_name);
        end
        assignment(g) = best_idx;
        pipe = catalog(best_idx);
        rows(g) = struct( ...
            'group_name', group_name, ...
            'required_size', required_size, ...
            'catalog_index', best_idx, ...
            'pipe_id', pipe.id, ...
            'size_class', pipe.size_class, ...
            'material_id', pipe.material_id, ...
            'mass_per_m_kg', best_mass_per_m);
    end

    lightest = struct();
    lightest.assignment = assignment;
    lightest.table = rows;
end

function evaluation = evaluateAssignment(nodes, connectivity, group_indices, assignment, group_names, catalog, materials, load_cases)
    elements_fem = buildElementsForSolver(connectivity, group_indices, assignment, catalog, materials);
    cases = struct('name', {}, 'success', {}, 'K_torsion_Nm_per_deg', {}, ...
        'mass_kg', {}, 'max_translation_m', {}, 'error', {});

    for ci = 1:numel(load_cases)
        try
            result = beam_fem_solver(nodes, elements_fem, load_cases(ci).bcs, load_cases(ci).loads);
            cases(ci).name = load_cases(ci).name;
            cases(ci).success = result.success;
            cases(ci).K_torsion_Nm_per_deg = result.K_torsion;
            cases(ci).mass_kg = result.mass;
            cases(ci).max_translation_m = extractMaxTranslation(result);
            cases(ci).error = '';
        catch ME
            cases(ci).name = load_cases(ci).name;
            cases(ci).success = false;
            cases(ci).K_torsion_Nm_per_deg = NaN;
            cases(ci).mass_kg = NaN;
            cases(ci).max_translation_m = NaN;
            cases(ci).error = ME.message;
        end
    end

    group_breakdown = computeGroupBreakdown(nodes, connectivity, group_indices, ...
        assignment, group_names, catalog, materials);
    mass_kg = cases(1).mass_kg;
    if ~isfinite(mass_kg)
        mass_kg = sum([group_breakdown.mass_kg]);
    end

    evaluation = struct();
    evaluation.mass_kg = mass_kg;
    evaluation.total_length_m = sum([group_breakdown.length_m]);
    evaluation.element_count = size(connectivity, 1);
    evaluation.torsion = cases(1);
    evaluation.load_cases = cases;
    evaluation.group_breakdown = group_breakdown;
end

function elements = buildElementsForSolver(connectivity, group_indices, assignment, catalog, materials)
    elements = zeros(size(connectivity, 1), 9);
    for e = 1:size(connectivity, 1)
        g = group_indices(e);
        pipe = catalog(assignment(g));
        mat = materials.(matlab.lang.makeValidName(pipe.material_id));
        shape_code = 1;
        if strcmpi(pipe.shape, 'square')
            shape_code = 2;
        end
        elements(e, :) = [connectivity(e, 1), connectivity(e, 2), ...
            pipe.D_outer_mm, pipe.t_mm, mat.E_GPa, mat.G_GPa, ...
            mat.rho_kg_m3, shape_code, 0.85];
    end
end

function breakdown = computeGroupBreakdown(nodes, connectivity, group_indices, assignment, group_names, catalog, materials)
    breakdown = struct('group_name', {}, 'pipe_id', {}, 'element_count', {}, ...
        'length_m', {}, 'mass_kg', {});
    for g = 1:numel(group_names)
        pipe = catalog(assignment(g));
        mat = materials.(matlab.lang.makeValidName(pipe.material_id));
        idx = find(group_indices == g);
        length_m = 0;
        mass_kg = 0;
        for k = 1:numel(idx)
            e = idx(k);
            L = norm(nodes(connectivity(e, 1), :) - nodes(connectivity(e, 2), :));
            length_m = length_m + L;
            mass_kg = mass_kg + L * pipe.A_mm2 * 1e-6 * mat.rho_kg_m3;
        end
        breakdown(g) = struct( ...
            'group_name', group_names{g}, ...
            'pipe_id', pipe.id, ...
            'element_count', numel(idx), ...
            'length_m', length_m, ...
            'mass_kg', mass_kg);
    end
end

function assignment = assignmentFromSelectedPipes(selected_pipes, group_count)
    assignment = ones(1, group_count);
    for i = 1:numel(selected_pipes)
        g = selected_pipes(i).group_index;
        if g >= 1 && g <= group_count
            assignment(g) = selected_pipes(i).catalog_index;
        end
    end
end

function node_config = inferV2NodeConfig(frame, node_count, total_mass_kg)
    rear_mid_on = getNestedDefault(frame, {'design_variables', 'decoded', 'geometry', 'rear_mid_on'}, 0);
    base_count = 25 + 4 * double(rear_mid_on ~= 0);
    fbh_count = 4;
    front_susp_idx = base_count + fbh_count + (1:8);
    rear_susp_idx = base_count + fbh_count + 8 + 8 + (1:8);
    diff_count = 0;
    if isfield(frame, 'diff_mount_points')
        diff_count = size(numericMatrix(frame.diff_mount_points), 1);
    end
    diff_idx = (node_count - diff_count + 1):node_count;

    node_config = struct();
    node_config.total_mass_kg = total_mass_kg;
    node_config.rear_susp_left_nodes = rear_susp_idx([1, 3, 5, 7]);
    node_config.rear_susp_right_nodes = rear_susp_idx([2, 4, 6, 8]);
    node_config.rear_lower_nodes = [rear_susp_idx(1), rear_susp_idx(2)];
    node_config.rear_upper_nodes = [rear_susp_idx(3), rear_susp_idx(4)];
    node_config.front_susp_left_nodes = front_susp_idx([1, 3, 5, 7]);
    node_config.front_susp_right_nodes = front_susp_idx([2, 4, 6, 8]);
    if diff_count > 0
        node_config.diff_mount_nodes = diff_idx;
    end
end

function [active_nodes, connectivity, group_indices, topology_report, original_to_active] = ...
        buildRegulationMinimalTopologyFromFrame(frame, nodes, group_names, connect_diff_mounts)
    original_node_count = size(nodes, 1);
    idx = inferOriginalV2NodeIndices(frame, original_node_count);
    edges = zeros(0, 2);
    group_indices = zeros(0, 1);

    % Continuous hoop approximations on the current V2 node set.
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'front_hoop', [1 3; 3 4; 4 2], original_node_count);
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'main_hoop', [9 11; 11 13; 13 16; 16 15; 15 17; 17 14; 14 12; 12 10], ...
        original_node_count);
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'shoulder_bar', [13 14], original_node_count);

    % Minimum side-impact approximation: lower, upper, and one diagonal per side.
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'side_impact', [1 5; 5 9; 2 6; 6 10; 3 7; 7 11; 4 8; 8 12; 5 11; 6 12], ...
        original_node_count);

    f = idx.fbh;
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'front_bulkhead', [f(1) f(2); f(2) f(4); f(4) f(3); f(3) f(1)], ...
        original_node_count);
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'fbh_support', [f(1) 1; f(2) 2; f(3) 3; f(4) 4; f(3) 1; f(4) 2], ...
        original_node_count);

    % Main Hoop Brace and the mandatory-ish rear trapezoid used as the current
    % diff/rear-suspension mounting datum.
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'mh_bracing', [16 18; 17 19], original_node_count);
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        'mh_brace_support', [18 19; 20 21; 18 20; 19 21; 13 18; 9 20; 14 19; 10 21], ...
        original_node_count);

    front_targets = [f(1), f(2), f(3), f(4), 1, 2, 3, 4];
    [edges, group_indices] = appendBracketEdges(edges, group_indices, group_names, ...
        'front_susp_mount', idx.front_susp, idx.front_inner, front_targets, original_node_count);

    rear_targets = [20, 21, 18, 19, 20, 21, 18, 19];
    [edges, group_indices] = appendBracketEdges(edges, group_indices, group_names, ...
        'component_protection', idx.rear_susp, idx.rear_inner, rear_targets, original_node_count);

    if connect_diff_mounts && ~isempty(idx.diff)
        diff_edges = nearestTargetEdges(nodes, idx.diff, [18 19 20 21], 2);
        [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
            'diff_mount', diff_edges, original_node_count);
    end

    [connectivity, group_indices] = uniqueUndirectedEdges(edges, group_indices);
    used_original_nodes = unique(connectivity(:));
    original_to_active = zeros(original_node_count, 1);
    original_to_active(used_original_nodes) = 1:numel(used_original_nodes);
    active_nodes = nodes(used_original_nodes, :);
    connectivity = original_to_active(connectivity);

    group_counts = struct();
    for g = 1:numel(group_names)
        name = matlab.lang.makeValidName(group_names{g});
        count = nnz(group_indices == g);
        if count > 0
            group_counts.(name) = count;
        end
    end

    notes = {
        'Regulation-required tubes are approximated on the current V2 node set.'
        'Front and rear suspension hardpoints are kept connected with minimum bracket links.'
        'Rear suspension bracket links terminate on the Main Hoop Brace trapezoid.'
        'The current V2 upper-side-impact nodes are not yet a full F.6.4 geometric model; this is a topology lower-bound probe, not a final compliance claim.'
    };
    if connect_diff_mounts
        notes{end + 1} = 'Differential mount points are connected to the nearest brace-trapezoid nodes to avoid isolated diff-reaction DOFs.';
    else
        notes{end + 1} = 'Differential mount points are pruned when unused, and diff-reaction load cases are omitted by node-config remapping.';
    end

    topology_report = struct();
    topology_report.mode = 'regulation_minimal';
    topology_report.connect_diff_mounts = connect_diff_mounts;
    topology_report.original_node_count = original_node_count;
    topology_report.active_node_count = size(active_nodes, 1);
    topology_report.original_element_count = size(numericMatrix(frame.geometry.connectivity), 1);
    topology_report.active_element_count = size(connectivity, 1);
    topology_report.group_element_counts = group_counts;
    topology_report.notes = notes;
end

function idx = inferOriginalV2NodeIndices(frame, node_count)
    rear_mid_on = getNestedDefault(frame, {'design_variables', 'decoded', 'geometry', 'rear_mid_on'}, 0);
    base_count = 25 + 4 * double(rear_mid_on ~= 0);
    idx.fbh = base_count + (1:4);
    idx.front_susp = base_count + 4 + (1:8);
    idx.front_inner = base_count + 4 + 8 + (1:8);
    idx.rear_susp = base_count + 4 + 8 + 8 + (1:8);
    idx.rear_inner = base_count + 4 + 8 + 8 + 8 + (1:8);

    diff_count = 0;
    if isfield(frame, 'diff_mount_points')
        diff_count = size(numericMatrix(frame.diff_mount_points), 1);
    end
    idx.diff = [];
    if diff_count > 0
        idx.diff = (node_count - diff_count + 1):node_count;
    end
end

function [edges, group_indices] = appendBracketEdges(edges, group_indices, group_names, ...
        group_name, bracket_nodes, inner_nodes, target_nodes, node_count)
    pairs = zeros(0, 2);
    for k = 1:min([numel(bracket_nodes), numel(inner_nodes), numel(target_nodes)])
        pairs(end + 1, :) = [bracket_nodes(k), inner_nodes(k)]; %#ok<AGROW>
        pairs(end + 1, :) = [inner_nodes(k), target_nodes(k)]; %#ok<AGROW>
    end
    [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        group_name, pairs, node_count);
end

function [edges, group_indices] = appendGroupedEdges(edges, group_indices, group_names, ...
        group_name, pairs, node_count)
    g = groupIndex(group_names, group_name);
    for i = 1:size(pairs, 1)
        a = pairs(i, 1);
        b = pairs(i, 2);
        if a < 1 || b < 1 || a > node_count || b > node_count || a == b
            continue;
        end
        edges(end + 1, :) = [a, b]; %#ok<AGROW>
        group_indices(end + 1, 1) = g; %#ok<AGROW>
    end
end

function g = groupIndex(group_names, group_name)
    g = find(strcmp(group_names, group_name), 1);
    if isempty(g)
        error('experiment_frame_v2_mass_cap_stiffness:missingGroup', ...
            'Required group "%s" is missing from artifact group_names.', group_name);
    end
end

function edges = nearestTargetEdges(nodes, source_nodes, target_nodes, target_count)
    edges = zeros(0, 2);
    for i = 1:numel(source_nodes)
        source = source_nodes(i);
        distances = vecnorm(nodes(target_nodes, :) - nodes(source, :), 2, 2);
        [~, order] = sort(distances, 'ascend');
        selected = target_nodes(order(1:min(target_count, numel(order))));
        for j = 1:numel(selected)
            edges(end + 1, :) = [source, selected(j)]; %#ok<AGROW>
        end
    end
end

function [edges, group_indices] = uniqueUndirectedEdges(edges, group_indices)
    if isempty(edges)
        return;
    end
    sorted_edges = sort(edges, 2);
    [~, keep] = unique(sorted_edges, 'rows', 'stable');
    edges = edges(keep, :);
    group_indices = group_indices(keep);
end

function node_config = remapNodeConfig(node_config, original_to_active)
    names = fieldnames(node_config);
    for i = 1:numel(names)
        name = names{i};
        if ~contains(name, 'nodes')
            continue;
        end
        values = node_config.(name);
        mapped = zeros(1, 0);
        for v = reshape(values, 1, [])
            if v >= 1 && v <= numel(original_to_active) && original_to_active(v) > 0
                mapped(end + 1) = original_to_active(v); %#ok<AGROW>
            end
        end
        if isempty(mapped)
            node_config = rmfield(node_config, name);
        else
            node_config.(name) = mapped;
        end
    end
end

function tf = pipeMeetsRequiredSizeLocal(pipe, required_size, reg_params)
    if strcmp(required_size, 'None')
        tf = true;
        return;
    end
    if ~isfield(reg_params, 'size_classes') || ~isfield(reg_params.size_classes, required_size)
        tf = false;
        return;
    end

    shape = 'round';
    if isfield(pipe, 'shape')
        shape = pipe.shape;
    end
    props = computeTubeSectionProperties(pipe.D_outer_mm, pipe.t_mm, shape);
    req = reg_params.size_classes.(required_size);
    tf = props.Iy_mm4 >= req.I_min_mm4 ...
        && props.A_mm2 >= req.A_min_mm2 ...
        && pipe.D_outer_mm >= req.D_min_mm ...
        && pipe.t_mm >= req.t_min_mm;
end

function max_translation = extractMaxTranslation(result)
    max_translation = NaN;
    if ~isfield(result, 'U') || isempty(result.U)
        return;
    end
    U = result.U;
    if size(U, 2) >= 3
        max_translation = max(max(abs(U(:, 1:3))));
    else
        max_translation = max(abs(U(:)));
    end
end

function text = renderMarkdown(report)
    current = report.current_assignment;
    lightest = report.lightest_compliant_assignment;
    lines = {};
    lines{end + 1} = '# Frame V2 Mass Cap Stiffness Experiment';
    lines{end + 1} = '';
    lines{end + 1} = sprintf('- Generated: %s', report.generated_at);
    lines{end + 1} = sprintf('- Artifact: `%s`', report.artifact_path);
    lines{end + 1} = sprintf('- Mass cap: %.2f kg', report.mass_cap_kg);
    lines{end + 1} = sprintf('- Topology mode: `%s`', report.topology.mode);
    lines{end + 1} = sprintf('- Nodes/elements: %d/%d -> %d/%d', ...
        report.topology.original_node_count, report.topology.original_element_count, ...
        report.topology.active_node_count, report.topology.active_element_count);
    lines{end + 1} = '';
    lines{end + 1} = '## Result';
    lines{end + 1} = '';
    lines{end + 1} = '| Case | Mass kg | Gap to cap kg | K torsion Nm/deg | Total length m | Elements |';
    lines{end + 1} = '| --- | ---: | ---: | ---: | ---: | ---: |';
    lines{end + 1} = sprintf('| Current artifact assignment | %.2f | %.2f | %.1f | %.2f | %d |', ...
        current.mass_kg, current.mass_kg - report.mass_cap_kg, ...
        current.torsion.K_torsion_Nm_per_deg, current.total_length_m, current.element_count);
    lines{end + 1} = sprintf('| Lightest compliant fixed topology | %.2f | %.2f | %.1f | %.2f | %d |', ...
        lightest.mass_kg, lightest.mass_kg - report.mass_cap_kg, ...
        lightest.torsion.K_torsion_Nm_per_deg, lightest.total_length_m, lightest.element_count);
    lines{end + 1} = '';
    if report.cap_feasible_in_fixed_topology
        lines{end + 1} = 'The mass cap is feasible in this fixed topology.';
    else
        lines{end + 1} = sprintf('The %.2f kg cap is infeasible in this fixed topology; the lower-bound assignment is %.2f kg over the cap.', ...
            report.mass_cap_kg, report.mass_gap_to_cap_kg);
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Topology Notes';
    lines{end + 1} = '';
    for i = 1:numel(report.topology.notes)
        lines{end + 1} = sprintf('- %s', report.topology.notes{i});
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Lightest Pipe Assignment';
    lines{end + 1} = '';
    lines{end + 1} = '| Group | Required | Pipe | Size | Material | kg/m |';
    lines{end + 1} = '| --- | --- | --- | --- | --- | ---: |';
    for i = 1:numel(report.lightest_pipe_table)
        r = report.lightest_pipe_table(i);
        lines{end + 1} = sprintf('| %s | %s | %s | %s | %s | %.3f |', ...
            r.group_name, r.required_size, r.pipe_id, r.size_class, ...
            r.material_id, r.mass_per_m_kg);
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Load Cases';
    lines{end + 1} = '';
    lines{end + 1} = '| Case | K torsion Nm/deg | Max translation m | Success |';
    lines{end + 1} = '| --- | ---: | ---: | --- |';
    for i = 1:numel(lightest.load_cases)
        c = lightest.load_cases(i);
        lines{end + 1} = sprintf('| %s | %.1f | %.6f | %d |', ...
            c.name, c.K_torsion_Nm_per_deg, c.max_translation_m, c.success);
    end
    text = strjoin(lines, newline);
end

function value = getNestedDefault(s, path, default_value)
    value = default_value;
    current = s;
    for i = 1:numel(path)
        if ~isstruct(current) || ~isfield(current, path{i})
            return;
        end
        current = current.(path{i});
    end
    value = current;
end

function names = toCellstr(value)
    if iscell(value)
        names = cellfun(@char, value, 'UniformOutput', false);
    elseif isstring(value)
        names = cellstr(value);
    else
        names = cellstr(value);
    end
end

function mat = numericMatrix(value)
    if isnumeric(value)
        mat = value;
    elseif iscell(value)
        mat = cell2mat(value);
    else
        mat = struct2array(value);
    end
end

function vec = numericVector(value)
    vec = numericMatrix(value);
    vec = reshape(vec, [], 1);
end

function writeTextFile(path, text)
    fid = fopen(path, 'w');
    if fid < 0
        error('experiment_frame_v2_mass_cap_stiffness:writeFailed', ...
            'Could not write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', text);
end
