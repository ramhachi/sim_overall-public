function report = investigate_frame_v2_bottlenecks(varargin)
% INVESTIGATE_FRAME_V2_BOTTLENECKS Inspect V2 frame mass and solver-speed issues.
%
% This is an investigation helper, not a production optimizer path. It reads
% a saved frame_result/v2 artifact, computes mass/length by regulation group,
% and optionally benchmarks the native MEX solver on the saved final frame.

    p = inputParser;
    addParameter(p, 'ArtifactPath', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'RunMexBenchmark', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'NumMexRuns', 20, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    parse(p, varargin{:});
    opts = p.Results;

    script_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(script_dir);
    project_root = fileparts(frame_root);
    addpath(fullfile(frame_root, 'matlab'));
    addpath(fullfile(project_root, 'matlab'));

    artifact_path = char(string(opts.ArtifactPath));
    if isempty(artifact_path)
        artifact_path = fullfile(project_root, 'output', 'runtime', ...
            'frame_v2_ga_15000', 'frame_v2_ga_15000_vehicle_artifact.json');
    end

    output_dir = char(string(opts.OutputDir));
    if isempty(output_dir)
        output_dir = fullfile(project_root, 'output', 'runtime', 'frame_v2_investigation');
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    artifact = jsondecode(fileread(artifact_path));
    frame = artifact.params.frame;
    data_dir = fullfile(frame_root, 'data');
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

    nodes = numericMatrix(frame.geometry.nodes_m);
    connectivity = numericMatrix(frame.geometry.connectivity);
    group_indices = numericVector(frame.geometry.group_indices);
    group_names = cellstr(frame.geometry.group_names);

    group_breakdown = computeGroupBreakdown(nodes, connectivity, group_indices, ...
        group_names, frame.selected_pipes, catalog, materials);
    mass_summary = summarizeMass(group_breakdown, frame);

    mex_summary = struct('enabled', logical(opts.RunMexBenchmark));
    if logical(opts.RunMexBenchmark)
        mex_summary = benchmarkMexSolver(frame_root, nodes, connectivity, group_indices, ...
            frame.selected_pipes, catalog, materials, frame, opts.NumMexRuns);
    end

    report = struct();
    report.schema_version = 'frame_v2_bottleneck_investigation/v1';
    report.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    report.artifact_path = artifact_path;
    report.output_dir = output_dir;
    report.mass_summary = mass_summary;
    report.group_breakdown = group_breakdown;
    report.mex_summary = mex_summary;
    report.observations = buildObservations(mass_summary, group_breakdown, mex_summary, frame);

    json_path = fullfile(output_dir, 'frame_v2_bottleneck_investigation.json');
    md_path = fullfile(output_dir, 'frame_v2_bottleneck_investigation.md');
    writeTextFile(json_path, jsonencode(report, PrettyPrint=true));
    writeTextFile(md_path, renderMarkdown(report));
    report.json_path = json_path;
    report.markdown_path = md_path;

    fprintf('=== Frame V2 bottleneck investigation ===\n');
    fprintf('Artifact: %s\n', artifact_path);
    fprintf('Mass %.2f kg, total length %.2f m, elements %d\n', ...
        mass_summary.artifact_mass_kg, mass_summary.total_length_m, mass_summary.num_elements);
    if isfield(mex_summary, 'median_s')
        fprintf('MEX torsion median %.6f s/eval, mean %.6f s/eval (%d runs)\n', ...
            mex_summary.median_s, mex_summary.mean_s, mex_summary.num_runs);
    end
    fprintf('Saved JSON: %s\n', json_path);
    fprintf('Saved MD: %s\n', md_path);
end

function breakdown = computeGroupBreakdown(nodes, connectivity, group_indices, group_names, selected_pipes, catalog, materials)
    breakdown = struct('group_index', {}, 'group_name', {}, 'pipe_id', {}, ...
        'shape', {}, 'material_id', {}, 'D_outer_mm', {}, 't_mm', {}, ...
        'element_count', {}, 'length_m', {}, 'mass_kg', {}, 'mass_fraction', {});

    total_mass = 0;
    masses = zeros(numel(group_names), 1);
    lengths = zeros(numel(group_names), 1);
    counts = zeros(numel(group_names), 1);

    pipe_by_group = buildPipeLookup(selected_pipes);
    for e = 1:size(connectivity, 1)
        g = group_indices(e);
        if g < 1 || g > numel(group_names)
            continue;
        end
        pipe_index = pipe_by_group(g);
        pipe = catalog(pipe_index);
        mat = materials.(pipe.material_id);
        L = norm(nodes(connectivity(e, 1), :) - nodes(connectivity(e, 2), :));
        area_mm2 = tubeAreaMm2(pipe.D_outer_mm, pipe.t_mm, pipe.shape);
        m = L * area_mm2 * 1e-6 * mat.rho_kg_m3;
        counts(g) = counts(g) + 1;
        lengths(g) = lengths(g) + L;
        masses(g) = masses(g) + m;
        total_mass = total_mass + m;
    end

    for g = 1:numel(group_names)
        pipe_index = pipe_by_group(g);
        pipe = catalog(pipe_index);
        fraction = 0;
        if total_mass > 0
            fraction = masses(g) / total_mass;
        end
        breakdown(g) = struct( ...
            'group_index', g, ...
            'group_name', group_names{g}, ...
            'pipe_id', pipe.id, ...
            'shape', pipe.shape, ...
            'material_id', pipe.material_id, ...
            'D_outer_mm', pipe.D_outer_mm, ...
            't_mm', pipe.t_mm, ...
            'element_count', counts(g), ...
            'length_m', lengths(g), ...
            'mass_kg', masses(g), ...
            'mass_fraction', fraction);
    end
end

function summary = summarizeMass(group_breakdown, frame)
    masses = [group_breakdown.mass_kg];
    lengths = [group_breakdown.length_m];
    summary = struct();
    summary.recomputed_mass_kg = sum(masses);
    summary.artifact_mass_kg = getNestedDefault(frame, {'performance', 'mass_kg'}, NaN);
    summary.mass_delta_kg = summary.recomputed_mass_kg - summary.artifact_mass_kg;
    summary.total_length_m = sum(lengths);
    summary.artifact_total_length_m = getNestedDefault(frame, {'performance', 'total_length_m'}, NaN);
    summary.sparsity_limit_m = getNestedDefault(frame, {'objective_config', 'sparsity_max_total_length_m'}, 45);
    summary.num_elements = sum([group_breakdown.element_count]);
    summary.artifact_num_elements = getNestedDefault(frame, {'performance', 'num_elements'}, NaN);
    [~, order] = sort(masses, 'descend');
    summary.top_mass_groups = {group_breakdown(order(1:min(5, numel(order)))).group_name};
end

function mex_summary = benchmarkMexSolver(frame_root, nodes, connectivity, group_indices, selected_pipes, catalog, materials, frame, num_runs)
    mex_summary = struct();
    mex_summary.enabled = true;
    mex_summary.num_runs = num_runs;
    mex_dir = fullfile(frame_root, 'mex');
    if exist(mex_dir, 'dir')
        addpath(mex_dir, '-begin');
    end
    solver_path = which('beam_fem_solver');
    mex_summary.solver_path = solver_path;
    mex_summary.available = contains(solver_path, ['.' mexext]);
    if ~mex_summary.available
        mex_summary.error = sprintf('Native MEX solver was not first on path. which() returned: %s', solver_path);
        return;
    end

    elements_fem = buildElementsForSolver(nodes, connectivity, group_indices, selected_pipes, catalog, materials);
    node_config = inferV2NodeConfig(frame, size(nodes, 1));
    mass_kg = getVehicleMassFromArtifact(frame);
    load_cases = generate_load_cases(nodes, mass_kg, 9.81, node_config);

    times = zeros(num_runs, 1);
    k_values = zeros(num_runs, 1);
    masses = zeros(num_runs, 1);
    for i = 1:num_runs
        t_start = tic;
        result = beam_fem_solver(nodes, elements_fem, load_cases(1).bcs, load_cases(1).loads);
        times(i) = toc(t_start);
        k_values(i) = result.K_torsion;
        masses(i) = result.mass;
    end

    mex_summary.times_s = times;
    mex_summary.mean_s = mean(times);
    mex_summary.median_s = median(times);
    mex_summary.min_s = min(times);
    mex_summary.max_s = max(times);
    mex_summary.evals_per_minute = 60 / mex_summary.mean_s;
    mex_summary.K_torsion_Nm_per_deg = median(k_values);
    mex_summary.mass_kg = median(masses);
end

function elements = buildElementsForSolver(~, connectivity, group_indices, selected_pipes, catalog, materials)
    pipe_by_group = buildPipeLookup(selected_pipes);
    elements = zeros(size(connectivity, 1), 9);
    for e = 1:size(connectivity, 1)
        g = group_indices(e);
        pipe = catalog(pipe_by_group(g));
        mat = materials.(pipe.material_id);
        shape_code = 1;
        if strcmpi(pipe.shape, 'square')
            shape_code = 2;
        end
        elements(e, :) = [connectivity(e, 1), connectivity(e, 2), ...
            pipe.D_outer_mm, pipe.t_mm, mat.E_GPa, mat.G_GPa, ...
            mat.rho_kg_m3, shape_code, 0.85];
    end
end

function node_config = inferV2NodeConfig(frame, node_count)
    rear_mid_on = 0;
    if isfield(frame.design_variables, 'geometry') && isfield(frame.design_variables.geometry, 'rear_mid_on')
        rear_mid_on = frame.design_variables.geometry.rear_mid_on;
    end
    base_count = 25 + 4 * double(rear_mid_on ~= 0);
    fbh_count = 4;
    front_susp_idx = base_count + fbh_count + (1:8);
    rear_susp_idx = base_count + fbh_count + 8 + 8 + (1:8);
    rear_inner_idx = rear_susp_idx(end) + (1:8); %#ok<NASGU>
    diff_count = 0;
    if isfield(frame, 'diff_mount_points')
        diff_count = size(numericMatrix(frame.diff_mount_points), 1);
    end
    diff_idx = (node_count - diff_count + 1):node_count;

    node_config = struct();
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

function mass_kg = getVehicleMassFromArtifact(frame)
    mass_kg = 250;
    if isfield(frame, 'load_cases') && ~isempty(frame.load_cases) && isfield(frame.load_cases(1), 'mass_kg')
        mass_kg = frame.load_cases(1).mass_kg;
    end
end

function lookup = buildPipeLookup(selected_pipes)
    max_group = max([selected_pipes.group_index]);
    lookup = ones(max_group, 1);
    for i = 1:numel(selected_pipes)
        lookup(selected_pipes(i).group_index) = selected_pipes(i).catalog_index;
    end
end

function area = tubeAreaMm2(D_outer, t, shape)
    D_inner = D_outer - 2 * t;
    if strcmpi(shape, 'square')
        area = D_outer^2 - D_inner^2;
    else
        area = pi / 4 * (D_outer^2 - D_inner^2);
    end
end

function observations = buildObservations(mass_summary, group_breakdown, mex_summary, frame)
    observations = {};
    observations{end + 1} = sprintf('Final artifact mass is %.2f kg against the 25 kg target.', ...
        mass_summary.artifact_mass_kg);
    observations{end + 1} = sprintf('Total tube length is %.2f m against the configured %.2f m sparsity limit.', ...
        mass_summary.total_length_m, mass_summary.sparsity_limit_m);
    [~, order] = sort([group_breakdown.mass_kg], 'descend');
    heaviest = group_breakdown(order(1));
    observations{end + 1} = sprintf('Heaviest group is %s: %.2f kg across %.2f m using %s.', ...
        heaviest.group_name, heaviest.mass_kg, heaviest.length_m, heaviest.pipe_id);
    if isfield(mex_summary, 'median_s')
        observations{end + 1} = sprintf('Native MEX torsion solve median is %.6f s/eval, so Docker/TCP lifecycle and multi-case evaluation dominate long-run cost.', ...
            mex_summary.median_s);
    end
    if isfield(frame.performance, 'penalties') && isfield(frame.performance.penalties, 'mass_target')
        observations{end + 1} = sprintf('Mass target penalty is %.2f, while stiffness penalty is %.2f; the current scalar objective still tolerates heavy feasible frames.', ...
            frame.performance.penalties.mass_target, frame.performance.penalties.stiffness);
    end
end

function text = renderMarkdown(report)
    lines = {};
    lines{end + 1} = '# Frame V2 Bottleneck Investigation';
    lines{end + 1} = '';
    lines{end + 1} = sprintf('- Generated: %s', report.generated_at);
    lines{end + 1} = sprintf('- Artifact: `%s`', report.artifact_path);
    lines{end + 1} = '';
    lines{end + 1} = '## Summary';
    for i = 1:numel(report.observations)
        lines{end + 1} = sprintf('- %s', report.observations{i});
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Mass By Group';
    lines{end + 1} = '';
    lines{end + 1} = '| Group | Pipe | Elements | Length m | Mass kg | Share |';
    lines{end + 1} = '| --- | --- | ---: | ---: | ---: | ---: |';
    for i = 1:numel(report.group_breakdown)
        g = report.group_breakdown(i);
        lines{end + 1} = sprintf('| %s | %s | %d | %.2f | %.2f | %.1f%% |', ...
            g.group_name, g.pipe_id, g.element_count, g.length_m, g.mass_kg, 100 * g.mass_fraction);
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Solver Timing';
    if isfield(report.mex_summary, 'median_s')
        lines{end + 1} = sprintf('- MEX solver: median %.6f s/eval, mean %.6f s/eval, %.0f eval/min', ...
            report.mex_summary.median_s, report.mex_summary.mean_s, report.mex_summary.evals_per_minute);
        lines{end + 1} = sprintf('- MEX path: `%s`', report.mex_summary.solver_path);
    elseif isfield(report.mex_summary, 'error')
        lines{end + 1} = sprintf('- MEX benchmark skipped/failed: %s', report.mex_summary.error);
    else
        lines{end + 1} = '- MEX benchmark disabled.';
    end
    lines{end + 1} = '';
    lines{end + 1} = '## Investigation Notes';
    lines{end + 1} = '- This script does not change optimizer behavior.';
    lines{end + 1} = '- It intentionally avoids Docker/TCP benchmarking by default.';
    lines{end + 1} = '- Use it to separate mass/topology issues from FEM server lifecycle cost.';
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
        error('investigate_frame_v2_bottlenecks:writeFailed', 'Could not write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', text);
end
