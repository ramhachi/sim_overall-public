function result = benchmark_frame_v2_loop_and_visualize(varargin)
% BENCHMARK_FRAME_V2_LOOP_AND_VISUALIZE Measure V2 objective speed and render final frame.

    p = inputParser;
    addParameter(p, 'NumObjectiveRuns', 5, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'OptimizerMode', 'local_refine', @(x) ischar(x) || isstring(x));
    addParameter(p, 'PopulationSize', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MaxGenerations', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'RandomSeed', 42, @(x) isnumeric(x) && isscalar(x) && isfinite(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    opts = p.Results;
    opts.OptimizerMode = char(string(opts.OptimizerMode));

    script_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(script_dir);
    project_root = fileparts(frame_root);
    addpath(fullfile(frame_root, 'matlab'));
    addpath(fullfile(project_root, 'matlab'));

    if isempty(char(string(opts.OutputDir)))
        output_dir = fullfile(project_root, 'output', 'runtime', 'frame_v2_benchmark');
    else
        output_dir = char(string(opts.OutputDir));
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    rng(opts.RandomSeed);
    fixture_path = fullfile(frame_root, 'tests', 'fixtures', 'canonical_suspension_fixture.json');
    benchmark_artifact_path = fullfile(output_dir, 'frame_v2_benchmark_vehicle_artifact.json');
    prepareBenchmarkArtifact(fixture_path, benchmark_artifact_path);

    data_dir = fullfile(frame_root, 'data');
    geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
    reg_params = jsondecode(fileread(fullfile(data_dir, 'regulation_params.json')));

    artifact = load_vehicle_artifact(benchmark_artifact_path);
    diff_config = load_diff_mount_config(artifact.params, geom);
    config = buildBenchmarkConfig(geom, fixture_path, diff_config, numel(catalog), opts.RandomSeed);
    objective_func = @(x) frameObjectiveFunction(x, [], [], [], config, catalog, materials, reg_params);
    x0 = config.design_schema.initial;

    fprintf('=== Frame V2 objective loop benchmark ===\n');
    fprintf('Runs: %d, optimizer: %s, output: %s\n', opts.NumObjectiveRuns, opts.OptimizerMode, output_dir);

    beam_fem_solver_tcp_java('start');
    cleanup_solver = onCleanup(@() beam_fem_solver_tcp_java('stop')); %#ok<NASGU>

    fprintf('Warmup objective evaluation...\n');
    [warmup_obj, warmup_results] = objective_func(x0);
    fprintf('Warmup: obj=%.6g, success=%d, cases=%d\n', ...
        warmup_obj, warmup_results.success, numel(warmup_results.load_case_results));

    objective_times_s = zeros(opts.NumObjectiveRuns, 1);
    objective_values = zeros(opts.NumObjectiveRuns, 1);
    for i = 1:opts.NumObjectiveRuns
        tic;
        [objective_values(i), run_results] = objective_func(x0); %#ok<NASGU>
        objective_times_s(i) = toc;
        fprintf('  objective run %02d/%02d: %.3f s, obj=%.6g\n', ...
            i, opts.NumObjectiveRuns, objective_times_s(i), objective_values(i));
    end

    objective_summary = struct();
    objective_summary.num_runs = opts.NumObjectiveRuns;
    objective_summary.times_s = objective_times_s;
    objective_summary.mean_s = mean(objective_times_s);
    objective_summary.median_s = median(objective_times_s);
    objective_summary.min_s = min(objective_times_s);
    objective_summary.max_s = max(objective_times_s);
    objective_summary.evals_per_minute = 60 / objective_summary.mean_s;
    objective_summary.objective_values = objective_values;
    objective_summary.warmup_objective = warmup_obj;
    objective_summary.warmup_mass_kg = warmup_results.mass;
    objective_summary.warmup_K_torsion = warmup_results.K_torsion;
    objective_summary.warmup_compliance_pass = warmup_results.compliance_pass;
    objective_summary.warmup_num_load_cases = numel(warmup_results.load_case_results);

    fprintf('\n=== Frame V2 mini optimization + visualization ===\n');
    opt_result = run_canonical_frame_optimization( ...
        'VehicleParamsFile', benchmark_artifact_path, ...
        'UseFrameDesignV2', true, ...
        'OptimizerMode', opts.OptimizerMode, ...
        'PopulationSize', opts.PopulationSize, ...
        'MaxGenerations', opts.MaxGenerations, ...
        'UseParallel', false, ...
        'ShowViewer', false, ...
        'SaveFigure', false, ...
        'WaitForFigureClose', false, ...
        'DisplayProgress', false, ...
        'RandomSeed', opts.RandomSeed);

    frame_png_path = fullfile(output_dir, 'final_frame_v2.png');
    renderFinalFrame(opt_result, catalog, frame_png_path);

    result = struct();
    result.schema_version = 'frame_v2_benchmark/v1';
    result.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    result.output_dir = output_dir;
    result.vehicle_artifact_path = benchmark_artifact_path;
    result.final_frame_png_path = frame_png_path;
    result.objective_loop = objective_summary;
    result.optimizer = struct();
    result.optimizer.mode = opts.OptimizerMode;
    result.optimizer.population_size = opts.PopulationSize;
    result.optimizer.max_generations = opts.MaxGenerations;
    result.optimizer.elapsed_s = opt_result.output.elapsed_s;
    result.optimizer.best_objective = opt_result.best_fval;
    result.optimizer.valid_individual_rate = getFieldOrDefault(opt_result.output, 'valid_individual_rate', NaN);
    result.final_frame = struct();
    result.final_frame.nodes = size(opt_result.nodes, 1);
    result.final_frame.elements = size(opt_result.elements_geom.connectivity, 1);
    result.final_frame.mass_kg = opt_result.final_results.mass;
    result.final_frame.K_torsion = opt_result.final_results.K_torsion;
    result.final_frame.cg_z_m = opt_result.final_results.cg_z;
    result.final_frame.compliance_pass = opt_result.final_results.compliance_pass;
    result.final_frame.compliance_penalty = opt_result.final_results.compliance_penalty;
    result.final_frame.load_cases = {opt_result.final_results.load_case_results.name};

    json_path = fullfile(output_dir, 'frame_v2_benchmark_summary.json');
    writeTextFile(json_path, jsonencode(result, PrettyPrint=true));
    result.summary_json_path = json_path;

    fprintf('\n=== Benchmark summary ===\n');
    fprintf('Objective mean: %.3f s/eval (%.1f eval/min), median %.3f s\n', ...
        result.objective_loop.mean_s, result.objective_loop.evals_per_minute, result.objective_loop.median_s);
    fprintf('Optimizer elapsed: %.3f s, best obj %.6g\n', ...
        result.optimizer.elapsed_s, result.optimizer.best_objective);
    fprintf('Final frame: %d nodes, %d elements, mass %.2f kg, K %.1f Nm/deg, compliance=%d\n', ...
        result.final_frame.nodes, result.final_frame.elements, result.final_frame.mass_kg, ...
        result.final_frame.K_torsion, result.final_frame.compliance_pass);
    fprintf('Saved PNG: %s\n', frame_png_path);
    fprintf('Saved JSON: %s\n', json_path);
end

function prepareBenchmarkArtifact(src_path, dst_path)
    artifact = load_vehicle_artifact(src_path);
    artifact.params = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
    artifact.params.suspension.K_chassis = 1200;
    save_vehicle_artifact(dst_path, artifact);
end

function config = buildBenchmarkConfig(geom, suspension_file, diff_config, catalog_count, random_seed)
    config = struct();
    config.W_mass = 5.0;
    config.W_cg = 1.0;
    config.mass_ref = 22.0;
    config.cg_ref = 0.300;
    config.K_min = 500;
    config.K_target = 1200;
    config.gamma_below = 8.0;
    config.gamma_hard = 50.0;
    config.compliance_penalty_scale = 1000;
    config.joint_factor = 0.85;
    config.max_cg_z = 0.380;
    config.sparsity_max_total_length_m = 45.0;
    config.sparsity_max_elements = 120;
    config.require_rear_suspension_hardpoints = true;
    config.require_total_mass_kg = true;
    config.geom = shiftGeometryExclusionZonesLocal(geom, 0.050);
    config.suspension_file = suspension_file;
    config.design_schema_version = 'frame_design/v2';
    config.design_schema = frame_design_schema_v2(catalog_count);
    config.diff_mount_points = diff_config.diff_mount_points;
    config.diff_envelope = diff_config.diff_envelope;
    config.diff_config_source = diff_config.source;
    config.optimizer_mode = 'benchmark_objective_loop';
    config.random_seed = random_seed;
    config.max_rear_free_pipe_length_m = 1.20;
    config.case_max_displacement_m = 0.025;
    config.diff_reaction_force_N = 1500;
    config.diff_reaction_torque_Nm = 500;
    config.exclusion_zones = buildExclusionZoneCellLocal(config.geom);
end

function renderFinalFrame(opt_result, catalog, png_path)
    fig = figure('Visible', 'off', 'Color', 'white', 'Position', [100, 100, 1400, 900]);
    ax = axes(fig);
    pipe_count = max(opt_result.elements_geom.group_indices);
    pipe_assignments = opt_result.best_x(1:min(pipe_count, numel(opt_result.best_x)));
    plot_frame(opt_result.nodes, opt_result.elements_geom, pipe_assignments, catalog, true);
    title(ax, sprintf('Frame V2 final: %.2f kg, K %.0f Nm/deg, obj %.3g', ...
        opt_result.final_results.mass, opt_result.final_results.K_torsion, opt_result.best_fval), ...
        'FontWeight', 'bold');
    view(ax, 35, 24);
    try
        exportgraphics(fig, png_path, 'Resolution', 200);
    catch
        saveas(fig, png_path);
    end
    close(fig);
end

function zones = buildExclusionZoneCellLocal(geom)
    zones = {};
    if ~isfield(geom, 'exclusion_zones') || isempty(geom.exclusion_zones)
        return;
    end
    for i = 1:length(geom.exclusion_zones)
        zone = geom.exclusion_zones(i);
        exclusion = struct();
        if isfield(zone, 'type')
            exclusion.type = zone.type;
        else
            exclusion.type = 'AABB';
        end
        if isfield(zone, 'min')
            exclusion.min_bounds = zone.min;
            exclusion.max_bounds = zone.max;
        else
            exclusion.min_bounds = zone.min_bounds;
            exclusion.max_bounds = zone.max_bounds;
        end
        if isfield(zone, 'name')
            exclusion.name = zone.name;
        else
            exclusion.name = sprintf('Zone_%d', i);
        end
        zones{end + 1} = exclusion; %#ok<AGROW>
    end
end

function geom = shiftGeometryExclusionZonesLocal(geom, z_offset)
    if ~isfield(geom, 'exclusion_zones')
        return;
    end
    for i = 1:length(geom.exclusion_zones)
        if isfield(geom.exclusion_zones(i), 'min') && isfield(geom.exclusion_zones(i), 'max')
            geom.exclusion_zones(i).min(3) = geom.exclusion_zones(i).min(3) + z_offset;
            geom.exclusion_zones(i).max(3) = geom.exclusion_zones(i).max(3) + z_offset;
        end
    end
end

function writeTextFile(path, text)
    fid = fopen(path, 'w');
    if fid < 0
        error('benchmark_frame_v2_loop_and_visualize:writeFailed', 'Could not write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s', text);
end

function value = getFieldOrDefault(s, field_name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    end
end
