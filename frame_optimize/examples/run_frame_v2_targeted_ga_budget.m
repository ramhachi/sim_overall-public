function result = run_frame_v2_targeted_ga_budget(varargin)
% RUN_FRAME_V2_TARGETED_GA_BUDGET Run V2 frame optimization with a fixed GA budget.

    p = inputParser;
    addParameter(p, 'EvaluationBudget', 15000, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'PopulationSize', 100, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'TargetStiffnessNmPerDeg', 7000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'MassTargetKg', 25, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'RandomSeed', 7000, @(x) isnumeric(x) && isscalar(x) && isfinite(x));
    addParameter(p, 'LoadCaseEvalMode', 'torsion_only', @(x) ischar(x) || isstring(x));
    addParameter(p, 'SolverBackend', 'mex', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MassPenaltyWeight', 1000, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(p, 'SparsityMaxTotalLengthM', 36, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
    addParameter(p, 'SparsityMaxElements', 105, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
    addParameter(p, 'SparsityPenaltyScale', 200, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(p, 'FrameCgLimitM', 0.55, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
    addParameter(p, 'StiffnessBelowPenaltyWeight', 1000, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(p, 'StiffnessHardPenaltyWeight', 5000, @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
    addParameter(p, 'EnableRearOptionalEdges', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'RearBoxDiagonalMode', 'minimal', @(x) ischar(x) || isstring(x));
    addParameter(p, 'VehicleParamsFile', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    opts = p.Results;

    script_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(script_dir);
    project_root = fileparts(frame_root);
    addpath(fullfile(frame_root, 'matlab'));
    addpath(fullfile(project_root, 'matlab'));

    if isempty(char(string(opts.OutputDir)))
        output_dir = fullfile(project_root, 'output', 'runtime', 'frame_v2_ga_15000');
    else
        output_dir = char(string(opts.OutputDir));
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    vehicle_artifact_path = prepareInputArtifact(opts, frame_root, output_dir);
    elite_count = max(1, ceil(0.05 * opts.PopulationSize));
    evals_per_generation_after_first = opts.PopulationSize - elite_count;
    if opts.EvaluationBudget <= 2 * opts.PopulationSize
        max_generations = max(1, ceil(opts.EvaluationBudget / opts.PopulationSize) - 1);
    else
        max_generations = 1 + ceil( ...
            (opts.EvaluationBudget - 2 * opts.PopulationSize) / evals_per_generation_after_first);
    end
    expected_budget = 2 * opts.PopulationSize ...
        + max(0, max_generations - 1) * evals_per_generation_after_first;

    fprintf('=== Frame V2 MATLAB GA fixed budget run ===\n');
    fprintf('Target K: %.0f Nm/deg, mass target: %.1f kg\n', ...
        opts.TargetStiffnessNmPerDeg, opts.MassTargetKg);
    fprintf('Population: %d, generations: %d, expected function evaluations: %d\n', ...
        opts.PopulationSize, max_generations, expected_budget);
    fprintf('GA elite count estimate: %d (new evaluations after first generation: %d/gen)\n', ...
        elite_count, evals_per_generation_after_first);
    fprintf('Inner load-case mode: %s (final result is re-evaluated with full cases)\n', ...
        char(string(opts.LoadCaseEvalMode)));
    fprintf('Solver: %s, mass gamma: %.1f, length limit: %.1f m, element limit: %.0f\n', ...
        char(string(opts.SolverBackend)), opts.MassPenaltyWeight, ...
        opts.SparsityMaxTotalLengthM, opts.SparsityMaxElements);
    fprintf('Penalty scales: sparsity %.1f, CG limit %.2f m, stiffness gamma %.1f/%.1f\n', ...
        opts.SparsityPenaltyScale, opts.FrameCgLimitM, ...
        opts.StiffnessBelowPenaltyWeight, opts.StiffnessHardPenaltyWeight);

    tic;
    opt_result = run_canonical_frame_optimization( ...
        'VehicleParamsFile', vehicle_artifact_path, ...
        'UseFrameDesignV2', true, ...
        'OptimizerMode', 'matlab_ga', ...
        'PopulationSize', opts.PopulationSize, ...
        'MaxGenerations', max_generations, ...
        'UseParallel', false, ...
        'ShowViewer', false, ...
        'SaveFigure', false, ...
        'WaitForFigureClose', false, ...
        'DisplayProgress', true, ...
        'RandomSeed', opts.RandomSeed, ...
        'StiffnessTargetNmPerDeg', opts.TargetStiffnessNmPerDeg, ...
        'MassTargetKg', opts.MassTargetKg, ...
        'LoadCaseEvalMode', char(string(opts.LoadCaseEvalMode)), ...
        'SolverBackend', char(string(opts.SolverBackend)), ...
        'MassPenaltyWeight', opts.MassPenaltyWeight, ...
        'SparsityMaxTotalLengthM', opts.SparsityMaxTotalLengthM, ...
        'SparsityMaxElements', opts.SparsityMaxElements, ...
        'SparsityPenaltyScale', opts.SparsityPenaltyScale, ...
        'FrameCgLimitM', opts.FrameCgLimitM, ...
        'StiffnessBelowPenaltyWeight', opts.StiffnessBelowPenaltyWeight, ...
        'StiffnessHardPenaltyWeight', opts.StiffnessHardPenaltyWeight, ...
        'EnableRearOptionalEdges', logical(opts.EnableRearOptionalEdges), ...
        'RearBoxDiagonalMode', char(string(opts.RearBoxDiagonalMode)), ...
        'SuppressObjectiveOutput', true);
    wall_time_s = toc;

    data_dir = fullfile(frame_root, 'data');
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

    png_path = fullfile(output_dir, 'frame_v2_ga_15000_final.png');
    renderFinalFrame(opt_result, catalog, png_path);

    result = struct();
    result.schema_version = 'frame_v2_targeted_ga_budget/v1';
    result.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    result.output_dir = output_dir;
    result.vehicle_artifact_path = vehicle_artifact_path;
    result.final_frame_png_path = png_path;
    result.target_stiffness_Nm_per_deg = opts.TargetStiffnessNmPerDeg;
    result.mass_target_kg = opts.MassTargetKg;
    result.solver_backend = char(string(opts.SolverBackend));
    result.mass_penalty_weight = opts.MassPenaltyWeight;
    result.sparsity_max_total_length_m = opts.SparsityMaxTotalLengthM;
    result.sparsity_max_elements = opts.SparsityMaxElements;
    result.sparsity_penalty_scale = opts.SparsityPenaltyScale;
    result.frame_cg_limit_m = opts.FrameCgLimitM;
    result.stiffness_below_penalty_weight = opts.StiffnessBelowPenaltyWeight;
    result.stiffness_hard_penalty_weight = opts.StiffnessHardPenaltyWeight;
    result.enable_rear_optional_edges = logical(opts.EnableRearOptionalEdges);
    result.rear_box_diagonal_mode = char(string(opts.RearBoxDiagonalMode));
    result.population_size = opts.PopulationSize;
    result.estimated_elite_count = elite_count;
    result.max_generations = max_generations;
    result.expected_evaluation_budget = expected_budget;
    result.actual_funccount = getFieldOrDefault(opt_result.output, 'funccount', NaN);
    result.finite_score_rate = getFieldOrDefault(opt_result.output, 'finite_score_rate', NaN);
    result.valid_individual_rate = getFieldOrDefault(opt_result.output, 'valid_individual_rate', NaN);
    result.wall_time_s = wall_time_s;
    result.optimizer_elapsed_s = opt_result.output.elapsed_s;
    result.seconds_per_funccount = wall_time_s / max(1, result.actual_funccount);
    result.best_objective = opt_result.best_fval;
    result.final_mass_kg = opt_result.final_results.mass;
    result.final_K_torsion_Nm_per_deg = opt_result.final_results.K_torsion;
    result.final_cg_z_m = opt_result.final_results.cg_z;
    result.final_nodes = size(opt_result.nodes, 1);
    result.final_elements = size(opt_result.elements_geom.connectivity, 1);
    result.compliance_pass = opt_result.final_results.compliance_pass;
    result.compliance_penalty = opt_result.final_results.compliance_penalty;
    result.load_cases = {opt_result.final_results.load_case_results.name};
    result.inner_load_case_mode = opt_result.output.inner_load_case_mode;
    result.inner_fem_solves_per_eval = opt_result.output.inner_fem_solves_per_eval;
    result.estimated_fem_solve_count = opt_result.output.total_fem_solve_count_estimate;
    result.final_load_case_count = opt_result.output.final_load_case_count;
    result.final_all_cases_success = opt_result.output.final_all_cases_success;
    result.final_compliance_pass = opt_result.output.final_compliance_pass;

    summary_path = fullfile(output_dir, 'frame_v2_ga_15000_summary.json');
    writeTextFile(summary_path, jsonencode(result, PrettyPrint=true));
    result.summary_json_path = summary_path;

    fprintf('\n=== Frame V2 MATLAB GA result ===\n');
    fprintf('Actual funccount: %.0f, wall time: %.1f s, %.4f s/eval\n', ...
        result.actual_funccount, result.wall_time_s, result.seconds_per_funccount);
    fprintf('Final finite score rate: %.1f%%, valid individual rate: %.1f%%\n', ...
        100 * result.finite_score_rate, 100 * result.valid_individual_rate);
    fprintf('Best obj: %.6g\n', result.best_objective);
    fprintf('Final mass: %.2f kg, K: %.1f Nm/deg, cg_z: %.3f m\n', ...
        result.final_mass_kg, result.final_K_torsion_Nm_per_deg, result.final_cg_z_m);
    fprintf('Final topology: %d nodes, %d elements, compliance=%d\n', ...
        result.final_nodes, result.final_elements, result.compliance_pass);
    fprintf('Inner mode: %s, estimated FEM solves: %.0f, final cases: %d (all success=%d)\n', ...
        result.inner_load_case_mode, result.estimated_fem_solve_count, ...
        result.final_load_case_count, result.final_all_cases_success);
    fprintf('Saved PNG: %s\n', png_path);
    fprintf('Saved JSON: %s\n', summary_path);
end

function vehicle_artifact_path = prepareInputArtifact(opts, frame_root, output_dir)
    explicit_path = char(string(opts.VehicleParamsFile));
    if ~isempty(explicit_path)
        vehicle_artifact_path = explicit_path;
        return;
    end

    fixture_path = fullfile(frame_root, 'tests', 'fixtures', 'canonical_suspension_fixture.json');
    vehicle_artifact_path = fullfile(output_dir, 'frame_v2_ga_15000_vehicle_artifact.json');
    artifact = load_vehicle_artifact(fixture_path);
    artifact.params = merge_nested_structs(create_vehicle_baseline_template(), artifact.params);
    artifact.params.suspension.K_chassis = opts.TargetStiffnessNmPerDeg;
    save_vehicle_artifact(vehicle_artifact_path, artifact);
end

function renderFinalFrame(opt_result, catalog, png_path)
    fig = figure('Visible', 'off', 'Color', 'white', 'Position', [100, 100, 1400, 900]);
    ax = axes(fig);
    pipe_count = max(opt_result.elements_geom.group_indices);
    pipe_assignments = opt_result.best_x(1:min(pipe_count, numel(opt_result.best_x)));
    plot_frame(opt_result.nodes, opt_result.elements_geom, pipe_assignments, catalog, true);
    title(ax, sprintf('Frame V2 GA: %.2f kg, K %.0f Nm/deg, obj %.3g', ...
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

function writeTextFile(path, text)
    fid = fopen(path, 'w');
    if fid < 0
        error('run_frame_v2_targeted_ga_budget:writeFailed', 'Could not write %s', path);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', text);
end

function value = getFieldOrDefault(s, field_name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    end
end
