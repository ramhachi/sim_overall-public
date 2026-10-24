function result = run_canonical_frame_optimization(varargin)
% RUN_CANONICAL_FRAME_OPTIMIZATION Execute the canonical frame optimization flow.
%
% This is the non-interactive API behind frame_optimize/examples/run_frame_optimization.m
% and the EasyStart launcher. It resolves the authoritative vehicle artifact,
% requires suspension hardpoints, runs the mixed-integer GA, visualizes the
% result, and persists the updated frame data back into the unified vehicle
% artifact JSON.

    p = inputParser;
    addParameter(p, 'VehicleParamsFile', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MassTargetKg', [], @(x) isempty(x) || (isscalar(x) && isnumeric(x) && isfinite(x) && x > 0));
    addParameter(p, 'GenerateSolidWorksMacro', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'GenerateOnshape3DSketch', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'WaitForFigureClose', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ShowViewer', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'SaveFigure', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'DisplayProgress', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'PopulationSize', 60, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MaxGenerations', 40, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'UseParallel', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'UseFrameDesignV2', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'OptimizerMode', 'mixed_ga', @(x) ischar(x) || isstring(x));
    addParameter(p, 'RandomSeed', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x)));
    addParameter(p, 'StiffnessTargetNmPerDeg', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
    addParameter(p, 'StiffnessMinNmPerDeg', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
    addParameter(p, 'LoadCaseEvalMode', 'full', @(x) ischar(x) || isstring(x));
    addParameter(p, 'SuppressObjectiveOutput', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'SolverBackend', 'auto', @(x) ischar(x) || isstring(x));
    addParameter(p, 'FemServerLifecycle', 'managed_once', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MassPenaltyWeight', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
    addParameter(p, 'SparsityMaxTotalLengthM', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
    addParameter(p, 'SparsityMaxElements', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
    addParameter(p, 'SparsityPenaltyScale', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
    addParameter(p, 'FrameCgLimitM', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
    addParameter(p, 'StiffnessBelowPenaltyWeight', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
    addParameter(p, 'StiffnessHardPenaltyWeight', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
    addParameter(p, 'EnableRearOptionalEdges', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'RearBoxDiagonalMode', 'minimal', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    opts = p.Results;
    opts.OptimizerMode = char(string(opts.OptimizerMode));
    opts.LoadCaseEvalMode = char(string(opts.LoadCaseEvalMode));
    opts.SolverBackend = char(string(opts.SolverBackend));
    opts.FemServerLifecycle = char(string(opts.FemServerLifecycle));
    opts.RearBoxDiagonalMode = char(string(opts.RearBoxDiagonalMode));
    if ~isempty(opts.RandomSeed)
        rng(opts.RandomSeed);
    end

    displayProgress = logical(opts.DisplayProgress);
    if displayProgress
        fprintf('=== Frame Optimization (Rear Suspension Integrated) ===\n\n');
    end

    runtime = loadCanonicalRuntime(opts, displayProgress);
    [config, reg_params] = buildCanonicalConfig(runtime, opts, displayProgress);
    [lb, ub, intcon, custom_options] = buildCanonicalGaOptions(opts, runtime.catalog, config, displayProgress);

    cleanup_solver = prepareSolverBackend(config, displayProgress); %#ok<NASGU>

    if displayProgress
        fprintf('\n5. Running optimization (%s)...\n', opts.OptimizerMode);
    end
    if logical(opts.SuppressObjectiveOutput)
        objective_func = @(x) quietFrameObjectiveFunction(x, config, runtime.catalog, runtime.materials, reg_params);
    else
        objective_func = @(x) frameObjectiveFunction(x, [], [], [], config, runtime.catalog, runtime.materials, reg_params);
    end
    tic;
    [best_x, best_fval, exitflag, output, population, scores] = ...
        run_frame_optimizer_mode(objective_func, numel(lb), lb, ub, intcon, custom_options);
    elapsed = toc;
    if ~isfinite(best_fval)
        error('run_canonical_frame_optimization:InvalidBestObjective', ...
            'Optimizer returned a non-finite best objective (%g).', best_fval);
    end

    if displayProgress
        fprintf('\n   Optimization completed in %.1f seconds.\n', elapsed);
        fprintf('   Best objective: %.4f\n', best_fval);
        fprintf('\n6. Regenerating best solution...\n');
    end

    [nodes, elements_geom, final_cases] = regenerateBestSolution(best_x, config, runtime.geom, runtime.suspension_file);
    final_config = config;
    if isfield(final_config, 'load_case_eval_mode')
        final_config.load_case_eval_mode = 'full';
    end
    if logical(opts.SuppressObjectiveOutput)
        final_objective_func = @(x) quietFrameObjectiveFunction(x, final_config, runtime.catalog, runtime.materials, reg_params);
    else
        final_objective_func = @(x) frameObjectiveFunction(x, [], [], [], final_config, runtime.catalog, runtime.materials, reg_params);
    end
    [final_fval, final_results] = final_objective_func(best_x);
    if ~isfield(final_results, 'success') || ~final_results.success ...
            || ~isfinite(final_fval)
        error('run_canonical_frame_optimization:FinalEvaluationFailed', ...
            ['Final full-load-case evaluation failed for the selected design ' ...
             '(objective=%g). Refusing to overwrite the vehicle artifact.'], final_fval);
    end

    fig_handle = [];
    if logical(opts.ShowViewer) || logical(opts.SaveFigure) || logical(opts.WaitForFigureClose)
        if displayProgress
            fprintf('\n7. Visualizing results...\n');
        end
        fig_handle = renderFrameResult(nodes, elements_geom, best_x, runtime.catalog, output, custom_options, config, final_cases);
        if logical(opts.SaveFigure)
            png_path = fullfile(runtime.examples_dir, 'optimized_frame_result.png');
            saveas(fig_handle, png_path);
            if displayProgress
                fprintf('   Saved to %s\n', png_path);
            end
        end
    end

    macro_path = '';
    if logical(opts.GenerateSolidWorksMacro)
        sw_opts = struct();
        sw_opts.output_dir = fullfile(runtime.examples_dir, 'output');
        sw_opts.export_individual_bodies = true;
        sw_opts.body_export_dir = fullfile(runtime.examples_dir, 'output', 'pipe_bodies');
        macro_path = generate_solidworks_macro(nodes, elements_geom, best_x, runtime.catalog, runtime.materials, sw_opts);
        if displayProgress
            fprintf('   SolidWorks macro saved to: %s\n', macro_path);
            fprintf('   使い方: SolidWorks VBAエディタ > File > Import Module\n');
        end
    end

    onshape_sketch_path = '';
    if logical(opts.GenerateOnshape3DSketch)
        onshape_opts = struct();
        onshape_opts.output_dir = fullfile(runtime.examples_dir, 'output');
        onshape_sketch_path = generate_onshape_3d_sketch(nodes, elements_geom, onshape_opts);
        if displayProgress
            fprintf('   Onshape 3D sketch FeatureScript saved to: %s\n', onshape_sketch_path);
        end
    end

    optimization_output = output;
    optimization_output.exitflag = exitflag;
    optimization_output.elapsed_s = elapsed;
    optimization_output.final_population = population;
    optimization_output.final_scores = scores;
    optimization_output.optimizer_mode = custom_options.OptimizerMode;
    optimization_output.random_seed = opts.RandomSeed;
    optimization_output.solver_backend = config.solver_backend;
    optimization_output.fem_server_lifecycle = config.fem_server_lifecycle;

    inner_mode = 'full';
    if isfield(config, 'load_case_eval_mode')
        inner_mode = char(string(config.load_case_eval_mode));
    end
    if isfield(final_results, 'load_case_results')
        final_cases = final_results.load_case_results;
    else
        final_cases = struct([]);
    end
    final_case_count = numel(final_cases);
    inner_case_count = final_case_count;
    if strcmpi(inner_mode, 'torsion_only')
        inner_case_count = 1;
    end
    optimization_output.inner_load_case_mode = inner_mode;
    optimization_output.inner_fem_solves_per_eval = inner_case_count;
    optimization_output.estimated_inner_fem_solve_count = output.funccount * inner_case_count;
    optimization_output.final_load_case_count = final_case_count;
    optimization_output.total_fem_solve_count_estimate = ...
        optimization_output.estimated_inner_fem_solve_count + final_case_count;
    if final_case_count > 0
        optimization_output.final_all_cases_success = ...
            all([final_cases.success]);
    else
        optimization_output.final_all_cases_success = false;
    end
    if isfield(final_results, 'compliance_pass')
        optimization_output.final_compliance_pass = final_results.compliance_pass;
    else
        optimization_output.final_compliance_pass = false;
    end

    if displayProgress
        printFrameSummary(best_x, best_fval, runtime.catalog, elements_geom, final_results);
    end

    artifactSaveResult = save_frame_optimization_artifact( ...
        'BaseArtifactPath', runtime.base_vehicle_artifact_path, ...
        'OutputPath', runtime.unified_vehicle_json_path, ...
        'CurrentParams', runtime.current_vehicle_params, ...
        'BestDesignVector', best_x, ...
        'BestObjectiveValue', best_fval, ...
        'Catalog', runtime.catalog, ...
        'ElementsGeom', elements_geom, ...
        'Nodes', nodes, ...
        'FinalResults', final_results, ...
        'Config', config, ...
        'OptimizationOutput', optimization_output, ...
        'ScriptName', fullfile(runtime.frame_root, 'examples', 'run_frame_optimization.m'));

    if displayProgress
        fprintf('   Unified vehicle JSON saved to: %s\n', artifactSaveResult.output_path);
        fprintf('\n   Objective Score: %.4f\n', best_fval);
        printObjectiveBreakdown(best_fval, final_results, config, elements_geom);
    end

    result = struct();
    result.best_x = best_x;
    result.best_fval = best_fval;
    result.exitflag = exitflag;
    result.output = optimization_output;
    result.final_results = final_results;
    result.nodes = nodes;
    result.elements_geom = elements_geom;
    result.figure_handle = fig_handle;
    result.macro_path = macro_path;
    result.onshape_sketch_path = onshape_sketch_path;
    result.output_path = artifactSaveResult.output_path;

    if logical(opts.WaitForFigureClose) && ishghandle(fig_handle)
        if displayProgress
            fprintf('\n✓ Done. Close figure window to exit.\n');
        end
        uiwait(fig_handle);
    elseif displayProgress
        fprintf('\n✓ Done.\n');
    end
end

function runtime = loadCanonicalRuntime(opts, displayProgress)
    matlab_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(matlab_dir);
    project_root = fileparts(frame_root);
    data_dir = fullfile(frame_root, 'data');
    examples_dir = fullfile(frame_root, 'examples');

    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(fullfile(project_root, 'matlab'));
    end

    if displayProgress
        fprintf('1. Loading geometry, materials, and suspension...\n');
    end

    geom = jsondecode(fileread(fullfile(data_dir, 'real_vehicle_geometry.json')));
    materials = loadMaterials(fullfile(data_dir, 'materials.json'));
    catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

    fallback_vehicle_json_path = fullfile(project_root, 'output', 'runtime', 'frame_optimized_vehicle_artifact.json');
    artifact_resolution = resolve_authoritative_vehicle_json(project_root, ...
        'ExplicitPath', char(string(opts.VehicleParamsFile)), ...
        'RequireExportReady', true, ...
        'RequireSuspensionHardpoints', true, ...
        'RequiredHardpointCorners', {'FL', 'RL'}, ...
        'FallbackOutputPath', fallback_vehicle_json_path);
    suspension_file = resolve_suspension_geometry_source(project_root, artifact_resolution.path, ...
        'ErrorIfMissing', true);

    vp_K_target = [];
    vp_K_min = [];
    vehicle_artifact = artifact_resolution.artifact;
    current_vehicle_params = artifact_resolution.params;
    unified_vehicle_json_path = artifact_resolution.path;
    base_vehicle_artifact_path = '';

    if artifact_resolution.exists
        base_vehicle_artifact_path = artifact_resolution.path;
        if displayProgress
            fprintf('   Loading vehicle params from: %s [%s]\n', ...
                artifact_resolution.path, artifact_resolution.source);
        end
        p = vehicle_artifact.params;
        if isfield(p, 'mass')
            if isfield(p.mass, 'm_total')
                geom.dimensions.total_mass_kg = p.mass.m_total;
            end
            if isfield(p.mass, 'wheelbase')
                geom.dimensions.wheelbase_m = p.mass.wheelbase;
            end
            if isfield(p.mass, 'track_f')
                geom.dimensions.track_front_m = p.mass.track_f;
            end
            if isfield(p.mass, 'track_r')
                geom.dimensions.track_rear_m = p.mass.track_r;
            end
            if isfield(p.mass, 'h_cg')
                geom.dimensions.cg_height_m = p.mass.h_cg;
            end
        end
        if isfield(p, 'suspension') && isfield(p.suspension, 'K_chassis')
            vp_K_target = p.suspension.K_chassis;
            vp_K_min = vp_K_target * 0.5;
            if displayProgress
                fprintf('   K_chassis from params: %.1f Nm/deg\n', vp_K_target);
            end
        end
    end

    if isempty(fieldnames(current_vehicle_params))
        current_vehicle_params = create_vehicle_baseline_template();
    end
    current_vehicle_params.mass.m_total = geom.dimensions.total_mass_kg;
    current_vehicle_params.mass.wheelbase = geom.dimensions.wheelbase_m;
    current_vehicle_params.mass.track_f = geom.dimensions.track_front_m;
    current_vehicle_params.mass.track_r = geom.dimensions.track_rear_m;
    current_vehicle_params.mass.h_cg = geom.dimensions.cg_height_m;

    if displayProgress
        fprintf('   WB: %.3f m, Mass: %.0f kg, Pipes: %d\n', ...
            geom.dimensions.wheelbase_m, geom.dimensions.total_mass_kg, length(catalog));
    end

    diff_config = load_diff_mount_config(current_vehicle_params, geom);

    runtime = struct();
    runtime.project_root = project_root;
    runtime.frame_root = frame_root;
    runtime.examples_dir = examples_dir;
    runtime.geom = shiftGeometryExclusionZones(geom, 0.050);
    runtime.materials = materials;
    runtime.catalog = catalog;
    runtime.suspension_file = suspension_file;
    runtime.current_vehicle_params = current_vehicle_params;
    runtime.base_vehicle_artifact_path = base_vehicle_artifact_path;
    runtime.unified_vehicle_json_path = unified_vehicle_json_path;
    runtime.vp_K_target = vp_K_target;
    runtime.vp_K_min = vp_K_min;
    runtime.diff_config = diff_config;
end

function [config, reg_params] = buildCanonicalConfig(runtime, opts, displayProgress)
    reg_path = fullfile(runtime.frame_root, 'data', 'regulation_params.json');
    if isfile(reg_path)
        reg_params = jsondecode(fileread(reg_path));
    else
        reg_params = struct();
    end
    if displayProgress
        fprintf('\n2. Configuring optimization...\n');
    end

    config = struct();
    config.W_mass = 5.0;
    config.W_cg = 1.0;
    config.mass_ref = 22.0;
    config.cg_ref = 0.300;
    config.K_min = 500;
    config.K_target = 1200;
    if ~isempty(runtime.vp_K_target)
        config.K_target = runtime.vp_K_target;
        config.K_min = runtime.vp_K_min;
        if displayProgress
            fprintf('   Stiffness targets overridden: K_target=%.0f, K_min=%.0f Nm/deg\n', ...
                config.K_target, config.K_min);
        end
    end
    if ~isempty(opts.StiffnessTargetNmPerDeg)
        config.K_target = opts.StiffnessTargetNmPerDeg;
        if isempty(opts.StiffnessMinNmPerDeg)
            config.K_min = 0.5 * opts.StiffnessTargetNmPerDeg;
        else
            config.K_min = opts.StiffnessMinNmPerDeg;
        end
        if displayProgress
            fprintf('   Stiffness targets set by option: K_target=%.0f, K_min=%.0f Nm/deg\n', ...
                config.K_target, config.K_min);
        end
    end
    if logical(opts.UseFrameDesignV2)
        config.gamma_below = 1000.0;
        config.gamma_hard = 5000.0;
    else
        config.gamma_below = 8.0;
        config.gamma_hard = 50.0;
    end
    if ~isempty(opts.StiffnessBelowPenaltyWeight)
        config.gamma_below = opts.StiffnessBelowPenaltyWeight;
    end
    if ~isempty(opts.StiffnessHardPenaltyWeight)
        config.gamma_hard = opts.StiffnessHardPenaltyWeight;
    end
    config.compliance_penalty_scale = 1000;
    if ~isempty(opts.MassTargetKg)
        config.mass_target = opts.MassTargetKg;
        if isempty(opts.MassPenaltyWeight)
            if logical(opts.UseFrameDesignV2)
                config.gamma_mass = 1000.0;
            else
                config.gamma_mass = 25.0;
            end
        else
            config.gamma_mass = opts.MassPenaltyWeight;
        end
        if displayProgress
            fprintf('   → 目標重量: %.1f kg (gamma_mass=%.1f)\n', opts.MassTargetKg, config.gamma_mass);
        end
    elseif displayProgress
        fprintf('   → 目標重量ペナルティ: OFF\n');
    end
    config.joint_factor = 0.85;
    if logical(opts.UseFrameDesignV2)
        config.max_cg_z = 0.550;
        config.sparsity_penalty_scale = 200.0;
    else
        config.max_cg_z = 0.380;
        config.sparsity_penalty_scale = 1e5;
    end
    if ~isempty(opts.FrameCgLimitM)
        config.max_cg_z = opts.FrameCgLimitM;
    end
    if ~isempty(opts.SparsityPenaltyScale)
        config.sparsity_penalty_scale = opts.SparsityPenaltyScale;
    end
    config.sparsity_max_total_length_m = 45.0;
    if ~isempty(opts.SparsityMaxTotalLengthM)
        config.sparsity_max_total_length_m = opts.SparsityMaxTotalLengthM;
    end
    config.sparsity_max_elements = 120;
    if ~isempty(opts.SparsityMaxElements)
        config.sparsity_max_elements = opts.SparsityMaxElements;
    end
    config.require_rear_suspension_hardpoints = true;
    config.require_total_mass_kg = true;
    config.geom = runtime.geom;
    config.suspension_file = runtime.suspension_file;
    config.optimizer_mode = char(string(opts.OptimizerMode));
    config.random_seed = opts.RandomSeed;
    config.load_case_eval_mode = char(string(opts.LoadCaseEvalMode));
    [config.solver_backend, config.solver_backend_requested] = resolveSolverBackend(opts.SolverBackend, runtime.frame_root);
    config.fem_server_lifecycle = char(string(opts.FemServerLifecycle));
    config.design_schema_version = 'frame_design/v1';
    config.max_rear_free_pipe_length_m = 1.20;
    config.case_max_displacement_m = 0.025;
    config.diff_reaction_force_N = 1500;
    config.diff_reaction_torque_Nm = 500;
    config.enable_rear_optional_edges = logical(opts.EnableRearOptionalEdges);
    config.rear_box_diagonal_mode = char(string(opts.RearBoxDiagonalMode));
    config.invalid_design_penalty = 1e9;
    config.finite_rejection_penalty = logical(opts.UseFrameDesignV2) && strcmpi(config.optimizer_mode, 'matlab_ga');
    if logical(opts.UseFrameDesignV2)
        if ~isfield(runtime.diff_config, 'diff_mount_points') || isempty(runtime.diff_config.diff_mount_points)
            error('run_canonical_frame_optimization:MissingDiffMountConfig', ...
                ['UseFrameDesignV2 requires params.frame.diff_mount_points or ' ...
                 'params.powertrain.diff_mount_points in the vehicle artifact.']);
        end
        config.design_schema_version = 'frame_design/v2';
        config.design_schema = frame_design_schema_v2(numel(runtime.catalog));
        config.diff_mount_points = runtime.diff_config.diff_mount_points;
        config.diff_envelope = runtime.diff_config.diff_envelope;
        config.diff_config_source = runtime.diff_config.source;
        if displayProgress
            fprintf('   V2 design schema enabled (%d variables). Diff config: %s\n', ...
                config.design_schema.num_variables, config.diff_config_source);
        end
    end
    if displayProgress
        fprintf('   Solver backend: %s (requested: %s), FEM lifecycle: %s\n', ...
            config.solver_backend, config.solver_backend_requested, config.fem_server_lifecycle);
        fprintf('   Sparsity limits: %.1f m, %.0f elements; rear optional edges=%d, rear box=%s\n', ...
            config.sparsity_max_total_length_m, config.sparsity_max_elements, ...
            config.enable_rear_optional_edges, config.rear_box_diagonal_mode);
    end
    config.exclusion_zones = buildExclusionZoneCell(runtime.geom);
    config.exclusion_zones = appendCockpitExclusionZones(config.exclusion_zones, runtime.geom, runtime.suspension_file, displayProgress, config);
end

function [lb, ub, intcon, custom_options] = buildCanonicalGaOptions(opts, catalog, config, displayProgress)
    if isfield(config, 'design_schema_version') && strcmp(config.design_schema_version, 'frame_design/v2')
        schema = config.design_schema;
        lb = schema.lower_bounds;
        ub = schema.upper_bounds;
        intcon = schema.integer_indices;
        custom_options = struct();
        custom_options.PopulationSize = opts.PopulationSize;
        custom_options.MaxGenerations = opts.MaxGenerations;
        custom_options.UseParallel = logical(opts.UseParallel);
        custom_options.InitialGuess = schema.initial;
        custom_options.InitialPopulationSigma = 0.08;
        custom_options.InvalidDesignPenalty = getConfigValue(config, 'invalid_design_penalty', Inf);
        custom_options.OptimizerMode = char(string(opts.OptimizerMode));
        custom_options.RandomSeed = opts.RandomSeed;
        custom_options.GaDisplay = 'iter';

        if displayProgress
            fprintf('\n3. Setting up V2 optimizer variables...\n');
            fprintf('   Variables: %d, Integer: %d, PopSize: %d, MaxGen: %d, Parallel: %s\n', ...
                numel(lb), numel(intcon), custom_options.PopulationSize, custom_options.MaxGenerations, string(custom_options.UseParallel));
        end
        return;
    end

    num_vars = 18;
    lb = ones(1, num_vars);
    ub = length(catalog) * ones(1, num_vars);

    lb(6) = 0.50; ub(6) = 0.80;
    lb(7) = 0.25; ub(7) = 0.45;
    lb(8) = 0.35; ub(8) = 0.55;
    lb(9) = 0.25; ub(9) = 0.75;

    lb(10) = 0.40; ub(10) = 0.80;
    lb(11) = 0.40; ub(11) = 0.80;
    lb(12) = 0.40; ub(12) = 0.80;

    lb(13) = 0; ub(13) = 1;
    lb(14) = 0; ub(14) = 1;

    lb(15) = 0.2; ub(15) = 0.8;
    lb(16) = 0.2; ub(16) = 0.8;
    lb(17) = 0.2; ub(17) = 0.8;
    lb(18) = 0.2; ub(18) = 0.8;

    intcon = [1, 2, 3, 4, 5, 13, 14];
    z_floor = 0.050;
    custom_options = struct();
    custom_options.PopulationSize = opts.PopulationSize;
    custom_options.MaxGenerations = opts.MaxGenerations;
    custom_options.UseParallel = logical(opts.UseParallel);
    custom_options.OptimizerMode = char(string(opts.OptimizerMode));
    custom_options.RandomSeed = opts.RandomSeed;
    custom_options.GaDisplay = 'iter';
    custom_options.InitialGuess = [
        5, 5, 5, 5, 5, ...
        0.70, 0.35, 0.45 + z_floor, 0.35, ...
        0.6, 0.65, 0.6, ...
        1, 1, ...
        0.5, 0.5, 0.5, 0.5 ...
    ];

    if displayProgress
        fprintf('\n3. Setting up GA variables...\n');
        fprintf('   PopSize: %d, MaxGen: %d, Parallel: %s\n', ...
            custom_options.PopulationSize, custom_options.MaxGenerations, string(custom_options.UseParallel));
    end
end

function [nodes, elements_geom, final_cases] = regenerateBestSolution(best_x, config, geom, suspension_file)
    schema = struct('schema_version', 'frame_design/v1');
    if isfield(config, 'design_schema')
        schema = config.design_schema;
    end
    design = decode_frame_design_vector(best_x, schema);

    if strcmp(design.schema_version, 'frame_design/v2')
        [nodes, ~, ~, ~, ~, ~, ~, susp_idx, rear_susp_idx, meta] = ...
            generate_parametric_nodes_v2(geom, suspension_file, design, config);
        elements_geom = generate_adaptive_topology_v2(nodes, meta, config, design.distance_thresholds);
        node_config = buildNodeConfig(rear_susp_idx, susp_idx, geom, config, meta);
    else
        [nodes, ~, ~, ~, fh_x, mh_x, fbh_idx, susp_idx, rear_susp_idx] = ...
            generate_parametric_nodes(geom, suspension_file, design.shape_params, design.node_toggles, design.node_ratios);

        elements_geom = generate_adaptive_topology(nodes, fbh_idx, susp_idx, fh_x, mh_x, config, design.distance_thresholds, rear_susp_idx);
        node_config = buildNodeConfig(rear_susp_idx, susp_idx, geom, config);
    end
    final_cases = generate_load_cases(nodes, node_config.total_mass_kg, 9.81, node_config);
end

function fig_handle = renderFrameResult(nodes, elements_geom, best_x, catalog, output, ~, config, final_cases)
    fsae_fig = findall(0, 'Type', 'figure', 'Name', 'FSAE Frame Optimization Dashboard');
    if ~isempty(fsae_fig)
        close(fsae_fig);
    end

    pareto_results = struct();
    pareto_results.history_K = output.history_K;
    pareto_results.history_Obj = output.history;
    pareto_results.gen_history = 1:numel(pareto_results.history_Obj);

    stiff_targets = struct('K_target', config.K_target, 'K_min', config.K_min);
    pipe_count = max(elements_geom.group_indices);
    pipe_assignments = best_x(1:min(pipe_count, numel(best_x)));
    fsae_viewer(nodes, elements_geom, pipe_assignments, catalog, pareto_results, ...
        final_cases(1).bcs, final_cases(1).loads, stiff_targets);
    fig_handle = gcf;
end

function node_config = buildNodeConfig(rear_susp_idx, susp_idx, geom, config, meta)
    if nargin < 5
        meta = struct();
    end
    node_config = struct();

    if isempty(rear_susp_idx)
        if isfield(config, 'require_rear_suspension_hardpoints') && config.require_rear_suspension_hardpoints
            error('run_canonical_frame_optimization:MissingRearSuspensionHardpoints', ...
                'Rear suspension hardpoints are required for the canonical frame optimization flow.');
        end
        node_config.rear_lower_nodes = [9, 10];
        node_config.rear_upper_nodes = [13, 14];
    else
        node_config.rear_susp_left_nodes = rear_susp_idx([1, 3, 5, 7]);
        node_config.rear_susp_right_nodes = rear_susp_idx([2, 4, 6, 8]);
        node_config.rear_lower_nodes = [rear_susp_idx(1), rear_susp_idx(2)];
        node_config.rear_upper_nodes = [rear_susp_idx(3), rear_susp_idx(4)];
    end

    node_config.front_susp_left_nodes = susp_idx([1, 3, 5, 7]);
    node_config.front_susp_right_nodes = susp_idx([2, 4, 6, 8]);
    if isfield(meta, 'diff_mount_idx') && ~isempty(meta.diff_mount_idx)
        node_config.diff_mount_nodes = meta.diff_mount_idx;
        node_config.diff_reaction_force_N = getConfigValue(config, 'diff_reaction_force_N', 1500);
        node_config.diff_reaction_torque_Nm = getConfigValue(config, 'diff_reaction_torque_Nm', 500);
    end

    if isfield(geom, 'dimensions') && isfield(geom.dimensions, 'total_mass_kg')
        node_config.total_mass_kg = geom.dimensions.total_mass_kg;
    elseif isfield(config, 'require_total_mass_kg') && config.require_total_mass_kg
        error('run_canonical_frame_optimization:MissingVehicleMass', ...
            'geom.dimensions.total_mass_kg is required for the canonical frame optimization flow.');
    else
        node_config.total_mass_kg = 230;
    end
end

function zones = appendCockpitExclusionZones(zones, geom, suspension_file, displayProgress, config)
    if displayProgress
        fprintf('\n   Adding cockpit exclusion zones...\n');
    end

    if nargin >= 5 && isfield(config, 'design_schema_version') ...
            && strcmp(config.design_schema_version, 'frame_design/v2')
        [X_fbh_ref, W_fbh_ref, Z_fbh_min, Z_fbh_max, X_fh_ref, X_mh_ref, si_upper_z] = ...
            cockpitReferenceV2(geom, suspension_file, config);
    else
        z_floor = 0.050;
        [~, init_fbh, ~, ~, X_fh_ref, X_mh_ref] = ...
            generate_parametric_nodes(geom, suspension_file, [0.95, 0.35, 0.45 + z_floor, 0.35], [1, 1], [0.5, 0.5, 0.5, 0.5]);
        X_fbh_ref = mean(init_fbh(:, 1));
        W_fbh_ref = max(abs(init_fbh(:, 2)));
        Z_fbh_min = min(init_fbh(:, 3));
        Z_fbh_max = max(init_fbh(:, 3));
        si_upper_z = 0.45 + z_floor;
    end
    margin = 0.030;

    cockpit_lower = struct();
    cockpit_lower.type = 'AABB';
    cockpit_lower.min_bounds = [X_fbh_ref + margin, -(W_fbh_ref - margin), Z_fbh_min + margin];
    cockpit_lower.max_bounds = [X_fh_ref - margin, W_fbh_ref - margin, Z_fbh_max - margin];
    cockpit_lower.name = 'Cockpit Lower (FBH-FH Legs)';
    zones{end + 1} = cockpit_lower;

    cockpit_upper = struct();
    cockpit_upper.type = 'AABB';
    cockpit_upper.min_bounds = [X_fh_ref + margin, -(0.250 - margin), si_upper_z + margin];
    cockpit_upper.max_bounds = [X_mh_ref - margin, 0.250 - margin, 1.20];
    cockpit_upper.name = 'Cockpit Upper (FH-MH Driver)';
    zones{end + 1} = cockpit_upper;

    if displayProgress
        fprintf('     Zone: %s  X=[%.3f, %.3f] Y=±%.3f Z=[%.3f, %.3f]\n', ...
            cockpit_lower.name, cockpit_lower.min_bounds(1), cockpit_lower.max_bounds(1), ...
            cockpit_lower.max_bounds(2), cockpit_lower.min_bounds(3), cockpit_lower.max_bounds(3));
        fprintf('     Zone: %s  X=[%.3f, %.3f] Y=±%.3f Z=[%.3f, %.3f]\n', ...
            cockpit_upper.name, cockpit_upper.min_bounds(1), cockpit_upper.max_bounds(1), ...
            cockpit_upper.max_bounds(2), cockpit_upper.min_bounds(3), cockpit_upper.max_bounds(3));
        fprintf('   Total exclusion zones: %d\n', length(zones));
    end
end

function zones = buildExclusionZoneCell(geom)
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

function geom = shiftGeometryExclusionZones(geom, z_offset)
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

function printFrameSummary(best_x, best_fval, catalog, elements_geom, final_results)
    fprintf('\n=== Optimization Summary ===\n');
    fprintf('   Nodes:    %d\n', size(final_results.nodes, 1));
    fprintf('   Elements: %d\n', size(elements_geom.connectivity, 1));
    pipe_count = min(max(elements_geom.group_indices), numel(best_x));
    fprintf('   Pipes:    [%s]\n', strjoin(arrayfun(@(i) catalog(round(best_x(i))).id, 1:pipe_count, 'UniformOutput', false), ', '));

    if isfield(final_results, 'mass') && isfield(final_results, 'cg_z') && isfield(final_results, 'K_torsion')
        fprintf('\n   --- Performance Metrics ---\n');
        fprintf('   Mass:              %.2f kg\n', final_results.mass);
        fprintf('   CG Height (Z):     %.3f m\n', final_results.cg_z);
        fprintf('   Torsional Def.:    %.1f Nm/deg\n', final_results.K_torsion);
    end

    fprintf('\n   Objective Score: %.4f\n', best_fval);
end

function printObjectiveBreakdown(best_fval, final_results, config, elements_geom)
    fprintf('\n=== 目的関数の詳細分析 ===\n');
    if ~(isfield(final_results, 'f_mass') && isfield(final_results, 'f_cg'))
        return;
    end

    fprintf('\n--- 主要項の寄与度 ---\n');
    f_mass_contrib = config.W_mass * final_results.f_mass;
    f_cg_contrib = config.W_cg * final_results.f_cg;
    P_stiff_contrib = final_results.P_stiff;
    P_mass_target_contrib = final_results.P_mass_target;

    fprintf('W_mass × f_mass  = %.1f × %.4f = %.4f\n', config.W_mass, final_results.f_mass, f_mass_contrib);
    fprintf('W_cg × f_cg      = %.1f × %.4f = %.4f\n', config.W_cg, final_results.f_cg, f_cg_contrib);
    fprintf('P_stiff          = %.4f (K=%.1f Nm/deg)\n', P_stiff_contrib, final_results.K_torsion);
    fprintf('P_mass_target    = %.4f\n', P_mass_target_contrib);

    total_main = f_mass_contrib + f_cg_contrib + P_stiff_contrib + P_mass_target_contrib;
    fprintf('Subtotal (main)  = %.4f\n', total_main);

    fprintf('\n--- ペナルティ項の寄与度 ---\n');
    mat_pen = getPenaltyOrZero(final_results, 'material_penalty');
    geom_pen = getPenaltyOrZero(final_results, 'geometry_penalty');
    cg_pen = getPenaltyOrZero(final_results, 'cg_penalty');
    excl_pen = getPenaltyOrZero(final_results, 'exclusion_penalty');
    sparse_pen = getPenaltyOrZero(final_results, 'sparsity_penalty');
    compliance_pen = getPenaltyOrZero(final_results, 'compliance_penalty');
    case_pen = getPenaltyOrZero(final_results, 'case_displacement_penalty');
    precheck_pen = getPenaltyOrZero(final_results, 'geometry_precheck_penalty');

    fprintf('material_penalty = %.1f (禁止素材や厚さ違反)\n', mat_pen);
    fprintf('geometry_penalty = %.1f (パイプ長<150mm など)\n', geom_pen);
    fprintf('cg_penalty       = %.1f (CG高さ>%.3f m)\n', cg_pen, config.max_cg_z);
    fprintf('exclusion_penalty= %.1f (エンジンベイ衝突など)\n', excl_pen);
    fprintf('sparsity_penalty = %.1f (要素数>%d or 長さ>%.1fm)\n', ...
        sparse_pen, config.sparsity_max_elements, config.sparsity_max_total_length_m);
    fprintf('compliance_penalty= %.1f (FSAE規則違反)\n', compliance_pen);
    fprintf('case_displacement= %.1f (荷重ケース変位超過)\n', case_pen);
    fprintf('geometry_precheck= %.1f (事前形状チェック)\n', precheck_pen);

    total_penalties = mat_pen + geom_pen + cg_pen + excl_pen + sparse_pen ...
        + compliance_pen + case_pen + precheck_pen;
    fprintf('Total Penalties  = %.1f\n', total_penalties);

    fprintf('\n--- 最終集計 ---\n');
    fprintf('Main terms       = %.4f (%.2f%%)\n', total_main, 100 * total_main / best_fval);
    fprintf('Penalties        = %.1f (%.2f%%)\n', total_penalties, 100 * total_penalties / best_fval);
    fprintf('Total obj_val    = %.1f\n\n', best_fval);

    if total_penalties > total_main * 10
        fprintf('⚠️  WARNING: ペナルティが支配的です！目的関数のバランスが不適切です。\n');
        fprintf('  → 考えられる原因:\n');
        if cg_pen > 0.1 * total_penalties
            fprintf('    1. CG高さ制約がきつすぎる (max_cg_z=%.3f m)\n', config.max_cg_z);
        end
        if excl_pen > 0.1 * total_penalties
            fprintf('    2. 除外ゾーン制約が満たせない (エンジンベイ配置の問題)\n');
        end
        if sparse_pen > 0.1 * total_penalties
            fprintf('    3. 要素数が多すぎる (現在%d要素、上限%d)\n', ...
                size(elements_geom.connectivity, 1), config.sparsity_max_elements);
        end
    end
end

function value = getPenaltyOrZero(s, field_name)
    value = 0;
    if isfield(s, field_name)
        value = s.(field_name);
    end
end

function value = getConfigValue(s, field_name, default_value)
    value = default_value;
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        value = s.(field_name);
    end
end

function [obj_val, results] = quietFrameObjectiveFunction(x, config, catalog, materials, reg_params)
    runner = @() frameObjectiveFunction(x, [], [], [], config, catalog, materials, reg_params); %#ok<NASGU>
    [~, obj_val, results] = evalc('runner()');
end

function cleanup_solver = prepareSolverBackend(config, displayProgress)
    cleanup_solver = [];
    backend = char(string(getConfigValue(config, 'solver_backend', 'tcp_java')));
    lifecycle = char(string(getConfigValue(config, 'fem_server_lifecycle', 'managed_once')));

    if strcmpi(backend, 'mex')
        if displayProgress
            fprintf('\n4. FEM server skipped (native MEX backend).\n');
        end
        return;
    end

    if ~strcmpi(backend, 'tcp_java')
        error('run_canonical_frame_optimization:UnknownSolverBackend', ...
            'Unknown solver backend "%s". Use auto/mex/tcp_java.', backend);
    end

    if displayProgress
        fprintf('\n4. Preparing FEM server (%s)...\n', lifecycle);
    end

    switch lower(lifecycle)
        case 'external'
            if ~beam_fem_server_manager('check_alive')
                error('run_canonical_frame_optimization:FemServerNotAlive', ...
                    'FemServerLifecycle=external requires an already-running FEM server on the configured port.');
            end
        case 'reuse_if_alive'
            if ~beam_fem_server_manager('check_alive')
                beam_fem_solver_tcp_java('start');
            elseif displayProgress
                fprintf('   Reusing already-running FEM server.\n');
            end
        case 'managed_once'
            beam_fem_solver_tcp_java('start');
            cleanup_solver = onCleanup(@() stopCanonicalSolver());
        otherwise
            error('run_canonical_frame_optimization:UnknownFemServerLifecycle', ...
                'Unknown FemServerLifecycle "%s". Use managed_once/reuse_if_alive/external.', lifecycle);
    end
end

function [backend, requested] = resolveSolverBackend(requested, frame_root)
    requested = lower(char(string(requested)));
    if strcmp(requested, 'auto')
        if ensureNativeMexSolver(frame_root)
            backend = 'mex';
        else
            backend = 'tcp_java';
        end
        return;
    end

    switch requested
        case 'mex'
            if ~ensureNativeMexSolver(frame_root)
                error('run_canonical_frame_optimization:MissingMexSolver', ...
                    'SolverBackend=mex was requested, but a native beam_fem_solver MEX was not found.');
            end
            backend = 'mex';
        case 'tcp_java'
            backend = 'tcp_java';
        otherwise
            error('run_canonical_frame_optimization:UnknownSolverBackend', ...
                'Unknown SolverBackend "%s". Use auto/mex/tcp_java.', requested);
    end
end

function available = ensureNativeMexSolver(frame_root)
    mex_dir = fullfile(frame_root, 'mex');
    if exist(mex_dir, 'dir')
        addpath(mex_dir, '-begin');
    end
    solver_path = which('beam_fem_solver');
    available = ~isempty(solver_path) && contains(solver_path, ['.' mexext]);
end

function stopCanonicalSolver()
    try
        beam_fem_solver_tcp_java('stop');
    catch
    end
end

function [X_fbh, W_fbh, Z_fbh_min, Z_fbh_max, X_fh, X_mh, si_upper_z] = ...
        cockpitReferenceV2(geom, suspension_file, config)
    design = decode_frame_design_vector(config.design_schema.initial, config.design_schema);
    [nodes, ~, ~, ~, X_fh, X_mh, ~, ~, ~, meta] = ...
        generate_parametric_nodes_v2(geom, suspension_file, design, config);
    fbh = nodes(meta.fbh_idx, :);
    X_fbh = mean(fbh(:, 1));
    W_fbh = max(abs(fbh(:, 2)));
    Z_fbh_min = min(fbh(:, 3));
    Z_fbh_max = max(fbh(:, 3));
    si_upper_z = max(nodes([7, 8], 3));
end
