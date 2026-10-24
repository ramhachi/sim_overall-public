function [best_x, best_fval, history] = runFrameOptimization(nodes, elements_geom, bcs, loads, config, catalog, materials, reg_params)
% RUNFRAMEOPTIMIZATION Legacy compatibility wrapper for the canonical frame flow.
%
% Canonical entrypoint:
%   frame_optimize/examples/run_frame_optimization.m
%
% Legacy fixed-node inputs are no longer accepted here because the canonical
% parametric SSOT flow ignores them. Callers must either:
%   - switch to frame_optimize/examples/run_frame_optimization.m, or
%   - call this wrapper only with [] placeholders for the legacy geometry inputs.

    %#ok<*INUSD>
    validateLegacyContract(nodes, elements_geom, bcs, loads);

    warning('runFrameOptimization:LegacyCompatibility', ...
        ['runFrameOptimization is a compatibility wrapper. ', ...
         'Use frame_optimize/examples/run_frame_optimization.m for the canonical SSOT flow.']);

    fprintf('--- Starting FSAE Frame Optimization (compatibility wrapper) ---\n');

    runtime_config = normalizeLegacyFrameConfig(config);
    [lb, ub, intcon, ga_options] = buildCanonicalGaSetup(runtime_config, catalog);
    objective_func = @(x) frameObjectiveFunction(x, [], [], [], runtime_config, catalog, materials, reg_params);

    fprintf('Delegating to canonical parametric objective (18 vars, %d catalog entries)...\n', length(catalog));

    server_started = false;
    try
        beam_fem_solver_tcp_java('start');
        server_started = true;
    catch ME
        warning('runFrameOptimization:SolverStartFailed', ...
            'Could not pre-start FEM server (%s). Proceeding anyway.', ME.message);
    end
    cleanup_obj = onCleanup(@() stopSolverIfStarted(server_started)); %#ok<NASGU>

    tic;
    [best_x, best_fval, exitflag, output, population, scores] = ...
        custom_mixed_ga(objective_func, numel(lb), lb, ub, intcon, ga_options);
    elapsed = toc;

    history = struct();
    history.exitflag = exitflag;
    history.generations = output.generations;
    history.fun_evals = NaN;
    history.final_population = population;
    history.final_scores = scores;
    history.time_s = elapsed;
    history.canonical_entrypoint = 'frame_optimize/examples/run_frame_optimization.m';
    history.compatibility_wrapper = true;

    fprintf('\n--- Optimization Complete ---\n');
    fprintf('Elapsed time: %.2f seconds.\n', elapsed);
    fprintf('Best Objective Function Value: %.4f\n', best_fval);
    disp('Best design vector:');
    disp(best_x);
end

function runtime_config = normalizeLegacyFrameConfig(config)
    runtime_config = config;
    matlab_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(matlab_dir);
    project_root = fileparts(frame_root);
    data_dir = fullfile(frame_root, 'data');
    loaded_default_geom = false;

    if ~isfield(runtime_config, 'geom') || isempty(runtime_config.geom)
        geom_path = fullfile(data_dir, 'real_vehicle_geometry.json');
        if ~isfile(geom_path)
            error('runFrameOptimization:MissingGeometry', ...
                'Missing %s. Use the canonical frame flow instead.', geom_path);
        end
        runtime_config.geom = jsondecode(fileread(geom_path));
        loaded_default_geom = true;
    end

    if loaded_default_geom && isfield(runtime_config.geom, 'exclusion_zones')
        runtime_config.geom.exclusion_zones = shiftGeometryExclusionZones(runtime_config.geom.exclusion_zones, 0.050);
    end

    if ~isfield(runtime_config, 'suspension_file') || isempty(runtime_config.suspension_file)
        runtime_config.suspension_file = resolve_suspension_geometry_source(project_root, '', ...
            'ErrorIfMissing', true);
    end

    runtime_config = withDefault(runtime_config, 'W_mass', 5.0);
    runtime_config = withDefault(runtime_config, 'W_cg', 1.0);
    runtime_config = withDefault(runtime_config, 'mass_ref', 22.0);
    runtime_config = withDefault(runtime_config, 'cg_ref', 0.300);
    runtime_config = withDefault(runtime_config, 'K_min', 500);
    runtime_config = withDefault(runtime_config, 'K_target', 1200);
    runtime_config = withDefault(runtime_config, 'gamma_below', 8.0);
    runtime_config = withDefault(runtime_config, 'gamma_hard', 50.0);
    runtime_config = withDefault(runtime_config, 'max_cg_z', 0.380);
    runtime_config = withDefault(runtime_config, 'sparsity_max_total_length_m', 45.0);
    runtime_config = withDefault(runtime_config, 'sparsity_max_elements', 120);
    runtime_config = withDefault(runtime_config, 'require_rear_suspension_hardpoints', true);
    runtime_config = withDefault(runtime_config, 'require_total_mass_kg', true);

    if ~isfield(runtime_config, 'exclusion_zones') || isempty(runtime_config.exclusion_zones)
        runtime_config.exclusion_zones = buildExclusionZoneCell(runtime_config.geom);
    end
end

function validateLegacyContract(nodes, elements_geom, bcs, loads)
    if ~isempty(nodes) || ~isempty(elements_geom) || ~isempty(bcs) || ~isempty(loads)
        error('runFrameOptimization:LegacyInputsUnsupported', ...
            ['runFrameOptimization no longer accepts legacy fixed-geometry inputs because ', ...
             'the canonical SSOT optimizer ignores nodes/elements_geom/bcs/loads. ', ...
             'Update the caller to use frame_optimize/examples/run_frame_optimization.m ', ...
             'or pass [] for the legacy geometry arguments when intentionally delegating ', ...
             'to the canonical parametric flow.']);
    end
end

function [lb, ub, intcon, ga_options] = buildCanonicalGaSetup(config, catalog)
    num_vars = 18;
    seed_pipe = min(5, max(1, length(catalog)));
    z_floor = 0.050;

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

    ga_options = struct();
    ga_options.PopulationSize = getConfigValue(config, 'pop_size', 60);
    ga_options.MaxGenerations = getConfigValue(config, 'max_generations', 40);
    ga_options.UseParallel = getConfigValue(config, 'use_parallel', false);
    ga_options.InitialGuess = [
        seed_pipe, seed_pipe, seed_pipe, seed_pipe, seed_pipe, ...
        0.70, 0.35, 0.45 + z_floor, 0.35, ...
        0.6, 0.65, 0.6, ...
        1, 1, ...
        0.5, 0.5, 0.5, 0.5 ...
    ];
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

function shifted_zones = shiftGeometryExclusionZones(zones, z_offset)
    shifted_zones = zones;

    for i = 1:length(shifted_zones)
        if isfield(shifted_zones(i), 'min') && isfield(shifted_zones(i), 'max')
            shifted_zones(i).min(3) = shifted_zones(i).min(3) + z_offset;
            shifted_zones(i).max(3) = shifted_zones(i).max(3) + z_offset;
        end
    end
end

function config = withDefault(config, field_name, default_value)
    if ~isfield(config, field_name) || isempty(config.(field_name))
        config.(field_name) = default_value;
    end
end

function value = getConfigValue(config, field_name, default_value)
    if isfield(config, field_name) && ~isempty(config.(field_name))
        value = config.(field_name);
    else
        value = default_value;
    end
end

function stopSolverIfStarted(server_started)
    if ~server_started
        return;
    end

    try
        beam_fem_solver_tcp_java('stop');
    catch
    end
end
