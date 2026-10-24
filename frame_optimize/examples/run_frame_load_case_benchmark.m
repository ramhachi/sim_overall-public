function result = run_frame_load_case_benchmark(varargin)
% RUN_FRAME_LOAD_CASE_BENCHMARK Compare full vs torsion-only inner load cases.
%
% The final candidate is always re-evaluated with all load cases by
% run_canonical_frame_optimization. Requires an already-running FEM server.
%
% Example:
%   run_frame_load_case_benchmark('VehicleParamsFile', '/path/to/vehicle.json', ...
%       'PopulationSize', 8, 'MaxGenerations', 3)

    p = inputParser;
    addParameter(p, 'VehicleParamsFile', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'PopulationSize', 8, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MaxGenerations', 3, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'OptimizerMode', 'local_refine', @(x) ischar(x) || isstring(x));
    addParameter(p, 'RandomSeed', 42, @(x) isnumeric(x) && isscalar(x) && isfinite(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'DisplayProgress', false, @(x) islogical(x) || isnumeric(x));
    parse(p, varargin{:});
    opts = p.Results;

    script_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(script_dir);
    project_root = fileparts(frame_root);
    addpath(fullfile(frame_root, 'matlab'));
    addpath(fullfile(project_root, 'matlab'));

    input_path = char(string(opts.VehicleParamsFile));
    if isempty(input_path) || ~isfile(input_path)
        error('run_frame_load_case_benchmark:MissingVehicleParams', ...
            'VehicleParamsFile must point to an export-ready vehicle artifact.');
    end
    if isempty(char(string(opts.OutputDir)))
        output_dir = fullfile(project_root, 'output', 'runtime', 'frame_load_case_benchmark');
    else
        output_dir = char(string(opts.OutputDir));
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    modes = {'full', 'torsion_only'};
    runs = struct('mode', {}, 'wall_time_s', {}, 'funccount', {}, ...
        'estimated_fem_solve_count', {}, 'final_load_case_count', {}, ...
        'final_all_cases_success', {}, 'final_compliance_pass', {}, ...
        'final_mass_kg', {}, 'final_K_torsion_Nm_per_deg', {}, 'objective', {}, ...
        'artifact_path', {});
    for i = 1:numel(modes)
        mode = modes{i};
        artifact_path = fullfile(output_dir, ['benchmark_' mode '_vehicle.json']);
        copyfile(input_path, artifact_path);
        fprintf('\n[%s] inner load cases: %s\n', mode, mode);
        tic;
        run_result = run_canonical_frame_optimization( ...
            'VehicleParamsFile', artifact_path, ...
            'UseFrameDesignV2', true, ...
            'OptimizerMode', char(string(opts.OptimizerMode)), ...
            'PopulationSize', opts.PopulationSize, ...
            'MaxGenerations', opts.MaxGenerations, ...
            'UseParallel', false, ...
            'ShowViewer', false, ...
            'SaveFigure', false, ...
            'WaitForFigureClose', false, ...
            'DisplayProgress', logical(opts.DisplayProgress), ...
            'RandomSeed', opts.RandomSeed, ...
            'LoadCaseEvalMode', mode, ...
            'SolverBackend', 'tcp_java', ...
            'FemServerLifecycle', 'external');
        wall_time_s = toc;

        runs(i).mode = mode;
        runs(i).wall_time_s = wall_time_s;
        runs(i).funccount = run_result.output.funccount;
        runs(i).estimated_fem_solve_count = ...
            run_result.output.total_fem_solve_count_estimate;
        runs(i).final_load_case_count = run_result.output.final_load_case_count;
        runs(i).final_all_cases_success = run_result.output.final_all_cases_success;
        runs(i).final_compliance_pass = run_result.output.final_compliance_pass;
        runs(i).final_mass_kg = run_result.final_results.mass;
        runs(i).final_K_torsion_Nm_per_deg = run_result.final_results.K_torsion;
        runs(i).objective = run_result.best_fval;
        runs(i).artifact_path = run_result.output_path;
    end

    full_idx = find(strcmp({runs.mode}, 'full'), 1);
    staged_idx = find(strcmp({runs.mode}, 'torsion_only'), 1);
    result = struct();
    result.runs = runs;
    result.speedup = runs(full_idx).wall_time_s / max(runs(staged_idx).wall_time_s, eps);
    result.fem_solve_reduction = ...
        runs(full_idx).estimated_fem_solve_count / ...
        max(runs(staged_idx).estimated_fem_solve_count, 1);
    result.staged_final_full_case_success = runs(staged_idx).final_all_cases_success;
    result.staged_final_compliance_pass = runs(staged_idx).final_compliance_pass;

    summary_path = fullfile(output_dir, 'frame_load_case_benchmark_summary.json');
    fid = fopen(summary_path, 'w');
    if fid < 0
        error('run_frame_load_case_benchmark:WriteFailed', 'Cannot write %s', summary_path);
    end
    fwrite(fid, jsonencode(result, 'PrettyPrint', true));
    fclose(fid);
    result.summary_path = summary_path;

    fprintf('\n=== Load-case benchmark ===\n');
    fprintf('Speedup (full/staged): %.2fx\n', result.speedup);
    fprintf('FEM solve estimate reduction: %.2fx\n', result.fem_solve_reduction);
    fprintf('Staged final full-case success: %d, compliance: %d\n', ...
        result.staged_final_full_case_success, result.staged_final_compliance_pass);
    fprintf('Summary: %s\n', summary_path);
end
