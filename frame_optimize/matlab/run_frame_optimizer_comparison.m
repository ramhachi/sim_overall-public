function report = run_frame_optimizer_comparison(fitnessfcn, nvars, lb, ub, intcon, options, modes, seeds, output_path)
% RUN_FRAME_OPTIMIZER_COMPARISON Compare optimizer modes on equal budgets.

    if nargin < 7 || isempty(modes)
        modes = {'mixed_ga', 'lhs_local', 'nsga2_like', 'local_refine'};
    end
    if nargin < 8 || isempty(seeds)
        seeds = [1, 2, 3];
    end
    if nargin < 9
        output_path = '';
    end

    rows = struct('mode', {}, 'seed', {}, 'best_objective', {}, ...
        'valid_individual_rate', {}, 'funccount', {}, 'generations', {});

    for mi = 1:numel(modes)
        for si = 1:numel(seeds)
            rng(seeds(si));
            run_options = options;
            run_options.OptimizerMode = modes{mi};
            [~, best_fval, ~, output, ~, scores] = ...
                run_frame_optimizer_mode(fitnessfcn, nvars, lb, ub, intcon, run_options);

            rows(end + 1) = struct( ... %#ok<AGROW>
                'mode', modes{mi}, ...
                'seed', seeds(si), ...
                'best_objective', best_fval, ...
                'valid_individual_rate', mean(isfinite(scores)), ...
                'funccount', getOutputValue(output, 'funccount', numel(scores)), ...
                'generations', getOutputValue(output, 'generations', NaN));
        end
    end

    report = struct();
    report.schema_version = 'frame_optimizer_comparison/v1';
    report.rows = rows;
    report.modes = modes;
    report.seeds = seeds;
    report.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    if ~isempty(output_path)
        writeComparisonMarkdown(report, output_path);
        report.output_path = output_path;
    end
end

function value = getOutputValue(output, field_name, default_value)
    value = default_value;
    if isstruct(output) && isfield(output, field_name) && ~isempty(output.(field_name))
        value = output.(field_name);
    end
end

function writeComparisonMarkdown(report, output_path)
    [out_dir, ~, ~] = fileparts(output_path);
    if ~isempty(out_dir) && ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    fid = fopen(output_path, 'w');
    if fid < 0
        error('run_frame_optimizer_comparison:cannotWrite', ...
            'Could not write optimizer comparison report: %s', output_path);
    end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, '# Frame Optimizer Comparison\n\n');
    fprintf(fid, '- schema_version: `%s`\n', report.schema_version);
    fprintf(fid, '- generated_at: `%s`\n\n', report.generated_at);
    fprintf(fid, '| mode | seed | best objective | valid rate | evaluations | generations |\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|\n');
    for i = 1:numel(report.rows)
        r = report.rows(i);
        fprintf(fid, '| `%s` | %d | %.6g | %.3f | %d | %.0f |\n', ...
            r.mode, r.seed, r.best_objective, r.valid_individual_rate, r.funccount, r.generations);
    end
end
