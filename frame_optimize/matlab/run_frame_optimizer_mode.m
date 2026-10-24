function [best_x, best_fval, exitflag, output, population, scores] = run_frame_optimizer_mode(fitnessfcn, nvars, lb, ub, intcon, options)
% RUN_FRAME_OPTIMIZER_MODE Dispatch frame optimization among comparable modes.
%
% Modes:
%   mixed_ga     - Existing custom mixed integer GA (default)
%   lhs_local    - Latin-hypercube-style sampling followed by local perturbation
%   nsga2_like   - Budget-compatible diversity sampler with scalarized objective
%   local_refine - Initial-guess centered local perturbation search
%   matlab_ga    - Optimization Toolbox genetic algorithm

    if nargin < 6 || isempty(options)
        options = struct();
    end
    mode = 'mixed_ga';
    if isfield(options, 'OptimizerMode') && ~isempty(options.OptimizerMode)
        mode = char(string(options.OptimizerMode));
    end

    switch lower(mode)
        case 'mixed_ga'
            [best_x, best_fval, exitflag, output, population, scores] = ...
                custom_mixed_ga(fitnessfcn, nvars, lb, ub, intcon, options);
            output.optimizer_mode = 'mixed_ga';
            output.valid_individual_rate = mean(isfinite(scores));
        case 'lhs_local'
            [best_x, best_fval, exitflag, output, population, scores] = ...
                runSampleAndRefine(fitnessfcn, nvars, lb, ub, intcon, options, 'lhs_local', 0.55);
        case 'nsga2_like'
            [best_x, best_fval, exitflag, output, population, scores] = ...
                runSampleAndRefine(fitnessfcn, nvars, lb, ub, intcon, options, 'nsga2_like', 0.75);
            output.mode_note = 'Scalarized objective with diversity-preserving sampling; full Pareto ranking is a later step.';
        case 'local_refine'
            [best_x, best_fval, exitflag, output, population, scores] = ...
                runLocalRefine(fitnessfcn, nvars, lb, ub, intcon, options);
        case 'matlab_ga'
            [best_x, best_fval, exitflag, output, population, scores] = ...
                runMatlabGa(fitnessfcn, nvars, lb, ub, intcon, options);
        otherwise
            error('run_frame_optimizer_mode:UnknownMode', ...
                'Unknown optimizer_mode "%s". Use mixed_ga/lhs_local/nsga2_like/local_refine/matlab_ga.', mode);
    end

    if ~isfield(output, 'funccount')
        output.funccount = numel(scores);
    end
    output.objective_funccount = output.funccount;
end

function [best_x, best_fval, exitflag, output, population, scores] = runMatlabGa(fitnessfcn, nvars, lb, ub, intcon, options)
    if exist('ga', 'file') ~= 2
        error('run_frame_optimizer_mode:MissingGa', ...
            'MATLAB ga() was not found. Install Optimization Toolbox or use mixed_ga.');
    end

    pop_size = getOption(options, 'PopulationSize', 100);
    max_gen = getOption(options, 'MaxGenerations', 149);
    use_parallel = getOption(options, 'UseParallel', false);
    display_mode = getOption(options, 'GaDisplay', 'iter');
    initial_guess = [];
    if isfield(options, 'InitialGuess') && ~isempty(options.InitialGuess)
        initial_guess = clampCandidate(options.InitialGuess, lb, ub, intcon);
    end

    history = [];
    best_history = [];
    ga_options = optimoptions('ga', ...
        'PopulationSize', pop_size, ...
        'MaxGenerations', max_gen, ...
        'UseParallel', logical(use_parallel), ...
        'Display', char(string(display_mode)), ...
        'FunctionTolerance', getOption(options, 'FunctionTolerance', 0), ...
        'MaxStallGenerations', getOption(options, 'MaxStallGenerations', max_gen), ...
        'EliteCount', max(1, ceil(0.05 * pop_size)), ...
        'OutputFcn', @recordGaProgress);

    if ~isempty(initial_guess)
        sigma = getOption(options, 'InitialPopulationSigma', 0.08);
        initial_population = initial_guess;
        if pop_size > 1
            initial_population = [initial_guess; ...
                perturbAround(initial_guess, pop_size - 1, lb, ub, intcon, sigma)];
        end
        ga_options = optimoptions(ga_options, 'InitialPopulationMatrix', initial_population);
    end

    [best_x, best_fval, exitflag, ga_output, population, scores] = ...
        ga(fitnessfcn, nvars, [], [], [], [], lb, ub, [], intcon, ga_options);

    output = ga_output;
    output.optimizer_mode = 'matlab_ga';
    output.history = best_history(:);
    output.history_K = NaN(size(output.history));
    output.generation_history = history(:);
    output.final_scores = scores;
    output.finite_score_rate = mean(isfinite(scores));
    invalid_threshold = getOption(options, 'InvalidDesignPenalty', Inf);
    if isfinite(invalid_threshold)
        output.valid_individual_rate = mean(isfinite(scores) & scores < invalid_threshold);
    else
        output.valid_individual_rate = output.finite_score_rate;
    end
    if ~isfield(output, 'funccount')
        output.funccount = numel(scores);
    end

    function [state, options_out, optchanged] = recordGaProgress(options_in, state, flag)
        options_out = options_in;
        optchanged = false;
        switch flag
            case {'init', 'iter', 'done'}
                history(end + 1, 1) = state.Generation;
                if isfield(state, 'Best') && ~isempty(state.Best)
                    best_history(end + 1, 1) = min(state.Best);
                elseif isfield(state, 'Score') && ~isempty(state.Score)
                    best_history(end + 1, 1) = min(state.Score);
                else
                    best_history(end + 1, 1) = NaN;
                end
        end
    end
end

function [best_x, best_fval, exitflag, output, population, scores] = runSampleAndRefine(fitnessfcn, nvars, lb, ub, intcon, options, mode, sample_fraction)
    budget = evaluationBudget(options);
    sample_count = max(1, round(budget * sample_fraction));
    refine_count = max(0, budget - sample_count);
    use_parallel = getOption(options, 'UseParallel', true);

    sample_pop = makeStratifiedPopulation(sample_count, nvars, lb, ub, intcon);
    if isfield(options, 'InitialGuess') && ~isempty(options.InitialGuess)
        sample_pop(1, :) = clampCandidate(options.InitialGuess, lb, ub, intcon);
    end
    sample_scores = evaluatePopulation(sample_pop, fitnessfcn, use_parallel);

    [best_fval, best_idx] = min(sample_scores);
    best_x = sample_pop(best_idx, :);
    population = sample_pop;
    scores = sample_scores;
    history = best_fval;
    history_K = extractBestStiffness(fitnessfcn, best_x);

    sigma0 = 0.12;
    batch_size = max(1, getOption(options, 'PopulationSize', 20));
    used = sample_count;
    while used < budget && refine_count > 0
        n_batch = min(batch_size, budget - used);
        sigma = sigma0 * max(0.15, 1 - used / budget);
        trial_pop = perturbAround(best_x, n_batch, lb, ub, intcon, sigma);
        trial_scores = evaluatePopulation(trial_pop, fitnessfcn, use_parallel);
        [trial_best, trial_idx] = min(trial_scores);
        if trial_best < best_fval
            best_fval = trial_best;
            best_x = trial_pop(trial_idx, :);
        end
        population = [population; trial_pop]; %#ok<AGROW>
        scores = [scores; trial_scores]; %#ok<AGROW>
        used = used + n_batch;
        history(end + 1, 1) = best_fval; %#ok<AGROW>
        history_K(end + 1, 1) = extractBestStiffness(fitnessfcn, best_x); %#ok<AGROW>
    end

    exitflag = 1;
    output = struct();
    output.optimizer_mode = mode;
    output.generations = numel(history);
    output.funccount = used + numel(history_K);
    output.history = history;
    output.history_K = history_K;
    output.valid_individual_rate = mean(isfinite(scores));
    output.final_scores = scores;
end

function [best_x, best_fval, exitflag, output, population, scores] = runLocalRefine(fitnessfcn, nvars, lb, ub, intcon, options)
    budget = evaluationBudget(options);
    use_parallel = getOption(options, 'UseParallel', true);
    if isfield(options, 'InitialGuess') && ~isempty(options.InitialGuess)
        best_x = clampCandidate(options.InitialGuess, lb, ub, intcon);
    else
        best_x = randomCandidate(nvars, lb, ub, intcon);
    end
    best_fval = fitnessfcn(best_x);
    population = best_x;
    scores = best_fval;
    history = best_fval;
    history_K = extractBestStiffness(fitnessfcn, best_x);
    used = 1;
    batch_size = max(1, getOption(options, 'PopulationSize', 20));

    while used < budget
        n_batch = min(batch_size, budget - used);
        sigma = 0.18 * max(0.10, 1 - used / budget);
        trial_pop = perturbAround(best_x, n_batch, lb, ub, intcon, sigma);
        trial_scores = evaluatePopulation(trial_pop, fitnessfcn, use_parallel);
        [trial_best, trial_idx] = min(trial_scores);
        if trial_best < best_fval
            best_fval = trial_best;
            best_x = trial_pop(trial_idx, :);
        end
        population = [population; trial_pop]; %#ok<AGROW>
        scores = [scores; trial_scores]; %#ok<AGROW>
        used = used + n_batch;
        history(end + 1, 1) = best_fval; %#ok<AGROW>
        history_K(end + 1, 1) = extractBestStiffness(fitnessfcn, best_x); %#ok<AGROW>
    end

    exitflag = 1;
    output = struct();
    output.optimizer_mode = 'local_refine';
    output.generations = numel(history);
    output.funccount = used + numel(history_K);
    output.history = history;
    output.history_K = history_K;
    output.valid_individual_rate = mean(isfinite(scores));
    output.final_scores = scores;
end

function budget = evaluationBudget(options)
    pop_size = getOption(options, 'PopulationSize', 20);
    max_gen = getOption(options, 'MaxGenerations', 10);
    budget = max(1, pop_size * (max_gen + 1));
end

function pop = makeStratifiedPopulation(count, nvars, lb, ub, intcon)
    pop = zeros(count, nvars);
    for j = 1:nvars
        perm = randperm(count);
        u = ((perm(:) - rand(count, 1)) / count)';
        pop(:, j) = lb(j) + (ub(j) - lb(j)) * u(:);
    end
    pop = clampPopulation(pop, lb, ub, intcon);
end

function pop = perturbAround(parent, count, lb, ub, intcon, sigma_fraction)
    nvars = numel(parent);
    pop = repmat(parent, count, 1);
    for i = 1:count
        for j = 1:nvars
            range_j = ub(j) - lb(j);
            if ismember(j, intcon)
                if rand < 0.35
                    pop(i, j) = pop(i, j) + sign(rand - 0.5);
                end
            else
                pop(i, j) = pop(i, j) + randn * sigma_fraction * range_j;
            end
        end
    end
    pop = clampPopulation(pop, lb, ub, intcon);
end

function x = randomCandidate(nvars, lb, ub, intcon)
    x = lb + (ub - lb) .* rand(1, nvars);
    x = clampCandidate(x, lb, ub, intcon);
end

function pop = clampPopulation(pop, lb, ub, intcon)
    for i = 1:size(pop, 1)
        pop(i, :) = clampCandidate(pop(i, :), lb, ub, intcon);
    end
end

function x = clampCandidate(x, lb, ub, intcon)
    x = reshape(x, 1, []);
    x = min(max(x, lb), ub);
    x(intcon) = round(x(intcon));
end

function scores = evaluatePopulation(pop, fitnessfcn, use_parallel)
    scores = zeros(size(pop, 1), 1);
    if use_parallel
        parfor i = 1:size(pop, 1)
            scores(i) = fitnessfcn(pop(i, :));
        end
    else
        for i = 1:size(pop, 1)
            scores(i) = fitnessfcn(pop(i, :));
        end
    end
end

function K = extractBestStiffness(fitnessfcn, x)
    K = NaN;
    try
        [~, res] = fitnessfcn(x);
        if isfield(res, 'K_torsion')
            K = res.K_torsion;
        end
    catch
    end
end

function value = getOption(options, field_name, default_value)
    value = default_value;
    if isstruct(options) && isfield(options, field_name) && ~isempty(options.(field_name))
        value = options.(field_name);
    end
end
