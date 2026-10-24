function [best_x, best_fval, exitflag, output, population, scores] = custom_mixed_ga(fitnessfcn, nvars, lb, ub, intcon, options)
% CUSTOM_MIXED_GA Specialized Mixed-Integer Genetic Algorithm
% Implements Elitism, Local Search (Perturbation), and Random Exploration.
%
% Variables:
%   x(intcon) - Discrete integer values (pipe catalog indices)
%   x(others) - Continuous values (geometry parameters)

    pop_size = options.PopulationSize;
    max_gen = options.MaxGenerations;
    if isfield(options, 'UseParallel')
        use_parallel = options.UseParallel;
    else
        use_parallel = true; % Default to true for speed
    end
    
    if isfield(options, 'InitialGuess')
        initial_guess = options.InitialGuess;
    else
        initial_guess = [];
    end
    
    % Strategy Ratios
    elite_ratio = 0.15;      % Top 15% kept as is
    perturb_ratio = 0.60;    % 60% are "evolved" from elites via local search
    % Remaining ~25% are completely random (Exploration)
    
    elite_count = min(pop_size, max(1, round(elite_ratio * pop_size)));
    perturb_count = min(pop_size - elite_count, max(0, round(perturb_ratio * pop_size)));
    random_count = max(0, pop_size - elite_count - perturb_count);
    
    % Local Search Params
    perturb_sigma = 0.15; % 15% of range for Gaussian perturbation
    
    fprintf('Initializing Custom Mixed GA (Pop: %d, Gens: %d)...\n', pop_size, max_gen);
    
    % 1. Initial Population - Batch generation with discard logic
    population = zeros(pop_size, nvars);
    scores = inf(pop_size, 1);
    evaluation_count = 0;
    
    surplus_factor = 10; % Increased from 3
    max_retries = 100;   % Increased from 20
    retry_count = 0;
    while any(isinf(scores))
        retry_count = retry_count + 1;
        if retry_count > max_retries
            error('custom_mixed_ga:InitializationFailed', ...
                'Failed to find valid initial population after %d attempts. Check FEM server logs.', max_retries);
        end
        
        needed = sum(isinf(scores));
        candidate_pop = zeros(needed * surplus_factor, nvars);
        
        for i = 1:(needed * surplus_factor)
            % If we have an initial guess and it's the first few retries, 
            % bias the generation towards the guess.
            if ~isempty(initial_guess) && retry_count == 1 && i == 1
                candidate_pop(i, :) = max(lb, min(ub, initial_guess));
                candidate_pop(i, intcon) = round(candidate_pop(i, intcon));
            elseif ~isempty(initial_guess) && retry_count <= 20 && i <= (needed * surplus_factor * 0.8)
                % 80% of candidates in early retries are seeded from the guess
                for j = 1:nvars
                    if ismember(j, intcon)
                        % For discrete: shift by +/- 1 occasionally
                        val = initial_guess(j);
                        if rand() < 0.2, val = val + (randi([0, 1])*2 - 1); end
                        candidate_pop(i, j) = max(lb(j), min(ub(j), round(val)));
                    else
                        % For continuous: Gaussian perturbation (sigma=15% of range)
                        range = ub(j) - lb(j);
                        sigma = range * 0.15;
                        val = initial_guess(j) + sigma * randn();
                        candidate_pop(i, j) = max(lb(j), min(ub(j), val));
                    end
                end
            else
                % Fully random generation (Exploration)
                for j = 1:nvars
                    if ismember(j, intcon)
                        candidate_pop(i, j) = randi([lb(j), ub(j)]);
                    else
                        candidate_pop(i, j) = lb(j) + (ub(j) - lb(j)) * rand();
                    end
                end
            end
        end
        candidate_scores = evaluate_population(candidate_pop, fitnessfcn, use_parallel);
        evaluation_count = evaluation_count + size(candidate_pop, 1);
        valid_idxs = find(~isinf(candidate_scores) & ~isnan(candidate_scores));
        
        fill_count = min(needed, length(valid_idxs));
        inf_idxs = find(isinf(scores));
        if fill_count > 0
            population(inf_idxs(1:fill_count), :) = candidate_pop(valid_idxs(1:fill_count), :);
            scores(inf_idxs(1:fill_count)) = candidate_scores(valid_idxs(1:fill_count));
        end
    end
    
    [best_fval, best_idx] = min(scores);
    best_x = population(best_idx, :);
    
    fprintf('Gen 0 | Best Obj: %.4f\n', best_fval);
    
    % History for output
    history = zeros(max_gen, 1);
    history_K = zeros(max_gen, 1);
    
    for gen = 1:max_gen
        new_population = zeros(pop_size, nvars);
        
        [~, sort_idx] = sort(scores);
        sorted_pop = population(sort_idx, :);
        elites = sorted_pop(1:elite_count, :);
        
        % --- Tier 1: Elitism ---
        new_population(1:elite_count, :) = elites;
        new_scores = zeros(pop_size, 1);
        new_scores(1:elite_count) = scores(sort_idx(1:elite_count));
        
        % --- Tier 2: Local Search (Perturbation) ---
        % Generate candidates first, then evaluate in parallel
        perturb_candidates = zeros(perturb_count, nvars);
        for i = 1:perturb_count
            parent = elites(randi(elite_count), :);
            child = parent;
            for j = 1:nvars
                range_j = ub(j) - lb(j);
                if ismember(j, intcon)
                    if rand < 0.20
                        shift = sign(rand - 0.5);
                        child(j) = max(lb(j), min(ub(j), child(j) + shift));
                    end
                else
                    perturb = randn * perturb_sigma * range_j;
                    child(j) = max(lb(j), min(ub(j), child(j) + perturb));
                end
            end
            perturb_candidates(i, :) = child;
        end
        
        % --- Tier 3: Random Exploration ---
        random_candidates = zeros(random_count, nvars);
        for i = 1:random_count
            for j = 1:nvars
                if ismember(j, intcon)
                    random_candidates(i, j) = randi([lb(j), ub(j)]);
                else
                    random_candidates(i, j) = lb(j) + (ub(j) - lb(j)) * rand();
                end
            end
        end

        % COMBINE AND EVALUATE IN PARALLEL
        trial_pop = [perturb_candidates; random_candidates];
        trial_scores = evaluate_population(trial_pop, fitnessfcn, use_parallel);
        evaluation_count = evaluation_count + size(trial_pop, 1);
        
        % Handle validity for Tier 2 and 3 (simple fallback for now to keep it parallel)
        % In a real robust system, we'd loop this specific part, but let's keep it simple for speed
        total_eval_count = perturb_count + random_count;
        for i = 1:total_eval_count
            idx = elite_count + i;
            if isinf(trial_scores(i)) || isnan(trial_scores(i))
                % Fallback: if Tier 2 fails, use parent. if Tier 3 fails, regenerate once or accept Inf
                if i <= perturb_count
                   new_population(idx, :) = elites(randi(elite_count), :); % Fallback to random elite
                   new_scores(idx) = scores(sort_idx(randi(elite_count)));
                else
                   new_population(idx, :) = trial_pop(i, :); % Keep it, GA will discard it anyway
                   new_scores(idx) = trial_scores(i);
                end
            else
                new_population(idx, :) = trial_pop(i, :);
                new_scores(idx) = trial_scores(i);
            end
        end
        
        population = new_population;
        scores = new_scores;
        
        [current_best_fval, current_best_idx] = min(scores);
        if current_best_fval < best_fval
            best_fval = current_best_fval;
            best_x = population(current_best_idx, :);
        end
        
        history(gen) = best_fval;
        
        % Extract extra physics results for the best individual
        try
            [~, best_res] = fitnessfcn(best_x);
            evaluation_count = evaluation_count + 1;
            if isfield(best_res, 'K_torsion')
                history_K(gen) = best_res.K_torsion;
            else
                history_K(gen) = NaN;
            end
        catch
            history_K(gen) = NaN;
        end
        
        if mod(gen, 5) == 0 || gen == max_gen
            fprintf('Gen %d | Best Obj: %.4f | Stiffness: %.1f | CG_limit_active: %s\n', ...
                gen, best_fval, history_K(gen), num2str(best_fval > 1e6));
        end
    end
    
    exitflag = 1;
    output.generations = max_gen;
    output.history = history;
    output.history_K = history_K;
    output.final_scores = scores;
    output.funccount = evaluation_count;
end

function scores = evaluate_population(pop, fitnessfcn, use_parallel)
    pop_size = size(pop, 1);
    scores = zeros(pop_size, 1);
    if use_parallel
        parfor i = 1:pop_size
            scores(i) = fitnessfcn(pop(i, :));
        end
    else
        for i = 1:pop_size
            scores(i) = fitnessfcn(pop(i, :));
        end
    end
end
