function options = configureGA(config)
% CONFIGUREGA Generates the options structure for the Genetic Algorithm.
%
% Inputs:
%   config - Struct containing high-level optimization settings
%            .pop_size        - Population size for GA
%            .max_generations - Maximum number of generations
%            .tol_fun         - Tolerance for objective function
%            .use_parallel    - Boolean indicating if parallel pool should be used
%
% Outputs:
%   options - GA optimization settings structure compatible with optimoptions

    % Default basic settings
    pop_size = 500;
    if isfield(config, 'pop_size')
        pop_size = config.pop_size;
    end
    
    max_gen = 200;
    if isfield(config, 'max_generations')
        max_gen = config.max_generations;
    end
    
    use_parallel = false;
    if isfield(config, 'use_parallel')
        use_parallel = config.use_parallel;
    end
    
    % Core Optimoptions setup
    options = optimoptions('ga', ...
        'PopulationSize', pop_size, ...
        'MaxGenerations', max_gen, ...
        'CrossoverFraction', 0.8, ...
        'EliteCount', round(0.05 * pop_size), ...
        'Display', 'iter', ...  % iter, final, or off
        'PlotFcn', {@gaplotbestf, @gaplotstopping, @gaplotscorediversity}, ...
        'FunctionTolerance', 1e-4);
        
    % Enable parallel processing if explicitly requested
    if use_parallel
        try
            pool = gcp('nocreate');
            if isempty(pool)
                parpool; % Starts the default profile
            end
            options = optimoptions(options, 'UseParallel', true);
        catch
            warning('Parallel pool could not be started. Using single thread mode.');
        end
    end
end
