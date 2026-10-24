function [pareto_front, pareto_indices] = extractParetoFront(objectives)
% EXTRACTPARETOFRONT Extracts non-dominated solutions from a set of objectives.
%
% This function finds the Pareto-optimal front from a given dataset representing
% trade-offs (e.g., Mass vs. Stiffness). Note that in optimization literature,
% lower values usually represent "better" values (minimization). If an objective 
% is to be maximized (like stiffness), it should be passed as its negative.
%
% Inputs:
%   objectives - N x M matrix where N is the number of solutions and M is the
%                number of objectives to MINIMIZE.
%
% Outputs:
%   pareto_front - K x M matrix containing only the points on the Pareto front.
%   pareto_indices - K x 1 vector of indices mapping front solutions back to original array.

    num_sols = size(objectives, 1);
    num_objs = size(objectives, 2);
    
    % Initialize all as optimal (non-dominated)
    is_pareto = true(num_sols, 1);
    
    for i = 1:num_sols
        % Only compare if not already dominated
        if is_pareto(i)
            % Check if solution i is dominated by any other solution j
            % Solution j dominates i if j is <= i in ALL objectives AND strictly < in AT LEAST ONE
            for j = 1:num_sols
                if i ~= j && is_pareto(j)
                    % Check dominance conditions
                    better_or_eq_all = all(objectives(j, :) <= objectives(i, :));
                    strictly_better_any = any(objectives(j, :) < objectives(i, :));
                    
                    if better_or_eq_all && strictly_better_any
                        is_pareto(i) = false; % i is dominated by j
                        break; 
                    end
                end
            end
        end
    end
    
    pareto_indices = find(is_pareto);
    pareto_front = objectives(pareto_indices, :);
    
    % Sort by the first objective for clean plotting
    [pareto_front, sort_idx] = sortrows(pareto_front, 1);
    pareto_indices = pareto_indices(sort_idx);
    
    % Remove identical duplicates
    [pareto_front, unique_idx, ~] = unique(pareto_front, 'rows', 'stable');
    pareto_indices = pareto_indices(unique_idx);
end
