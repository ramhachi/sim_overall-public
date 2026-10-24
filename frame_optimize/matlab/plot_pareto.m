function plot_pareto(history_results, pareto_front, title_str)
% PLOT_PARETO Visualizes the Mass vs Torsional Stiffness Pareto tradeoff graph.
%
% Inputs:
%   history_results - Raw [M x 2] matrix of all evaluated points for background scattering
%                     (Assumes col 1 = Mass_kg, col 2 = Setup Torsional Stiffness_Nm_deg)
%   pareto_front    - [K x 2] matrix of just the extracted optimal points
%   title_str       - Optional title override

    if nargin < 3
        title_str = 'FSAE Frame Optimization: Mass vs. Stiffness Trade-off';
    end

    hold on;
    grid on;
    box on;
    
    % 1. Plot all background solutions (light grey)
    if ~isempty(history_results)
        scatter(history_results(:, 1), history_results(:, 2), 15, ...
                [0.7 0.7 0.7], 'o', 'filled', 'MarkerEdgeAlpha', 0.2, 'MarkerFaceAlpha', 0.5);
    end
    
    % 2. Plot Pareto line mapping
    if ~isempty(pareto_front)
        % Sort strictly by mass to build an ascending line
        [sorted_pareto, ~] = sortrows(pareto_front, 1);
        
        plot(sorted_pareto(:, 1), sorted_pareto(:, 2), '-b', 'LineWidth', 2);
        
        % 3. Scatter Pareto Frontier prominent dots
        scatter(sorted_pareto(:, 1), sorted_pareto(:, 2), 60, ...
                [0.1 0.4 0.8], 'o', 'filled', 'MarkerEdgeColor', 'k');
    end
    
    title(title_str, 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Frame Mass [kg]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Torsional Stiffness [Nm/deg]', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Add basic legendary marker
    legend('All Sampled Configurations', 'Pareto Boundary Constraint', 'Optimal Solutions', ...
           'Location', 'northwest');
       
    set(gca, 'FontSize', 11);
end
