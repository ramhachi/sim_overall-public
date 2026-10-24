function fsae_viewer(nodes, elements_geom, pipe_assignments, catalog, pareto_results, bcs, loads, stiff_targets)
% FSAE_VIEWER Master Visualizer UI. Opens a combined dashboard of the frame
% and pareto progress side-by-side using the underlying functions.
%
% Inputs:
%   nodes            - Node geometry
%   elements_geom    - Connectivity array
%   pipe_assignments - Chosen configuration from catalog
%   catalog          - Structural database
%   pareto_results   - Extracted pareto outputs struct with history
%   bcs              - Optional boundary conditions constraint array
%   loads            - Optional applied loads constraint array
%   stiff_targets    - Optional struct with K_target and K_min [Nm/deg]

    if nargin < 6
        bcs = [];
    end
    if nargin < 7
        loads = [];
    end
    if nargin < 8 || isempty(stiff_targets)
        stiff_targets = struct('K_target', 1200, 'K_min', 500);
    end

    fig = figure('Name', 'FSAE Frame Optimization Dashboard', ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 100, 1400, 700], ...
                 'Color', 'white');
                 
    % --- Left Panel: 3D Render --- 
    subplot(1, 2, 1);
    
    % Title formatting based on result parameters limits (if we passed results in params, extract the mass here)
    title('Optimized Frame Topology - Pipe Render');
    
    plot_frame(nodes, elements_geom, pipe_assignments, catalog, true);
    
    % Add UI Color Legend
    sizes = {'Size A (Red)', 'Size B (Orange)', 'Size C (Green)', 'Size D (Blue)'};
    colors = {[0.8, 0.2, 0.2], [0.9, 0.5, 0.1], [0.2, 0.7, 0.3], [0.2, 0.4, 0.8]};
    
    ax = gca;
    for i = 1:length(sizes)
        text(ax.XLim(1), ax.YLim(1), ax.ZLim(2) - (i * 0.05 * (ax.ZLim(2)-ax.ZLim(1))), ...
             sizes{i}, 'Color', colors{i}, 'FontWeight', 'bold', 'FontSize', 10);
    end

    % --- Plot Analysis Points (BCs and Loads) ---
    if ~isempty(bcs)
        % BCs are usually fixed points, display as magenta triangles
        bc_nodes = bcs(:, 1); 
        scatter3(nodes(bc_nodes, 1), nodes(bc_nodes, 2), nodes(bc_nodes, 3), 100, 'm^', 'filled', 'DisplayName', 'Fixed BCs');
    end
    
    if ~isempty(loads)
        % Loads format: [node_id, dof, value] (dof 1=x, 2=y, 3=z)
        unique_load_nodes = unique(loads(:, 1));
        
        for idx = 1:length(unique_load_nodes)
            L_node = unique_load_nodes(idx);
            vec = [0, 0, 0];
            
            % Accumulate force vectors on this node
            node_loads = loads(loads(:, 1) == L_node, :);
            for k = 1:size(node_loads, 1)
                dof = node_loads(k, 2);
                val = node_loads(k, 3);
                if dof >= 1 && dof <= 3
                    vec(dof) = vec(dof) + val;
                end
            end
            
            % Normalize vector for rendering (we only care about direction for standard UI, but scale a bit)
            if norm(vec) > 0
                vec_render = (vec / norm(vec)) * 0.2; % 200mm arrow
                quiver3(nodes(L_node, 1), nodes(L_node, 2), nodes(L_node, 3), ...
                        vec_render(1), vec_render(2), vec_render(3), 0, ...
                        'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 2, 'ShowArrowHead', 'on');
            end
        end
        % Just plot stars at load origins
        scatter3(nodes(unique_load_nodes, 1), nodes(unique_load_nodes, 2), nodes(unique_load_nodes, 3), 100, 'c*', 'DisplayName', 'Load Points');
    end

    % --- Right Panel: Torsional Stiffness History Graph ---
    subplot(1, 2, 2);
    
    if nargin < 5 || isempty(pareto_results) || ~isfield(pareto_results, 'history_K')
        text(0.5, 0.5, 'Torsional Stiffness data not provided.', ...
             'HorizontalAlignment', 'center', 'FontSize', 12);
        axis off;
    else
        % Plot Torsional Stiffness History
        plot(pareto_results.gen_history, pareto_results.history_K, '-bo', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        grid on;
        xlabel('Generation', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Torsional Stiffness (Nm/deg)', 'FontSize', 12, 'FontWeight', 'bold');
        title('GA Convergence: Torsional Stiffness vs Generation', 'FontSize', 14, 'FontWeight', 'bold');
        
        % Add target reference lines (dynamic from stiff_targets)
        yline(stiff_targets.K_target, 'r--', sprintf('Target (%.0f Nm/deg)', stiff_targets.K_target), 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
        yline(stiff_targets.K_min, 'k--', sprintf('Min (%.0f Nm/deg)', stiff_targets.K_min), 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');

        xlim([1, length(pareto_results.gen_history)]);
        ylim([0, max(max(pareto_results.history_K)*1.2, stiff_targets.K_target * 1.1)]);
    end
    
    drawnow;
end
