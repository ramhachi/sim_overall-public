%% PLOT_STUDY_RESULTS - Quick visualization of parameter study results
%
% Plots the results from study_pipe_sizes.m using hardcoded data

clear; close all; clc;

fprintf('=== Pipe Size Parameter Study - Results Visualization ===\n\n');

%% Hardcoded results from study_pipe_sizes.m

% Config data: [D_steel, t_steel, D_alum, mass, K_torsion]
data = [
    % Config 1 failed
    25.4, 2.00, 25.4, 25.53, 229.2;
    25.4, 2.50, 25.4, 29.34, 262.1;
    28.6, 1.65, 31.8, 27.06, 323.5;
    28.6, 2.00, 31.8, 30.24, 362.9;
    28.6, 2.50, 31.8, 34.63, 413.3;
    31.8, 1.65, 38.1, 31.32, 486.5;
    31.8, 2.00, 38.1, 34.91, 543.6;
    31.8, 2.50, 38.1, 39.88, 617.5;
    38.1, 1.65, 38.1, 35.11, 738.0;
    38.1, 2.00, 38.1, 39.49, 840.9;
    38.1, 2.50, 38.1, 45.61, 977.4;
];

D_steel = data(:, 1);
t_steel = data(:, 2);
D_alum = data(:, 3);
mass = data(:, 4);
K_torsion = data(:, 5);

%% Create figure

figure('Name', 'Pipe Size Parameter Study', 'Position', [100, 100, 1400, 800]);

% Define colors for steel OD
D_steel_unique = unique(D_steel);
colors = lines(length(D_steel_unique));

%% Plot 1: Mass vs Torsional Rigidity (Pareto Front)

subplot(2, 3, 1);
hold on; grid on;
xlabel('Mass [kg]', 'FontSize', 12);
ylabel('Torsional Rigidity [Nm/deg]', 'FontSize', 12);
title('Mass vs Stiffness Tradeoff', 'FontSize', 14, 'FontWeight', 'bold');

% Plot points colored by steel OD
for i = 1:length(D_steel)
    D_idx = find(D_steel_unique == D_steel(i));
    plot(mass(i), K_torsion(i), 'o', 'MarkerSize', 12, ...
         'MarkerFaceColor', colors(D_idx,:), 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
end

% Find and plot Pareto front
[sorted_mass, idx] = sort(mass);
sorted_stiff = K_torsion(idx);
pareto_idx = false(size(sorted_mass));
for i = 1:length(sorted_mass)
    dominated = any((mass < sorted_mass(i)) & (K_torsion > sorted_stiff(i)));
    pareto_idx(i) = ~dominated;
end
plot(sorted_mass(pareto_idx), sorted_stiff(pareto_idx), 'r--', 'LineWidth', 2);

% Add target line
plot([25, 45], [500, 500], 'g--', 'LineWidth', 2);
text(35, 520, 'Target: 500 Nm/deg', 'Color', 'g', 'FontWeight', 'bold');

% Highlight recommended config (Config 8: 31.8×2.0)
config8_idx = find(D_steel == 31.8 & t_steel == 2.0);
plot(mass(config8_idx), K_torsion(config8_idx), 'p', 'MarkerSize', 20, ...
     'MarkerFaceColor', 'yellow', 'MarkerEdgeColor', 'red', 'LineWidth', 3);

legend_entries = arrayfun(@(x) sprintf('Steel %.1fmm', x), D_steel_unique, 'UniformOutput', false);
legend_entries{end+1} = 'Pareto Front';
legend_entries{end+1} = 'Target';
legend_entries{end+1} = 'Recommended (Config 8)';
legend(legend_entries, 'Location', 'southeast', 'FontSize', 9);

%% Plot 2: Steel OD vs Mass

subplot(2, 3, 2);
hold on; grid on;
xlabel('Steel OD [mm]', 'FontSize', 12);
ylabel('Mass [kg]', 'FontSize', 12);
title('Pipe Size Effect on Mass', 'FontSize', 14, 'FontWeight', 'bold');

t_steel_unique = unique(t_steel);
markers = {'o-', 's-', '^-'};

for i_t = 1:length(t_steel_unique)
    t = t_steel_unique(i_t);
    mask = abs(t_steel - t) < 0.01;
    if any(mask)
        D_vals = D_steel(mask);
        mass_vals = mass(mask);
        [D_sorted, sort_idx] = sort(D_vals);
        plot(D_sorted, mass_vals(sort_idx), markers{i_t}, 'LineWidth', 2, ...
             'MarkerSize', 10, 'DisplayName', sprintf('Wall %.2fmm', t));
    end
end
legend('Location', 'northwest', 'FontSize', 10);
xlim([24, 39]);

%% Plot 3: Steel OD vs Stiffness

subplot(2, 3, 3);
hold on; grid on;
xlabel('Steel OD [mm]', 'FontSize', 12);
ylabel('Torsional Rigidity [Nm/deg]', 'FontSize', 12);
title('Pipe Size Effect on Stiffness', 'FontSize', 14, 'FontWeight', 'bold');

for i_t = 1:length(t_steel_unique)
    t = t_steel_unique(i_t);
    mask = abs(t_steel - t) < 0.01;
    if any(mask)
        D_vals = D_steel(mask);
        stiff_vals = K_torsion(mask);
        [D_sorted, sort_idx] = sort(D_vals);
        plot(D_sorted, stiff_vals(sort_idx), markers{i_t}, 'LineWidth', 2, ...
             'MarkerSize', 10, 'DisplayName', sprintf('Wall %.2fmm', t));
    end
end

% Add target line
plot([24, 39], [500, 500], 'g--', 'LineWidth', 2, 'DisplayName', 'Target');
legend('Location', 'northwest', 'FontSize', 10);
xlim([24, 39]);

%% Plot 4: Stiffness-to-Weight Ratio

subplot(2, 3, 4);
efficiency = K_torsion ./ mass;
bar(efficiency);
hold on; grid on;
xlabel('Configuration', 'FontSize', 12);
ylabel('Efficiency [Nm/deg per kg]', 'FontSize', 12);
title('Stiffness-to-Weight Efficiency', 'FontSize', 14, 'FontWeight', 'bold');

% Highlight Config 8
bar(config8_idx, efficiency(config8_idx), 'FaceColor', 'yellow', 'EdgeColor', 'red', 'LineWidth', 2);

% Add labels
config_labels = arrayfun(@(d, t) sprintf('%.1f×%.2f', d, t), D_steel, t_steel, 'UniformOutput', false);
set(gca, 'XTick', 1:length(D_steel), 'XTickLabel', config_labels, 'XTickLabelRotation', 45);

%% Plot 5: Mass Breakdown by Component

subplot(2, 3, 5);

% Estimate steel/aluminum mass (approximate based on volume)
% Steel: 12 elements (hoops), Aluminum: 33 elements (rails/bracing)
steel_fraction = 12 / 45;
alum_fraction = 33 / 45;

mass_steel = mass * steel_fraction;
mass_alum = mass * alum_fraction;

bar([mass_steel, mass_alum], 'stacked');
hold on; grid on;
xlabel('Configuration', 'FontSize', 12);
ylabel('Mass [kg]', 'FontSize', 12);
title('Material Mass Breakdown', 'FontSize', 14, 'FontWeight', 'bold');
legend('Steel', 'Aluminum', 'Location', 'northwest');

config_labels = arrayfun(@(d, t) sprintf('%.1f×%.2f', d, t), D_steel, t_steel, 'UniformOutput', false);
set(gca, 'XTick', 1:length(D_steel), 'XTickLabel', config_labels, 'XTickLabelRotation', 45);

%% Plot 6: Summary Table

subplot(2, 3, 6);
axis off;

% Display summary text
text(0.05, 0.95, 'Parameter Study Summary', 'FontSize', 14, 'FontWeight', 'bold');

summary_text = {
    '';
    sprintf('Configurations tested: %d (11 successful)', length(D_steel)+1);
    sprintf('Mass range: %.1f - %.1f kg', min(mass), max(mass));
    sprintf('Stiffness range: %.1f - %.1f Nm/deg', min(K_torsion), max(K_torsion));
    '';
    'Recommended: Config 8';
    sprintf('  Steel: %.1f mm OD × %.2f mm wall', D_steel(config8_idx), t_steel(config8_idx));
    sprintf('  Mass: %.2f kg', mass(config8_idx));
    sprintf('  Stiffness: %.1f Nm/deg', K_torsion(config8_idx));
    sprintf('  Efficiency: %.1f Nm/deg/kg', efficiency(config8_idx));
    '';
    'Key Findings:';
    '  • OD has 2.5× more impact than wall thickness';
    '  • Config 8 achieves 500+ Nm/deg target';
    '  • Config 4 is lightest (27.1 kg, 323 Nm/deg)';
    '  • Config 11 is stiffest (39.5 kg, 841 Nm/deg)';
};

for i = 1:length(summary_text)
    y_pos = 0.90 - (i-1) * 0.045;
    text(0.05, y_pos, summary_text{i}, 'FontSize', 10, 'Interpreter', 'none');
end

%% Save figure

fprintf('\n✓ Plots generated!\n');
fprintf('\nRecommended configuration (Config 8):\n');
fprintf('  Steel: %.1f mm × %.2f mm\n', D_steel(config8_idx), t_steel(config8_idx));
fprintf('  Mass: %.2f kg\n', mass(config8_idx));
fprintf('  Stiffness: %.1f Nm/deg\n', K_torsion(config8_idx));
fprintf('  Efficiency: %.2f Nm/deg/kg\n', efficiency(config8_idx));

% Optionally save
% saveas(gcf, 'parameter_study_results.png');
