%% STUDY_PIPE_SIZES - Parameter study for pipe size effects
%
% This script runs the real vehicle frame demo with different pipe sizes
% to understand the mass vs stiffness tradeoff.

clear; close all; clc;

fprintf('=== Pipe Size Parameter Study ===\n\n');

% Add paths
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(fullfile(project_dir, 'matlab'));

% Load base data
data_dir = fullfile(project_dir, 'data');
geom_file = fullfile(data_dir, 'real_vehicle_geometry.json');
geom = jsondecode(fileread(geom_file));
materials = loadMaterials(fullfile(data_dir, 'materials.json'));

%% Define parameter sweep

% Steel pipe outer diameters to test [mm]
D_steel_test = [25.4, 28.6, 31.8, 38.1];  % 1", 1.125", 1.25", 1.5"

% Steel wall thicknesses to test [mm]
t_steel_test = [1.65, 2.0, 2.5];

% Aluminum pipe outer diameters [mm]
D_alum_test = [25.4, 31.8, 38.1];  % 1", 1.25", 1.5"

% Pre-allocate results
n_configs = length(D_steel_test) * length(t_steel_test);
results = struct('D_steel', {}, 't_steel', {}, 'D_alum', {}, ...
                 'mass', {}, 'K_torsion', {}, 'max_disp_torsion', {}, ...
                 'max_disp_lateral', {}, 'max_disp_longitudinal', {});

config_idx = 1;

%% Run parameter sweep

fprintf('Running %d configurations...\n\n', n_configs);

for i_D = 1:length(D_steel_test)
    for i_t = 1:length(t_steel_test)
        D_steel = D_steel_test(i_D);
        t_steel = t_steel_test(i_t);
        D_alum = D_alum_test(min(i_D, length(D_alum_test)));  % Scale aluminum with steel

        fprintf('Config %d/%d: Steel %.1f×%.2fmm, Alum %.1fmm...\n', ...
                config_idx, n_configs, D_steel, t_steel, D_alum);

        % Create frame geometry (simplified, same node positions)
        [nodes, elements, bcs, load_cases] = create_frame_geometry(...
            geom, materials, D_steel, t_steel, D_alum);

        % Run FEM analysis
        try
            result_torsion = beam_fem_solver_tcp_java(nodes, elements, bcs, load_cases(1).loads);
            result_lateral = beam_fem_solver_tcp_java(nodes, elements, bcs, load_cases(2).loads);
            result_longitudinal = beam_fem_solver_tcp_java(nodes, elements, bcs, load_cases(3).loads);

            % Store results
            results(config_idx).D_steel = D_steel;
            results(config_idx).t_steel = t_steel;
            results(config_idx).D_alum = D_alum;
            results(config_idx).mass = result_torsion.mass;
            results(config_idx).K_torsion = result_torsion.K_torsion;
            results(config_idx).max_disp_torsion = max(abs(result_torsion.U(:))) * 1000;
            results(config_idx).max_disp_lateral = max(abs(result_lateral.U(:))) * 1000;
            results(config_idx).max_disp_longitudinal = max(abs(result_longitudinal.U(:))) * 1000;

            fprintf('  Mass: %.2f kg, K_torsion: %.1f Nm/deg\n', ...
                    results(config_idx).mass, results(config_idx).K_torsion);
        catch ME
            warning('Config %d failed: %s', config_idx, ME.message);
        end

        config_idx = config_idx + 1;
    end
end

% Shutdown server
beam_fem_solver_tcp_java('shutdown');

%% Plot results

fprintf('\n=== Plotting Results ===\n');

figure('Name', 'Pipe Size Parameter Study', 'Position', [100, 100, 1400, 600]);

% Plot 1: Mass vs Torsional Rigidity
subplot(1, 3, 1);
hold on; grid on;
xlabel('Mass [kg]'); ylabel('Torsional Rigidity [Nm/deg]');
title('Mass vs Stiffness Tradeoff');

% Color by steel OD
colors = lines(length(D_steel_test));
for i = 1:length(results)
    % Find closest match for D_steel (handle floating point comparison)
    [~, D_idx] = min(abs(D_steel_test - results(i).D_steel));
    plot(results(i).mass, results(i).K_torsion, 'o', ...
         'MarkerSize', 10, 'MarkerFaceColor', colors(D_idx,:), ...
         'MarkerEdgeColor', 'k');
end

% Add Pareto front
masses = [results.mass];
stiffnesses = [results.K_torsion];
[sorted_mass, idx] = sort(masses);
sorted_stiff = stiffnesses(idx);

% Find Pareto front (no point dominates)
pareto_idx = false(size(sorted_mass));
for i = 1:length(sorted_mass)
    % Point is Pareto-optimal if no other point has both lower mass AND higher stiffness
    dominated = any((masses < sorted_mass(i)) & (stiffnesses > sorted_stiff(i)));
    pareto_idx(i) = ~dominated;
end

plot(sorted_mass(pareto_idx), sorted_stiff(pareto_idx), 'r--', 'LineWidth', 2);

legend_entries = arrayfun(@(x) sprintf('Steel OD %.1fmm', x), D_steel_test, 'UniformOutput', false);
legend_entries{end+1} = 'Pareto Front';
legend(legend_entries, 'Location', 'best');

% Plot 2: Pipe Size vs Mass
subplot(1, 3, 2);
hold on; grid on;
xlabel('Steel OD [mm]'); ylabel('Mass [kg]');
title('Pipe Size Effect on Mass');

for i_t = 1:length(t_steel_test)
    t_steel = t_steel_test(i_t);
    % Handle floating point comparison with tolerance
    mask = abs([results.t_steel] - t_steel) < 0.01;
    if any(mask)
        D_vals = [results(mask).D_steel];
        mass_vals = [results(mask).mass];
        [D_sorted, sort_idx] = sort(D_vals);
        plot(D_sorted, mass_vals(sort_idx), 'o-', 'LineWidth', 2, ...
             'DisplayName', sprintf('Wall %.2fmm', t_steel));
    end
end
legend('Location', 'best');

% Plot 3: Pipe Size vs Stiffness
subplot(1, 3, 3);
hold on; grid on;
xlabel('Steel OD [mm]'); ylabel('Torsional Rigidity [Nm/deg]');
title('Pipe Size Effect on Stiffness');

for i_t = 1:length(t_steel_test)
    t_steel = t_steel_test(i_t);
    % Handle floating point comparison with tolerance
    mask = abs([results.t_steel] - t_steel) < 0.01;
    if any(mask)
        D_vals = [results(mask).D_steel];
        stiff_vals = [results(mask).K_torsion];
        [D_sorted, sort_idx] = sort(D_vals);
        plot(D_sorted, stiff_vals(sort_idx), 'o-', 'LineWidth', 2, ...
             'DisplayName', sprintf('Wall %.2fmm', t_steel));
    end
end
legend('Location', 'best');

%% Display summary table

fprintf('\n=== Results Summary ===\n');
fprintf('%-10s %-10s %-10s %-10s %-15s %-15s\n', ...
        'D_steel', 't_steel', 'D_alum', 'Mass[kg]', 'K_torsion', 'Disp_tor[mm]');
fprintf('%s\n', repmat('-', 1, 80));

for i = 1:length(results)
    fprintf('%-10.1f %-10.2f %-10.1f %-10.2f %-15.1f %-15.1f\n', ...
            results(i).D_steel, results(i).t_steel, results(i).D_alum, ...
            results(i).mass, results(i).K_torsion, results(i).max_disp_torsion);
end

fprintf('\n✓ Parameter study complete!\n');

%% Helper function to create frame geometry
function [nodes, elements, bcs, load_cases] = create_frame_geometry(geom, materials, D_steel_mm, t_steel_mm, D_alum_mm)
    % Extract dimensions
    L = geom.dimensions.wheelbase_m;
    W_f = geom.dimensions.track_front_m;
    W_r = geom.dimensions.track_rear_m;
    H_front = 1.0;
    H_main = 1.35;
    H_mid = 0.9;

    % Create nodes (same as demo_real_vehicle_frame.m)
    nodes = [
        % Front hoop (1-4)
        0, -W_f/2, 0;
        0,  W_f/2, 0;
        0, -W_f/2, H_front;
        0,  W_f/2, H_front;

        % Side impact (5-8)
        0.7, -W_f/2, 0.3;
        0.7,  W_f/2, 0.3;
        0.7, -W_f/2, H_mid;
        0.7,  W_f/2, H_mid;

        % Main hoop (9-12)
        L, -W_r/2, 0;
        L,  W_r/2, 0;
        L, -W_r/2, H_main;
        L,  W_r/2, H_main;

        % Additional structure (13-28)
        0.35, -W_f/2, 0;
        0.35,  W_f/2, 0;
        1.05, -W_r/2, 0;
        1.05,  W_r/2, 0;
        0.35, 0, H_front/2;
        1.05, 0, H_mid;
        0.35, -W_f/2, H_front;
        0.35,  W_f/2, H_front;
        1.05, -W_r/2, H_mid;
        1.05,  W_r/2, H_mid;
        0.5, -0.2, H_front-0.1;
        0.5,  0.2, H_front-0.1;
        1.0, -0.3, 0.25;
        1.0,  0.3, 0.25;
        L-0.05, -0.3, 0.4;
        L-0.05,  0.3, 0.4;
    ];

    % Material properties
    E_steel = materials.AISI4130.E_GPa;
    G_steel = materials.AISI4130.G_GPa;
    rho_steel = materials.AISI4130.rho_kg_m3;

    t_alum_mm = 2.5;
    E_alum = materials.Al6061T6.E_GPa;
    G_alum = materials.Al6061T6.G_GPa;
    rho_alum = materials.Al6061T6.rho_kg_m3;

    shape = 1;
    joint_factor = 0.85;

    % Create elements (same connectivity as demo)
    elements = [
        % Front hoop (steel)
        1, 2, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        2, 4, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        4, 3, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        3, 1, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;

        % Main hoop (steel)
        9, 10, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        10, 12, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        12, 11, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        11, 9, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;

        % Side impact (steel)
        5, 6, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        6, 8, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        8, 7, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        7, 5, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;

        % Rails (aluminum) - 16 elements
        1, 13, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        2, 14, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        13, 5, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        14, 6, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        5, 15, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        6, 16, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        15, 9, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        16, 10, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        3, 19, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        4, 20, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        19, 7, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        20, 8, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        7, 21, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        8, 22, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        21, 11, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        22, 12, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;

        % Bracing (aluminum) - remaining elements
        1, 17, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        2, 17, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        17, 7, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        17, 8, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        9, 18, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        10, 18, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        18, 11, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        18, 12, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        23, 24, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        3, 23, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        4, 24, D_steel_mm, t_steel_mm, E_steel, G_steel, rho_steel, shape, joint_factor;
        25, 26, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        26, 28, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        28, 27, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        27, 25, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        15, 25, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
        16, 26, D_alum_mm, t_alum_mm, E_alum, G_alum, rho_alum, shape, joint_factor;
    ];

    % Boundary conditions (3-point system)
    bcs = [
        1, 1, 0; 1, 2, 0; 1, 3, 0; 1, 4, 0; 1, 5, 0; 1, 6, 0;  % Node 1 all DOF
        2, 1, 0; 2, 2, 0; 2, 3, 0;  % Node 2 XYZ
        9, 3, 0;  % Node 9 Z
    ];

    % Load cases
    load_cases = generate_load_cases(nodes, geom.dimensions.total_mass_kg, 9.81);
end
