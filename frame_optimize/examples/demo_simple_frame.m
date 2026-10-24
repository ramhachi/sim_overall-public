%% DEMO_SIMPLE_FRAME - Demonstrate MEX FEM solver with simple frame
%
% This script creates a simple 16-node rectangular frame and computes
% torsional rigidity using the MEX FEM solver.
%
% Expected output:
%   - Torsional rigidity: ~200-300 Nm/deg (depending on geometry)
%   - Mass: ~8-12 kg
%   - Computation time: <10 ms

clear; close all; clc;

fprintf('=== FSAE Frame Optimization Demo ===\n\n');

%% 1. Load materials and pipe catalog

% Get paths
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');

% Add MATLAB functions to path
addpath(matlab_dir);

fprintf('1. Loading materials and pipe catalog...\n');
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

fprintf('   Loaded %d materials, %d pipes\n', length(fieldnames(materials)), length(catalog));

%% 2. Create simple frame geometry (16 nodes, rectangular box)

fprintf('\n2. Creating simple frame geometry...\n');

% Dimensions (m)
L = 2.4;  % Length (front to rear)
W = 1.2;  % Width
H_front = 1.0;  % Front hoop height
H_rear = 1.4;   % Rear hoop height

% Nodes: [x, y, z] in meters
% Front hoop (nodes 1-4)
nodes = [
    0, -W/2, 0;      % 1: Front lower left
    0, W/2, 0;       % 2: Front lower right
    0, -W/2, H_front; % 3: Front upper left
    0, W/2, H_front;  % 4: Front upper right

    % Rear hoop (nodes 5-8)
    L, -W/2, 0;      % 5: Rear lower left
    L, W/2, 0;       % 6: Rear lower right
    L, -W/2, H_rear; % 7: Rear upper left
    L, W/2, H_rear;  % 8: Rear upper right

    % Mid nodes for diagonal bracing (nodes 9-12)
    L/2, -W/2, 0;    % 9: Mid lower left
    L/2, W/2, 0;     % 10: Mid lower right
    L/2, -W/2, H_front; % 11: Mid upper left
    L/2, W/2, H_front;  % 12: Mid upper right

    % Additional nodes for cross bracing (nodes 13-16)
    L/4, 0, H_front/2;   % 13
    3*L/4, 0, H_rear/2;  % 14
    L/2, 0, 0;           % 15
    L/2, 0, H_front;     % 16
];

% Elements: [node_i, node_j, D_outer, t, E, G, rho, shape, joint_factor]
% Use AISI 4130, 25.4mm OD, 1.65mm wall, round tube
D_outer_mm = 25.4;
t_mm = 1.65;
E_GPa = materials.AISI4130.E_GPa;
G_GPa = materials.AISI4130.G_GPa;
rho_kg_m3 = materials.AISI4130.rho_kg_m3;
shape = 1;  % 1=round
joint_factor = 0.75;  % Joint correction

elements = [
    % Front hoop (4 elements)
    1, 2, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Lower horizontal
    2, 4, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Right vertical
    4, 3, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Upper horizontal
    3, 1, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Left vertical

    % Rear hoop (4 elements)
    5, 6, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    6, 8, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    8, 7, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    7, 5, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;

    % Lower rails (2 elements)
    1, 5, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Left lower
    2, 6, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Right lower

    % Upper rails (2 elements)
    3, 7, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Left upper
    4, 8, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;  % Right upper

    % Lower cross bracing (4 elements: nodes 9, 10)
    1, 9, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    9, 5, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    2, 10, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    10, 6, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;

    % Upper cross bracing (4 elements: nodes 11, 12)
    3, 11, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    11, 7, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    4, 12, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    12, 8, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;

    % Diagonal bracing (4 elements: nodes 13, 14, 15, 16)
    1, 13, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    13, 8, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    9, 15, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    15, 10, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    11, 16, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    16, 12, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    5, 14, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
    14, 4, D_outer_mm, t_mm, E_GPa, G_GPa, rho_kg_m3, shape, joint_factor;
];

n_nodes = size(nodes, 1);
n_elems = size(elements, 1);

fprintf('   Created %d nodes, %d elements\n', n_nodes, n_elems);

%% 3. Define boundary conditions and loads

fprintf('\n3. Setting up boundary conditions and loads...\n');

% Fix all DOFs at nodes 1, 2 (front lower corners)
% Also fix XYZ at node 5 to prevent rigid body modes
% Pre-allocate for performance
bcs = zeros(15, 3);  % 2 nodes × 6 DOFs + 3 extra constraints = 15 rows
bc_idx = 1;
for node = [1, 2]
    for dof = 1:6
        bcs(bc_idx, :) = [node, dof, 0.0];
        bc_idx = bc_idx + 1;
    end
end
% Fix XYZ displacements at node 5 (rear lower left) to prevent rigid body modes
for dof = 1:3  % ux, uy, uz
    bcs(bc_idx, :) = [5, dof, 0.0];
    bc_idx = bc_idx + 1;
end

% Apply torsional load: ±1000 Nm torque at rear corners
torque = 1000;  % Nm
loads = [
    7, 4, torque;   % Node 7 (rear upper left): +Mx
    8, 4, -torque;  % Node 8 (rear upper right): -Mx
];

fprintf('   BCs: %d constraints\n', size(bcs, 1));
fprintf('   Loads: Applied ±%.0f Nm torsion\n', torque);

%% 4. Call FEM solver (Docker-based)

fprintf('\n4. Running FEM solver (Docker-based)...\n');

% Check if Docker binary exists
bin_dir = fullfile(script_dir, '..', 'bin');
solver_binary = fullfile(bin_dir, 'beam_fem_standalone');

if ~isfile(solver_binary)
    fprintf('   ✗ Solver binary not found: %s\n', solver_binary);
    fprintf('   Build the solver first using Docker:\n');
    fprintf('   docker compose run --rm fsae-sim bash -c "cd /workspace/frame_optimize/cpp/build && cmake .. && make && make install"\n');
    return;
end

% Solve
tic;
result = beam_fem_solver(nodes, elements, bcs, loads, struct());
elapsed = toc;

fprintf('   ✓ Solve completed in %.3f ms\n', elapsed * 1000);

%% 5. Display results

fprintf('\n=== RESULTS ===\n');
fprintf('Mass:               %.2f kg\n', result.mass);
fprintf('Torsional rigidity: %.1f Nm/deg\n', result.K_torsion);
fprintf('Max displacement:   %.3f mm\n', max(abs(result.U(:))) * 1000);

% Check if results are reasonable
if result.K_torsion < 100 || result.K_torsion > 2000
    warning('Torsional rigidity %.1f Nm/deg is outside expected range (100-2000)', ...
            result.K_torsion);
end

if result.mass < 5 || result.mass > 20
    warning('Mass %.2f kg is outside expected range (5-20)', result.mass);
end

%% 6. Visualize frame

fprintf('\n6. Visualizing frame...\n');

figure('Name', 'Simple Frame Demo', 'Position', [100, 100, 1000, 600]);

% Plot elements
subplot(1, 2, 1);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Frame Geometry');

for i = 1:n_elems
    node_i = elements(i, 1);
    node_j = elements(i, 2);
    p1 = nodes(node_i, :);
    p2 = nodes(node_j, :);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'b-', 'LineWidth', 2);
end

% Plot nodes
scatter3(nodes(:,1), nodes(:,2), nodes(:,3), 50, 'r', 'filled');

% Mark fixed nodes
scatter3(nodes([1,2],1), nodes([1,2],2), nodes([1,2],3), 100, 'k', 'filled', 'square');

view(3); rotate3d on;

% Plot displacements
subplot(1, 2, 2);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Deformed Shape (Magnified 1000×)');

scale = 1000;  % Magnification factor
for i = 1:n_elems
    node_i = elements(i, 1);
    node_j = elements(i, 2);
    p1 = nodes(node_i, :) + scale * result.U(node_i, 1:3);
    p2 = nodes(node_j, :) + scale * result.U(node_j, 1:3);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r-', 'LineWidth', 2);
end

view(3); rotate3d on;

fprintf('\n✓ Demo completed successfully!\n');
