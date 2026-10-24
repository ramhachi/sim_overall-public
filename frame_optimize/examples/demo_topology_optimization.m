%% DEMO_TOPOLOGY_OPTIMIZATION - Ground Structure法によるトポロジー最適化
%
% Ground Structure候補から最適なパイプセットを選択
% - Binary GA: 各候補パイプを使う(1)/使わない(0)で最適化
% - 左右対称を構造的に保証（Y=0平面で対称グループ化）
% - 目的: 質量最小化 + ねじり剛性確保
% - 制約: FSAE規則、阻止領域、必須構造

clear; close all; clc;

fprintf('=== Topology Optimization with Ground Structure ===\n\n');

%% 1. Load Data
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

fprintf('1. Loading materials and pipe catalogs...\n');
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

%% 2. Define Nodes
fprintf('\n2. Defining nodes...\n');

L = 2.4; W = 1.2; H_front = 1.0; H_rear = 1.4;

base_nodes = [
    0, -W/2, 0;           0, W/2, 0;           0, -W/2, H_front;     0, W/2, H_front;
    L, -W/2, 0;           L, W/2, 0;           L, -W/2, H_rear;      L, W/2, H_rear;
];

additional_nodes = [
    L/2, -W/2, 0;         L/2, W/2, 0;         L/2, -W/2, H_front;   L/2, W/2, H_front;
    L/4, -W/2, H_front/2; L/4, W/2, H_front/2; 3*L/4, -W/2, H_rear*0.7; 3*L/4, W/2, H_rear*0.7;
    L/3, 0, H_front*0.8;  2*L/3, 0, H_rear*0.8;
];

all_nodes = [base_nodes; additional_nodes];
n_nodes = size(all_nodes, 1);
fprintf('  Total nodes: %d\n', n_nodes);

%% 3. Build Node Mirror Map (Y=0 symmetry plane)
fprintf('\n3. Building symmetry map...\n');

node_mirror = zeros(n_nodes, 1);
tol = 1e-6;
for i = 1:n_nodes
    mirrored_pos = [all_nodes(i,1), -all_nodes(i,2), all_nodes(i,3)];
    for j = 1:n_nodes
        if norm(all_nodes(j,:) - mirrored_pos) < tol
            node_mirror(i) = j;
            break;
        end
    end
end

% Report
n_center = sum(node_mirror == (1:n_nodes)');
n_paired = (n_nodes - n_center) / 2;
fprintf('  Center nodes (Y=0): %d, Paired nodes: %d pairs\n', n_center, n_paired);

%% 4. Define Exclusion Zones
fprintf('\n4. Defining exclusion zones...\n');

config = struct();
config.exclusion_zones = {};

% IA/Nose area (F.8.4: IA must be ≥200mm fwd of FBH, ≥200mm wide, ≥100mm high)
% FBH square nodes and AIP zone — no frame tubes here
nose_ia = struct();
nose_ia.type = 'AABB';
nose_ia.min_bounds = [-0.4, -0.15, 0];
nose_ia.max_bounds = [0.0, 0.15, 0.28];
config.exclusion_zones{end+1} = nose_ia;

% Driver cockpit (T.1.1/T.1.2 template zone)
% Template inserted from top and nose side, 450mm wide × max ~800mm tall
cockpit = struct();
cockpit.type = 'AABB';
cockpit.min_bounds = [0.10, -0.225, 0];
cockpit.max_bounds = [1.10, 0.225, 0.80];
config.exclusion_zones{end+1} = cockpit;

% Front Hoop sweep zone — driver helmet/head clearance
% FH upper region where tubes would interfere with driver
fh_sweep = struct();
fh_sweep.type = 'AABB';
fh_sweep.min_bounds = [-0.05, -0.25, 0.80];
fh_sweep.max_bounds = [0.45, 0.25, 1.05];
config.exclusion_zones{end+1} = fh_sweep;

% Engine bay
engine = struct();
engine.type = 'AABB';
engine.min_bounds = [1.8, -0.4, 0];
engine.max_bounds = [2.4, 0.4, 0.6];
config.exclusion_zones{end+1} = engine;

%% 5. Generate Ground Structure
fprintf('\n5. Generating ground structure...\n');

% Shorter edges preferred for cowl fitment
config.max_length_mm = 1200;  % Reduced from 2000mm for better cowl compatibility
candidate_elements = generateGroundStructure(all_nodes, config);
n_candidates = size(candidate_elements, 1);

fprintf('  Candidate pipes: %d\n', n_candidates);

%% 6. Define Mandatory Elements (FSAE required)
fprintf('\n6. Defining mandatory elements...\n');

% Front hoop (FSAE F.6.3)
% Main hoop (FSAE F.6.5)
% Lower side rails (FSAE F.2.8 Side Impact)
% Upper side rails (FSAE F.2.7)
mandatory_elements = [
    % Front hoop
    1, 2;  % Front lower
    2, 4;  % Front right
    4, 3;  % Front upper
    3, 1;  % Front left
    % Main hoop
    5, 6;  % Rear lower
    6, 8;  % Rear right
    8, 7;  % Rear upper
    7, 5;  % Rear left
    % Lower side rails (front-to-mid-to-rear, both sides)
    1, 9;   % Front-left-bottom to mid-left-bottom
    9, 5;   % Mid-left-bottom to rear-left-bottom
    2, 10;  % Front-right-bottom to mid-right-bottom
    10, 6;  % Mid-right-bottom to rear-right-bottom
    % Upper side rails (front-to-mid-to-rear, both sides)
    3, 11;  % Front-left-top to mid-left-top
    11, 7;  % Mid-left-top to rear-left-top
    4, 12;  % Front-right-top to mid-right-top
    12, 8;  % Mid-right-top to rear-right-top
];

% Verify all mandatory elements exist in candidate list; add missing ones
for i = 1:size(mandatory_elements, 1)
    n1 = mandatory_elements(i, 1);
    n2 = mandatory_elements(i, 2);
    found = false;
    for j = 1:n_candidates
        if (candidate_elements(j,1) == n1 && candidate_elements(j,2) == n2) || ...
           (candidate_elements(j,1) == n2 && candidate_elements(j,2) == n1)
            found = true;
            break;
        end
    end
    if ~found
        fprintf('  WARNING: Mandatory element [%d-%d] not in candidates, adding it.\n', n1, n2);
        candidate_elements(end+1, :) = [n1, n2]; %#ok<SAGROW>
        n_candidates = n_candidates + 1;
    end
end

% Mark mandatory elements in candidate list
is_mandatory = false(n_candidates, 1);
for i = 1:size(mandatory_elements, 1)
    for j = 1:n_candidates
        if (candidate_elements(j,1) == mandatory_elements(i,1) && ...
            candidate_elements(j,2) == mandatory_elements(i,2)) || ...
           (candidate_elements(j,1) == mandatory_elements(i,2) && ...
            candidate_elements(j,2) == mandatory_elements(i,1))
            is_mandatory(j) = true;
        end
    end
end

fprintf('  Mandatory pipes: %d / %d\n', sum(is_mandatory), n_candidates);

%% 7. Build Element Symmetry Groups
fprintf('\n7. Building element symmetry groups...\n');

% For each candidate element, find its mirror element
% Elements in the same symmetry group share the same GA variable
symmetry_group = zeros(n_candidates, 1);
group_id = 0;

for i = 1:n_candidates
    if symmetry_group(i) > 0
        continue;  % Already assigned to a group
    end

    ni = candidate_elements(i, 1);
    nj = candidate_elements(i, 2);
    mi = node_mirror(ni);  % Mirror of node i
    mj = node_mirror(nj);  % Mirror of node j

    % Check if this element is its own mirror (both nodes on center or symmetric pair)
    is_self_mirror = (ni == mi && nj == mj) || (ni == mj && nj == mi);

    if is_self_mirror
        % Self-symmetric element (e.g., center cross-brace)
        group_id = group_id + 1;
        symmetry_group(i) = group_id;
    else
        % Find the mirror element in the candidate list
        mirror_found = false;
        for j = (i+1):n_candidates
            if symmetry_group(j) > 0
                continue;
            end
            nj1 = candidate_elements(j, 1);
            nj2 = candidate_elements(j, 2);
            if (nj1 == mi && nj2 == mj) || (nj1 == mj && nj2 == mi)
                group_id = group_id + 1;
                symmetry_group(i) = group_id;
                symmetry_group(j) = group_id;
                mirror_found = true;
                break;
            end
        end

        if ~mirror_found
            % No mirror found (shouldn't happen with symmetric nodes, but be safe)
            group_id = group_id + 1;
            symmetry_group(i) = group_id;
        end
    end
end

n_groups = group_id;
config.symmetry_group = symmetry_group;

% Find which groups contain mandatory elements
is_mandatory_group = false(n_groups, 1);
for i = 1:n_candidates
    if is_mandatory(i)
        is_mandatory_group(symmetry_group(i)) = true;
    end
end

fprintf('  Symmetry groups: %d (from %d candidates)\n', n_groups, n_candidates);
fprintf('  Variable reduction: %.0f%%\n', 100 * (1 - n_groups / n_candidates));
fprintf('  Mandatory groups: %d / %d\n', sum(is_mandatory_group), n_groups);

%% 8. Setup Optimization Problem
fprintf('\n8. Setting up symmetric binary topology optimization...\n');

% GA variables = number of symmetry groups (about half of candidates)
n_vars = n_groups;

% Fixed pipe size for topology optimization
fixed_pipe_idx = 5;
fixed_pipe = catalog(fixed_pipe_idx);
fprintf('  Using fixed pipe: %s (D=%.1fmm, t=%.2fmm, Size %s)\n', ...
        fixed_pipe.id, fixed_pipe.D_outer_mm, fixed_pipe.t_mm, fixed_pipe.size_class);

% FSAE-correct boundary conditions for torsion test
% Rear lower nodes = suspension mounts (fixed translation)
% Front lower nodes = minimal constraint + couple force for torque
bc_config = struct();
bc_config.rear_lower_nodes = [5, 6];    % Rear lower left/right
bc_config.front_lower_nodes = [1, 2];   % Front lower left/right
bc_config.rear_upper_nodes = [7, 8];    % Rear upper left/right  
bc_config.front_upper_nodes = [3, 4];   % Front upper left/right
bc_config.torque_Nm = 1000;

[bcs, loads] = generate_fsae_boundary_conditions(all_nodes, 'torsion', bc_config);

% Material properties
mat = materials.(fixed_pipe.material_id);

% Optimization configuration
config.W_mass = 0.4;        % Weight for mass term
config.W_stiff = 0.6;       % Weight for stiffness term
config.mass_ref = 15.0;     % Reference mass [kg]
config.K_min = 500;         % Minimum stiffness [Nm/deg]
config.K_target = 1200;     % Target stiffness [Nm/deg]
config.gamma_stiff = 100;   % Penalty multiplier for low stiffness

fprintf('  Optimization weights: Mass=%.1f, Stiffness=%.1f\n', ...
        config.W_mass, config.W_stiff);
fprintf('  K_min=%.0f, K_target=%.0f, gamma_stiff=%.0f\n', ...
        config.K_min, config.K_target, config.gamma_stiff);

% Objective function (x is per-group, evaluateTopology expands via symmetry_group)
objective_func = @(x) evaluateTopology(x, all_nodes, candidate_elements, ...
                                       is_mandatory, fixed_pipe, mat, ...
                                       bcs, loads, config);

%% 9. GA Configuration
fprintf('\n9. Configuring GA for symmetric binary topology optimization...\n');

% Binary variables: 0 or 1, one per symmetry group
IntCon = 1:n_vars;  % All integer
lb = zeros(n_vars, 1);
ub = ones(n_vars, 1);

A = [];
b = [];
Aeq = [];
beq = [];

% Custom initial population with diverse density tiers
PopSize = 100;
InitialPopulationMatrix = zeros(PopSize, n_vars);
optional_idx = find(~is_mandatory_group);
n_optional = length(optional_idx);

for i = 1:PopSize
    % Always include mandatory groups
    InitialPopulationMatrix(i, is_mandatory_group) = 1;

    if i <= 10
        % Tier 1: Full structures (60-80% optional elements)
        n_add = randi([round(0.6*n_optional), round(0.8*n_optional)]);
    elseif i <= 30
        % Tier 2: Medium density (35-55% optional elements)
        n_add = randi([round(0.35*n_optional), round(0.55*n_optional)]);
    else
        % Tier 3: Sparse exploration (20-40% optional elements)
        n_add = randi([round(0.2*n_optional), round(0.4*n_optional)]);
    end

    n_add = min(n_add, n_optional);  % Safety clamp
    add_idx = optional_idx(randperm(n_optional, n_add));
    InitialPopulationMatrix(i, add_idx) = 1;
end

ga_opts = optimoptions('ga', ...
    'PopulationSize', PopSize, ...
    'MaxGenerations', 60, ...
    'EliteCount', 10, ...
    'CrossoverFraction', 0.8, ...
    'MutationFcn', {@mutationuniform, 0.05}, ...
    'Display', 'iter', ...
    'PlotFcn', @gaplotbestf, ...
    'UseParallel', false, ...
    'InitialPopulationMatrix', InitialPopulationMatrix);

fprintf('  Population: %d, Variables: %d (symmetric groups)\n', ...
        ga_opts.PopulationSize, n_vars);
fprintf('  Generations: %d, EliteCount: %d, MutationRate: 0.05\n', ...
        ga_opts.MaxGenerations, ga_opts.EliteCount);

%% 10. Run Optimization
fprintf('\n10. Running GA topology optimization (symmetric)...\n');
fprintf('    (This may take several minutes...)\n\n');

tic;
[best_x, best_fval, exitflag, output] = ga(objective_func, n_vars, ...
    A, b, Aeq, beq, lb, ub, [], IntCon, ga_opts);
elapsed = toc;

fprintf('\n=== Optimization Complete ===\n');
fprintf('Elapsed time: %.1f seconds\n', elapsed);
fprintf('Best objective: %.4f\n', best_fval);
fprintf('Exit flag: %d\n', exitflag);

%% 11. Analyze Result
fprintf('\n11. Analyzing optimal topology...\n');

% Expand group variables to full element selection
best_x_full = best_x(symmetry_group);
best_x_full(is_mandatory) = 1;  % Enforce mandatory
selected_idx = (best_x_full > 0.5);
selected_elements = candidate_elements(selected_idx, :);
n_selected = sum(selected_idx);

fprintf('  Selected pipes: %d / %d candidates (%.1f%%)\n', ...
        n_selected, n_candidates, 100*n_selected/n_candidates);
fprintf('  Mandatory pipes: %d (all included)\n', sum(is_mandatory & selected_idx));

% Evaluate final design
[~, results] = evaluateTopology(best_x, all_nodes, candidate_elements, ...
                                is_mandatory, fixed_pipe, mat, bcs, loads, config);

fprintf('\n=== Final Design Performance ===\n');
fprintf('Mass:               %.2f kg\n', results.mass);
fprintf('Torsional rigidity: %.1f Nm/deg\n', results.K_torsion);
fprintf('Exclusion penalty:  %.0f\n', results.exclusion_penalty);
if isfield(results, 'connectivity_penalty')
    fprintf('Connectivity penalty: %.1f\n', results.connectivity_penalty);
end
if isfield(results, 'edge_length_penalty')
    fprintf('Edge length penalty:  %.4f\n', results.edge_length_penalty);
end

%% 12. Structural Health Check
fprintf('\n12. Structural health check...\n');

health_ok = true;

% Check lower side rails exist
lower_rails = [1,9; 9,5; 2,10; 10,6];
for i = 1:size(lower_rails, 1)
    n1 = lower_rails(i, 1);
    n2 = lower_rails(i, 2);
    found = false;
    for j = 1:n_selected
        if (selected_elements(j,1)==n1 && selected_elements(j,2)==n2) || ...
           (selected_elements(j,1)==n2 && selected_elements(j,2)==n1)
            found = true;
            break;
        end
    end
    if found
        fprintf('  [OK] Lower rail %d-%d present\n', n1, n2);
    else
        fprintf('  [NG] Lower rail %d-%d MISSING\n', n1, n2);
        health_ok = false;
    end
end

% Check upper side rails exist
upper_rails = [3,11; 11,7; 4,12; 12,8];
for i = 1:size(upper_rails, 1)
    n1 = upper_rails(i, 1);
    n2 = upper_rails(i, 2);
    found = false;
    for j = 1:n_selected
        if (selected_elements(j,1)==n1 && selected_elements(j,2)==n2) || ...
           (selected_elements(j,1)==n2 && selected_elements(j,2)==n1)
            found = true;
            break;
        end
    end
    if found
        fprintf('  [OK] Upper rail %d-%d present\n', n1, n2);
    else
        fprintf('  [NG] Upper rail %d-%d MISSING\n', n1, n2);
        health_ok = false;
    end
end

% Check all structural nodes are connected (BFS)
max_node_sel = max(selected_elements(:));
adj_check = cell(max_node_sel, 1);
for i = 1:n_selected
    u = selected_elements(i, 1);
    v = selected_elements(i, 2);
    adj_check{u}(end+1) = v;
    adj_check{v}(end+1) = u;
end

visited_check = false(max_node_sel, 1);
queue_check = 1;
visited_check(1) = true;
while ~isempty(queue_check)
    curr = queue_check(1);
    queue_check(1) = [];
    for k = 1:length(adj_check{curr})
        nb = adj_check{curr}(k);
        if ~visited_check(nb)
            visited_check(nb) = true;
            queue_check(end+1) = nb; %#ok<SAGROW>
        end
    end
end

n_disconnected = 0;
for i = 1:8
    if ~visited_check(i)
        fprintf('  [NG] Structural node %d is DISCONNECTED\n', i);
        n_disconnected = n_disconnected + 1;
        health_ok = false;
    end
end
if n_disconnected == 0
    fprintf('  [OK] All 8 structural nodes connected\n');
end

% Check element count range
if n_selected >= 20 && n_selected <= 45
    fprintf('  [OK] Element count %d in target range [20-45]\n', n_selected);
else
    fprintf('  [WARN] Element count %d outside target range [20-45]\n', n_selected);
end

% Check torsional stiffness
if results.K_torsion >= 500
    fprintf('  [OK] K_torsion = %.1f Nm/deg >= 500\n', results.K_torsion);
else
    fprintf('  [NG] K_torsion = %.1f Nm/deg < 500\n', results.K_torsion);
    health_ok = false;
end

% Check mass range
if results.mass >= 8 && results.mass <= 14
    fprintf('  [OK] Mass = %.2f kg in target range [8-14]\n', results.mass);
else
    fprintf('  [WARN] Mass = %.2f kg outside target range [8-14]\n', results.mass);
end

if health_ok
    fprintf('\n  >>> STRUCTURAL HEALTH CHECK: ALL PASSED <<<\n');
else
    fprintf('\n  >>> STRUCTURAL HEALTH CHECK: ISSUES FOUND <<<\n');
end

%% 13. Visualize Result
fprintf('\n13. Visualizing optimized frame...\n');

figure('Name', 'Topology Optimization Result', 'Position', [100, 100, 1400, 600]);

% Left: Selected topology
subplot(1, 2, 1);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Optimized Topology (%d pipes, %.1f kg, K=%.0f Nm/deg)', ...
      n_selected, results.mass, results.K_torsion));

for i = 1:n_selected
    n1 = all_nodes(selected_elements(i, 1), :);
    n2 = all_nodes(selected_elements(i, 2), :);

    % Find original index for mandatory check
    orig_idx = find(selected_idx, i, 'first');
    orig_idx = orig_idx(end);

    % Color: red for mandatory, blue for optimized
    if is_mandatory(orig_idx)
        color = 'r';
        width = 3;
    else
        color = 'b';
        width = 2;
    end

    plot3([n1(1), n2(1)], [n1(2), n2(2)], [n1(3), n2(3)], ...
          'Color', color, 'LineWidth', width);
end

scatter3(all_nodes(:,1), all_nodes(:,2), all_nodes(:,3), 50, 'k', 'filled');

% Plot exclusion zones
for i = 1:length(config.exclusion_zones)
    zone = config.exclusion_zones{i};
    if strcmp(zone.type, 'AABB')
        xmin = zone.min_bounds(1); xmax = zone.max_bounds(1);
        ymin = zone.min_bounds(2); ymax = zone.max_bounds(2);
        zmin = zone.min_bounds(3); zmax = zone.max_bounds(3);

        vertices = [
            xmin, ymin, zmin; xmax, ymin, zmin; xmax, ymax, zmin; xmin, ymax, zmin;
            xmin, ymin, zmax; xmax, ymin, zmax; xmax, ymax, zmax; xmin, ymax, zmax;
        ];
        faces = [1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8];

        patch('Vertices', vertices, 'Faces', faces, ...
              'FaceColor', 'red', 'FaceAlpha', 0.1, 'EdgeColor', 'red');
    end
end

view(3); rotate3d on;
legend({'Mandatory', 'Optimized', 'Nodes', 'Exclusion'}, 'Location', 'best');

% Right: Convergence
subplot(1, 2, 2);
hold on; grid on;
xlabel('Generation'); ylabel('Objective Value');
title('Optimization Convergence');

if isfield(output, 'bestfval')
    plot(output.bestfval, 'b-', 'LineWidth', 2);
    legend('Best Objective');
end

fprintf('\n=== Topology Optimization Demo Complete ===\n');
