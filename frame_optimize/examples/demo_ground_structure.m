%% DEMO_GROUND_STRUCTURE - Ground Structure法によるトポロジー最適化
%
% このスクリプトは、ノード追加とトポロジー最適化を実演します：
% 1. 基本ノード（フープ）を定義
% 2. Ground Structure法で候補要素を自動生成
% 3. 阻止領域（コクピット、エンジン）を考慮
% 4. GAで最適なトポロジーを探索

clear; close all; clc;

fprintf('=== Ground Structure Topology Optimization Demo ===\n\n');

%% 1. Load Data
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

fprintf('1. Loading materials and pipe catalogs...\n');
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

%% 2. Define Minimal Base Nodes (FSAE規則で必須の構造のみ)
fprintf('\n2. Defining base nodes (Front Hoop, Main Hoop only)...\n');

L = 2.4; W = 1.2; H_front = 1.0; H_rear = 1.4;

% 必須ノード（8点）：フロントフープとメインフープのみ
base_nodes = [
    % Front Hoop (nodes 1-4)
    0, -W/2, 0;           % 1: Front lower left
    0, W/2, 0;            % 2: Front lower right
    0, -W/2, H_front;     % 3: Front upper left
    0, W/2, H_front;      % 4: Front upper right

    % Main Hoop (nodes 5-8)
    L, -W/2, 0;           % 5: Rear lower left
    L, W/2, 0;            % 6: Rear lower right
    L, -W/2, H_rear;      % 7: Rear upper left
    L, W/2, H_rear;       % 8: Rear upper right
];

% 追加候補ノード（トポロジー最適化で使用可能）
additional_nodes = [
    % Mid points (potential side impact rails)
    L/2, -W/2, 0;         % 9
    L/2, W/2, 0;          % 10
    L/2, -W/2, H_front;   % 11
    L/2, W/2, H_front;    % 12

    % Diagonal support points
    L/4, -W/2, H_front/2; % 13
    L/4, W/2, H_front/2;  % 14
    3*L/4, -W/2, H_rear*0.7; % 15
    3*L/4, W/2, H_rear*0.7;  % 16

    % Additional bracing points
    L/3, 0, H_front*0.8;  % 17
    2*L/3, 0, H_rear*0.8; % 18
];

% 全ノード結合
all_nodes = [base_nodes; additional_nodes];
n_nodes = size(all_nodes, 1);

fprintf('  Base nodes: %d (mandatory hoops)\n', size(base_nodes, 1));
fprintf('  Additional nodes: %d (optional bracing)\n', size(additional_nodes, 1));
fprintf('  Total nodes: %d\n', n_nodes);

%% 3. Define Exclusion Zones
fprintf('\n3. Defining exclusion zones...\n');

config = struct();
config.exclusion_zones = {};

% Cockpit (driver space)
cockpit = struct();
cockpit.type = 'AABB';
cockpit.min_bounds = [0.3, -0.3, 0];
cockpit.max_bounds = [1.5, 0.3, 1.0];
cockpit.name = 'Cockpit';
config.exclusion_zones{end+1} = cockpit;

% Engine
engine = struct();
engine.type = 'AABB';
engine.min_bounds = [1.8, -0.4, 0];
engine.max_bounds = [2.4, 0.4, 0.6];
engine.name = 'Engine';
config.exclusion_zones{end+1} = engine;

fprintf('  Defined %d exclusion zones\n', length(config.exclusion_zones));

%% 4. Generate Ground Structure
fprintf('\n4. Generating ground structure...\n');

config.max_length_mm = 2000;  % 最大2mのパイプ

% Ground Structure生成（阻止領域を避けて候補要素を生成）
candidate_elements = generateGroundStructure(all_nodes, config);

fprintf('  Total candidate pipes: %d (from %d possible)\n', ...
        size(candidate_elements, 1), n_nodes*(n_nodes-1)/2);

%% 5. Visualize Ground Structure
fprintf('\n5. Visualizing ground structure...\n');

figure('Name', 'Ground Structure', 'Position', [100, 100, 1200, 600]);

% Left: All candidate elements
subplot(1, 2, 1);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Ground Structure (%d candidates)', size(candidate_elements, 1)));

% Plot all candidate elements in gray
for i = 1:size(candidate_elements, 1)
    n1 = all_nodes(candidate_elements(i, 1), :);
    n2 = all_nodes(candidate_elements(i, 2), :);
    plot3([n1(1), n2(1)], [n1(2), n2(2)], [n1(3), n2(3)], ...
          'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5);
end

% Plot nodes
scatter3(all_nodes(:,1), all_nodes(:,2), all_nodes(:,3), 50, 'r', 'filled');

% Plot base nodes in black (mandatory)
scatter3(base_nodes(:,1), base_nodes(:,2), base_nodes(:,3), 100, 'k', 'filled');

% Plot exclusion zones
for i = 1:length(config.exclusion_zones)
    zone = config.exclusion_zones{i};
    if strcmp(zone.type, 'AABB')
        % Draw bounding box
        xmin = zone.min_bounds(1); xmax = zone.max_bounds(1);
        ymin = zone.min_bounds(2); ymax = zone.max_bounds(2);
        zmin = zone.min_bounds(3); zmax = zone.max_bounds(3);

        vertices = [
            xmin, ymin, zmin; xmax, ymin, zmin; xmax, ymax, zmin; xmin, ymax, zmin;
            xmin, ymin, zmax; xmax, ymin, zmax; xmax, ymax, zmax; xmin, ymax, zmax;
        ];
        faces = [
            1,2,3,4; 5,6,7,8; 1,2,6,5; 2,3,7,6; 3,4,8,7; 4,1,5,8;
        ];

        patch('Vertices', vertices, 'Faces', faces, ...
              'FaceColor', 'red', 'FaceAlpha', 0.2, 'EdgeColor', 'red');
    end
end

view(3); rotate3d on;
legend({'Candidates', 'Nodes', 'Mandatory', 'Exclusion'}, 'Location', 'best');

% Right: Mandatory structure only
subplot(1, 2, 2);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Mandatory Structure (Front + Main Hoop)');

% Front hoop
plot3([base_nodes(1,1), base_nodes(2,1)], [base_nodes(1,2), base_nodes(2,2)], [base_nodes(1,3), base_nodes(2,3)], 'b-', 'LineWidth', 3);
plot3([base_nodes(2,1), base_nodes(4,1)], [base_nodes(2,2), base_nodes(4,2)], [base_nodes(2,3), base_nodes(4,3)], 'b-', 'LineWidth', 3);
plot3([base_nodes(4,1), base_nodes(3,1)], [base_nodes(4,2), base_nodes(3,2)], [base_nodes(4,3), base_nodes(3,3)], 'b-', 'LineWidth', 3);
plot3([base_nodes(3,1), base_nodes(1,1)], [base_nodes(3,2), base_nodes(1,2)], [base_nodes(3,3), base_nodes(1,3)], 'b-', 'LineWidth', 3);

% Main hoop
plot3([base_nodes(5,1), base_nodes(6,1)], [base_nodes(5,2), base_nodes(6,2)], [base_nodes(5,3), base_nodes(6,3)], 'r-', 'LineWidth', 3);
plot3([base_nodes(6,1), base_nodes(8,1)], [base_nodes(6,2), base_nodes(8,2)], [base_nodes(6,3), base_nodes(8,3)], 'r-', 'LineWidth', 3);
plot3([base_nodes(8,1), base_nodes(7,1)], [base_nodes(8,2), base_nodes(7,2)], [base_nodes(8,3), base_nodes(7,3)], 'r-', 'LineWidth', 3);
plot3([base_nodes(7,1), base_nodes(5,1)], [base_nodes(7,2), base_nodes(5,2)], [base_nodes(7,3), base_nodes(5,3)], 'r-', 'LineWidth', 3);

scatter3(base_nodes(:,1), base_nodes(:,2), base_nodes(:,3), 100, 'k', 'filled');
view(3); rotate3d on;

%% Summary
fprintf('\n=== Ground Structure Generation Complete ===\n');
fprintf('Total nodes: %d\n', n_nodes);
fprintf('  - Mandatory (hoops): %d\n', size(base_nodes, 1));
fprintf('  - Optional (bracing): %d\n', size(additional_nodes, 1));
fprintf('Candidate pipes: %d (%.1f%% of theoretical maximum)\n', ...
        size(candidate_elements, 1), ...
        100 * size(candidate_elements, 1) / (n_nodes*(n_nodes-1)/2));
fprintf('\nNext step: Run GA optimization to select optimal topology\n');
fprintf('  (This would optimize which pipes to include from candidates)\n');
