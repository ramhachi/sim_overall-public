%% TCP版FEMソルバーの性能テスト
% TCP通信版（beam_fem_solver_tcp）と従来のDocker版（beam_fem_solver）の
% 性能を比較するスクリプト

clear; close all; clc;

fprintf('=== TCP版FEMソルバー 性能テスト ===\n\n');

%% 1. データ準備（16ノード、16要素フレーム）
fprintf('1. テストデータ準備中...\n');

% Add path
script_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(script_dir);
addpath(fullfile(project_dir, 'matlab'));
addpath(fullfile(project_dir, 'data'));

% Load materials and pipes
materials = loadMaterials(fullfile(project_dir, 'data', 'materials.json'));
pipes = loadPipeCatalog(fullfile(project_dir, 'data', 'pipe_catalog.json'), materials);

% Create simple frame geometry (same as demo_simple_frame)
nodes = [
    0.0, -0.6, 0.0;    % 1: Front-left bottom
    0.0,  0.6, 0.0;    % 2: Front-right bottom
    0.0, -0.6, 1.0;    % 3: Front-left top
    0.0,  0.6, 1.0;    % 4: Front-right top
    1.2, -0.6, 0.0;    % 5: Mid-left bottom
    1.2,  0.6, 0.0;    % 6: Mid-right bottom
    1.2, -0.6, 1.0;    % 7: Mid-left top
    1.2,  0.6, 1.0;    % 8: Mid-right top
    2.4, -0.6, 0.0;    % 9: Rear-left bottom
    2.4,  0.6, 0.0;    % 10: Rear-right bottom
    2.4, -0.6, 1.4;    % 11: Rear-left top
    2.4,  0.6, 1.4;    % 12: Rear-right top
    1.2, -0.6, 1.4;    % 13: Mid-left upper
    1.2,  0.6, 1.4;    % 14: Mid-right upper
    0.0, -0.6, 1.4;    % 15: Front-left upper
    0.0,  0.6, 1.4;    % 16: Front-right upper
];

% Use standard AISI 4130 pipe: 25.4mm x 1.65mm (Size B)
pipe_idx = find(strcmp({pipes.id}, 'STL-R-25.4x1.65'));
if isempty(pipe_idx)
    error('Could not find standard pipe in catalog');
end
pipe = pipes(pipe_idx);
mat = materials.(pipe.material_id);

% Element connectivity (16 elements)
connectivity = [
    1,3; 3,4; 4,2; 2,1;       % Front hoop (4 elements)
    5,7; 7,8; 8,6; 6,5;       % Mid section (4 elements)
    9,11; 11,12; 12,10; 10,9; % Rear hoop (4 elements)
    1,5; 2,6; 3,7; 4,8;       % Longitudinal rails (4 elements)
];

n_elem = size(connectivity, 1);
elements = zeros(n_elem, 9);

% Extract values as scalars
D_out = double(pipe.D_outer_mm);
t_val = double(pipe.t_mm);
E_val = double(mat.E_GPa);
G_val = double(mat.G_GPa);
rho_val = double(mat.rho_kg_m3);

fprintf('Debug: D_out=%g, t_val=%g, E_val=%g, G_val=%g, rho_val=%g\n', ...
    D_out, t_val, E_val, G_val, rho_val);
fprintf('Debug: sizes - D_out:%s, t_val:%s, E_val:%s, G_val:%s, rho_val:%s\n', ...
    mat2str(size(D_out)), mat2str(size(t_val)), mat2str(size(E_val)), ...
    mat2str(size(G_val)), mat2str(size(rho_val)));

for e = 1:n_elem
    row = [double(connectivity(e,1)), double(connectivity(e,2)), ...
           D_out, t_val, E_val, G_val, rho_val, 1.0, 0.7];
    fprintf('Debug: element %d, row length=%d\n', e, length(row));
    if length(row) ~= 9
        error('Row length is %d, expected 9', length(row));
    end
    elements(e,:) = row;
end

% Boundary conditions: Fix front lower corners (nodes 1, 2)
bcs = [
    1, 1, 0;  1, 2, 0;  1, 3, 0;  1, 4, 0;  1, 5, 0;  1, 6, 0;
    2, 1, 0;  2, 2, 0;  2, 3, 0;  2, 4, 0;  2, 5, 0;  2, 6, 0;
];

% Loads: Apply torsion (±1000 Nm) at rear corners (nodes 9, 10)
loads = [
    9, 4, -1000;   % Rear-left: -1000 Nm
    10, 4, 1000;   % Rear-right: +1000 Nm
];

fprintf('   準備完了: %d nodes, %d elements\n\n', size(nodes,1), n_elem);

%% 2. TCP版の性能測定
fprintf('2. TCP版の性能測定\n');
fprintf('   (初回はサーバー起動で約2秒かかります)\n');

tcp_times = zeros(10, 1);
for i = 1:10
    tic;
    result_tcp = beam_fem_solver_tcp(nodes, elements, bcs, loads);
    tcp_times(i) = toc;
    if i == 1
        fprintf('   初回: %.3f 秒 (サーバー起動含む)\n', tcp_times(i));
        tcp_mass = result_tcp.mass;
        tcp_K = result_tcp.K_torsion;
    elseif i == 2
        fprintf('   2回目以降: %.1f ms\n', tcp_times(i)*1000);
    end
end

% サーバーをシャットダウン
beam_fem_solver_tcp('shutdown');

fprintf('   平均時間 (2-10回目): %.1f ms\n', mean(tcp_times(2:end))*1000);
fprintf('   標準偏差: %.1f ms\n', std(tcp_times(2:end))*1000);
fprintf('   質量: %.2f kg, ねじり剛性: %.1f Nm/deg\n\n', tcp_mass, tcp_K);

%% 3. Docker版の性能測定（比較用）
fprintf('3. Docker版の性能測定（比較用）\n');

docker_times = zeros(5, 1);
for i = 1:5
    tic;
    result_docker = beam_fem_solver(nodes, elements, bcs, loads);
    docker_times(i) = toc;
    if i == 1
        docker_mass = result_docker.mass;
        docker_K = result_docker.K_torsion;
    end
end

fprintf('   平均時間: %.1f ms\n', mean(docker_times)*1000);
fprintf('   標準偏差: %.1f ms\n', std(docker_times)*1000);
fprintf('   質量: %.2f kg, ねじり剛性: %.1f Nm/deg\n\n', docker_mass, docker_K);

%% 4. 結果比較
fprintf('=== 性能比較 ===\n');
fprintf('TCP版 (2回目以降):  %.1f ms\n', mean(tcp_times(2:end))*1000);
fprintf('Docker版:           %.1f ms\n', mean(docker_times)*1000);
speedup = mean(docker_times) / mean(tcp_times(2:end));
fprintf('高速化率:           %.1f倍\n\n', speedup);

% 結果の一致性確認
fprintf('=== 結果の一致性 ===\n');
fprintf('質量誤差:           %.2e kg (%.2e%%)\n', ...
    abs(tcp_mass - docker_mass), abs(tcp_mass - docker_mass)/docker_mass*100);
fprintf('剛性誤差:           %.2e Nm/deg (%.2e%%)\n', ...
    abs(tcp_K - docker_K), abs(tcp_K - docker_K)/docker_K*100);

if abs(tcp_mass - docker_mass)/docker_mass < 1e-10 && ...
   abs(tcp_K - docker_K)/docker_K < 1e-10
    fprintf('✓ TCP版とDocker版の結果が一致しました\n\n');
else
    fprintf('⚠ TCP版とDocker版の結果に差異があります\n\n');
end

%% 5. 可視化
figure('Position', [100, 100, 800, 500]);

subplot(1,2,1);
bar([mean(tcp_times(2:end))*1000, mean(docker_times)*1000]);
set(gca, 'XTickLabel', {'TCP版', 'Docker版'});
ylabel('計算時間 [ms]');
title('性能比較');
grid on;

subplot(1,2,2);
plot(2:10, tcp_times(2:end)*1000, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
plot(1:5, docker_times*1000, 's-', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('評価回数');
ylabel('計算時間 [ms]');
legend('TCP版', 'Docker版', 'Location', 'best');
title('評価時間の推移');
grid on;

fprintf('✓ 性能テスト完了\n');
