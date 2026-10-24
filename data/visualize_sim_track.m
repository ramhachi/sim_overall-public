
% visualize_sim_track.m
% ASE_autocross_sim.csv (シミュレーション用) を可視化して確認するスクリプト

filename = 'ASE_autocross_sim.csv';
if ~isfile(filename)
    error('File not found: %s', filename);
end

data = readtable(filename);
x = data.x;
y = data.y;
w = data.width;

figure('Name', 'Check Simulation Track', 'Color', 'w');
hold on; grid on; axis equal;

% ドットでプロット（順序がわかるように色を変える）
scatter(x, y, 20, 1:length(x), 'filled');
colormap(jet);
colorbar;
title('Simulation Track Points (Color = Index)');
xlabel('X [m]');
ylabel('Y [m]');

% 開始点と終了点
plot(x(1), y(1), 'go', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Start');
plot(x(end), y(end), 'rx', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'End');
legend('Location', 'best');

% 接続線
plot(x, y, 'k-', 'Color', [0.5 0.5 0.5 0.3]);

% データ分析
dists = sqrt(diff(x).^2 + diff(y).^2);
fprintf('Total points: %d\n', length(x));
fprintf('Min dist: %.4f m\n', min(dists));
fprintf('Max dist: %.4f m\n', max(dists));
fprintf('Mean dist: %.4f m\n', mean(dists));

% 急激な変化の検出 (平均の3倍以上)
threshold = mean(dists) * 3;
jumps = find(dists > threshold);
if ~isempty(jumps)
    fprintf('Warning: Large jumps detected at indices:\n');
    disp(jumps);
    plot(x(jumps), y(jumps), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end
