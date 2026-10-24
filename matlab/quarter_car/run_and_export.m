%% run_and_export.m
% Quarter Carシミュレーション実行＆CSV出力
% C++結果との比較用

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' MATLAB Quarter Car Simulation\n');
fprintf('==========================================\n\n');

%% 1. シミュレーション実行
fprintf('1. Running simulation...\n');

tic;
[t, x, data] = quarter_car_ode();
elapsed = toc;

fprintf('   Done in %.3f seconds\n', elapsed);
fprintf('   Data points: %d\n', length(t));

%% 2. CSVに出力（C++と同じ形式）
fprintf('\n2. Exporting to CSV...\n');

% C++と同じ形式でCSV作成
output_data = table(...
    t', ...
    x(:,1) * 1000, ...  % z_s [mm]
    x(:,2) * 1000, ...  % z_u [mm]
    x(:,3) * 1000, ...  % dz_s [mm/s]
    x(:,4) * 1000, ...  % dz_u [mm/s]
    data.F_spring', ... % F_spring [N]
    data.F_damper', ... % F_damper [N]
    data.F_tire', ...   % F_tire [N]
    data.z_road', ...   % z_road [mm]
    'VariableNames', {'time', 'z_s', 'z_u', 'dz_s', 'dz_u', 'F_spring', 'F_damper', 'F_tire', 'z_road'});

% 出力先
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
output_file = fullfile(output_dir, 'quarter_car_matlab.csv');
writetable(output_data, output_file);
fprintf('   Saved: %s\n', output_file);

%% 3. 結果サマリー
fprintf('\n3. Results Summary:\n');
fprintf('   Duration: %.2f s\n', t(end));
fprintf('   Max z_s: %.2f mm\n', max(x(:,1)) * 1000);
fprintf('   Max z_u: %.2f mm\n', max(x(:,2)) * 1000);
fprintf('   Final z_s: %.4f mm\n', x(end,1) * 1000);
fprintf('   Final z_u: %.4f mm\n', x(end,2) * 1000);

%% 4. プロット
fprintf('\n4. Plotting...\n');

figure('Position', [100, 100, 1200, 800]);

% 変位
subplot(2, 2, 1);
plot(t, x(:,1)*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Sprung (z_s)');
hold on;
plot(t, x(:,2)*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Unsprung (z_u)');
plot(t, data.z_road, 'k--', 'LineWidth', 1, 'DisplayName', 'Road input');
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Suspension Displacement');
legend('Location', 'best');
grid on;

% 速度
subplot(2, 2, 2);
plot(t, x(:,3)*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Sprung');
hold on;
plot(t, x(:,4)*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Unsprung');
xlabel('Time [s]');
ylabel('Velocity [mm/s]');
title('Suspension Velocity');
legend('Location', 'best');
grid on;

% 力
subplot(2, 2, 3);
plot(t, data.F_spring, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Spring');
hold on;
plot(t, data.F_damper, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Damper');
plot(t, data.F_tire, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Tire');
xlabel('Time [s]');
ylabel('Force [N]');
title('Suspension Forces');
legend('Location', 'best');
grid on;

% F-Vループ
subplot(2, 2, 4);
v_rel = (x(:,3) - x(:,4)) * 1000;
scatter(v_rel, data.F_damper, 3, t, 'filled');
colorbar;
xlabel('Relative Velocity [mm/s]');
ylabel('Damper Force [N]');
title('Damper Hysteresis');
grid on;

sgtitle('MATLAB Quarter Car Simulation (Ground Truth)');

% 画像保存
plot_file = fullfile(output_dir, 'quarter_car_matlab.png');
saveas(gcf, plot_file);
fprintf('   Saved: %s\n', plot_file);

fprintf('\n==========================================\n');
fprintf(' Complete!\n');
fprintf('==========================================\n');
