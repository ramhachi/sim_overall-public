%% run_14dof_verification.m
% 14DOF C++ vs MATLAB 検証スクリプト

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' 14DOF Full Vehicle Model Verification\n');
fprintf('==========================================\n\n');

%% パラメータ設定（C++と完全一致）
params = struct();

% 質量・慣性
params.mass.m_sprung = 200;
params.mass.m_unsprung = [10, 10, 10, 10];
params.mass.a = 0.813;
params.mass.b = 0.787;
params.mass.L = params.mass.a + params.mass.b;
params.mass.track_f = 1.2;
params.mass.track_r = 1.18;
params.mass.h_cg = 0.28;
params.mass.Ixx = 32;
params.mass.Iyy = 95;
params.mass.Izz = 85;
params.mass.Iw = 0.15;

% サスペンション
params.suspension.k_spring = [25000, 25000, 30000, 30000];
params.suspension.motion_ratio = [0.9, 0.9, 0.9, 0.9];

% Bouc-Wenパラメータ
params.suspension.bw_c0 = 2000;
params.suspension.bw_k0 = 0;
params.suspension.bw_alpha = 1500;
params.suspension.bw_A = 100;
params.suspension.bw_beta = 300;
params.suspension.bw_gamma = 0;
params.suspension.bw_n = 1;

% LuGreパラメータ
params.suspension.lugre_sigma0 = 10000;
params.suspension.lugre_sigma1 = 10;
params.suspension.lugre_sigma2 = 0;
params.suspension.lugre_Fc = 20;
params.suspension.lugre_Fs = 30;
params.suspension.lugre_vs = 0.01;

% ARB
params.suspension.k_arb_f = 5000;
params.suspension.k_arb_r = 3000;

% バンプストップ
params.suspension.bump_stop_gap = 0.03;
params.suspension.bump_stop_k = 100000;

% タイヤ
params.tire.radius = 0.228;
params.tire.mu_x_max = 1.6;
params.tire.mu_y_max = 2.0;   % レーシングタイヤ（実測ベース）
params.tire.Fz_nom = 1500;
params.tire.Lx = 0.15;
params.tire.Ly = 0.25;
params.tire.Cx = 80000;
params.tire.Cy = 60000;
params.tire.v_N = 0.5;
params.tire.k_tire = 200000;
params.tire.c_tire = 500;

% 空力
params.aero.rho = 1.2;
params.aero.CL_A = 2.5;
params.aero.CD_A = 1.0;

% パワートレイン
params.powertrain.max_torque = 55;
params.powertrain.gear_ratio = 3.0;
params.powertrain.efficiency = 0.9;

% 初速
params.v0 = 15;

%% シミュレーション設定
t_span = [0, 5];
dt = 0.001;

% 入力関数
t_steer = 1.0;
delta = 0.1;
input_func = @(t) struct('throttle', 0.1, 'steering', (t >= t_steer) * delta);

%% MATLAB シミュレーション実行
fprintf('Running MATLAB 14DOF simulation...\n');
tic;
[t_m, state_m, data_m] = vehicle_14dof_model(params, t_span, dt, input_func);
fprintf('MATLAB simulation time: %.2f s\n', toc);

% 静的荷重確認
W = params.mass.m_sprung * 9.81;
Wf = W * params.mass.b / params.mass.L;
Wr = W * params.mass.a / params.mass.L;
fprintf('Static Fz: FL=%.0f, FR=%.0f, RL=%.0f, RR=%.0f N\n', Wf/2, Wf/2, Wr/2, Wr/2);

% 初期タイヤ荷重確認
fprintf('Initial Fz: FL=%.0f, FR=%.0f, RL=%.0f, RR=%.0f N\n', ...
    data_m.Fz(1,1), data_m.Fz(1,2), data_m.Fz(1,3), data_m.Fz(1,4));

%% 結果プロット
figure('Position', [50, 50, 1600, 900]);

% 軌跡
subplot(2,4,1);
plot(state_m(:,1), state_m(:,2), 'b-', 'LineWidth', 1.5);
xlabel('X [m]');
ylabel('Y [m]');
title('Vehicle Trajectory');
axis equal;
grid on;

% 速度
subplot(2,4,2);
plot(t_m, state_m(:,7), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('vx [m/s]');
title('Forward Velocity');
grid on;

% ロール角
subplot(2,4,3);
plot(t_m, state_m(:,4)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Roll [deg]');
title('Roll Angle');
grid on;

% ピッチ角
subplot(2,4,4);
plot(t_m, state_m(:,5)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Pitch [deg]');
title('Pitch Angle');
grid on;

% タイヤ垂直荷重
subplot(2,4,5);
plot(t_m, data_m.Fz);
xlabel('Time [s]');
ylabel('Force [N]');
title('Tire Vertical Loads');
legend('FL', 'FR', 'RL', 'RR');
grid on;

% Bouc-Wen力
subplot(2,4,6);
plot(t_m, data_m.F_bw);
xlabel('Time [s]');
ylabel('Force [N]');
title('Bouc-Wen Damper Forces');
legend('FL', 'FR', 'RL', 'RR');
grid on;

% LuGre力
subplot(2,4,7);
plot(t_m, data_m.F_lugre);
xlabel('Time [s]');
ylabel('Force [N]');
title('LuGre Friction Forces');
legend('FL', 'FR', 'RL', 'RR');
grid on;

% ヨーレート
subplot(2,4,8);
plot(t_m, state_m(:,12)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Yaw Rate [deg/s]');
title('Yaw Rate');
grid on;

sgtitle('14DOF Full Vehicle Model - MATLAB');

%% 結果サマリー
fprintf('\n=== Results Summary ===\n');
fprintf('Final position: (%.2f, %.2f) m\n', state_m(end,1), state_m(end,2));
fprintf('Final heading: %.1f deg\n', state_m(end,6)*180/pi);
fprintf('Final speed: %.2f m/s\n', sqrt(state_m(end,7)^2 + state_m(end,8)^2));
fprintf('Max roll angle: %.3f deg\n', max(abs(state_m(:,4)))*180/pi);
fprintf('Max pitch angle: %.3f deg\n', max(abs(state_m(:,5)))*180/pi);

% 旋回半径
final_speed = sqrt(state_m(end,7)^2 + state_m(end,8)^2);
final_yaw_rate = abs(state_m(end,12));
if final_yaw_rate > 0.01
    turning_radius = final_speed / final_yaw_rate;
    fprintf('Turning radius: %.1f m\n', turning_radius);
end

%% C++ 結果読み込み（オプショナル）
cpp_file = '';
search_paths = {'../output/14dof_result.csv', ...
    '../../output/14dof_result.csv', ...
    '14dof_result.csv'};
for i = 1:length(search_paths)
    if exist(search_paths{i}, 'file')
        cpp_file = search_paths{i};
        fprintf('\nFound C++ result: %s\n', cpp_file);
        break;
    end
end

if ~isempty(cpp_file)
    cpp_data = readtable(cpp_file);
    fprintf('Loaded C++ result: %d data points\n', height(cpp_data));
    
    % 比較プロット
    figure('Position', [100, 100, 1400, 600]);
    
    t_cpp = cpp_data.time;
    idx_m = round(t_cpp / dt) + 1;
    idx_m = min(idx_m, size(state_m, 1));
    
    subplot(2,3,1);
    plot(cpp_data.x, cpp_data.y, 'b-', 'LineWidth', 1.5); hold on;
    plot(state_m(idx_m,1), state_m(idx_m,2), 'r--', 'LineWidth', 1.5);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Trajectory Comparison');
    legend('C++', 'MATLAB', 'Location', 'best');
    axis equal; grid on;
    
    subplot(2,3,2);
    plot(t_cpp, cpp_data.phi, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_cpp, state_m(idx_m,4)*180/pi, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Roll [deg]');
    title('Roll Angle Comparison');
    legend('C++', 'MATLAB', 'Location', 'best');
    grid on;
    
    subplot(2,3,3);
    plot(t_cpp, cpp_data.vx, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_cpp, state_m(idx_m,7), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('vx [m/s]');
    title('Forward Velocity Comparison');
    legend('C++', 'MATLAB', 'Location', 'best');
    grid on;
    
    subplot(2,3,4);
    if ismember('Fz_FL', cpp_data.Properties.VariableNames)
        plot(t_cpp, cpp_data.Fz_FL, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_cpp, data_m.Fz(idx_m,1), 'r--', 'LineWidth', 1.5);
        xlabel('Time [s]'); ylabel('Force [N]');
        title('Tire Fz (FL) Comparison');
        legend('C++', 'MATLAB', 'Location', 'best');
        grid on;
    end
    
    subplot(2,3,5);
    err_phi = cpp_data.phi - state_m(idx_m,4)*180/pi;
    plot(t_cpp, err_phi, 'k-', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('Error [deg]');
    title(sprintf('Roll Angle Error (RMSE=%.4f deg)', sqrt(mean(err_phi.^2))));
    grid on;
    
    subplot(2,3,6);
    err_x = cpp_data.x - state_m(idx_m,1);
    err_y = cpp_data.y - state_m(idx_m,2);
    err_pos = sqrt(err_x.^2 + err_y.^2);
    plot(t_cpp, err_pos, 'k-', 'LineWidth', 1.0);
    xlabel('Time [s]'); ylabel('Error [m]');
    title(sprintf('Position Error (max=%.3f m)', max(err_pos)));
    grid on;
    
    sgtitle('14DOF Model C++ vs MATLAB Verification');
    
    % 検証
    max_err_phi = max(abs(err_phi));
    max_err_pos = max(err_pos);
    
    fprintf('\n=== Verification ===\n');
    fprintf('Max roll error: %.4f deg\n', max_err_phi);
    fprintf('Max position error: %.3f m\n', max_err_pos);
    
    if max_err_phi < 1.0 && max_err_pos < 1.0
        fprintf('Status: ✓ PASS\n');
    else
        fprintf('Status: ✗ FAIL\n');
    end
    
    saveas(gcf, '../../output/verify_14dof.png');
    fprintf('\nSaved comparison plot to output/verify_14dof.png\n');
end

fprintf('\n==========================================\n');
