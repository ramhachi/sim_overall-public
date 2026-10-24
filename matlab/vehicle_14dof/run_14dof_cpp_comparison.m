%% run_14dof_cpp_comparison.m
% 14DOF MATLAB vs C++ 比較検証スクリプト
% TMeasyタイヤモデル使用版（NNモデルはC++側で問題あり調査中）

clear; clc; close all;

fprintf('=================================================================\n');
fprintf(' 14DOF Full Vehicle Model: MATLAB vs C++ Comparison\n');
fprintf(' (Using TMeasy Tire Model for consistent comparison)\n');
fprintf('=================================================================\n\n');

%% パラメータ設定（C++と完全一致）
fprintf('Step 1: Parameter Setup\n');
params = setup_params_cpp_compatible();
fprintf('  ✓ Parameters configured (C++ compatible)\n\n');

%% シミュレーション設定
fprintf('Step 2: Simulation Setup\n');
t_span = [0, 5];
dt = 0.001;
t_steer = 1.0;
delta = 0.1;  % 5.7 deg

% 入力関数
input_func = @(t) struct('throttle', 0.3, ...
                         'steering', (t >= t_steer) * delta, ...
                         'gear', 3);

fprintf('  Duration:    %.1f s\n', t_span(2));
fprintf('  Time step:   %.3f s\n', dt);
fprintf('  Steer time:  %.1f s\n', t_steer);
fprintf('  Steer angle: %.1f deg\n', delta * 180/pi);
fprintf('  Throttle:    %.1f\n', 0.3);
fprintf('\n');

%% MATLABシミュレーション実行（TMeasyタイヤモデル使用）
fprintf('Step 3: Running MATLAB Simulation (TMeasy Tire Model)\n');
tic;
[t_m, state_m, data_m] = vehicle_14dof_model(params, t_span, dt, input_func);
t_matlab = toc;
fprintf('  ✓ MATLAB simulation completed in %.2f s\n', t_matlab);
fprintf('  Data points: %d\n', length(t_m));
fprintf('  Using TMeasy Tire Model (same as C++)\n\n');

%% 結果分析
fprintf('Step 4: Results Analysis\n');

% 最終状態
x_final = state_m(end, 1);
y_final = state_m(end, 2);
psi_final = state_m(end, 6) * 180/pi;
vx_final = state_m(end, 7);
vy_final = state_m(end, 8);
r_final = state_m(end, 12) * 180/pi;

fprintf('  Final State (t=%.1f s):\n', t_span(2));
fprintf('    Position:  x=%.1f m, y=%.1f m\n', x_final, y_final);
fprintf('    Yaw:       %.1f deg\n', psi_final);
fprintf('    Velocity:  vx=%.2f m/s, vy=%.2f m/s\n', vx_final, vy_final);
fprintf('    Yaw rate:  %.1f deg/s\n', r_final);
fprintf('\n');

% タイヤ荷重統計
Fz_mean = mean(data_m.Fz, 1);
fprintf('  Mean Tire Loads:\n');
fprintf('    FL: %.0f N, FR: %.0f N\n', Fz_mean(1), Fz_mean(2));
fprintf('    RL: %.0f N, RR: %.0f N\n', Fz_mean(3), Fz_mean(4));
fprintf('\n');

%% 結果プロット
fprintf('Step 5: Plotting Results\n');

figure('Position', [50, 50, 1600, 900], 'Name', 'MATLAB 14DOF Results');

% 1. 軌跡
subplot(2,4,1);
plot(state_m(:,1), state_m(:,2), 'b-', 'LineWidth', 1.5);
xlabel('X [m]');
ylabel('Y [m]');
title('Vehicle Trajectory');
axis equal;
grid on;

% 2. 速度
subplot(2,4,2);
plot(t_m, state_m(:,7), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_m, state_m(:,8), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocity');
legend('vx', 'vy');
grid on;

% 3. ヨーレート
subplot(2,4,3);
plot(t_m, state_m(:,12)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Yaw Rate [deg/s]');
title('Yaw Rate');
grid on;

% 4. ロール角
subplot(2,4,4);
plot(t_m, state_m(:,4)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Roll [deg]');
title('Roll Angle');
grid on;

% 5. タイヤ垂直荷重
subplot(2,4,5);
plot(t_m, data_m.Fz, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Force [N]');
title('Tire Vertical Loads');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

% 6. 加速度
subplot(2,4,6);
plot(t_m, data_m.ax, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_m, data_m.ay, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Acceleration [m/s²]');
title('Acceleration');
legend('ax', 'ay');
grid on;

% 7. タイヤ横力
subplot(2,4,7);
plot(t_m, data_m.Fy, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Force [N]');
title('Tire Lateral Forces');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

% 8. タイヤ前後力
subplot(2,4,8);
plot(t_m, data_m.Fx, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Force [N]');
title('Tire Longitudinal Forces');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

fprintf('  ✓ Plots generated\n\n');

%% CSV出力（C++との比較用）
fprintf('Step 6: Exporting CSV for C++ Comparison\n');

% データサイズ確認（デバッグ用）
fprintf('  Data sizes:\n');
fprintf('    t_m:            %d x %d\n', size(t_m));
fprintf('    state_m:        %d x %d\n', size(state_m));
fprintf('    data_m.ax:      %d x %d\n', size(data_m.ax));
fprintf('    data_m.ay:      %d x %d\n', size(data_m.ay));
fprintf('    data_m.Fz:      %d x %d\n', size(data_m.Fz));
fprintf('    state_m(:,1):   %d x %d\n', size(state_m(:,1)));
fprintf('    state_m(:,6)*180/pi: %d x %d\n', size(state_m(:,6)*180/pi));

% CSV用にデータを整形（各列を明示的に列ベクトル化）
csv_filename = 'matlab_14dof_result.csv';

% 各列を作成
col_time = t_m(:);                          % time
col_x = state_m(:,1);                       % x
col_y = state_m(:,2);                       % y
col_psi = state_m(:,6) * 180/pi;           % psi [deg]
col_vx = state_m(:,7);                      % vx
col_vy = state_m(:,8);                      % vy
col_r = state_m(:,12) * 180/pi;            % r [deg/s]
col_ax = data_m.ax(:);                      % ax
col_ay = data_m.ay(:);                      % ay
col_Fz = data_m.Fz;                         % Fz (4列: FL, FR, RL, RR)

% サイズ確認
fprintf('  Column sizes after vectorization:\n');
fprintf('    col_time: %d x %d\n', size(col_time));
fprintf('    col_Fz:   %d x %d\n', size(col_Fz));

% 結合
csv_data = [col_time, col_x, col_y, col_psi, col_vx, col_vy, col_r, ...
            col_ax, col_ay, col_Fz];

fprintf('  CSV data size: %d x %d (expected: %d x 13)\n', ...
        size(csv_data, 1), size(csv_data, 2), length(t_m));

header = 'time,x,y,psi,vx,vy,r,ax,ay,Fz_FL,Fz_FR,Fz_RL,Fz_RR';
fid = fopen(csv_filename, 'w');
fprintf(fid, '%s\n', header);
fclose(fid);
dlmwrite(csv_filename, csv_data, '-append', 'precision', '%.6f');

fprintf('  ✓ Saved to: %s\n\n', csv_filename);

%% C++結果との比較（CSVがあれば）
fprintf('Step 7: Compare with C++ Results (if available)\n');
cpp_csv = '../../cpp/build/step_steer_result.csv';

if exist(cpp_csv, 'file')
    fprintf('  Loading C++ results from: %s\n', cpp_csv);
    cpp_data = readtable(cpp_csv);
    
    % 比較プロット
    figure('Position', [100, 100, 1600, 600], 'Name', 'MATLAB vs C++ Comparison');
    
    % 軌跡比較
    subplot(1,3,1);
    plot(state_m(:,1), state_m(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    hold on;
    plot(cpp_data.x, cpp_data.y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Trajectory Comparison');
    legend('Location', 'best');
    axis equal;
    grid on;
    
    % 速度比較
    subplot(1,3,2);
    plot(t_m, state_m(:,7), 'b-', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    hold on;
    plot(cpp_data.time, cpp_data.vx, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
    xlabel('Time [s]');
    ylabel('vx [m/s]');
    title('Forward Velocity Comparison');
    legend('Location', 'best');
    grid on;
    
    % ヨーレート比較
    subplot(1,3,3);
    plot(t_m, state_m(:,12)*180/pi, 'b-', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    hold on;
    plot(cpp_data.time, cpp_data.r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
    xlabel('Time [s]');
    ylabel('Yaw Rate [deg/s]');
    title('Yaw Rate Comparison');
    legend('Location', 'best');
    grid on;
    
    % 差分計算
    % 時間を合わせる（補間）
    vx_cpp_interp = interp1(cpp_data.time, cpp_data.vx, t_m, 'linear', 'extrap');
    r_cpp_interp = interp1(cpp_data.time, cpp_data.r, t_m, 'linear', 'extrap');
    
    vx_diff = abs(state_m(:,7) - vx_cpp_interp);
    r_diff = abs(state_m(:,12)*180/pi - r_cpp_interp);
    
    fprintf('\n  Comparison Statistics:\n');
    fprintf('    vx RMS error:  %.4f m/s\n', rms(vx_diff));
    fprintf('    r RMS error:   %.4f deg/s\n', rms(r_diff));
    fprintf('    vx max error:  %.4f m/s\n', max(vx_diff));
    fprintf('    r max error:   %.4f deg/s\n\n', max(r_diff));
else
    fprintf('  ⚠ C++ results not found at: %s\n', cpp_csv);
    fprintf('  Run C++ simulation first:\n');
    fprintf('    cd cpp/build && ./fsae_14dof step_steer\n\n');
end

fprintf('=================================================================\n');
fprintf('✅ MATLAB 14DOF simulation and comparison complete\n');
fprintf('=================================================================\n');

%% ヘルパー関数
function params = setup_params_cpp_compatible()
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
    params.mass.h_roll = 0.28;
    params.mass.Ixx = 32;
    params.mass.Iyy = 95;
    params.mass.Izz = 85;
    params.mass.Iw = 0.15;
    
    % サスペンション
    params.suspension.k_spring = [25000, 25000, 30000, 30000];
    params.suspension.motion_ratio = [0.9, 0.9, 0.9, 0.9];
    params.suspension.bw_c0 = 2000;
    params.suspension.bw_k0 = 0;
    params.suspension.bw_alpha = 1500;
    params.suspension.bw_A = 100;
    params.suspension.bw_beta = 300;
    params.suspension.bw_gamma = 0;
    params.suspension.bw_n = 1;
    params.suspension.lugre_sigma0 = 10000;
    params.suspension.lugre_sigma1 = 10;
    params.suspension.lugre_sigma2 = 0;
    params.suspension.lugre_Fc = 20;
    params.suspension.lugre_Fs = 30;
    params.suspension.lugre_vs = 0.01;
    params.suspension.k_arb_f = 5000;
    params.suspension.k_arb_r = 3000;
    params.suspension.bump_stop_gap = 0.03;
    params.suspension.bump_stop_k = 100000;
    
    % タイヤ（実測データベース: Hoosier 18.0×6.0-10-R20）
    params.tire.radius = 0.228;
    params.tire.mu_x_max = 1.8;   % レーシングタイヤ（実測ベース）
    params.tire.mu_y_max = 2.0;   % レーシングタイヤ（実測ベース、保守的調整）
    params.tire.Fz_nom = 1500;
    params.tire.Lx = 0.15;
    params.tire.Ly = 0.25;
    params.tire.Cx = 80000;       % 縦スティフネス
    params.tire.Cy = 22000;       % 横スティフネス（実測: 22436 N/rad）
    params.tire.v_N = 0.5;
    params.tire.k_tire = 200000;
    params.tire.c_tire = 500;
    
    % 空力（4Dマップ用）
    params.aero.rho = 1.225;
    params.aero.A_ref = 1.0;
    params.aero.L_ref = 1.6;
    params.aero.x_cg = 0.8;
    params.aero.CL_default = -2.5;
    params.aero.CD_default = 1.0;
    params.aero.CoP_default = 0.45;
    
    % パワートレイン
    params.powertrain.gear_ratio = 3.0;
    params.powertrain.efficiency = 0.95;
    
    % 初速（C++と統一）
    params.v0 = 20;  % m/s
end
