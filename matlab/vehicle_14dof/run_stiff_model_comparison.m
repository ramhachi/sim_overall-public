%% run_stiff_model_comparison.m
% 高剛性モデルでRosenbrock法 vs RK4比較

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' High Stiffness Model Comparison\n');
fprintf('==========================================\n\n');

%% 高剛性パラメータ設定
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

% サスペンション（高剛性）
params.suspension.k_spring = [50000, 50000, 60000, 60000];  % 2x増加
params.suspension.motion_ratio = [0.9, 0.9, 0.9, 0.9];
params.suspension.bw_c0 = 5000;      % 2.5x増加
params.suspension.bw_k0 = 0;
params.suspension.bw_alpha = 3000;   % 2x増加
params.suspension.bw_A = 200;        % 2x増加
params.suspension.bw_beta = 500;
params.suspension.bw_gamma = 0;
params.suspension.bw_n = 1;
params.suspension.lugre_sigma0 = 50000;  % 5x増加（非常に剛性が高い）
params.suspension.lugre_sigma1 = 50;     % 5x増加
params.suspension.lugre_sigma2 = 0;
params.suspension.lugre_Fc = 50;
params.suspension.lugre_Fs = 80;
params.suspension.lugre_vs = 0.01;
params.suspension.k_arb_f = 10000;   % 2x増加
params.suspension.k_arb_r = 6000;    % 2x増加
params.suspension.bump_stop_gap = 0.02;
params.suspension.bump_stop_k = 500000;  % 5x増加

% タイヤ（非常に高剛性）
params.tire.radius = 0.228;
params.tire.mu_x_max = 1.6;
params.tire.mu_y_max = 2.0;   % レーシングタイヤ（実測ベース）
params.tire.Fz_nom = 1500;
params.tire.Lx = 0.05;   % 緩和長短縮（より高周波応答）
params.tire.Ly = 0.08;
params.tire.Cx = 150000;  % 2x増加
params.tire.Cy = 120000;  % 2x増加
params.tire.v_N = 0.5;
params.tire.k_tire = 1000000;  % 5x増加（非常に剛性が高い）
params.tire.c_tire = 2000;     % 4x増加

% 空力
params.aero.rho = 1.2;
params.aero.CL_A = 2.5;
params.aero.CD_A = 1.0;

% パワートレイン
params.powertrain.max_torque = 55;
params.powertrain.gear_ratio = 3.0;
params.powertrain.efficiency = 0.9;

params.v0 = 15;

%% シミュレーション設定
t_span = [0, 3];
t_steer = 0.5;  % 早めにステアリング入力
delta = 0.15;   % 大きめの舵角
input_func = @(t) struct('throttle', 0.1, 'steering', (t >= t_steer) * delta);

fprintf('=== Stiffness Parameters ===\n');
fprintf('k_tire = %.0f N/m (5x normal)\n', params.tire.k_tire);
fprintf('c_tire = %.0f Ns/m (4x normal)\n', params.tire.c_tire);
fprintf('lugre_sigma0 = %.0f N/m (5x normal)\n', params.suspension.lugre_sigma0);
fprintf('k_spring = [%.0f, %.0f, %.0f, %.0f] N/m (2x normal)\n', params.suspension.k_spring);
fprintf('\n');

%% RK4 (dt=1ms) - 通常ステップ
fprintf('Running RK4 (dt=1ms)...\n');
dt1 = 0.001;
tic;
try
    [t1, state1, data1] = vehicle_14dof_model(params, t_span, dt1, input_func);
    time1 = toc;
    fprintf('  Time: %.3f s, Steps: %d\n', time1, length(t1)-1);
    rk4_1ms_ok = true;
catch ME
    fprintf('  ERROR: %s\n', ME.message);
    time1 = NaN;
    rk4_1ms_ok = false;
end

%% RK4 (dt=0.1ms) - 高精度
fprintf('Running RK4 (dt=0.1ms)...\n');
dt2 = 0.0001;
tic;
try
    [t2, state2, data2] = vehicle_14dof_model(params, t_span, dt2, input_func);
    time2 = toc;
    fprintf('  Time: %.3f s, Steps: %d\n', time2, length(t2)-1);
    rk4_01ms_ok = true;
catch ME
    fprintf('  ERROR: %s\n', ME.message);
    time2 = NaN;
    rk4_01ms_ok = false;
end

%% RK4 (dt=0.01ms) - 非常に高精度（剛性系用）
fprintf('Running RK4 (dt=0.01ms)...\n');
dt3 = 0.00001;
tic;
try
    [t3, state3, data3] = vehicle_14dof_model(params, t_span, dt3, input_func);
    time3 = toc;
    fprintf('  Time: %.3f s, Steps: %d\n', time3, length(t3)-1);
    rk4_001ms_ok = true;
catch ME
    fprintf('  ERROR: %s\n', ME.message);
    time3 = NaN;
    rk4_001ms_ok = false;
end

%% 結果比較
fprintf('\n=== Results Comparison ===\n');

if rk4_1ms_ok && rk4_01ms_ok
    % 最終状態
    idx1 = length(t1);
    idx2 = length(t2);
    
    fprintf('\nFinal Position:\n');
    fprintf('  RK4 (1ms):    x=%.2f, y=%.2f m\n', state1(idx1,1), state1(idx1,2));
    fprintf('  RK4 (0.1ms):  x=%.2f, y=%.2f m\n', state2(idx2,1), state2(idx2,2));
    
    if rk4_001ms_ok
        idx3 = length(t3);
        fprintf('  RK4 (0.01ms): x=%.2f, y=%.2f m\n', state3(idx3,1), state3(idx3,2));
    end
    
    fprintf('\nFinal Velocity:\n');
    fprintf('  RK4 (1ms):    vx=%.2f m/s\n', state1(idx1,7));
    fprintf('  RK4 (0.1ms):  vx=%.2f m/s\n', state2(idx2,7));
    
    fprintf('\nMax Roll Angle:\n');
    fprintf('  RK4 (1ms):    %.3f deg\n', max(abs(state1(:,4)))*180/pi);
    fprintf('  RK4 (0.1ms):  %.3f deg\n', max(abs(state2(:,4)))*180/pi);
    
    fprintf('\nComputation Time:\n');
    fprintf('  RK4 (1ms):    %.3f s\n', time1);
    fprintf('  RK4 (0.1ms):  %.3f s (%.1fx slower)\n', time2, time2/time1);
    if rk4_001ms_ok
        fprintf('  RK4 (0.01ms): %.3f s (%.1fx slower)\n', time3, time3/time1);
    end
    
    % 誤差計算
    t_common = t1;
    state2_interp = interp1(t2, state2, t_common, 'linear', 'extrap');
    
    err_pos = sqrt((state1(:,1) - state2_interp(:,1)).^2 + (state1(:,2) - state2_interp(:,2)).^2);
    err_roll = abs(state1(:,4) - state2_interp(:,4)) * 180/pi;
    
    fprintf('\n=== Accuracy (1ms vs 0.1ms) ===\n');
    fprintf('Max position error: %.4f m\n', max(err_pos));
    fprintf('Max roll error: %.4f deg\n', max(err_roll));
    fprintf('Final position error: %.4f m\n', err_pos(end));
    
    %% プロット
    figure('Position', [50, 50, 1400, 900]);
    
    % 軌跡
    subplot(2,3,1);
    plot(state1(:,1), state1(:,2), 'b-', 'LineWidth', 2); hold on;
    plot(state2(:,1), state2(:,2), 'r--', 'LineWidth', 1);
    if rk4_001ms_ok
        plot(state3(:,1), state3(:,2), 'g:', 'LineWidth', 1);
        legend('RK4 (1ms)', 'RK4 (0.1ms)', 'RK4 (0.01ms)');
    else
        legend('RK4 (1ms)', 'RK4 (0.1ms)');
    end
    xlabel('X [m]'); ylabel('Y [m]');
    title('Trajectory');
    axis equal; grid on;
    
    % ロール角
    subplot(2,3,2);
    plot(t1, state1(:,4)*180/pi, 'b-', 'LineWidth', 2); hold on;
    plot(t2, state2(:,4)*180/pi, 'r--', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Roll [deg]');
    title('Roll Angle');
    legend('RK4 (1ms)', 'RK4 (0.1ms)');
    grid on;
    
    % 速度
    subplot(2,3,3);
    plot(t1, state1(:,7), 'b-', 'LineWidth', 2); hold on;
    plot(t2, state2(:,7), 'r--', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('vx [m/s]');
    title('Forward Velocity');
    legend('RK4 (1ms)', 'RK4 (0.1ms)');
    grid on;
    
    % 位置誤差
    subplot(2,3,4);
    plot(t_common, err_pos, 'k-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Position Error [m]');
    title('Position Error (1ms vs 0.1ms)');
    grid on;
    
    % タイヤ力
    subplot(2,3,5);
    plot(t1, data1.Fz(:,1), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Force [N]');
    title('Tire Fz (FL) - High Stiffness');
    grid on;
    
    % 計算時間
    subplot(2,3,6);
    times = [time1, time2];
    labels = {'RK4 (1ms)', 'RK4 (0.1ms)'};
    if rk4_001ms_ok
        times = [times, time3];
        labels{3} = 'RK4 (0.01ms)';
    end
    bar(times);
    xticklabels(labels);
    ylabel('Computation Time [s]');
    title('Computation Time');
    grid on;
    
    sgtitle('High Stiffness Model - RK4 Step Size Comparison');
    
    saveas(gcf, '../../output/stiff_model_comparison.png');
    fprintf('\nSaved plot to output/stiff_model_comparison.png\n');
    
    %% 剛性による安定性評価
    fprintf('\n=== Stiffness Stability Analysis ===\n');
    
    if max(err_pos) < 0.01
        fprintf('✓ RK4 (1ms) is stable and accurate for this stiffness level\n');
        fprintf('  Rosenbrock not required\n');
    elseif max(err_pos) < 0.1
        fprintf('! RK4 (1ms) shows noticeable error\n');
        fprintf('  Consider smaller step or Rosenbrock\n');
    else
        fprintf('✗ RK4 (1ms) has significant error (%.4f m)\n', max(err_pos));
        fprintf('  Rosenbrock recommended for this stiffness\n');
    end
end

fprintf('\n==========================================\n');
