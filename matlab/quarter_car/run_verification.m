%% run_verification.m
% C++とMATLABの出力比較検証スクリプト

clear; clc; close all;

%% 1. MATLABでGround Truth生成
fprintf('=== FSAE Sim Verification ===\n\n');
fprintf('1. Generating MATLAB Ground Truth...\n');

% パラメータ設定
params = default_params();
params.use_bouc_wen = true;

% 初期条件
x0 = [0; 0; 0; 0; 0];  % [z_s, z_u, dz_s, dz_u, z_bw]

% 路面入力（ステップ入力）
z_step = 0.01;  % 10mm段差
input_func = @(t) z_step * (t >= 0.1);

% シミュレーション実行
t_span = [0, 1];
[t_matlab, x_matlab] = quarter_car_ode(params, t_span, x0, input_func);

fprintf('   Completed: %d steps, t_end = %.3f s\n', length(t_matlab), t_matlab(end));

%% 2. C++結果の読み込み（存在する場合）
cpp_result_file = '../../output/quarter_car_cpp.csv';
if exist(cpp_result_file, 'file')
    fprintf('\n2. Loading C++ results...\n');
    cpp_data = readmatrix(cpp_result_file);
    t_cpp = cpp_data(:, 1);
    x_cpp = cpp_data(:, 2:end);
    
    % 補間してマッチング
    x_cpp_interp = interp1(t_cpp, x_cpp, t_matlab, 'linear', 'extrap');
    
    % 誤差計算
    error_abs = abs(x_matlab - x_cpp_interp);
    error_rms = sqrt(mean(error_abs.^2, 1));
    error_max = max(error_abs, [], 1);
    
    fprintf('   RMS Error:  z_s=%.2e, z_u=%.2e, dz_s=%.2e, dz_u=%.2e\n', error_rms(1:4));
    fprintf('   Max Error:  z_s=%.2e, z_u=%.2e, dz_s=%.2e, dz_u=%.2e\n', error_max(1:4));
    
    has_cpp = true;
else
    fprintf('\n2. C++ result file not found: %s\n', cpp_result_file);
    fprintf('   Run C++ simulation first.\n');
    has_cpp = false;
end

%% 3. プロット
fprintf('\n3. Plotting results...\n');

figure('Position', [100, 100, 1200, 800]);

% ばね上変位
subplot(2, 2, 1);
plot(t_matlab, x_matlab(:, 1) * 1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
if has_cpp
    hold on;
    plot(t_cpp, x_cpp(:, 1) * 1000, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
end
xlabel('Time [s]');
ylabel('Sprung Mass Displacement [mm]');
title('Sprung Mass Response');
legend('Location', 'best');
grid on;

% ばね下変位
subplot(2, 2, 2);
plot(t_matlab, x_matlab(:, 2) * 1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
if has_cpp
    hold on;
    plot(t_cpp, x_cpp(:, 2) * 1000, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
end
xlabel('Time [s]');
ylabel('Unsprung Mass Displacement [mm]');
title('Unsprung Mass Response');
legend('Location', 'best');
grid on;

% ばね上速度
subplot(2, 2, 3);
plot(t_matlab, x_matlab(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
if has_cpp
    hold on;
    plot(t_cpp, x_cpp(:, 3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'C++');
end
xlabel('Time [s]');
ylabel('Sprung Mass Velocity [m/s]');
title('Sprung Mass Velocity');
legend('Location', 'best');
grid on;

% 誤差（C++がある場合）
subplot(2, 2, 4);
if has_cpp
    semilogy(t_matlab, error_abs(:, 1), 'b-', 'DisplayName', 'z_s');
    hold on;
    semilogy(t_matlab, error_abs(:, 2), 'r-', 'DisplayName', 'z_u');
    semilogy(t_matlab, error_abs(:, 3), 'g-', 'DisplayName', 'dz_s');
    xlabel('Time [s]');
    ylabel('Absolute Error');
    title('MATLAB vs C++ Error');
    legend('Location', 'best');
    grid on;
else
    text(0.5, 0.5, 'Run C++ simulation to see comparison', ...
         'HorizontalAlignment', 'center', 'Units', 'normalized');
    title('Error (C++ result not available)');
end

sgtitle('Quarter Car Model Verification');

% 保存
saveas(gcf, '../../output/verification_plot.png');
fprintf('   Saved: ../../output/verification_plot.png\n');

%% 4. Ground Truth保存
fprintf('\n4. Saving MATLAB Ground Truth...\n');
output_data = [t_matlab, x_matlab];
writematrix(output_data, '../../output/quarter_car_matlab.csv');
fprintf('   Saved: ../../output/quarter_car_matlab.csv\n');

%% 検証結果サマリー
fprintf('\n=== Verification Summary ===\n');
if has_cpp
    pass_threshold = 1e-4;
    if all(error_rms(1:4) < pass_threshold)
        fprintf('PASS: All RMS errors below %.0e\n', pass_threshold);
    else
        fprintf('FAIL: Some RMS errors exceed %.0e\n', pass_threshold);
    end
else
    fprintf('PENDING: C++ results required for verification\n');
end

fprintf('\nDone.\n');

%% ヘルパー関数
function p = default_params()
    p.m_s = 62.5;       % 1/4車両ばね上質量 [kg]
    p.m_u = 12;         % ばね下質量 [kg]
    p.k_s = 25000;      % ばねレート [N/m]
    p.c_s = 1500;       % 減衰係数 [Ns/m]
    p.k_t = 200000;     % タイヤ剛性 [N/m]
    p.use_bouc_wen = true;
    p.bw_alpha = 5000;
    p.bw_A = 1;
    p.bw_beta = 500;
    p.bw_gamma = 100;
    p.bw_n = 2;
end
