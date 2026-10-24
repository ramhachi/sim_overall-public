%% run_8dof_bw_verification.m
% 8DOF + Bouc-Wen C++ vs MATLAB 検証スクリプト

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' 8DOF + Bouc-Wen Vehicle Model Verification\n');
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

% Bouc-Wenパラメータ（C++テストと一致）
params.suspension.bw_c0 = 2000;      % 粘性減衰
params.suspension.bw_k0 = 0;         % 剛性（通常0）
params.suspension.bw_alpha = 1500;   % ヒステリシス強度
params.suspension.bw_A = 100;        % 形状パラメータ
params.suspension.bw_beta = 300;     % 形状パラメータ
params.suspension.bw_gamma = 0;      % 形状パラメータ
params.suspension.bw_n = 1;          % 形状指数

% ARB
params.suspension.k_arb_f = 5000;   % [Nm/rad]
params.suspension.k_arb_r = 3000;   % [Nm/rad]

% タイヤ
params.tire.radius = 0.228;
params.tire.mu_x_max = 1.6;
params.tire.mu_y_max = 1.8;
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
t_span = [0, 3];
dt = 0.001;

% 入力関数
t_steer = 1.0;
delta = 0.15;
input_func = @(t) struct('throttle', 0.1, 'steering', (t >= t_steer) * delta);

%% MATLAB シミュレーション実行
fprintf('Running MATLAB 8DOF+Bouc-Wen simulation...\n');
tic;
[t_m, state_m, data_m] = vehicle_8dof_bw_model(params, t_span, dt, input_func);
fprintf('MATLAB simulation time: %.2f s\n', toc);

%% C++ 結果読み込み
% ファイル検索順序
cpp_file = '';
search_paths = {'../output/bouc_wen_comparison.csv', ...
    '../../output/bouc_wen_comparison.csv', ...
    'bouc_wen_comparison.csv'};
for i = 1:length(search_paths)
    if exist(search_paths{i}, 'file')
        cpp_file = search_paths{i};
        fprintf('Found C++ result: %s\n', cpp_file);
        break;
    end
end

if ~isempty(cpp_file)
    cpp_data = readtable(cpp_file);
    fprintf('Loaded C++ result: %d data points\n', height(cpp_data));
    
    % 時間軸を合わせる
    t_cpp = cpp_data.time;
    
    % 比較用データ抽出（CSVカラム名に基づく）
    if ismember('phi_bw', cpp_data.Properties.VariableNames)
        phi_cpp = cpp_data.phi_bw;  % 度
        phi_m = state_m(:,4) * 180 / pi;  % MATLAB: rad -> deg
        
        % 時間解像度を合わせる
        idx_m = round(t_cpp / dt) + 1;
        idx_m = min(idx_m, length(phi_m));
        phi_m_interp = phi_m(idx_m);
        
        % 誤差計算
        err = phi_cpp - phi_m_interp;
        max_err = max(abs(err));
        rmse = sqrt(mean(err.^2));
        
        fprintf('\n=== Roll Angle Comparison ===\n');
        fprintf('Max Error:  %.4f deg\n', max_err);
        fprintf('RMSE:       %.4f deg\n', rmse);
        
        if max_err < 1.0  % 1.0度未満
            fprintf('Status: ✓ PASS\n');
        else
            fprintf('Status: ✗ FAIL (max error > 1.0 deg)\n');
        end
        
        % プロット
        figure('Position', [100, 100, 1200, 600]);
        
        subplot(2,2,1);
        plot(t_cpp, phi_cpp, 'b-', 'LineWidth', 1.5); hold on;
        plot(t_cpp, phi_m_interp, 'r--', 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel('Roll Angle [deg]');
        title('Roll Angle Comparison');
        legend('C++ (Bouc-Wen)', 'MATLAB', 'Location', 'best');
        grid on;
        
        subplot(2,2,2);
        plot(t_cpp, err, 'k-', 'LineWidth', 1.0);
        xlabel('Time [s]');
        ylabel('Error [deg]');
        title(sprintf('Roll Angle Error (RMSE=%.4f deg)', rmse));
        grid on;
        
        % Bouc-Wen力比較
        if ismember('bw_force_fl', cpp_data.Properties.VariableNames)
            subplot(2,2,3);
            bw_force_cpp = cpp_data.bw_force_fl;
            bw_force_m = data_m.F_damper_bw(idx_m, 1);
            
            plot(t_cpp, bw_force_cpp, 'b-', 'LineWidth', 1.5); hold on;
            plot(t_cpp, bw_force_m, 'r--', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Damper Force [N]');
            title('Bouc-Wen Damper Force (FL)');
            legend('C++', 'MATLAB', 'Location', 'best');
            grid on;
        end
        
        % 履歴変数比較
        if ismember('damper_z_fl', cpp_data.Properties.VariableNames)
            subplot(2,2,4);
            z_cpp = cpp_data.damper_z_fl;
            z_m = state_m(idx_m, 33);  % damper_z(1)
            
            plot(t_cpp, z_cpp, 'b-', 'LineWidth', 1.5); hold on;
            plot(t_cpp, z_m, 'r--', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Hysteresis Variable z');
            title('Bouc-Wen Hysteresis Variable (FL)');
            legend('C++', 'MATLAB', 'Location', 'best');
            grid on;
        end
        
        sgtitle('8DOF + Bouc-Wen Vehicle Model Verification');
        
        % 保存
        saveas(gcf, '../../output/verify_8dof_bw.png');
        fprintf('\nSaved plot to output/verify_8dof_bw.png\n');
    end
else
    % MATLAB単体モード
    warning('C++ result file not found. Running MATLAB-only mode.');
    fprintf('\nRunning in MATLAB-only mode (no C++ comparison)\n');
    
    % 結果プロット
    t = t_m;
    phi = state_m(:,4) * 180/pi;
    damper_z = state_m(:,33:36);
    
    figure('Position', [100, 100, 1200, 600]);
    
    subplot(2,2,1);
    plot(t, phi, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Roll Angle [deg]');
    title('Roll Angle');
    grid on;
    
    subplot(2,2,2);
    plot(t, state_m(:,7), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('vx [m/s]');
    title('Forward Velocity');
    grid on;
    
    subplot(2,2,3);
    plot(t, data_m.F_damper_bw);
    xlabel('Time [s]');
    ylabel('Force [N]');
    title('Bouc-Wen Damper Forces');
    legend('FL', 'FR', 'RL', 'RR');
    grid on;
    
    subplot(2,2,4);
    plot(t, damper_z);
    xlabel('Time [s]');
    ylabel('z');
    title('Bouc-Wen Hysteresis Variables');
    legend('FL', 'FR', 'RL', 'RR');
    grid on;
    
    sgtitle('8DOF + Bouc-Wen Vehicle Model (MATLAB)');
    
    fprintf('\nMax roll angle: %.3f deg\n', max(abs(phi)));
    fprintf('Final hysteresis z: [%.4f, %.4f, %.4f, %.4f]\n', damper_z(end,:));
end

fprintf('\n==========================================\n');
