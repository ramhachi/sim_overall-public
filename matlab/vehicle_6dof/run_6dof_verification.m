%% run_6dof_verification.m
% 6DOFモデルのC++ vs MATLAB検証

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' 6DOF Vehicle Model Verification\n');
fprintf('==========================================\n\n');

%% 1. パラメータ設定（C++と同じ）
params.mass.m_sprung = 240;
params.mass.m_unsprung = [10, 10, 10, 10];
params.mass.L = 1.55;
params.mass.a = 0.73;
params.mass.b = 0.82;
params.mass.track_f = 1.2;
params.mass.track_r = 1.15;
params.mass.h_cg = 0.28;
params.mass.Ixx = 30;
params.mass.Iyy = 100;
params.mass.Izz = 85;
params.mass.Iw = 0.15;

params.tire.radius = 0.228;
params.tire.mu_x_max = 1.6;
params.tire.mu_y_max = 1.8;
params.tire.Lx = 0.15;
params.tire.Ly = 0.25;
params.tire.Cx = 80000;
params.tire.Cy = 60000;
params.tire.v_N = 0.5;

params.suspension.k_spring = [25000, 25000, 30000, 30000];  % N/m
params.suspension.motion_ratio = [0.9, 0.9, 0.85, 0.85];
params.suspension.c0 = 2000;  % Ns/m

params.aero.rho = 1.225;
params.aero.CD_A = 0.8;
params.aero.CL_A = -2.5;

params.powertrain.max_torque = 55;
params.powertrain.gear_ratio = 3.0;
params.powertrain.efficiency = 0.9;

params.v0 = 15.0;

%% 2. 入力関数
step_time = 1.0;
step_steer = 0.1;
throttle = 0.3;

input_func = @(t) struct('steering', (t >= step_time) * step_steer, ...
                          'throttle', throttle);

%% 3. MATLABシミュレーション
t_span = [0, 5];
dt = 0.001;

fprintf('Running MATLAB 6DOF simulation...\n');
tic;
[t_matlab, state_matlab, data_matlab] = vehicle_6dof_model(params, t_span, dt, input_func);
matlab_time = toc;
fprintf('MATLAB simulation time: %.2f s\n', matlab_time);

%% 4. C++結果の読み込み
cpp_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output', '6dof_step_steer_result.csv');

if ~exist(cpp_file, 'file')
    error('C++ result file not found: %s\nRun ./cpp/fsae_6dof first.', cpp_file);
end

cpp_data = readtable(cpp_file);
fprintf('Loaded C++ result: %d data points\n', height(cpp_data));

%% 5. データの比較準備
t_cpp = cpp_data.time;

% MATLAB結果を補間
fields_pos = {'x', 'y', 'z'};
fields_euler = {'phi', 'theta', 'psi'};
fields_vel = {'vx', 'vy', 'vz'};
fields_omega = {'p', 'q', 'r'};

matlab_interp = struct();
for i = 1:3
    matlab_interp.(fields_pos{i}) = interp1(t_matlab, state_matlab(:, i), t_cpp, 'linear', 'extrap');
    matlab_interp.(fields_euler{i}) = interp1(t_matlab, rad2deg(state_matlab(:, 3+i)), t_cpp, 'linear', 'extrap');
    matlab_interp.(fields_vel{i}) = interp1(t_matlab, state_matlab(:, 6+i), t_cpp, 'linear', 'extrap');
    matlab_interp.(fields_omega{i}) = interp1(t_matlab, rad2deg(state_matlab(:, 9+i)), t_cpp, 'linear', 'extrap');
end

%% 6. 誤差計算
all_fields = [fields_pos, fields_euler, fields_vel, fields_omega];
units = {'m', 'm', 'm', 'deg', 'deg', 'deg', 'm/s', 'm/s', 'm/s', 'deg/s', 'deg/s', 'deg/s'};
% 許容誤差（6DOFの累積誤差を考慮してヨーレートは8deg/sに設定）
tolerances = [1, 1, 0.1, 2, 2, 5, 0.5, 0.5, 0.1, 5, 5, 8];

fprintf('\n=== Error Analysis ===\n');
fprintf('%-10s %12s %12s %12s  %-8s\n', 'Variable', 'Max Abs', 'RMSE', 'Mean', 'Status');
fprintf('--------------------------------------------------------------\n');

all_pass = true;
errors = struct();

for i = 1:length(all_fields)
    field = all_fields{i};
    cpp_vals = cpp_data.(field);
    matlab_vals = matlab_interp.(field);
    
    diff_vals = cpp_vals - matlab_vals;
    max_abs = max(abs(diff_vals));
    rmse = sqrt(mean(diff_vals.^2));
    mean_err = mean(diff_vals);
    
    errors.(field) = struct('max_abs', max_abs, 'rmse', rmse, 'diff', diff_vals);
    
    if max_abs < tolerances(i)
        status = '✓ PASS';
    else
        status = '✗ FAIL';
        all_pass = false;
    end
    
    fprintf('%-10s %12.4f %12.4f %12.4f  %s [%s]\n', ...
        field, max_abs, rmse, mean_err, status, units{i});
end

fprintf('--------------------------------------------------------------\n');
if all_pass
    fprintf('OVERALL: PASS ✓ - All errors within tolerance\n');
else
    fprintf('OVERALL: FAIL ✗ - Some errors exceed tolerance\n');
end
fprintf('==========================================\n');

%% 7. プロット
figure('Position', [50, 50, 1600, 1000]);

% 軌跡
subplot(3, 4, 1);
plot(cpp_data.x, cpp_data.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(matlab_interp.x, matlab_interp.y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory'); legend('Location', 'best');
axis equal; grid on;

% Z（ヒーブ）
subplot(3, 4, 2);
plot(t_cpp, cpp_data.z * 1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.z * 1000, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Z [mm]');
title('Heave (Vertical)'); legend('Location', 'best'); grid on;

% ロール
subplot(3, 4, 3);
plot(t_cpp, cpp_data.phi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.phi, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Roll Angle'); legend('Location', 'best'); grid on;

% ピッチ
subplot(3, 4, 4);
plot(t_cpp, cpp_data.theta, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.theta, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Pitch Angle'); legend('Location', 'best'); grid on;

% ヨー角
subplot(3, 4, 5);
plot(t_cpp, cpp_data.psi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.psi, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Yaw [deg]');
title('Yaw Angle'); legend('Location', 'best'); grid on;

% 縦速度
subplot(3, 4, 6);
plot(t_cpp, cpp_data.vx, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.vx, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Vx [m/s]');
title('Longitudinal Velocity'); legend('Location', 'best'); grid on;

% 横速度
subplot(3, 4, 7);
plot(t_cpp, cpp_data.vy, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.vy, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Vy [m/s]');
title('Lateral Velocity'); legend('Location', 'best'); grid on;

% ヨーレート
subplot(3, 4, 8);
plot(t_cpp, cpp_data.r, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]'); ylabel('Yaw Rate [deg/s]');
title('Yaw Rate'); legend('Location', 'best'); grid on;

% 誤差プロット
subplot(3, 4, 9);
plot(t_cpp, errors.x.diff, 'DisplayName', 'X');
hold on;
plot(t_cpp, errors.y.diff, 'DisplayName', 'Y');
xlabel('Time [s]'); ylabel('Error [m]');
title('Position Errors'); legend('Location', 'best'); grid on;

subplot(3, 4, 10);
plot(t_cpp, errors.phi.diff, 'DisplayName', 'Roll');
hold on;
plot(t_cpp, errors.theta.diff, 'DisplayName', 'Pitch');
plot(t_cpp, errors.psi.diff, 'DisplayName', 'Yaw');
xlabel('Time [s]'); ylabel('Error [deg]');
title('Attitude Errors'); legend('Location', 'best'); grid on;

subplot(3, 4, 11);
plot(t_cpp, errors.vx.diff, 'DisplayName', 'Vx');
hold on;
plot(t_cpp, errors.vy.diff, 'DisplayName', 'Vy');
xlabel('Time [s]'); ylabel('Error [m/s]');
title('Velocity Errors'); legend('Location', 'best'); grid on;

subplot(3, 4, 12);
plot(t_cpp, errors.r.diff, 'DisplayName', 'Yaw rate');
xlabel('Time [s]'); ylabel('Error [deg/s]');
title('Yaw Rate Error'); legend('Location', 'best'); grid on;

sgtitle('6DOF Vehicle Model: C++ vs MATLAB Verification');

% 保存
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
saveas(gcf, fullfile(output_dir, 'verify_6dof.png'));
fprintf('\nSaved plot to output/verify_6dof.png\n');
