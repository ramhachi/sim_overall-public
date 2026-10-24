%% run_3dof_verification.m
% 3DOFモデルのC++ vs MATLAB検証

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' 3DOF Vehicle Model Verification\n');
fprintf('==========================================\n\n');

%% 1. パラメータ設定（C++と同じ）
params.mass.m_sprung = 240;        % kg
params.mass.m_unsprung = [10, 10, 10, 10];  % kg
params.mass.L = 1.55;              % m (wheelbase)
params.mass.a = 0.73;              % m (CG to front)
params.mass.b = 0.82;              % m (CG to rear)
params.mass.track_f = 1.2;         % m
params.mass.track_r = 1.15;        % m
params.mass.Izz = 85;              % kg*m^2
params.mass.Iw = 0.15;             % kg*m^2

params.tire.radius = 0.228;        % m
params.tire.mu_x_max = 1.6;
params.tire.mu_y_max = 1.8;
params.tire.Lx = 0.15;             % m
params.tire.Ly = 0.25;             % m
params.tire.Cx = 80000;            % N/m
params.tire.Cy = 60000;            % N/m
params.tire.v_N = 0.5;             % m/s

params.aero.rho = 1.225;           % kg/m^3
params.aero.CD_A = 0.8;            % m^2

params.powertrain.max_torque = 55; % Nm
params.powertrain.gear_ratio = 3.0;
params.powertrain.efficiency = 0.9;

params.v0 = 15.0;                  % m/s (初期速度)

%% 2. 入力関数（ステップステア）
step_time = 1.0;
step_steer = 0.1;  % rad (~5.7 deg)
throttle = 0.3;

input_func = @(t) struct('steering', (t >= step_time) * step_steer, ...
                          'throttle', throttle);

%% 3. MATLABシミュレーション実行
t_span = [0, 5];
dt = 0.001;

fprintf('Running MATLAB simulation...\n');
tic;
[t_matlab, state_matlab, data_matlab] = vehicle_3dof_model(params, t_span, dt, input_func);
matlab_time = toc;
fprintf('MATLAB simulation time: %.2f s\n', matlab_time);

%% 4. C++結果の読み込み
cpp_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'build', '3dof_step_steer_result.csv');
if ~exist(cpp_file, 'file')
    cpp_file = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output', '3dof_step_steer_result.csv');
end

if ~exist(cpp_file, 'file')
    error('C++ result file not found: %s\nRun ./cpp/fsae_3dof first.', cpp_file);
end

cpp_data = readtable(cpp_file);
fprintf('Loaded C++ result: %d data points\n', height(cpp_data));

% MATLAB結果を同じ形式に変換
matlab_result = struct();
matlab_result.time = t_matlab';
matlab_result.x = state_matlab(:, 1);
matlab_result.y = state_matlab(:, 2);
matlab_result.psi = rad2deg(state_matlab(:, 3));
matlab_result.vx = state_matlab(:, 4);
matlab_result.vy = state_matlab(:, 5);
matlab_result.r = rad2deg(state_matlab(:, 6));

%% 5. データの補間（タイムステップが異なる場合）
t_cpp = cpp_data.time;
n_cpp = length(t_cpp);

% MATLABデータをC++のタイムステップに合わせて補間
matlab_interp.x = interp1(matlab_result.time, matlab_result.x, t_cpp, 'linear', 'extrap');
matlab_interp.y = interp1(matlab_result.time, matlab_result.y, t_cpp, 'linear', 'extrap');
matlab_interp.psi = interp1(matlab_result.time, matlab_result.psi, t_cpp, 'linear', 'extrap');
matlab_interp.vx = interp1(matlab_result.time, matlab_result.vx, t_cpp, 'linear', 'extrap');
matlab_interp.vy = interp1(matlab_result.time, matlab_result.vy, t_cpp, 'linear', 'extrap');
matlab_interp.r = interp1(matlab_result.time, matlab_result.r, t_cpp, 'linear', 'extrap');

%% 6. 誤差計算
errors = struct();
fields = {'x', 'y', 'psi', 'vx', 'vy', 'r'};
units = {'m', 'm', 'deg', 'm/s', 'm/s', 'deg/s'};

fprintf('\n=== Error Analysis ===\n');
fprintf('%-10s %12s %12s %12s\n', 'Variable', 'Max Abs', 'RMSE', 'Mean');
fprintf('----------------------------------------------------\n');

all_pass = true;
tolerances = [1.0, 1.0, 5.0, 0.5, 0.5, 5.0];  % 各変数の許容誤差

for i = 1:length(fields)
    field = fields{i};
    cpp_vals = cpp_data.(field);
    matlab_vals = matlab_interp.(field);
    
    diff = cpp_vals - matlab_vals;
    max_abs = max(abs(diff));
    rmse = sqrt(mean(diff.^2));
    mean_err = mean(diff);
    
    errors.(field) = struct('max_abs', max_abs, 'rmse', rmse, 'mean', mean_err, 'diff', diff);
    
    if max_abs < tolerances(i)
        status = '✓';
    else
        status = '✗';
        all_pass = false;
    end
    
    fprintf('%-10s %12.4f %12.4f %12.4f %s [%s]\n', ...
        field, max_abs, rmse, mean_err, units{i}, status);
end

fprintf('----------------------------------------------------\n');
if all_pass
    fprintf('OVERALL: PASS ✓ - All errors within tolerance\n');
else
    fprintf('OVERALL: FAIL ✗ - Some errors exceed tolerance\n');
end
fprintf('==========================================\n');

%% 7. プロット
figure('Position', [100, 100, 1400, 900]);

% 軌跡
subplot(2, 3, 1);
plot(cpp_data.x, cpp_data.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(matlab_interp.x, matlab_interp.y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('X [m]');
ylabel('Y [m]');
title('Trajectory');
legend('Location', 'best');
axis equal;
grid on;

% ヨー角
subplot(2, 3, 2);
plot(t_cpp, cpp_data.psi, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.psi, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]');
ylabel('Yaw [deg]');
title('Yaw Angle');
legend('Location', 'best');
grid on;

% 縦速度
subplot(2, 3, 3);
plot(t_cpp, cpp_data.vx, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.vx, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]');
ylabel('Vx [m/s]');
title('Longitudinal Velocity');
legend('Location', 'best');
grid on;

% 横速度
subplot(2, 3, 4);
plot(t_cpp, cpp_data.vy, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.vy, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]');
ylabel('Vy [m/s]');
title('Lateral Velocity');
legend('Location', 'best');
grid on;

% ヨーレート
subplot(2, 3, 5);
plot(t_cpp, cpp_data.r, 'b-', 'LineWidth', 1.5, 'DisplayName', 'C++');
hold on;
plot(t_cpp, matlab_interp.r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
xlabel('Time [s]');
ylabel('Yaw Rate [deg/s]');
title('Yaw Rate');
legend('Location', 'best');
grid on;

% 誤差
subplot(2, 3, 6);
plot(t_cpp, errors.vx.diff, 'b-', 'DisplayName', 'Vx error');
hold on;
plot(t_cpp, errors.vy.diff, 'r-', 'DisplayName', 'Vy error');
plot(t_cpp, errors.r.diff / 10, 'g-', 'DisplayName', 'r error / 10');
xlabel('Time [s]');
ylabel('Error');
title('Velocity Errors');
legend('Location', 'best');
grid on;

sgtitle('3DOF Vehicle Model: C++ vs MATLAB Verification');

% 保存
output_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
saveas(gcf, fullfile(output_dir, 'verify_3dof.png'));
fprintf('\nSaved plot to output/verify_3dof.png\n');

%% 8. MATLAB結果をCSV出力
output_file = fullfile(output_dir, '3dof_matlab_result.csv');
output_table = table(t_matlab', state_matlab(:,1), state_matlab(:,2), ...
    rad2deg(state_matlab(:,3)), state_matlab(:,4), state_matlab(:,5), ...
    rad2deg(state_matlab(:,6)), ...
    'VariableNames', {'time', 'x', 'y', 'psi', 'vx', 'vy', 'r'});
writetable(output_table, output_file);
fprintf('Saved MATLAB result to %s\n', output_file);
