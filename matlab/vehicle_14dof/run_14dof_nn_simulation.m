%% run_14dof_nn_simulation.m
% 14DOF NN タイヤモデル統合シミュレーション実行スクリプト
%
% 目的:
%   - NNタイヤモデルを統合した14DOF車両シミュレーションを実行
%   - C++版と同じ条件でシミュレーション
%   - 結果をCSVに保存してC++と比較可能にする

clear;
clc;

fprintf('\n');
fprintf('================================================================\n');
fprintf('14DOF Vehicle Simulation with NN Tire Model\n');
fprintf('================================================================\n\n');

%% Step 1: 学習済みモデルの読み込み
fprintf('Step 1: Loading trained NN model...\n');
model_path = '../dl_tire_model/output/ultra_deep_validation_model.mat';

if ~isfile(model_path)
    error('Model file not found: %s\nPlease train the model first.', model_path);
end

model_data = load(model_path);
nn_model = model_data.trained_net;
norm_params = model_data.norm_params;

fprintf('  ✓ Model loaded: %s\n\n', model_path);

%% Step 2: 車両パラメータ設定
fprintf('Step 2: Setting vehicle parameters...\n');

params = struct();

% 質量パラメータ（C++と一致）
params.mass.m_sprung = 200.0;                    % ばね上質量 [kg]
params.mass.m_unsprung = [10.0, 10.0, 10.0, 10.0]; % ばね下質量 [kg]
params.mass.a = 0.813;                           % 前軸から重心 [m]
params.mass.b = 0.787;                           % 後軸から重心 [m]
params.mass.L = params.mass.a + params.mass.b;  % ホイールベース [m]
params.mass.track_f = 1.2;                       % 前トレッド [m]
params.mass.track_r = 1.2;                       % 後トレッド [m]
params.mass.h_cg = 0.3;                          % 重心高さ [m]

% 慣性モーメント
params.mass.Ixx = 50.0;   % ロール慣性 [kg·m²]
params.mass.Iyy = 100.0;  % ピッチ慣性 [kg·m²]
params.mass.Izz = 80.0;   % ヨー慣性 [kg·m²]
params.mass.Iw = 0.5;     % 車輪慣性 [kg·m²]

% タイヤパラメータ
params.tire.radius = 0.2286;  % タイヤ半径 [m]
params.tire.k_tire = 150000;  % タイヤ剛性 [N/m]
params.tire.c_tire = 100;     % タイヤ減衰 [N·s/m]

% サスペンションパラメータ
params.suspension.k_spring = [30000, 30000, 30000, 30000];  % ばね定数 [N/m]
params.suspension.motion_ratio = [1.0, 1.0, 1.0, 1.0];     % モーションレシオ

% Bouc-Wenパラメータ（C++と一致）
params.suspension.bw_c0 = 2000.0;
params.suspension.bw_k0 = 0.0;
params.suspension.bw_alpha = 1500.0;
params.suspension.bw_A = 100.0;
params.suspension.bw_beta = 300.0;
params.suspension.bw_gamma = 0.0;
params.suspension.bw_n = 1.0;

% LuGreパラメータ（C++と一致）
params.suspension.lugre_sigma0 = 10000.0;
params.suspension.lugre_sigma1 = 10.0;
params.suspension.lugre_sigma2 = 0.0;
params.suspension.lugre_Fc = 20.0;
params.suspension.lugre_Fs = 30.0;
params.suspension.lugre_vs = 0.01;

% ARB
params.suspension.k_arb_f = 5000.0;  % 前ARB剛性 [N·m/rad]
params.suspension.k_arb_r = 3000.0;  % 後ARB剛性 [N·m/rad]

% バンプストップ
params.suspension.bump_stop_gap = 0.05;  % バンプストップギャップ [m]
params.suspension.bump_stop_k = 100000;  % バンプストップ剛性 [N/m]

% パワートレイン
params.powertrain.max_torque = 200.0;    % 最大トルク [N·m]
params.powertrain.gear_ratio = 10.0;     % ギア比
params.powertrain.efficiency = 0.9;      % 伝達効率

% 空力
params.aero.rho = 1.225;   % 空気密度 [kg/m³]
params.aero.CL_A = 2.0;    % 揚力係数×面積 [m²]
params.aero.CD_A = 0.8;    % 抗力係数×面積 [m²]

% 初速
params.v0 = 15.0;  % [m/s]

% 環境条件
params.environment.T_ambient = 25.0;  % 外気温 [°C]

fprintf('  ✓ Parameters configured\n\n');

%% Step 3: 入力設定
fprintf('Step 3: Configuring driver input...\n');

% ステアリング入力（C++と同じ：1秒後に0.1rad）
t_steer = 1.0;
delta = 0.1;

input_func = @(t) struct(...
    'steering', (t >= t_steer) * delta, ...
    'throttle', 0.1, ...
    'brake', 0.0 ...
    );

fprintf('  ✓ Input: steering step at t=%.1fs, delta=%.2frad\n\n', t_steer, delta);

%% Step 4: シミュレーション実行
fprintf('Step 4: Running 14DOF simulation...\n');

t_span = [0, 5.0];
dt = 0.001;

tic;
[t, state, data] = vehicle_14dof_nn(params, t_span, dt, input_func, nn_model, norm_params);
sim_time = toc;

fprintf('  ✓ Simulation completed (%.2f seconds)\n\n', sim_time);

%% Step 5: 結果の抽出
fprintf('Step 5: Extracting results...\n');

% 状態変数の展開
x = state(:, 1);
y = state(:, 2);
z = state(:, 3);
phi = state(:, 4);
theta = state(:, 5);
psi = state(:, 6);
vx = state(:, 7);
vy = state(:, 8);
vz = state(:, 9);
p = state(:, 10);
q = state(:, 11);
r = state(:, 12);

speed = sqrt(vx.^2 + vy.^2);

% 最終状態の表示
fprintf('\n=== Results ===\n');
fprintf('Final position: (%.2f, %.2f) m\n', x(end), y(end));
fprintf('Final heading: %.1f°\n', psi(end) * 180/pi);
fprintf('Final speed: %.2f m/s\n', speed(end));
fprintf('Max roll angle: %.3f°\n', max(abs(phi)) * 180/pi);

% 旋回半径
if abs(r(end)) > 0.01
    turn_radius = speed(end) / abs(r(end));
    fprintf('Turning radius: %.1f m\n', turn_radius);
end

% タイヤ荷重
fprintf('\nFinal tire vertical loads:\n');
fprintf('  FL: %.0f N, FR: %.0f N\n', data.Fz(end, 1), data.Fz(end, 2));
fprintf('  RL: %.0f N, RR: %.0f N\n', data.Fz(end, 3), data.Fz(end, 4));

% ダンパー力
fprintf('\nFinal damper forces (BW/LuGre):\n');
fprintf('  FL: %.1f/%.1f N\n', data.F_bw(end, 1), data.F_lugre(end, 1));
fprintf('  FR: %.1f/%.1f N\n', data.F_bw(end, 2), data.F_lugre(end, 2));

% タイヤ温度
fprintf('\nFinal tire surface temperatures:\n');
fprintf('  FL: %.1f°C, FR: %.1f°C\n', data.T_surf(end, 1), data.T_surf(end, 2));
fprintf('  RL: %.1f°C, RR: %.1f°C\n\n', data.T_surf(end, 3), data.T_surf(end, 4));

%% Step 6: CSV保存
fprintf('Step 6: Saving results to CSV...\n');

output_file = '14dof_nn_result.csv';

% CSVヘッダー
csv_data = [t, x, y, psi*180/pi, vx, vy, phi*180/pi, theta*180/pi, ...
    data.ax, data.ay, data.Fz];

csv_header = 'time,x,y,psi,vx,vy,phi,theta,ax,ay,Fz_FL,Fz_FR,Fz_RL,Fz_RR';

% ファイル書き込み
fid = fopen(output_file, 'w');
fprintf(fid, '%s\n', csv_header);
fclose(fid);

csvwrite_with_header(output_file, csv_data, csv_header);

fprintf('  ✓ Saved to %s\n\n', output_file);

fprintf('================================================================\n');
fprintf('Simulation complete!\n');
fprintf('================================================================\n\n');

%% Helper function
function csvwrite_with_header(filename, data, header)
% ヘッダー付きCSV書き込み
dlmwrite(filename, data, '-append', 'precision', '%.6f');
end
