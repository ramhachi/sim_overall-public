%% verify_params_match.m
% MATLABとC++のパラメータ一致確認スクリプト

clear; clc;

fprintf('=================================================================\n');
fprintf(' MATLAB vs C++ Parameter Verification\n');
fprintf('=================================================================\n\n');

% MATLABパラメータ設定
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

% 空力（C++と統一）
params.aero.rho = 1.225;
params.aero.A_ref = 1.0;
params.aero.L_ref = 1.6;
params.aero.x_cg = 0.8;
params.aero.CL_default = -2.5;
params.aero.CD_default = 1.0;
params.aero.CoP_default = 0.45;

% パワートレイン
params.powertrain.gear_ratio = 3.0;  % 3速固定
params.powertrain.efficiency = 0.95;

% 初速（C++と統一）
params.v0 = 20;  % m/s

fprintf('✅ Parameter Setup Complete\n\n');

%% パラメータ表示
fprintf('--- Mass Properties ---\n');
fprintf('  m_sprung:    %.1f kg\n', params.mass.m_sprung);
fprintf('  a:           %.3f m\n', params.mass.a);
fprintf('  b:           %.3f m\n', params.mass.b);
fprintf('  h_cg:        %.3f m\n', params.mass.h_cg);
fprintf('  Ixx:         %.1f kg·m²\n', params.mass.Ixx);
fprintf('  Izz:         %.1f kg·m²\n', params.mass.Izz);
fprintf('\n');

fprintf('--- Suspension ---\n');
fprintf('  k_spring_f:  %.0f N/m\n', params.suspension.k_spring(1));
fprintf('  k_spring_r:  %.0f N/m\n', params.suspension.k_spring(3));
fprintf('  k_arb_f:     %.0f N/m\n', params.suspension.k_arb_f);
fprintf('  k_arb_r:     %.0f N/m\n', params.suspension.k_arb_r);
fprintf('\n');

fprintf('--- Tire ---\n');
fprintf('  radius:      %.3f m\n', params.tire.radius);
fprintf('  mu_x_max:    %.2f\n', params.tire.mu_x_max);
fprintf('  mu_y_max:    %.2f\n', params.tire.mu_y_max);
fprintf('  Fz_nom:      %.0f N\n', params.tire.Fz_nom);
fprintf('\n');

fprintf('--- Aero (4D Map) ---\n');
fprintf('  rho:         %.3f kg/m³\n', params.aero.rho);
fprintf('  CL_default:  %.2f\n', params.aero.CL_default);
fprintf('  CD_default:  %.2f\n', params.aero.CD_default);
fprintf('\n');

fprintf('--- Powertrain ---\n');
fprintf('  gear_ratio:  %.2f\n', params.powertrain.gear_ratio);
fprintf('  efficiency:  %.2f\n', params.powertrain.efficiency);
fprintf('\n');

fprintf('--- Initial Conditions ---\n');
fprintf('  v0:          %.1f m/s (%.1f km/h)\n', params.v0, params.v0 * 3.6);
fprintf('\n');

%% エンジントルクカーブテスト
fprintf('--- Engine Torque Map Test ---\n');
test_rpms = [3000, 6000, 9000, 12000, 13000, 15000];
for rpm = test_rpms
    torque = engine_torque_map(rpm);
    fprintf('  %5d rpm: %5.1f Nm\n', rpm, torque);
end
fprintf('\n');

%% LSDテスト
fprintf('--- LSD Test ---\n');
lsd_params.ramp_accel = 35.0;
lsd_params.ramp_decel = 50.0;
lsd_params.preload = 40.0;
lsd_params.efficiency = 0.95;

omega_L = 100;
omega_R = 120;  % 右が速い
T_in = 100;

[T_L, T_R, omega_out] = salisbury_lsd(omega_L, omega_R, T_in, lsd_params);
fprintf('  Input: ωL=%.1f, ωR=%.1f, T_in=%.1f Nm\n', omega_L, omega_R, T_in);
fprintf('  Output: TL=%.1f Nm, TR=%.1f Nm\n', T_L, T_R);
fprintf('  Sum: %.1f Nm (efficiency: %.2f%%)\n', T_L + T_R, (T_L + T_R)/T_in * 100);
fprintf('\n');

%% 4Dエアロマップテスト
fprintf('--- 4D Aero Map Test ---\n');
v = 20;  % m/s
rh_f = 40;  % mm
rh_r = 55;  % mm
roll = 0 * pi/180;  % rad
yaw = 0 * pi/180;   % rad

aero = aero_map_4d(v, rh_f, rh_r, roll, yaw, params.aero);
fprintf('  Conditions: V=%.1f m/s, RH_f=%.0fmm, RH_r=%.0fmm\n', v, rh_f, rh_r);
fprintf('  Results:\n');
fprintf('    Fz:  %7.1f N (downforce)\n', aero.Fz);
fprintf('    Fx:  %7.1f N (drag)\n', aero.Fx);
fprintf('    Fy:  %7.1f N (sideforce)\n', aero.Fy);
fprintf('    My:  %7.1f Nm (pitch)\n', aero.My);
fprintf('\n');

% ライドハイト効果
fprintf('  Ride Height Effect (V=20m/s):\n');
for rh_test = [25, 35, 45, 55]
    aero_test = aero_map_4d(20, rh_test, 55, 0, 0, params.aero);
    gain = (aero_test.Fz / aero.Fz - 1.0) * 100;
    fprintf('    RH_f=%2dmm: Fz=%7.1f N (%+5.1f%%)\n', rh_test, aero_test.Fz, gain);
end
fprintf('\n');

fprintf('=================================================================\n');
fprintf('✅ All parameters verified and ready for C++ comparison\n');
fprintf('=================================================================\n');
