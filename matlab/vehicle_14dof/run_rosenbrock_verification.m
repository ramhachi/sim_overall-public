%% run_rosenbrock_verification.m
% Rosenbrock法 vs RK4 比較検証

clear; clc; close all;

fprintf('==========================================\n');
fprintf(' Rosenbrock vs RK4 Integration Test\n');
fprintf('==========================================\n\n');

%% パラメータ設定
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
t_span = [0, 10];  % 10秒に延長
t_steer = 1.0;
delta = 0.1;
input_func = @(t) struct('throttle', 0.1, 'steering', (t >= t_steer) * delta);

%% RK4 (dt=1ms) - 標準
fprintf('Running RK4 (dt=1ms)...\n');
dt1 = 0.001;
tic;
[t1, state1, data1] = vehicle_14dof_model(params, t_span, dt1, input_func);
time1 = toc;
fprintf('  Time: %.3f s, Steps: %d\n', time1, length(t1)-1);

%% RK4 (dt=0.1ms) - 高精度
fprintf('Running RK4 (dt=0.1ms)...\n');
dt2 = 0.0001;
tic;
[t2, state2, data2] = vehicle_14dof_model(params, t_span, dt2, input_func);
time2 = toc;
fprintf('  Time: %.3f s, Steps: %d\n', time2, length(t2)-1);

%% MATLABのode23s（Rosenbrock法）
fprintf('Running ode23s (Rosenbrock method)...\n');

% 静的平衡計算
W = params.mass.m_sprung * 9.81;
Wf = W * params.mass.b / params.mass.L;
Wr = W * params.mass.a / params.mass.L;
static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2];

k_avg = 0;
for i = 1:4
    mr = params.suspension.motion_ratio(i);
    k_avg = k_avg + params.suspension.k_spring(i) * mr^2;
end
k_avg = k_avg / 4;
z_eq = W / (4 * k_avg);

k_tire = params.tire.k_tire;
z_u_eq = zeros(1, 4);
for i = 1:4
    total_load = static_Fz(i) + params.mass.m_unsprung(i) * 9.81;
    z_u_eq(i) = total_load / k_tire;
end

% 初期状態
x0 = zeros(40, 1);
x0(3) = z_eq;
x0(7) = params.v0;
for i = 1:4
    x0(12+i) = z_u_eq(i);
    x0(20+i) = params.v0 / params.tire.radius;
end

% ode23s用のODE関数
odefun = @(t, x) ode_14dof(t, x, input_func(t), params, static_Fz, z_eq, z_u_eq);

% 許容誤差を厳しく設定
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

tic;
[t3, state3] = ode23s(odefun, t_span, x0, options);
time3 = toc;
fprintf('  Time: %.3f s, Steps: %d\n', time3, length(t3)-1);

%% 結果比較
fprintf('\n=== Results Comparison ===\n');

% 最終状態
idx1 = length(t1);
idx2 = length(t2);
idx3 = length(t3);

fprintf('\nFinal Position:\n');
fprintf('  RK4 (1ms):    x=%.2f, y=%.2f m\n', state1(idx1,1), state1(idx1,2));
fprintf('  RK4 (0.1ms):  x=%.2f, y=%.2f m\n', state2(idx2,1), state2(idx2,2));
fprintf('  ode23s:       x=%.2f, y=%.2f m\n', state3(idx3,1), state3(idx3,2));

fprintf('\nFinal Velocity:\n');
fprintf('  RK4 (1ms):    vx=%.2f m/s\n', state1(idx1,7));
fprintf('  RK4 (0.1ms):  vx=%.2f m/s\n', state2(idx2,7));
fprintf('  ode23s:       vx=%.2f m/s\n', state3(idx3,7));

fprintf('\nMax Roll Angle:\n');
fprintf('  RK4 (1ms):    %.3f deg\n', max(abs(state1(:,4)))*180/pi);
fprintf('  RK4 (0.1ms):  %.3f deg\n', max(abs(state2(:,4)))*180/pi);
fprintf('  ode23s:       %.3f deg\n', max(abs(state3(:,4)))*180/pi);

fprintf('\nComputation Time:\n');
fprintf('  RK4 (1ms):    %.3f s\n', time1);
fprintf('  RK4 (0.1ms):  %.3f s (%.1fx slower)\n', time2, time2/time1);
fprintf('  ode23s:       %.3f s (%.1fx vs RK4 1ms)\n', time3, time3/time1);

%% プロット
figure('Position', [50, 50, 1400, 800]);

% 軌跡
subplot(2,3,1);
plot(state1(:,1), state1(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(state2(:,1), state2(:,2), 'r--', 'LineWidth', 1);
plot(state3(:,1), state3(:,2), 'g:', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory');
legend('RK4 (1ms)', 'RK4 (0.1ms)', 'ode23s');
axis equal; grid on;

% ロール角
subplot(2,3,2);
plot(t1, state1(:,4)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t2, state2(:,4)*180/pi, 'r--', 'LineWidth', 1);
plot(t3, state3(:,4)*180/pi, 'g:', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Roll [deg]');
title('Roll Angle');
legend('RK4 (1ms)', 'RK4 (0.1ms)', 'ode23s');
grid on;

% 速度
subplot(2,3,3);
plot(t1, state1(:,7), 'b-', 'LineWidth', 1.5); hold on;
plot(t2, state2(:,4), 'r--', 'LineWidth', 1);
plot(t3, state3(:,7), 'g:', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('vx [m/s]');
title('Forward Velocity');
legend('RK4 (1ms)', 'RK4 (0.1ms)', 'ode23s');
grid on;

% 計算時間バー
subplot(2,3,4);
bar([time1, time2, time3]);
xticklabels({'RK4 (1ms)', 'RK4 (0.1ms)', 'ode23s'});
ylabel('Time [s]');
title('Computation Time');
grid on;

% ステップ数
subplot(2,3,5);
bar([length(t1)-1, length(t2)-1, length(t3)-1]);
xticklabels({'RK4 (1ms)', 'RK4 (0.1ms)', 'ode23s'});
ylabel('Steps');
title('Number of Steps');
grid on;

% 誤差（RK4 0.1msを基準）
t_common = t1;
state2_interp = interp1(t2, state2, t_common, 'linear', 'extrap');
state3_interp = interp1(t3, state3, t_common, 'linear', 'extrap');

err1 = sqrt((state1(:,1) - state2_interp(:,1)).^2 + (state1(:,2) - state2_interp(:,2)).^2);
err3 = sqrt((state3_interp(:,1) - state2_interp(:,1)).^2 + (state3_interp(:,2) - state2_interp(:,2)).^2);

subplot(2,3,6);
plot(t_common, err1, 'b-', 'LineWidth', 1.5); hold on;
plot(t_common, err3, 'g-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Error vs RK4 (0.1ms)');
legend('RK4 (1ms)', 'ode23s');
grid on;

sgtitle('Rosenbrock (ode23s) vs RK4 Comparison');

saveas(gcf, '../../output/rosenbrock_comparison.png');
fprintf('\nSaved plot to output/rosenbrock_comparison.png\n');

%% 検証結果
fprintf('\n=== Verification ===\n');

% 位置誤差
pos_err_rk4 = sqrt((state1(end,1)-state2(end,1))^2 + (state1(end,2)-state2(end,2))^2);
pos_err_ode23s = sqrt((state3(end,1)-state2(end,1))^2 + (state3(end,2)-state2(end,2))^2);

if pos_err_rk4 < 0.5
    fprintf('✓ RK4 (1ms) accurate: error = %.4f m\n', pos_err_rk4);
else
    fprintf('✗ RK4 (1ms) error too large: %.4f m\n', pos_err_rk4);
end

if pos_err_ode23s < 0.5
    fprintf('✓ ode23s accurate: error = %.4f m\n', pos_err_ode23s);
else
    fprintf('✗ ode23s error too large: %.4f m\n', pos_err_ode23s);
end

if time3 < time2
    fprintf('✓ ode23s faster than fine RK4: %.1fx speedup\n', time2/time3);
else
    fprintf('! ode23s slower than fine RK4\n');
end

fprintf('\n==========================================\n');

%% ODE関数（14DOF）
function dxdt = ode_14dof(t, x, input, params, static_Fz, z_eq, z_u_eq)
% 状態変数の展開
z = x(3);
phi = x(4);
theta = x(5);
psi = x(6);
vx = x(7);
vy = x(8);
vz = x(9);
p = x(10);
q = x(11);
r = x(12);
z_u = x(13:16);
vz_u = x(17:20);
omega = x(21:24);
tire_xi = x(25:28);
tire_eta = x(29:32);
damper_z = x(33:36);
friction_z = x(37:40);

% 定数
m = params.mass.m_sprung;
g = 9.81;
Ixx = params.mass.Ixx;
Iyy = params.mass.Iyy;
Izz = params.mass.Izz;
k_tire = params.tire.k_tire;
c_tire = params.tire.c_tire;
h_cg = params.mass.h_cg;

% 車輪位置
a_dist = params.mass.a;
b_dist = params.mass.b;
tf = params.mass.track_f / 2;
tr = params.mass.track_r / 2;
wheel_x = [a_dist, a_dist, -b_dist, -b_dist];
wheel_y = [-tf, tf, -tr, tr];

% 回転行列
R = euler_to_rotation(phi, theta, psi);
E = angular_to_euler_rate(phi, theta);

% 運動学
v_body = [vx; vy; vz];
v_inertial = R' * v_body;
euler_rate = E * [p; q; r];

% ステアリング角
delta = input.steering;
steer = [delta, delta, 0, 0];

% 初期化
Fx_wheel = zeros(1, 4);
Fy_wheel = zeros(1, 4);
F_spring = zeros(1, 4);
F_bw = zeros(1, 4);
F_lugre = zeros(1, 4);
az_u_vec = zeros(4, 1);
dxi = zeros(4, 1);
deta = zeros(4, 1);
d_damper_z = zeros(4, 1);
d_friction_z = zeros(4, 1);
susp_travel = zeros(1, 4);

for i = 1:4
    % サスペンション変位
    z_sprung = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi);
    susp_travel(i) = z_sprung - z_u(i);
    
    vz_sprung = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi);
    susp_vel = vz_sprung - vz_u(i);
    
    susp_from_eq = susp_travel(i) - (z_eq - z_u_eq(i));
    
    % スプリング力
    mr = params.suspension.motion_ratio(i);
    k_eff = params.suspension.k_spring(i) * mr^2;
    F_spring(i) = static_Fz(i) + k_eff * susp_from_eq;
    
    % Bouc-Wenダンパー
    z_bw = damper_z(i);
    F_bw(i) = params.suspension.bw_c0 * susp_vel + params.suspension.bw_alpha * z_bw;
    d_damper_z(i) = params.suspension.bw_A * susp_vel ...
        - params.suspension.bw_beta * abs(susp_vel) * abs(z_bw)^(params.suspension.bw_n-1) * z_bw;
    
    % LuGre摩擦
    z_fr = friction_z(i);
    g_v = params.suspension.lugre_Fc + (params.suspension.lugre_Fs - params.suspension.lugre_Fc) ...
        * exp(-(susp_vel/params.suspension.lugre_vs)^2);
    if abs(susp_vel) > 1e-10
        sigma0_over_g = params.suspension.lugre_sigma0 / g_v;
        d_friction_z(i) = susp_vel - sigma0_over_g * abs(susp_vel) * z_fr;
    else
        d_friction_z(i) = 0;
    end
    F_lugre(i) = params.suspension.lugre_sigma0 * z_fr + params.suspension.lugre_sigma1 * d_friction_z(i);
    
    % タイヤ垂直力
    tire_def = z_u(i) - z_u_eq(i);
    Fz_tire = static_Fz(i) + params.mass.m_unsprung(i) * g + k_tire * tire_def + c_tire * vz_u(i);
    Fz_tire = max(Fz_tire, 0);
    
    % タイヤ力計算
    vx_c = vx - r * wheel_y(i);
    vy_c = vy + r * wheel_x(i);
    cs = cos(steer(i)); sn = sin(steer(i));
    vx_w = cs * vx_c + sn * vy_c;
    vy_w = -sn * vx_c + cs * vy_c;
    
    omega_r = omega(i) * params.tire.radius;
    vsx = vx_w - omega_r;
    vsy = vy_w;
    vcx = max(abs(omega_r), 0.1);
    
    [Fx_t, Fy_t, dxi(i), deta(i)] = tmeasy(tire_xi(i), tire_eta(i), Fz_tire, vsx, vsy, vcx, params.tire);
    
    Fx_wheel(i) = cs * Fx_t - sn * Fy_t;
    Fy_wheel(i) = sn * Fx_t + cs * Fy_t;
    
    % ばね下
    m_u = params.mass.m_unsprung(i);
    F_tire_react = -k_tire * tire_def - c_tire * vz_u(i);
    F_susp = F_spring(i) + F_bw(i) + F_lugre(i);
    az_u_vec(i) = (F_tire_react + F_susp) / m_u;
end

% ARB
F_arb = compute_arb(susp_travel, params);

% 合計力・モーメント
Fx_tot = sum(Fx_wheel);
Fy_tot = sum(Fy_wheel);
Fz_tot = 0; Mx_tot = 0; My_tot = 0; Mz_tot = 0;

for i = 1:4
    F_susp = F_spring(i) + F_bw(i) + F_lugre(i) + F_arb(i);
    Fz_tot = Fz_tot - F_susp;
    Mx_tot = Mx_tot - wheel_y(i) * F_susp + Fy_wheel(i) * h_cg;
    My_tot = My_tot + wheel_x(i) * F_susp;
    Mz_tot = Mz_tot + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i);
end

% 空力
v2 = vx^2 + vy^2;
Fz_tot = Fz_tot + 0.5 * params.aero.rho * v2 * params.aero.CL_A;
Fx_tot = Fx_tot - 0.5 * params.aero.rho * v2 * params.aero.CD_A;

% 重力
g_body = R * [0; 0; g];

% ばね上運動
ax = Fx_tot / m + g_body(1) + r * vy - q * vz;
ay = Fy_tot / m + g_body(2) - r * vx + p * vz;
az = Fz_tot / m + g_body(3) + q * vx - p * vy;

dp = Mx_tot / Ixx;
dq = My_tot / Iyy;
dr = Mz_tot / Izz;

% 車輪回転
Iw = params.mass.Iw;
R_tire = params.tire.radius;
engine_torque = input.throttle * params.powertrain.max_torque;
drive_torque = engine_torque * params.powertrain.gear_ratio * params.powertrain.efficiency / 2;

domega = zeros(4, 1);
for i = 1:4
    T_drive = (i >= 3) * drive_torque;
    cs = cos(steer(i)); sn = sin(steer(i));
    Fx_w = cs * Fx_wheel(i) + sn * Fy_wheel(i);
    T_tire = -Fx_w * R_tire;
    domega(i) = (T_drive + T_tire) / Iw;
end

% 出力
dxdt = [v_inertial; euler_rate; ax; ay; az; dp; dq; dr; ...
    vz_u(:); az_u_vec(:); domega(:); dxi(:); deta(:); d_damper_z(:); d_friction_z(:)];
end

function R = euler_to_rotation(phi, theta, psi)
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [cpsi*cth, spsi*cth, -sth;
    cpsi*sth*sphi-spsi*cphi, spsi*sth*sphi+cpsi*cphi, cth*sphi;
    cpsi*sth*cphi+spsi*sphi, spsi*sth*cphi-cpsi*sphi, cth*cphi];
end

function E = angular_to_euler_rate(phi, theta)
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); tth = tan(theta);
E = [1, sphi*tth, cphi*tth; 0, cphi, -sphi; 0, sphi/cth, cphi/cth];
end

function F = compute_arb(susp, params)
F = zeros(1, 4);
track_f = params.mass.track_f;
roll_f = (susp(1) - susp(2)) / track_f;
torque_f = params.suspension.k_arb_f * roll_f;
F(1) = torque_f / (track_f/2);
F(2) = -torque_f / (track_f/2);

track_r = params.mass.track_r;
roll_r = (susp(3) - susp(4)) / track_r;
torque_r = params.suspension.k_arb_r * roll_r;
F(3) = torque_r / (track_r/2);
F(4) = -torque_r / (track_r/2);
end

function [Fx, Fy, dxi, deta] = tmeasy(xi, eta, Fz, vsx, vsy, vcx, tire)
dxi = vsx - (vcx / tire.Lx) * xi;
deta = vsy - (vcx / tire.Ly) * eta;

xi_n = tire.Cx * xi / (tire.mu_x_max * Fz + 1);
eta_n = tire.Cy * eta / (tire.mu_y_max * Fz + 1);

s = sqrt(xi_n^2 + eta_n^2);
if s < 1
    mu = s * (2 - s);
else
    mu = 1;
end

if s > 1e-6
    Fx = -mu * tire.mu_x_max * Fz * xi_n / s;
    Fy = -mu * tire.mu_y_max * Fz * eta_n / s;
else
    Fx = 0;
    Fy = 0;
end
end
