%% vehicle_14dof_model.m
% 14DOF完全車両モデル
%
% 自由度構成:
%   - 6 DOF ばね上質量 (x, y, z, φ, θ, ψ)
%   - 4 DOF ばね下垂直 (z_u × 4)
%   - 4 DOF 車輪回転 (ω × 4)
%
% 統合サブシステム:
%   - TMeasyタイヤ
%   - Bouc-Wenダンパー
%   - LuGre摩擦
%   - アンチロールバー
%   - バンプストップ
%   - 空力
%
% 状態変数: 40
%   [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r,
%    z_u(4), vz_u(4), ω(4),
%    tire_xi(4), tire_eta(4),
%    damper_z(4), friction_z(4)]

function [t, state, data] = vehicle_14dof_model(params, t_span, dt, input_func)
% 静的平衡計算
[static_Fz, z_eq, z_u_eq] = compute_equilibrium(params);

% 初期状態 (40変数)
x0 = zeros(40, 1);
x0(3) = z_eq;  % z_sprung at equilibrium
x0(7) = params.v0;  % vx
for i = 1:4
    x0(12+i) = z_u_eq(i);  % z_u
    x0(20+i) = params.v0 / params.tire.radius;  % omega
    x0(32+i) = 0;  % damper_z
    x0(36+i) = 0;  % friction_z
end

% RK4積分
t = t_span(1):dt:t_span(2);
n_steps = length(t);
state = zeros(n_steps, 40);
state(1, :) = x0';

% データ記録用
data.ax = zeros(n_steps, 1);
data.ay = zeros(n_steps, 1);
data.Fz = zeros(n_steps, 4);
data.Fx = zeros(n_steps, 4);
data.Fy = zeros(n_steps, 4);
data.F_spring = zeros(n_steps, 4);
data.F_bw = zeros(n_steps, 4);
data.F_lugre = zeros(n_steps, 4);

for k = 1:(n_steps-1)
    x = state(k, :)';
    input = input_func(t(k));
    
    % RK4
    [k1, ~] = derivatives(x, input, params, static_Fz, z_eq, z_u_eq);
    [k2, ~] = derivatives(x + 0.5*dt*k1, input, params, static_Fz, z_eq, z_u_eq);
    [k3, ~] = derivatives(x + 0.5*dt*k2, input, params, static_Fz, z_eq, z_u_eq);
    [k4, forces] = derivatives(x + dt*k3, input, params, static_Fz, z_eq, z_u_eq);
    
    state(k+1, :) = (x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4))';
    
    data.ax(k) = forces.ax;
    data.ay(k) = forces.ay;
    data.Fz(k, :) = forces.Fz;
    data.Fx(k, :) = forces.Fx;
    data.Fy(k, :) = forces.Fy;
    data.F_spring(k, :) = forces.F_spring;
    data.F_bw(k, :) = forces.F_bw;
    data.F_lugre(k, :) = forces.F_lugre;
end

% 最終ステップのデータ
data.ax(end) = data.ax(end-1);
data.ay(end) = data.ay(end-1);
data.Fz(end, :) = data.Fz(end-1, :);
data.Fx(end, :) = data.Fx(end-1, :);
data.Fy(end, :) = data.Fy(end-1, :);
data.F_spring(end, :) = data.F_spring(end-1, :);
data.F_bw(end, :) = data.F_bw(end-1, :);
data.F_lugre(end, :) = data.F_lugre(end-1, :);
end

function [static_Fz, z_eq, z_u_eq] = compute_equilibrium(params)
m = params.mass.m_sprung;
g = 9.81;
a = params.mass.a;
b = params.mass.b;
L = params.mass.L;

W = m * g;
Wf = W * b / L;
Wr = W * a / L;

static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2];

% ばね上平均ばね定数
k_avg = 0;
for i = 1:4
    mr = params.suspension.motion_ratio(i);
    k_avg = k_avg + params.suspension.k_spring(i) * mr^2;
end
k_avg = k_avg / 4;
z_eq = W / (4 * k_avg);

% ばね下平衡位置
k_tire = params.tire.k_tire;
z_u_eq = zeros(1, 4);
for i = 1:4
    total_load = static_Fz(i) + params.mass.m_unsprung(i) * g;
    z_u_eq(i) = total_load / k_tire;
end
end

function [dxdt, forces] = derivatives(x, input, params, static_Fz, z_eq, z_u_eq)
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

% Bouc-Wenパラメータ
bw_c0 = params.suspension.bw_c0;
bw_k0 = params.suspension.bw_k0;
bw_alpha = params.suspension.bw_alpha;
bw_A = params.suspension.bw_A;
bw_beta = params.suspension.bw_beta;
bw_gamma = params.suspension.bw_gamma;
bw_n = params.suspension.bw_n;

% LuGreパラメータ
lugre_sigma0 = params.suspension.lugre_sigma0;
lugre_sigma1 = params.suspension.lugre_sigma1;
lugre_sigma2 = params.suspension.lugre_sigma2;
lugre_Fc = params.suspension.lugre_Fc;
lugre_Fs = params.suspension.lugre_Fs;
lugre_vs = params.suspension.lugre_vs;

% バンプストップ
bump_gap = params.suspension.bump_stop_gap;
bump_k = params.suspension.bump_stop_k;

% 車輪位置
a = params.mass.a;
b = params.mass.b;
tf = params.mass.track_f / 2;
tr = params.mass.track_r / 2;

wheel_x = [a, a, -b, -b];
wheel_y = [-tf, tf, -tr, tr];

% 回転行列
R = euler_to_rotation_matrix(phi, theta, psi);
E = angular_velocity_to_euler_rate(phi, theta);

% 運動学
v_body = [vx; vy; vz];
v_inertial = R' * v_body;
euler_rate = E * [p; q; r];

% ステアリング角
delta = input.steering;
steer = [delta, delta, 0, 0];

% 各輪の力計算
Fx_wheel = zeros(1, 4);
Fy_wheel = zeros(1, 4);
F_spring = zeros(1, 4);
F_bw = zeros(1, 4);
F_lugre = zeros(1, 4);
F_bump = zeros(1, 4);
Fz_tire = zeros(1, 4);
dxi = zeros(4, 1);
deta = zeros(4, 1);
az_u = zeros(4, 1);
d_damper_z = zeros(4, 1);
d_friction_z = zeros(4, 1);
susp_travel = zeros(1, 4);

for i = 1:4
    % サスペンション変位
    z_sprung_at_wheel = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi);
    susp_travel(i) = z_sprung_at_wheel - z_u(i);
    
    % サスペンション速度
    vz_sprung_at_wheel = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi);
    susp_velocity = vz_sprung_at_wheel - vz_u(i);
    
    % 平衡からの偏差
    susp_travel_from_eq = susp_travel(i) - (z_eq - z_u_eq(i));
    
    % スプリング力
    mr = params.suspension.motion_ratio(i);
    k_eff = params.suspension.k_spring(i) * mr^2;
    F_spring(i) = static_Fz(i) + k_eff * susp_travel_from_eq;
    
    % Bouc-Wenダンパー力
    z_bw = damper_z(i);
    F_bw(i) = bw_c0 * susp_velocity + bw_k0 * susp_travel_from_eq + bw_alpha * z_bw;
    
    abs_z_n_minus_1 = abs(z_bw)^(bw_n - 1);
    abs_z_n = abs_z_n_minus_1 * abs(z_bw);
    d_damper_z(i) = bw_A * susp_velocity ...
        - bw_beta * abs(susp_velocity) * z_bw * abs_z_n_minus_1 ...
        - bw_gamma * susp_velocity * abs_z_n;
    
    % LuGre摩擦力
    z_fr = friction_z(i);
    g_v = lugre_Fc + (lugre_Fs - lugre_Fc) * exp(-(susp_velocity/lugre_vs)^2);
    
    if abs(susp_velocity) > 1e-10
        sigma0_over_g = lugre_sigma0 / g_v;
        d_friction_z(i) = susp_velocity - sigma0_over_g * abs(susp_velocity) * z_fr;
    else
        z_ss = lugre_Fs / lugre_sigma0;
        if abs(z_fr) > z_ss
            d_friction_z(i) = -z_fr / (z_ss / lugre_sigma1);
        else
            d_friction_z(i) = 0;
        end
    end
    
    F_lugre(i) = lugre_sigma0 * z_fr + lugre_sigma1 * d_friction_z(i) + lugre_sigma2 * susp_velocity;
    
    % バンプストップ力
    if susp_travel_from_eq > bump_gap
        F_bump(i) = bump_k * (susp_travel_from_eq - bump_gap);
    elseif susp_travel_from_eq < -bump_gap
        F_bump(i) = bump_k * (susp_travel_from_eq + bump_gap);
    else
        F_bump(i) = 0;
    end
    
    % タイヤ垂直力
    tire_deflection = z_u(i) - z_u_eq(i);
    tire_velocity = vz_u(i);
    Fz_tire(i) = static_Fz(i) + params.mass.m_unsprung(i) * g ...
        + k_tire * tire_deflection + c_tire * tire_velocity;
    Fz_tire(i) = max(Fz_tire(i), 0);
    
    % タイヤ滑り計算
    vx_contact = vx - r * wheel_y(i);
    vy_contact = vy + r * wheel_x(i);
    
    c = cos(steer(i));
    s = sin(steer(i));
    vx_wheel = c * vx_contact + s * vy_contact;
    vy_wheel = -s * vx_contact + c * vy_contact;
    
    omega_r = omega(i) * params.tire.radius;
    vsx = vx_wheel - omega_r;
    vsy = vy_wheel;
    vcx = abs(omega_r);
    
    % TMeasyタイヤ力
    [Fx_tire, Fy_tire, dxi(i), deta(i)] = tmeasy_force(...
        tire_xi(i), tire_eta(i), Fz_tire(i), vsx, vsy, max(vcx, 0.1), params.tire);
    
    Fx_wheel(i) = c * Fx_tire - s * Fy_tire;
    Fy_wheel(i) = s * Fx_tire + c * Fy_tire;
    
    % ばね下運動方程式
    m_u = params.mass.m_unsprung(i);
    F_tire_reaction = -k_tire * tire_deflection - c_tire * tire_velocity;
    F_susp_on_unsprung = F_spring(i) + F_bw(i) + F_lugre(i) + F_bump(i);
    az_u(i) = (F_tire_reaction + F_susp_on_unsprung) / m_u;
end

% ARB力
F_arb = compute_arb_forces(susp_travel, params);

% 合計力・モーメント
Fx_total = sum(Fx_wheel);
Fy_total = sum(Fy_wheel);
Fz_total = 0;
Mx_total = 0;
My_total = 0;
Mz_total = 0;

for i = 1:4
    F_susp = F_spring(i) + F_bw(i) + F_lugre(i) + F_bump(i) + F_arb(i);
    Fz_total = Fz_total - F_susp;
    
    Mx_total = Mx_total - wheel_y(i) * F_susp;
    Mx_total = Mx_total + Fy_wheel(i) * h_cg;
    
    My_total = My_total + wheel_x(i) * F_susp;
    Mz_total = Mz_total + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i);
end

% 空力（4Dエアロマップ）
v_magnitude = sqrt(vx^2 + vy^2);

% ライドハイト計算
rh_front = 0.5 * ((z - z_u(1)) + (z - z_u(2))) * 1000;  % mm
rh_rear = 0.5 * ((z - z_u(3)) + (z - z_u(4))) * 1000;   % mm

% ロール角
roll_rad = phi;

% ヨー角（車体スリップ角）
if v_magnitude > 0.1
    beta = atan2(vy, vx);
else
    beta = 0;
end

% 4Dエアロマップから力を計算
aero_result = aero_map_4d(v_magnitude, rh_front, rh_rear, roll_rad, beta, params.aero);

Fx_total = Fx_total + aero_result.Fx;  % ドラッグ
Fy_total = Fy_total + aero_result.Fy;  % サイドフォース
Fz_total = Fz_total + aero_result.Fz;  % ダウンフォース

% エアロモーメント
Mx_total = Mx_total + aero_result.Mx;
My_total = My_total + aero_result.My;
Mz_total = Mz_total + aero_result.Mz;

% 重力
g_body = gravity_in_body(phi, theta, g);

% ばね上運動方程式
ax = Fx_total / m + g_body(1) + r * vy - q * vz;
ay = Fy_total / m + g_body(2) - r * vx + p * vz;
az = Fz_total / m + g_body(3) + q * vx - p * vy;

dp = Mx_total / Ixx;
dq = My_total / Iyy;
dr = Mz_total / Izz;

% 車輪回転
Iw = params.mass.Iw;
R_tire = params.tire.radius;

% エンジンRPM計算（後輪平均）
omega_rear_avg = 0.5 * (omega(3) + omega(4));
omega_diff = omega_rear_avg;
engine_rpm = omega_diff * params.powertrain.gear_ratio * 30 / pi;

% エンジントルク（トルクカーブから）
engine_torque = input.throttle * engine_torque_map(engine_rpm);

% LSDパラメータ
lsd_params = struct();
lsd_params.ramp_accel = 35.0;
lsd_params.ramp_decel = 50.0;
lsd_params.preload = 40.0;
lsd_params.efficiency = params.powertrain.efficiency;

% LSDによるトルク配分
T_in = engine_torque * params.powertrain.gear_ratio;
[T_L, T_R, ~] = salisbury_lsd(omega(3), omega(4), T_in, lsd_params);

domega = zeros(4, 1);
for i = 1:4
    if i == 3
        T_drive = T_L;  % 左後輪
    elseif i == 4
        T_drive = T_R;  % 右後輪
    else
        T_drive = 0;    % 前輪
    end
    
    c = cos(steer(i));
    s = sin(steer(i));
    Fx_w = c * Fx_wheel(i) + s * Fy_wheel(i);
    T_tire = -Fx_w * R_tire;
    
    domega(i) = (T_drive + T_tire) / Iw;
end

% 出力（全て列ベクトルに統一）
dxdt = [v_inertial; euler_rate; ax; ay; az; dp; dq; dr; ...
    vz_u(:); az_u(:); domega(:); dxi(:); deta(:); d_damper_z(:); d_friction_z(:)];

forces.ax = ax;
forces.ay = ay + vx * r;
forces.Fz = Fz_tire;
forces.Fx = Fx_wheel;
forces.Fy = Fy_wheel;
forces.F_spring = F_spring;
forces.F_bw = F_bw;
forces.F_lugre = F_lugre;
end

function F_arb = compute_arb_forces(susp_travel, params)
F_arb = zeros(1, 4);

track_f = params.mass.track_f;
roll_diff_f = susp_travel(1) - susp_travel(2);
roll_angle_f = roll_diff_f / track_f;
torque_arb_f = params.suspension.k_arb_f * roll_angle_f;
half_track_f = track_f / 2;
F_arb(1) = torque_arb_f / half_track_f;
F_arb(2) = -torque_arb_f / half_track_f;

track_r = params.mass.track_r;
roll_diff_r = susp_travel(3) - susp_travel(4);
roll_angle_r = roll_diff_r / track_r;
torque_arb_r = params.suspension.k_arb_r * roll_angle_r;
half_track_r = track_r / 2;
F_arb(3) = torque_arb_r / half_track_r;
F_arb(4) = -torque_arb_r / half_track_r;
end

function R = euler_to_rotation_matrix(phi, theta, psi)
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

R = [cpsi*cth,                 spsi*cth,                -sth;
    cpsi*sth*sphi-spsi*cphi, spsi*sth*sphi+cpsi*cphi, cth*sphi;
    cpsi*sth*cphi+spsi*sphi, spsi*sth*cphi-cpsi*sphi, cth*cphi];
end

function E = angular_velocity_to_euler_rate(phi, theta)
cphi = cos(phi); sphi = sin(phi);
cth = cos(theta); tth = tan(theta);

E = [1, sphi*tth,  cphi*tth;
    0, cphi,     -sphi;
    0, sphi/cth,  cphi/cth];
end

function g_body = gravity_in_body(phi, theta, g)
R = euler_to_rotation_matrix(phi, theta, 0);
g_inertial = [0; 0; g];
g_body = R * g_inertial;
end

function [Fx, Fy, dxi, deta] = tmeasy_force(xi, eta, Fz, vsx, vsy, vcx, tire)
mu_x = tire.mu_x_max;
mu_y = tire.mu_y_max;
Cx = tire.Cx;
Cy = tire.Cy;
Lx = tire.Lx;
Ly = tire.Ly;

dxi = vsx - (vcx / Lx) * xi;
deta = vsy - (vcx / Ly) * eta;

xi_norm = Cx * xi / (mu_x * Fz + 1);
eta_norm = Cy * eta / (mu_y * Fz + 1);

s = sqrt(xi_norm^2 + eta_norm^2);

if s < 1
    mu_eff = s * (2 - s);
else
    mu_eff = 1;
end

if s > 1e-6
    Fx = -mu_eff * mu_x * Fz * xi_norm / s;
    Fy = -mu_eff * mu_y * Fz * eta_norm / s;
else
    Fx = 0;
    Fy = 0;
end
end
