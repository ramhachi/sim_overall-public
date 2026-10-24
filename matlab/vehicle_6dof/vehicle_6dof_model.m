%% vehicle_6dof_model.m
% 6DOFビークルモデル（C++との検証用）
%
% 自由度: x, y, z, φ(roll), θ(pitch), ψ(yaw)
% 状態変数: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r, omega(4), tire_xi(4), tire_eta(4)]

function [t, state, data] = vehicle_6dof_model(params, t_span, dt, input_func)
% 静的平衡計算
[static_Fz, z_eq] = compute_equilibrium(params);

% 初期状態 (24変数)
x0 = zeros(24, 1);
x0(3) = z_eq;  % z at equilibrium
x0(7) = params.v0;  % vx
for i = 1:4
    x0(12+i) = params.v0 / params.tire.radius;  % omega
end

% RK4積分
t = t_span(1):dt:t_span(2);
n_steps = length(t);
state = zeros(n_steps, 24);
state(1, :) = x0';

% データ記録用
data.ax = zeros(n_steps, 1);
data.ay = zeros(n_steps, 1);
data.Fz = zeros(n_steps, 4);

for k = 1:(n_steps-1)
    x = state(k, :)';
    input = input_func(t(k));
    
    % RK4
    [k1, ~] = derivatives(x, input, params, static_Fz, z_eq);
    [k2, ~] = derivatives(x + 0.5*dt*k1, input, params, static_Fz, z_eq);
    [k3, ~] = derivatives(x + 0.5*dt*k2, input, params, static_Fz, z_eq);
    [k4, forces] = derivatives(x + dt*k3, input, params, static_Fz, z_eq);
    
    state(k+1, :) = (x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4))';
    
    data.ax(k) = forces.ax;
    data.ay(k) = forces.ay;
    data.Fz(k, :) = forces.Fz;
end

data.ax(end) = data.ax(end-1);
data.ay(end) = data.ay(end-1);
data.Fz(end, :) = data.Fz(end-1, :);
end

function [static_Fz, z_eq] = compute_equilibrium(params)
m = params.mass.m_sprung;
g = 9.81;
a = params.mass.a;
b = params.mass.b;
L = params.mass.L;

W = m * g;
Wf = W * b / L;
Wr = W * a / L;

static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2];

% 平均ばね定数
k_avg = 0;
for i = 1:4
    mr = params.suspension.motion_ratio(i);
    k_avg = k_avg + params.suspension.k_spring(i) * mr^2;
end
k_avg = k_avg / 4;

z_eq = W / (4 * k_avg);
end

function [dxdt, forces] = derivatives(x, input, params, static_Fz, z_eq)
% 状態変数の展開
% x(1:6) = [x, y, z, phi, theta, psi]
% x(7:12) = [vx, vy, vz, p, q, r]
% x(13:16) = omega[4]
% x(17:20) = tire_xi[4]
% x(21:24) = tire_eta[4]

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
omega = x(13:16);
tire_xi = x(17:20);
tire_eta = x(21:24);

m = params.mass.m_sprung;
g = 9.81;
Ixx = params.mass.Ixx;
Iyy = params.mass.Iyy;
Izz = params.mass.Izz;

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

% 位置微分（慣性座標系）
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
F_damper = zeros(1, 4);
Fz_tire = zeros(1, 4);
dxi = zeros(4, 1);
deta = zeros(4, 1);

for i = 1:4
    % サスペンション変位（符号修正 - C++と一致）
    % SAE座標系: +z = 下向き、+theta = ノーズアップ、+phi = 右下がり
    z_sprung_at_wheel = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi);
    susp_travel = z_sprung_at_wheel - z_eq;
    
    susp_velocity = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi);
    
    % スプリング力（静的成分 + 偏差成分）
    mr = params.suspension.motion_ratio(i);
    k_eff = params.suspension.k_spring(i) * mr^2;
    F_spring_static = static_Fz(i);  % 平衡時の力
    F_spring_delta = k_eff * susp_travel;  % 偏差による力
    F_spring(i) = F_spring_static + F_spring_delta;
    
    % ダンパー力（線形減衰）
    c_eff = params.suspension.c0 * mr^2;
    F_damper(i) = c_eff * susp_velocity;
    
    % タイヤ荷重 = サスペンション力（路面からの反力）
    Fz_tire(i) = F_spring(i) + F_damper(i);
    Fz_tire(i) = max(Fz_tire(i), 0);
    
    % タイヤ力計算
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
    
    [Fx_tire, Fy_tire, dxi(i), deta(i)] = tmeasy_force(...
        tire_xi(i), tire_eta(i), Fz_tire(i), vsx, vsy, max(vcx, 0.1), params.tire);
    
    Fx_wheel(i) = c * Fx_tire - s * Fy_tire;
    Fy_wheel(i) = s * Fx_tire + c * Fy_tire;
end

% 合計力とモーメント
Fx_total = sum(Fx_wheel);
Fy_total = sum(Fy_wheel);
Fz_total = 0;
Mx_total = 0;
My_total = 0;
Mz_total = 0;

h_cg = params.mass.h_cg;  % 重心高

for i = 1:4
    F_susp = F_spring(i) + F_damper(i);
    Fz_total = Fz_total - F_susp;
    
    % ロールモーメント = サスペンション力による成分 + タイヤ横力×重心高
    Mx_total = Mx_total - wheel_y(i) * F_susp;
    Mx_total = Mx_total + Fy_wheel(i) * h_cg;
    
    My_total = My_total + wheel_x(i) * F_susp;
    Mz_total = Mz_total + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i);
end

% 空力
v2 = vx^2 + vy^2;
Fz_aero = 0.5 * params.aero.rho * v2 * params.aero.CL_A;
Fx_aero = -0.5 * params.aero.rho * v2 * params.aero.CD_A;

Fx_total = Fx_total + Fx_aero;
Fz_total = Fz_total + Fz_aero;

% 重力（車体座標系）
g_body = gravity_in_body(phi, theta, g);

% 並進運動方程式
ax = Fx_total / m + g_body(1) + r * vy - q * vz;
ay = Fy_total / m + g_body(2) - r * vx + p * vz;
az = Fz_total / m + g_body(3) + q * vx - p * vy;

% 回転運動方程式
dp = Mx_total / Ixx;
dq = My_total / Iyy;
dr = Mz_total / Izz;

% 車輪回転
Iw = params.mass.Iw;
R_tire = params.tire.radius;

engine_torque = input.throttle * params.powertrain.max_torque;
drive_torque = engine_torque * params.powertrain.gear_ratio * params.powertrain.efficiency / 2;

domega = zeros(4, 1);
for i = 1:4
    if i >= 3
        T_drive = drive_torque;
    else
        T_drive = 0;
    end
    
    c = cos(steer(i));
    s = sin(steer(i));
    Fx_w = c * Fx_wheel(i) + s * Fy_wheel(i);
    T_tire = -Fx_w * R_tire;
    
    domega(i) = (T_drive + T_tire) / Iw;
end

% 出力
dxdt = [v_inertial; euler_rate; ax; ay; az; dp; dq; dr; domega; dxi; deta];

forces.ax = ax;
forces.ay = ay + vx * r;
forces.Fz = Fz_tire;
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
