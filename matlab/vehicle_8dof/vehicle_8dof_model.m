%% vehicle_8dof_model.m
% 8DOFビークルモデル（6DOF + 4輪ばね下垂直動）
%
% 状態変数: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r,
%            z_u(4), vz_u(4), omega(4), tire_xi(4), tire_eta(4)]
% 合計: 12 + 8 + 4 + 8 = 32 状態変数

function [t, state, data] = vehicle_8dof_model(params, t_span, dt, input_func)
% 静的平衡計算
[static_Fz, z_eq, z_u_eq] = compute_equilibrium(params);

% 初期状態 (32変数)
x0 = zeros(32, 1);
x0(3) = z_eq;  % z_sprung at equilibrium
x0(7) = params.v0;  % vx
for i = 1:4
    x0(12+i) = z_u_eq(i);  % z_u
    x0(20+i) = params.v0 / params.tire.radius;  % omega
end

% RK4積分
t = t_span(1):dt:t_span(2);
n_steps = length(t);
state = zeros(n_steps, 32);
state(1, :) = x0';

% データ記録用
data.ax = zeros(n_steps, 1);
data.ay = zeros(n_steps, 1);
data.Fz = zeros(n_steps, 4);

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
end

data.ax(end) = data.ax(end-1);
data.ay(end) = data.ay(end-1);
data.Fz(end, :) = data.Fz(end-1, :);
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

m = params.mass.m_sprung;
g = 9.81;
Ixx = params.mass.Ixx;
Iyy = params.mass.Iyy;
Izz = params.mass.Izz;
k_tire = params.tire.k_tire;
c_tire = params.tire.c_tire;

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

% 位置微分
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
az_u = zeros(4, 1);  % ばね下加速度

for i = 1:4
    % ばね上の車輪位置でのz座標
    z_sprung_at_wheel = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi);
    
    % サスペンション変位 = ばね上 - ばね下
    susp_travel = z_sprung_at_wheel - z_u(i);
    
    % サスペンション速度
    vz_sprung_at_wheel = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi);
    susp_velocity = vz_sprung_at_wheel - vz_u(i);
    
    % スプリング・ダンパー力
    mr = params.suspension.motion_ratio(i);
    k_eff = params.suspension.k_spring(i) * mr^2;
    c_eff = params.suspension.c0 * mr^2;
    
    % 平衡からの偏差
    susp_travel_from_eq = susp_travel - (z_eq - z_u_eq(i));
    F_spring(i) = static_Fz(i) + k_eff * susp_travel_from_eq;
    F_damper(i) = c_eff * susp_velocity;
    
    % タイヤ垂直力
    tire_deflection = z_u(i) - z_u_eq(i);
    tire_velocity = vz_u(i);
    Fz_tire(i) = static_Fz(i) + params.mass.m_unsprung(i) * g ...
        + k_tire * tire_deflection + c_tire * tire_velocity;
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
    
    % ばね下運動方程式
    m_u = params.mass.m_unsprung(i);
    F_tire_reaction = -k_tire * tire_deflection - c_tire * tire_velocity;
    F_susp_on_unsprung = F_spring(i) + F_damper(i);
    az_u(i) = (F_tire_reaction + F_susp_on_unsprung) / m_u;
end

% ばね上の力とモーメント
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
    Mx_total = Mx_total + Fy_wheel(i) * h_cg;  % 横力による重心周りのロールモーメント
    
    My_total = My_total + wheel_x(i) * F_susp;
    Mz_total = Mz_total + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i);
end

% 空力
v2 = vx^2 + vy^2;
Fz_total = Fz_total + 0.5 * params.aero.rho * v2 * params.aero.CL_A;
Fx_total = Fx_total - 0.5 * params.aero.rho * v2 * params.aero.CD_A;

% 重力
g_body = gravity_in_body(phi, theta, g);

% ばね上並進運動
ax = Fx_total / m + g_body(1) + r * vy - q * vz;
ay = Fy_total / m + g_body(2) - r * vx + p * vz;
az = Fz_total / m + g_body(3) + q * vx - p * vy;

% ばね上回転運動
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
dxdt = [v_inertial; euler_rate; ax; ay; az; dp; dq; dr; ...
    vz_u; az_u; domega; dxi; deta];

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
