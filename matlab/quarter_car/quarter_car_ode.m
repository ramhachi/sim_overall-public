%% quarter_car_ode.m
% Quarter Carモデル（C++検証用Ground Truth生成）
% パラメータはC++実装と完全一致

function [t, x, data] = quarter_car_ode(params, t_span, x0, input_func)
% QUARTER_CAR_ODE Quarter Carモデルのシミュレーション
%
% 入力:
%   params     - 車両パラメータ構造体
%   t_span     - [t_start, t_end]
%   x0         - 初期状態 [z_s; z_u; dz_s; dz_u; z_bw]
%   input_func - 路面入力関数 @(t) -> z_road
%
% 出力:
%   t    - 時刻ベクトル
%   x    - 状態履歴 [N x 5]
%   data - 追加データ（力など）

    % デフォルトパラメータ（C++と同じ値）
    if nargin < 1 || isempty(params)
        params = default_params();
    end
    if nargin < 2
        t_span = [0, 2];
    end
    if nargin < 3
        x0 = zeros(5, 1);
    end
    if nargin < 4
        % デフォルト: t=0.2sで20mmステップ入力
        step_time = 0.2;
        step_height = 0.02;  % 20mm
        input_func = @(t) step_height * (t >= step_time);
    end
    
    % ode45オプション（固定ステップでC++と比較しやすく）
    dt = 0.001;  % 1ms（C++と同じ）
    t = t_span(1):dt:t_span(2);
    
    % RK4で積分（C++と同じ方法）
    x = zeros(length(t), 5);
    x(1, :) = x0';
    
    for i = 1:length(t)-1
        ti = t(i);
        xi = x(i, :)';
        z_road = input_func(ti);
        
        k1 = dynamics(ti, xi, params, z_road);
        k2 = dynamics(ti + dt/2, xi + dt/2*k1, params, z_road);
        k3 = dynamics(ti + dt/2, xi + dt/2*k2, params, z_road);
        k4 = dynamics(ti + dt, xi + dt*k3, params, z_road);
        
        x(i+1, :) = (xi + dt/6 * (k1 + 2*k2 + 2*k3 + k4))';
    end
    
    % 力データを計算
    data.F_spring = zeros(size(t));
    data.F_damper = zeros(size(t));
    data.F_tire = zeros(size(t));
    data.z_road = zeros(size(t));
    
    for i = 1:length(t)
        z_road = input_func(t(i));
        data.z_road(i) = z_road * 1000;  % mm
        
        z_s = x(i, 1);
        z_u = x(i, 2);
        dz_s = x(i, 3);
        dz_u = x(i, 4);
        z_bw = x(i, 5);
        
        delta_s = z_s - z_u;
        v_s = dz_s - dz_u;
        
        data.F_spring(i) = params.k_s * delta_s;
        data.F_damper(i) = bouc_wen_force(params, delta_s, v_s, z_bw);
        data.F_tire(i) = max(params.k_t * (z_road - z_u), 0);
    end
end

%% デフォルトパラメータ（C++と完全一致）
function p = default_params()
    % 質量
    p.m_s = 62.5;       % ばね上質量 [kg] (1/4車両)
    p.m_u = 12.0;       % ばね下質量 [kg]
    
    % サスペンション
    p.k_s = 25000;      % ばねレート [N/m]
    
    % タイヤ
    p.k_t = 200000;     % タイヤ剛性 [N/m]
    
    % Bouc-Wen（C++と同じ値）
    p.c0 = 1500;        % 粘性係数
    p.alpha = 5000;     % ヒステリシス強度
    p.A = 1;
    p.beta = 500;
    p.gamma = 100;
    p.n = 2;
end

%% 運動方程式
function dxdt = dynamics(t, x, p, z_road)
    % 状態変数展開
    z_s   = x(1);   % ばね上変位
    z_u   = x(2);   % ばね下変位
    dz_s  = x(3);   % ばね上速度
    dz_u  = x(4);   % ばね下速度
    z_bw  = x(5);   % Bouc-Wen履歴変数
    
    % 相対変位・速度
    delta_s = z_s - z_u;
    v_s = dz_s - dz_u;
    
    % サスペンション力
    F_spring = p.k_s * delta_s;
    F_damper = bouc_wen_force(p, delta_s, v_s, z_bw);
    dz_bw = bouc_wen_dz(p, v_s, z_bw);
    
    % タイヤ力
    F_tire = p.k_t * (z_road - z_u);
    F_tire = max(F_tire, 0);  % タイヤ離陸対応
    
    % 運動方程式
    ddz_s = (-F_spring - F_damper) / p.m_s;
    ddz_u = (F_spring + F_damper + F_tire) / p.m_u;
    
    dxdt = [dz_s; dz_u; ddz_s; ddz_u; dz_bw];
end

%% Bouc-Wen力（C++と同じ実装）
function F = bouc_wen_force(p, x, v, z)
    F = p.c0 * v + p.alpha * z;
end

%% Bouc-Wen履歴変数の微分（C++と同じ実装）
function dz = bouc_wen_dz(p, v, z)
    abs_z_n_minus_1 = abs(z)^(p.n - 1);
    abs_z_n = abs_z_n_minus_1 * abs(z);
    dz = p.A * v - p.beta * abs(v) * z * abs_z_n_minus_1 - p.gamma * v * abs_z_n;
end
