function T_surf_new = tire_thermal_update(T_surf, Fx, Fy, vsx, vsy, v_wind, T_amb, dt)
% TIRE_THERMAL_UPDATE タイヤ表面温度の動的更新
%
% Inputs:
%   T_surf  - 現在のタイヤ表面温度 [°C]
%   Fx      - 前後力 [N]
%   Fy      - 横力 [N]
%   vsx     - タイヤ座標系での縦滑り速度 [m/s]
%   vsy     - タイヤ座標系での横滑り速度 [m/s]
%   v_wind  - 風速（車速で近似可）[m/s]
%   T_amb   - 外気温 [°C]
%   dt      - タイムステップ [s]
%
% Outputs:
%   T_surf_new - 更新後のタイヤ表面温度 [°C]
%
% 物理モデル:
%   熱収支: m_tread * c_p * dT/dt = Q_gen - Q_conv - Q_cond
%   発熱: Q_gen = |Fx * vsx| + |Fy * vsy| (摩擦仕事)
%   対流冷却: Q_conv = h_conv * A * (T_surf - T_amb) * (1 + 0.1*v_wind)

% 物理パラメータ（定数）
m_tread = 2.0;          % トレッド質量 [kg]
c_p = 1000.0;           % 比熱 [J/(kg·K)]
h_conv = 50.0;          % 熱伝達係数 [W/(m²·K)]
A_tread = 0.05;         % トレッド接地面積 [m²]

% 発熱量（摩擦仕事率）
Q_gen = abs(Fx * vsx) + abs(Fy * vsy);  % [W]

% 対流冷却（風速依存）
Q_conv = h_conv * A_tread * (T_surf - T_amb) * (1 + 0.1 * abs(v_wind));  % [W]

% 温度変化率
dT_dt = (Q_gen - Q_conv) / (m_tread * c_p);  % [K/s = °C/s]

% オイラー法で更新
T_surf_new = T_surf + dT_dt * dt;

% 物理的な温度範囲に制限
T_surf_new = max(min(T_surf_new, 120.0), 0.0);  % 0-120°C
end
