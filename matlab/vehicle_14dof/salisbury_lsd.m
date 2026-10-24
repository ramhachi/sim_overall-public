function [T_L, T_R, omega_out] = salisbury_lsd(omega_L, omega_R, T_in, params)
% salisbury_lsd - Salisbury式リミテッドスリップデファレンシャル
%
% C++実装と完全一致するLSDモデル
% cpp/include/fsae/physics/powertrain/salisbury_lsd.hpp と同一
%
% Inputs:
%   omega_L - 左車輪角速度 [rad/s]
%   omega_R - 右車輪角速度 [rad/s]
%   T_in    - 入力トルク [Nm]
%   params  - LSDパラメータ構造体
%             .ramp_accel  - 加速側ランプ角 [deg]
%             .ramp_decel  - 減速側ランプ角 [deg]
%             .preload     - プリロードトルク [Nm]
%             .efficiency  - 効率 [0-1]
%
% Outputs:
%   T_L       - 左車輪出力トルク [Nm]
%   T_R       - 右車輪出力トルク [Nm]
%   omega_out - デフ出力軸角速度 [rad/s]

% デフォルトパラメータ
if nargin < 4
    params = struct();
end
if ~isfield(params, 'ramp_accel')
    params.ramp_accel = 35.0;  % deg
end
if ~isfield(params, 'ramp_decel')
    params.ramp_decel = 50.0;  % deg
end
if ~isfield(params, 'preload')
    params.preload = 40.0;  % Nm
end
if ~isfield(params, 'efficiency')
    params.efficiency = 0.95;
end

% 角度をラジアンに変換
ramp_accel_rad = params.ramp_accel * pi / 180;
ramp_decel_rad = params.ramp_decel * pi / 180;

% デフ出力軸角速度（左右平均）
omega_out = 0.5 * (omega_L + omega_R);

% 速度差
delta_omega = omega_R - omega_L;

% 加速 or 減速判定
if T_in >= 0
    % 加速時
    ramp_angle = ramp_accel_rad;
else
    % 減速時
    ramp_angle = ramp_decel_rad;
end

% ランプ係数（tan(ramp_angle)）
K_ramp = tan(ramp_angle);

% LSDによるトルク制限
T_lsd = params.preload + abs(T_in) * K_ramp;

% トルク配分比率計算
if abs(delta_omega) < 0.01
    % ほぼ同速度 → 均等配分
    torque_bias = 0.5;
else
    % 速度差に応じた配分
    % 遅い方の車輪により多くのトルクを配分
    if delta_omega > 0
        % 右が速い → 左に多く配分
        torque_bias = 0.5 + min(0.3, T_lsd / (2 * abs(T_in) + 1e-6));
    else
        % 左が速い → 右に多く配分
        torque_bias = 0.5 - min(0.3, T_lsd / (2 * abs(T_in) + 1e-6));
    end
    torque_bias = max(0.2, min(0.8, torque_bias));
end

% 効率考慮
T_out = T_in * params.efficiency;

% 左右トルク配分
T_L = T_out * torque_bias;
T_R = T_out * (1 - torque_bias);

end
