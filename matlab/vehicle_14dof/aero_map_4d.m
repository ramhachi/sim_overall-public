function result = aero_map_4d(velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, params)
% aero_map_4d - 4次元エアロダイナミクスマップ
%
% C++実装と完全一致する4Dエアロマップ
% cpp/include/fsae/physics/aero/aero_map_4d.hpp と同一
%
% Inputs:
%   velocity    - 車速 [m/s]
%   rh_front_mm - フロントライドハイト [mm]
%   rh_rear_mm  - リアライドハイト [mm]
%   roll_rad    - ロール角 [rad]
%   yaw_rad     - ヨー角（車体スリップ角） [rad]
%   params      - エアロパラメータ構造体（オプション）
%
% Output:
%   result - 構造体
%            .Fx        - ドラッグ [N] (負=後方)
%            .Fy        - サイドフォース [N]
%            .Fz        - リフト [N] (負=ダウンフォース)
%            .Mx        - ロールモーメント [Nm]
%            .My        - ピッチモーメント [Nm]
%            .Mz        - ヨーモーメント [Nm]
%            .Fz_front  - 前輪ダウンフォース [N]
%            .Fz_rear   - 後輪ダウンフォース [N]

% デフォルトパラメータ
if nargin < 6
    params = struct();
end
if ~isfield(params, 'rho')
    params.rho = 1.225;  % kg/m³
end
if ~isfield(params, 'A_ref')
    params.A_ref = 1.0;  % m²
end
if ~isfield(params, 'L_ref')
    params.L_ref = 1.6;  % m
end
if ~isfield(params, 'x_cg')
    params.x_cg = 0.8;  % m
end
if ~isfield(params, 'CL_default')
    params.CL_default = -2.5;
end
if ~isfield(params, 'CD_default')
    params.CD_default = 1.0;
end
if ~isfield(params, 'CoP_default')
    params.CoP_default = 0.45;  % m
end

% グリッド定義（C++と同一）
grid.rh_f = [20, 30, 40, 50, 60];  % mm
grid.rh_r = [30, 40, 50, 60, 70, 80];  % mm
grid.roll = [-5, -2.5, 0, 2.5, 5] * pi/180;  % rad
grid.yaw = [-15, -10, -5, 0, 5, 10, 15] * pi/180;  % rad

% 4D線形補間用の係数計算
coef = compute_coefficients(rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, ...
                            grid, params);

% 動圧
q = 0.5 * params.rho * velocity^2;

% 力計算
result.Fx = -q * params.A_ref * coef.CD;  % ドラッグ（負=後方）
result.Fy = q * params.A_ref * coef.CY;   % サイドフォース
result.Fz = q * params.A_ref * coef.CL;   % リフト（負=ダウンフォース）

% モーメント計算
lever_arm = coef.CoP_x - params.x_cg;
result.My = result.Fz * lever_arm;  % ピッチモーメント

% ヨー角によるモーメント
if abs(yaw_rad) > 0.01
    result.Mz = result.Fy * lever_arm;
else
    result.Mz = 0;
end

% ロールモーメント（簡易）
result.Mx = 0;

% 前後荷重配分
result.Fz_front = result.Fz * coef.CoP_dist;
result.Fz_rear = result.Fz * (1.0 - coef.CoP_dist);

end

function coef = compute_coefficients(rh_f, rh_r, roll, yaw, grid, params)
% 4次元補間により係数を計算

% ベース係数
CL_base = params.CL_default;  % -2.5
CD_base = params.CD_default;  % 1.0
CoP_base = params.CoP_default;  % 0.45

% ライドハイト効果（地面効果）
% 低いほどダウンフォース増加
rh_effect = 1.0 + 0.015 * (40 - rh_f) + 0.010 * (55 - rh_r);
rh_effect = max(0.6, min(1.4, rh_effect));

% ロール効果（フロアシール損失）
roll_loss = 1.0 - 3.0 * abs(roll);
roll_loss = max(0.7, min(1.0, roll_loss));

% ヨー効果
yaw_effect = cos(yaw * 2);
yaw_effect = max(0.6, min(1.0, yaw_effect));
CY_val = sin(yaw * 2) * 0.5;

% 最終係数
coef.CL = CL_base * rh_effect * roll_loss * yaw_effect;
coef.CD = CD_base * (1.0 + 2.0 * yaw^2);
coef.CY = CY_val;

% 圧力中心
CoP = CoP_base;
CoP = CoP + (rh_f - 40) * 0.003;  % フロント下がるとCoP前進
CoP = CoP - (rh_r - 55) * 0.002;  % リア下がるとCoP後退
coef.CoP_x = CoP;

% 前後配分
dist = 0.5 + (rh_f - rh_r + 15) * 0.003;
coef.CoP_dist = max(0.35, min(0.65, dist));

end
