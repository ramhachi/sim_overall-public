function torque = engine_torque_map(rpm)
% engine_torque_map - CBR600RRエンジントルクカーブ
%
% C++実装と完全一致するトルクマップ
% cpp/include/fsae/physics/powertrain/engine_torque_map.hpp と同一
%
% Input:
%   rpm - エンジン回転数 [rpm]
%
% Output:
%   torque - エンジントルク [Nm]

% トルクカーブデータポイント（17点）
rpm_points = [
    3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, ...
    11000, 11500, 12000, 12500, 13000, 13500, 14000, 14500, 15000
];

torque_points = [
    40.0, 45.0, 50.0, 54.0, 57.0, 59.5, 61.0, 62.0, ...
    62.5, 62.7, 62.8, 62.7, 62.5, 61.8, 60.5, 58.0, 55.0
];

% 線形補間
if rpm < rpm_points(1)
    % 下限外挿
    torque = torque_points(1);
elseif rpm > rpm_points(end)
    % 上限外挿
    torque = torque_points(end);
else
    % 線形補間
    torque = interp1(rpm_points, torque_points, rpm, 'linear');
end

end
