% verify_skidpad_time.m
% 最適化されたコンセプトパラメータでSkidpadのラップタイムを検証する

clear;
clc;

project_root = fileparts(fileparts(mfilename('fullpath')));  % matlab/ → sim_overall/
runner = VehicleSimRunner(project_root);

% ベースパラメータ
base_params = struct();
base_params.mass = struct('m_unsprung', [12, 12, 14, 14], 'wheelbase', 1.55);
base_params.aero = struct();
base_params.powertrain = struct();
base_params.suspension = struct();

% 最適化された全設計変数 (from optimized_full_vehicle.json)
design_vector = [
    200, 0.6, 0.25, 1.53, 1.4, 1.4, ... % Track Width forced to 1.4
    15000, 15000, 500, 500, 0, 3.03, ...
    -0.5, -0.06, 0.71, -0.20, 0, ...
    1.5, 20, 30, 3
];

% 物理パラメータへ変換
params = apply_full_design(base_params, design_vector);

fprintf('=== Simulating Optimized Concept on Skidpad ===\n');
fprintf('Mass: %.1f kg\n', params.mass.m_total);
fprintf('Aero CLA: %.2f\n', params.aero.CL_A);
fprintf('Distribution: %.1f %% Front\n', design_vector(2)*100);
fprintf('Tire Mu Y: %.4f\n', params.tire.mu_y_max);

% Skidpad実行
% 半径 9.125m
[res, success] = runner.run(params, 'skidpad', 9.125, 20.0);

if success && res.lap_completed
    fprintf('\nSUCCESS\n');
    fprintf('Lap Time: %.4f s\n', res.lap_time);
    dist = 4 * pi * 9.125;
    fprintf('Avg Speed: %.2f km/h\n', (dist / res.lap_time) * 3.6);
    fprintf('Max Lat Error: %.4f m\n', res.max_lateral_error);
else
    fprintf('\nFAILED\n');
    if isempty(res)
        fprintf('No result returned.\n');
    else
        fprintf('Time: %.2f (Not completed)\n', res.lap_time);
    end
end
