% optimize_concept_design.m
% 車両コンセプト（質量・空力・重量配分）のトップダウン最適化

clear;
clc;

% 設定
project_root = fileparts(fileparts(mfilename('fullpath')));  % matlab/ → sim_overall/
runner = VehicleSimRunner(project_root);

% 安定性のためシリアル実行 (Parpool使用時は true に変更)
use_parallel = false; 

% 設計変数の定義
% 1. Total Mass [kg]
% 2. Front Weight Dist [%] (0.0 - 1.0)
% 3. Aero CLA [m^2]
% 4. Engine Torque Scale [-]
% 5. Aero Balance Front Fraction [-]
shared_extra_specs = build_extended_vehicle_tunable_specs();
shared_extra_specs(strcmp({shared_extra_specs.name}, 'EngineTorqueScale')) = [];
extra_bounds = zeros(numel(shared_extra_specs), 2);
extra_names = cell(1, numel(shared_extra_specs));
for i = 1:numel(shared_extra_specs)
    shared_extra_specs(i).index = 5 + i;
    extra_bounds(i, :) = [shared_extra_specs(i).lower, shared_extra_specs(i).upper];
    extra_names{i} = shared_extra_specs(i).name;
end

variable_bounds = [
    200,   300;    % 1. Mass: 200kg - 300kg (Realistic FSAE range)
    0.40,  0.55;   % 2. Front Dist: 40% - 55%
    0.0,   3.5;    % 3. Aero CLA: 0 (No wing) to 3.5 (High downforce)
    0.8,   1.1;    % 4. Engine Scale: 80% to 110% (Constraint: Reliability/Fuel)
    0.30,  0.70;   % 5. Aero Balance Front Fraction
    extra_bounds
];

variable_names = [{'mass_total', 'front_dist', 'cl_a', 'engine_torque_scale', 'aero_balance'}, extra_names];

nvars = size(variable_bounds, 1);
lb = variable_bounds(:, 1);
ub = variable_bounds(:, 2);

% 基準パラメータの読み込み（構造体のテンプレートとして使用）
base_params = create_concept_base_params();

% GA設定
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', 20, ...
    'MaxGenerations', 15, ...
    'UseParallel', use_parallel, ...
    'PlotFcn', {@gaplotbestf, @gaplotstopping});

fprintf('Starting Conceptual Design Optimization...\n');
tic;
cost_func = @(x) concept_cost_function(x, base_params, runner, shared_extra_specs);
[x_opt, fval] = ga(cost_func, nvars, [], [], [], [], lb, ub, [], options);
elapsed_time = toc;

fprintf('\nOptimization Completed in %.2f seconds\n', elapsed_time);
fprintf('Best Lap Time: %.4f s\n', fval);

% 最適解の物理パラメータ変換と表示
final_params = apply_vehicle_tradeoffs(base_params, x_opt, shared_extra_specs);

fprintf('\n=== Optimized Concept ===\n');
fprintf('Mass:            %.1f kg\n', final_params.mass.m_total);
fprintf('Weight Dist:     %.1f %% Front\n', x_opt(2)*100);
fprintf('Downforce (CLA): %.2f m^2\n', final_params.aero.CL_A);
fprintf('Drag (CDA):      %.2f m^2 (L/D = %.2f)\n', final_params.aero.CD_A, final_params.aero.CL_A / final_params.aero.CD_A);
fprintf('Engine Scale:    %.0f %%\n', x_opt(4)*100);
fprintf('Aero Balance:    %.1f %% Front\n', final_params.aero.aero_balance*100);

% JSON保存
final_params.lap_time_prediction = fval;
[final_skid, skid_success] = runner.run(final_params, 'skidpad', 9.125, 20.0);
[final_circle, circle_success] = runner.run(final_params, 'circle', 30.0, 30.0);
skid_event = normalize_event_result(final_skid, skid_success, 'skidpad');
circle_event = normalize_event_result(final_circle, circle_success, 'circle');

stage_entry = struct();
stage_entry.name = 'optimize_concept_design';
stage_entry.optimizer = 'ga';
stage_entry.design_vector = x_opt;
stage_entry.variable_names = variable_names;
stage_entry.objective_value = fval;
stage_entry.skidpad = skid_event;
stage_entry.circle = circle_event;

artifact = build_vehicle_artifact(final_params, 'stage_entry', stage_entry);
artifact = build_vehicle_artifact( ...
    final_params, ...
    'base_artifact', artifact, ...
    'event_name', 'skidpad', ...
    'event_result', skid_event);
artifact = build_vehicle_artifact( ...
    final_params, ...
    'base_artifact', artifact, ...
    'event_name', 'circle', ...
    'event_result', circle_event);

save_vehicle_artifact('optimized_concept.json', artifact);
fprintf('Saved to optimized_concept.json\n');

% ---------------------------------------------------------
function cost = concept_cost_function(x, base_params, runner, shared_extra_specs)
    % 1. 設計変数から物理パラメータへ変換（トレードオフ適用）
    params = apply_vehicle_tradeoffs(base_params, x, shared_extra_specs);
    
    % 2. シミュレーション実行用パラメータの整形
    % VehicleSimRunnerへ渡すparams構造体は、C++側が期待するJson構造と一致させる必要あり
    % apply_vehicle_tradeoffs で生成された params は既にある程度整っている前提
    
    % 3. シミュレーション実行 (Skidpad & Acceleration & Track?)
    % 複合コースでの評価が望ましいが、一旦Skidpadでテスト
    % 複数コースの重み付け和をコストにするのが一般的
    
    track = 'skidpad';
    radius = 9.125;
    
    % 高速トラック（円旋回 R30m）での評価を追加して、空力の影響を見る
    track_hi = 'circle';
    radius_hi = 30.0;
    
    % Run 1: Low speed (Mechanical Grip focus)
    % Skidpad ~60m. 10m/s -> 6s. Allow 20s.
    [res1, s1] = runner.run(params, track, radius, 20.0);
    
    % Run 2: High speed (Aero focus)
    % R30 ~190m. 15m/s -> 12.6s. 10m/s -> 19s. Allow 30s.
    [res2, s2] = runner.run(params, track_hi, radius_hi, 30.0);
    
    if ~s1 || ~s2 || ~res1.lap_completed || ~res2.lap_completed
        % デバッグ出力: 失敗理由の推測
        if ~s1, fprintf('Run 1 (Skidpad) Crash\n'); end
        if ~s2, fprintf('Run 2 (Circle) Crash\n'); end
        if s1 && ~res1.lap_completed, fprintf('Run 1 (Skidpad) DNF (Time: %.2f, Dist: %.1f)\n', res1.lap_time, res1.distance_traveled); end
        if s2 && ~res2.lap_completed, fprintf('Run 2 (Circle) DNF (Time: %.2f, Dist: %.1f)\n', res2.lap_time, res2.distance_traveled); end
        
        cost = 1000.0; % Penalty
        return;
    end
    
    % コスト関数: 単純和（時間は10秒オーダーなのでそのまま足してOK）
    % Skidpad (~5s) + R30 Circle (~6s)
    cost = res1.lap_time + res2.lap_time;
end

function base_params = create_concept_base_params()
base_params = create_vehicle_baseline_template();
end

function params = apply_vehicle_tradeoffs(base_params, x, shared_extra_specs)
params = base_params;
if nargin < 3
    shared_extra_specs = struct([]);
end

mass_total = x(1);
front_dist = x(2);
cl_a = x(3);
engine_scale = x(4);
aero_balance = 0.5;
if length(x) >= 5
    aero_balance = x(5);
end

params.mass.m_total = mass_total;
params.mass.b = params.mass.wheelbase * front_dist;
params.mass.a = params.mass.wheelbase - params.mass.b;
params.mass.m_sprung = params.mass.m_total - sum(params.mass.m_unsprung);

scale_mass = params.mass.m_total / 250.0;
params.mass.Ixx = 32.0 * scale_mass;
params.mass.Iyy = 95.0 * scale_mass;
params.mass.Izz = 85.0 * scale_mass;

params.aero.CL_A = cl_a;
params.aero.CD_A = compute_aero_cd_a(cl_a, 'cd0', 0.30, 'k_induced', 0.10, 'power', 2.0);
params.aero.aero_balance = max(0.20, min(0.80, aero_balance));

params.powertrain.max_torque = 63.0 * engine_scale;
if isfield(params.powertrain, 'engine_torque_curve') && ~isempty(params.powertrain.engine_torque_curve)
    params.powertrain.engine_torque_curve = params.powertrain.engine_torque_curve * engine_scale;
end

if ~isempty(shared_extra_specs)
    params = apply_vehicle_extra_tunables(params, x, shared_extra_specs);
end
end

function event_result = normalize_event_result(raw_result, success, event_name)
if nargin < 1 || isempty(raw_result)
    event_result = struct();
else
    event_result = raw_result;
end

if ~isfield(event_result, 'name') || isempty(event_result.name)
    event_result.name = event_name;
end
event_result.success = logical(success);
if ~isfield(event_result, 'lap_completed')
    event_result.lap_completed = false;
end
end
