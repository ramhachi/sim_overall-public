% optimize_multi_event.m
% スキッドパッドとアクセラレーションの同時最適化（Blend Optimization）
% With Flexible Trade-off System (Discrete Variables)
%
% Strategy:
% - Shared Variables: MassBase, Geometry, [Discrete] AeroPackage, Final Drive
% - Tunable Variables: Suspension, LSD, Aero Setting
% 
% Cost = (T_skid / Ref_Skid) + (T_accel / Ref_Accel)

clear;
clc;

project_root = fileparts(fileparts(mfilename('fullpath')));  % matlab/ → sim_overall/
runner = VehicleSimRunner(project_root);
config = ConceptTradeOffs(); % Load tradeoff config

% === 設定 ===
use_parallel = false; 
max_generations = 30;
pop_size = 100;       

% 基準タイム
ref_skid = 8.49;
ref_accel = 4.12;

% 重み (Accel Only)
w_skid = 0.0;
w_accel = 1.0;

% === 設計変数の定義 ===
% 構造: [Shared(8) | SharedVehiclePackage(N) | Tunable_Skid(15) | Tunable_Accel(15)]

% -- Shared Variables (1-8) --
% 1. Mass Base [kg] (Chassis only, No Aero)
% 2. Front Dist [%]
% 3. CG Height [m]
% 4. Wheelbase [m]
% 5. Track Width F [m]
% 6. Track Width R [m]
% 7. Final Drive Ratio [-]
% 8. [INT] Aero Package ID (1=None, 2=Light, 3=Heavy)
bounds_shared = [
    150.0, 250.0;    % Mass Base (Lighter base, aero adds penalty)
    0.40,  0.60;
    0.25,  0.35;
    1.53,  1.65;
    1.10,  1.40;
    1.10,  1.40;
    3.0,   5.0;
    1,     3;        % Aero Package ID (Integer)
];

shared_names = {'MassBase', 'FrontDist', 'CGHeight', 'Wheelbase', 'TrackF', 'TrackR', 'FinalDrive', 'AeroPackageId'};
shared_extra_specs = build_extended_vehicle_tunable_specs();
bounds_shared_extra = zeros(numel(shared_extra_specs), 2);
shared_extra_names = cell(1, numel(shared_extra_specs));
for i = 1:numel(shared_extra_specs)
    shared_extra_specs(i).index = size(bounds_shared, 1) + i;
    bounds_shared_extra(i, :) = [shared_extra_specs(i).lower, shared_extra_specs(i).upper];
    shared_extra_names{i} = shared_extra_specs(i).name;
end

% -- Tunable Variables Template (15 vars, x2 events) --
% 1-15: Spring, Damp, ARB, Camber, Toe, LSD, AeroFrac, AeroBalance
bounds_tunable = [
    15000, 60000; % Spring F
    15000, 60000; % Spring R
    500,   3000;  % Damper F
    500,   3000;  % Damper R
    0,     1000;  % ARB F
    0,     1000;  % ARB R
    -3.5,  -0.5;  % Camber F
    -3.0,  0.0;   % Camber R
    -1.0,  1.0;   % Toe F
    -0.5,  0.5;   % Toe R
    0,     100;   % LSD Preload
    20,    60;    % LSD Acc
    30,    80;    % LSD Dec
    0.0,   1.0;   % Aero Fraction (0=Min Drag/Downforce, 1=Max)
    0.20,  0.80;  % Aero Balance Front Fraction
];

% 全変数結合
variable_bounds = [
    bounds_shared;      % 1-8
    bounds_shared_extra;
    bounds_tunable;     % 9-23 (Skidpad)
    bounds_tunable;     % 24-38 (Accel)
];

variable_names = [ ...
    shared_names, ...
    shared_extra_names, ...
    strcat('Skid_', {'SpringF', 'SpringR', 'DamperF', 'DamperR', 'ARBF', 'ARBR', 'CamberF', 'CamberR', 'ToeF', 'ToeR', 'LSDPreload', 'LSDRampAcc', 'LSDRampDec', 'AeroFraction', 'AeroBalance'}), ...
    strcat('Accel_', {'SpringF', 'SpringR', 'DamperF', 'DamperR', 'ARBF', 'ARBR', 'CamberF', 'CamberR', 'ToeF', 'ToeR', 'LSDPreload', 'LSDRampAcc', 'LSDRampDec', 'AeroFraction', 'AeroBalance'}) ...
];

nvars = size(variable_bounds, 1);
lb = variable_bounds(:, 1);
ub = variable_bounds(:, 2);

% Integer Constraints (Index of integer variables)
% Variable 8 (Aero Package ID) is integer
int_con = 8;

% ベースパラメータ
base_params = create_vehicle_baseline_template();

% GA設定
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'PopulationSize', pop_size, ...
    'MaxGenerations', max_generations, ...
    'UseParallel', use_parallel, ...
    'PlotFcn', {@gaplotbestf, @gaplotstopping});

fprintf('Starting Multi-Event Optimization (With Discrete Trade-offs)...\n');
fprintf('Variables: %d (IntCon: AeroID)\n', nvars);
tic;

design_context = struct('shared_extra_specs', shared_extra_specs);
cost_func = @(x) blend_cost(x, base_params, runner, config, ref_skid, ref_accel, w_skid, w_accel, design_context);

% ga calls with IntCon
[x_opt, fval] = ga(cost_func, nvars, [], [], [], [], lb, ub, [], int_con, options);
elapsed_time = toc;

fprintf('\nOptimization Completed in %.2f seconds\n', elapsed_time);
fprintf('Best Combined Cost: %.4f\n', fval);

% 結果解析
[~, res_skid, res_accel, params_skid, params_accel] = blend_cost(x_opt, base_params, runner, config, ref_skid, ref_accel, w_skid, w_accel, design_context);

% 選択された構成を表示
aero_id = round(x_opt(8));
aero_info = config.aero_packages(aero_id);
fprintf('Selected Config:\n');
fprintf('  Aero Package: %s (Mass +%.1f kg, Max CL %.1f)\n', aero_info.name, aero_info.mass_penalty, aero_info.max_cl_a);
fprintf('  Mass Total:   %.1f kg\n', params_skid.mass.m_total);
fprintf('  Track Width:  %.2f m\n', mean([params_skid.mass.track_f, params_skid.mass.track_r]));

if isfield(res_skid, 'simulated') && ~res_skid.simulated && isfield(res_skid, 'reference_lap_time')
    fprintf('Skidpad: not simulated (reference %.4f s)\n', res_skid.reference_lap_time);
else
    fprintf('Skidpad: %.4f s\n', res_skid.lap_time);
end
fprintf('Accel:   %.4f s\n', res_accel.lap_time);

% 結果構造体
stage_entry = struct();
stage_entry.name = 'optimize_multi_event';
stage_entry.optimizer = 'ga';
stage_entry.objective_value = fval;
stage_entry.config_name = aero_info.name;
stage_entry.design_vector = x_opt;
stage_entry.variable_names = variable_names;
stage_entry.primary_params_event = 'acceleration';
stage_entry.skidpad = struct('result', res_skid, 'params', params_skid);
stage_entry.acceleration = struct('result', res_accel, 'params', params_accel);

artifact = build_vehicle_artifact( ...
    params_accel, ...
    'stage_entry', stage_entry);
if isfield(res_skid, 'simulated') && res_skid.simulated
    artifact = build_vehicle_artifact( ...
        params_accel, ...
        'base_artifact', artifact, ...
        'event_name', 'skidpad', ...
        'event_result', res_skid);
end
if isfield(res_accel, 'simulated') && res_accel.simulated
    artifact = build_vehicle_artifact( ...
        params_accel, ...
        'base_artifact', artifact, ...
        'event_name', 'acceleration', ...
        'event_result', res_accel);
end
artifact.primary_params_event = 'acceleration';
artifact.event_params = struct('skidpad', params_skid, 'acceleration', params_accel);
save_vehicle_artifact('optimized_accel_new_engine.json', artifact);
fprintf('Saved result to optimized_accel_new_engine.json\n');


% ---------------------------------------------------------
function [cost, res_skid, res_accel, params_skid, params_accel] = blend_cost(x, base_params, runner, config, ref_skid, ref_accel, w_skid, w_accel, design_context)
    
    % --- 変数展開 ---
    % Shared (1-8 + shared extra tunables)
    n_shared = 8;
    if nargin >= 9 && isfield(design_context, 'shared_extra_specs') && ~isempty(design_context.shared_extra_specs)
        n_shared = n_shared + numel(design_context.shared_extra_specs);
    end
    x_shared = x(1:n_shared);
    n_tunable = (length(x) - n_shared) / 2;

    % Tunable
    x_skid_tunable = x(n_shared + 1:n_shared + n_tunable);
    x_accel_tunable = x(n_shared + n_tunable + 1:end);
    
    % --- Build Vectors for apply_tradeoff_design (23 vars) ---
    % apply_tradeoff_design expects: [Shared(8) | Tunable(15)]
    
    vec_skid = [x_shared, x_skid_tunable];
    vec_accel = [x_shared, x_accel_tunable];
    
    % --- Apply Design ---
    params_skid = apply_tradeoff_design(base_params, vec_skid, config, design_context);
    params_accel = apply_tradeoff_design(base_params, vec_accel, config, design_context);
    
    % --- Simulation ---
    % 1. Skidpad (Cost = 0 if w_skid = 0, but calculate if needed?)
    % If w_skid is 0, we can skip skidpad simulation to save time!
    if w_skid > 0
        [res_skid, success_skid] = runner.run(params_skid, 'skidpad', 9.125, 20.0);
        if ~success_skid || ~res_skid.lap_completed
            cost = 1000.0;
            res_skid = make_failure_result('skidpad', 1000.0, 'skidpad simulation failed');
            res_accel = make_reference_result('acceleration', 1000.0, 'acceleration run was skipped after skidpad failure');
            return;
        end
        res_skid.name = 'skidpad';
        res_skid.simulated = true;
        res_skid.success = true;
    else
        res_skid = make_reference_result('skidpad', ref_skid, 'skidpad objective weight is zero');
    end
    
    % 2. Acceleration
    % Acceleration 75m, run for 10s max (or enough to finish)
    [res_accel, success_accel] = runner.run(params_accel, 'acceleration', 75.0, 10.0);
    if ~success_accel
        cost = 1000.0;
        res_accel = normalize_failure_result(res_accel, 'acceleration', 1000.0, 'acceleration simulation failed');
        return;
    end
    if ~res_accel.lap_completed
        cost = 1000.0;
        res_accel = normalize_failure_result(res_accel, 'acceleration', 1000.0, 'acceleration lap not completed');
        return;
    end
    res_accel.name = 'acceleration';
    res_accel.simulated = true;
    res_accel.success = true;
    
    % --- Cost ---
    skid_cost = 0.0;
    if isfield(res_skid, 'simulated') && res_skid.simulated
        skid_cost = w_skid * (res_skid.lap_time / ref_skid);
    end
    cost = skid_cost + w_accel * (res_accel.lap_time / ref_accel);

end

function result = make_failure_result(event_name, lap_time, reason)
result = struct( ...
    'name', event_name, ...
    'lap_time', lap_time, ...
    'simulated', true, ...
    'success', false, ...
    'lap_completed', false, ...
    'reason', reason);
end

function result = make_reference_result(event_name, lap_time, reason)
result = struct( ...
    'name', event_name, ...
    'lap_time', lap_time, ...
    'reference_lap_time', lap_time, ...
    'simulated', false, ...
    'success', false, ...
    'lap_completed', false, ...
    'reason', reason);
end

function result = normalize_failure_result(raw_result, event_name, default_lap_time, reason)
if nargin < 1 || isempty(raw_result)
    result = struct();
else
    result = raw_result;
end

result.name = event_name;
if ~isfield(result, 'lap_time') || isempty(result.lap_time) || result.lap_time <= 0
    result.lap_time = default_lap_time;
end
result.simulated = true;
result.success = false;
result.lap_completed = false;
if ~isfield(result, 'reason') || isempty(result.reason)
    result.reason = reason;
end
end
