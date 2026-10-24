function run_full_pipeline(track_type, radius, varargin)
% run_full_pipeline - PID最適化→シミュレーション→可視化を自動実行
%
% 使い方:
%   run_full_pipeline('circle', 30)                    % 最適化スキップ、デフォルトゲインで実行
%   run_full_pipeline('circle', 30, 'optimize', true)  % 最適化を実行
%   run_full_pipeline('circle', 30, 'optimize', true, 'method', 'grid')  % グリッドサーチ
%
% パラメータ:
%   track_type - トラックタイプ ('circle', 'figure_eight', 'skidpad', 'acceleration', 'straight_corner')
%   radius     - トラック半径 [m] (accelerationの場合は距離 [m])
%
% オプション (Name-Value):
%   'optimize'     - 最適化を実行するか (default: false)
%   'method'       - 最適化手法 ('bayesian' or 'grid', default: 'bayesian')
%   'n_iter'       - ベイズ最適化の反復回数 (default: 60)
%   'sim_time'     - シミュレーション時間 [s] (default: 180, accelerationは10)
%   'initial_speed'- 初速 [m/s] (default: 15.0, accelerationは0, autocrossは6.0)
%   'save_report'  - レポート保存 (default: true)
%
% 例:
%   % 円形トラック、デフォルトゲインで実行
%   run_full_pipeline('circle', 30)
%
%   % 円形トラック、PID最適化（30回）→シミュレーション→可視化
%   run_full_pipeline('circle', 30, 'optimize', true, 'n_iter', 60)
%
%   % 8の字トラック、グリッドサーチ→シミュレーション→可視化
%   run_full_pipeline('figure_eight', 25, 'optimize', true, 'method', 'grid')
%
%   % スキッドパッド (SAE公式半径9.125m)
%   run_full_pipeline('skidpad', 9.125)
%
%   % 加速イベント (75m直線)
%   run_full_pipeline('acceleration', 75)
%
%   % Autocross (CSVファイルベース、半径0指定)
%   run_full_pipeline('ASE_autocross', 0, 'optimize', true, 'n_iter', 60)
%
%   % POU Hardpoint最適化 + PID最適化 + シミュレーション
%   run_full_pipeline('circle', 30, 'pou_optimize', true, 'optimize', true)

%% パラメータ解析
defaults = struct('n_iter', 60, 'sim_time', 180.0, 'initial_speed', 15.0);
p = inputParser;
addRequired(p, 'track_type', @ischar);
addRequired(p, 'radius', @isnumeric);
addParameter(p, 'optimize', false, @islogical);
addParameter(p, 'method', 'bayesian', @(x) ismember(x, {'bayesian', 'grid'}));
addParameter(p, 'n_iter', defaults.n_iter, @isnumeric);
addParameter(p, 'sim_time', defaults.sim_time, @isnumeric);
addParameter(p, 'initial_speed', defaults.initial_speed, @isnumeric);
addParameter(p, 'save_report', true, @islogical);
addParameter(p, 'drs_enabled', false, @islogical);
addParameter(p, 'visualize', true, @islogical);
addParameter(p, 'pou_optimize', false, @islogical);
addParameter(p, 'pou_wheels', [0, 1], @isnumeric);
addParameter(p, 'allow_pou_failure', false, @islogical);
addParameter(p, 'custom_params_file', '', @ischar);
addParameter(p, 'driver_mode', 'pid', @(x) ischar(x) || isstring(x));
addParameter(p, 'rl_actor_dir', '', @(x) ischar(x) || isstring(x));
addParameter(p, 'lpv_model', 'steering6', @(x) ischar(x) || isstring(x));
addParameter(p, 'tire_model_family', 'pinn', @(x) ischar(x) || isstring(x));
addParameter(p, 'tire_model_name', '', @(x) ischar(x) || isstring(x));
% fast_mode: opt-in flag for large search campaigns.
% When true, forces PID driver only. Tire runtime remains deployable PINN.
addParameter(p, 'fast_mode', false, @islogical);
parse(p, track_type, radius, varargin{:});

opt = p.Results;
opt.driver_mode = lower(char(opt.driver_mode));
opt.rl_actor_dir = char(opt.rl_actor_dir);
opt.lpv_model = lower(char(opt.lpv_model));
opt.tire_model_family = lower(char(opt.tire_model_family));
opt.tire_model_name = char(opt.tire_model_name);
if ~ismember(opt.driver_mode, {'pid', 'rl', 'lpv_lqr'})
    error('driver_mode must be ''pid'', ''rl'', or ''lpv_lqr''.');
end
if strcmp(opt.driver_mode, 'lpv_lqr') && ~strcmp(opt.lpv_model, 'steering6')
    error('The maintained LPV driver contract requires lpv_model=''steering6''.');
end

% fast_mode override: PID only. Tire fallback is disabled for production.
if opt.fast_mode
    opt.driver_mode = 'pid';
    fprintf('[fast_mode] Override: driver=pid, tire remains PINN\n');
end

function base = create_default_vehicle_params()
base = create_vehicle_baseline_template();
end

function base = normalize_vehicle_param_structure(base)
if isfield(base, 'K_chassis_Nm_deg') && ~isfield(base, 'suspension')
    base.suspension = struct();
end
if isfield(base, 'K_chassis_Nm_deg')
    base.suspension.K_chassis = base.K_chassis_Nm_deg;
end
if isfield(base, 'I_twist_kgm2')
    base.suspension.I_twist = base.I_twist_kgm2;
end

if isfield(base, 'steering_torque_mode') && ~isfield(base, 'steering')
    base.steering = struct();
end
if isfield(base, 'steering_torque_mode')
    base.steering.torque_mode = base.steering_torque_mode;
end
if isfield(base, 'steering_I_steer')
    base.steering.I_steer = base.steering_I_steer;
end
if isfield(base, 'steering_R_steer')
    base.steering.R_steer = base.steering_R_steer;
end

if ~isfield(base, 'mass')
    base.mass = struct();
end
if ~isfield(base.mass, 'm_total'), base.mass.m_total = 250; end
if ~isfield(base.mass, 'm_sprung'), base.mass.m_sprung = 194; end

if ~isfield(base, 'aero')
    base.aero = struct();
end
if ~isfield(base.aero, 'CL_A'), base.aero.CL_A = 2.5; end
if ~isfield(base.aero, 'CD_A'), base.aero.CD_A = 0.75; end

if ~isfield(base, 'tire')
    base.tire = struct();
end
if ~isfield(base.tire, 'ambient_temperature_c')
    base.tire.ambient_temperature_c = 25.0;
end
end

function base = apply_tire_model_selection(base, opt, project_root)
[base, ~, ~] = apply_named_tire_model_selection( ...
    base, opt.tire_model_family, opt.tire_model_name, project_root);
end
if ~strcmp(opt.tire_model_family, 'pinn')
    error('tire_model_family must be ''pinn''; TMeasy/Magic Formula runtime fallback is disabled.');
end
if isempty(opt.tire_model_name)
    error('tire_model_name is required for PINN runtime to avoid rim/compound ambiguity.');
end
if ~strcmp(opt.driver_mode, 'pid') && opt.optimize
    fprintf('Driver mode=%s のため PID最適化はスキップします。\n', opt.driver_mode);
    opt.optimize = false;
end

% Accelerationトラック用の自動パラメータ調整
if strcmp(track_type, 'acceleration')
    % accelerationは短時間、初速ゼロが適切
    if ~any(strcmp(varargin, 'sim_time'))
        opt.sim_time = 10.0;  % デフォルト10秒
    end
    if ~any(strcmp(varargin, 'initial_speed'))
        opt.initial_speed = 0.0;  % デフォルト停止状態
    end
end

% Autocross用の自動パラメータ調整
if strcmp(track_type, 'ASE_autocross')
    if ~any(strcmp(varargin, 'initial_speed'))
        opt.initial_speed = 6.0;  % Autocrossはタイトコーナーが多いため低初速が適切
    end
    if ~any(strcmp(varargin, 'sim_time'))
        opt.sim_time = 180.0;  % コース長が長いため時間確保
    end
end

%% ヘッダー表示
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║          FSAE Lap Time Simulation - Full Pipeline            ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');
fprintf('Track Type:      %s\n', track_type);
if strcmp(track_type, 'acceleration')
    fprintf('Length:          %.1f m\n', radius);
else
    fprintf('Radius:          %.1f m\n', radius);
end
fprintf('Optimize:        %s\n', mat2str(opt.optimize));
fprintf('DRS:             %s\n', mat2str(opt.drs_enabled));
fprintf('Driver Mode:     %s\n', opt.driver_mode);
fprintf('Tire Model:      %s\n', opt.tire_model_family);
if ~isempty(opt.tire_model_name)
    fprintf('Tire Dataset:    %s\n', opt.tire_model_name);
end
fprintf('Fast Mode:       %s\n', mat2str(opt.fast_mode));
fprintf('POU Optimize:    %s\n', mat2str(opt.pou_optimize));
fprintf('Allow POU Fail:  %s\n', mat2str(opt.allow_pou_failure));
if opt.optimize
    fprintf('Method:          %s\n', opt.method);
    if strcmp(opt.method, 'bayesian')
        fprintf('Iterations:      %d\n', opt.n_iter);
    end
end
fprintf('Sim Time:        %.3f s\n', opt.sim_time);
fprintf('Initial Speed:   %.3f m/s\n', opt.initial_speed);
fprintf('\n');

%% プロジェクトルートの設定
project_root = fileparts(pwd);
if ~endsWith(pwd, 'matlab')
    warning('MATLABディレクトリから実行してください。現在のディレクトリ: %s', pwd);
    return;
end

%% Docker PATHの設定（バッチモード対応）
current_path = getenv('PATH');
if ismac
    docker_dir = '/opt/homebrew/bin';
    if ~contains(current_path, docker_dir)
        setenv('PATH', [docker_dir, ':', current_path]);
    end
elseif ispc
    docker_dir = 'C:\Program Files\Docker\Docker\resources\bin';
    if ~contains(current_path, docker_dir)
        setenv('PATH', [docker_dir, ';', current_path]);
    end
end

%% Phase 0: DRSパラメータファイル事前生成（最適化・シミュレーション共通で使用）
params_file_arg = '';

% ベースとなるパラメータファイルの決定
if strcmp(track_type, 'ASE_autocross')
    opt_file = 'optimized_autocross.json';
else
    opt_file = sprintf('optimized_%s.json', track_type);
end

legacy_opt_file = fullfile(project_root, 'matlab', opt_file);
preferred_base_artifact = opt.custom_params_file;
if isempty(preferred_base_artifact)
    latest_ga_artifact = fullfile(project_root, 'output', 'vehicle_ga_runs', ...
        'latest_best_vehicle_artifact.json');
    if strcmp(track_type, 'ASE_autocross') && isfile(latest_ga_artifact)
        preferred_base_artifact = latest_ga_artifact;
    elseif isfile(legacy_opt_file)
        preferred_base_artifact = legacy_opt_file;
    end
end

% Only delegate to the SSOT helper when we have an explicit artifact to resolve.
% When preferred_base_artifact is empty (no event-specific optimized_<track>.json and
% no custom_params_file), skip the helper to prevent resolve_default_vehicle_json from
% silently returning a globally highest-priority artifact unrelated to this event.
if ~isempty(preferred_base_artifact)
    artifact_resolution = resolve_authoritative_vehicle_json(project_root, ...
        'ExplicitPath', preferred_base_artifact, ...
        'RequireExportReady', true, ...
        'LegacyFallbackPath', legacy_opt_file);
else
    artifact_resolution = struct( ...
        'path',     '', ...
        'exists',   false, ...
        'source',   'default_template', ...
        'artifact', struct(), ...
        'params',   create_vehicle_baseline_template());
end
opt_file = artifact_resolution.path;
base_artifact = artifact_resolution.artifact;
base = artifact_resolution.params;

if artifact_resolution.exists
    fprintf('ベースパラメータ: %s [%s]\n', opt_file, artifact_resolution.source);
else
    fprintf('ベースパラメータ: 内蔵デフォルト\n');
end

base = normalize_vehicle_param_structure(base);
base = apply_tire_model_selection(base, opt, project_root);

runtime_output_dir = fullfile(project_root, 'output', 'runtime');
laptime_output_dir = fullfile(project_root, 'output', 'laptime');
report_output_dir = fullfile(project_root, 'output', 'reports');
if ~exist(runtime_output_dir, 'dir'), mkdir(runtime_output_dir); end
if ~exist(laptime_output_dir, 'dir'), mkdir(laptime_output_dir); end
if ~exist(report_output_dir, 'dir'), mkdir(report_output_dir); end

needs_generated_params = opt.drs_enabled || artifact_resolution.exists || ...
    ~strcmp(opt.tire_model_family, 'tmeasy');

if opt.drs_enabled
    switch track_type
        case 'skidpad',      drs_fraction = 0.05;
        case 'acceleration', drs_fraction = 0.65;
        otherwise,           drs_fraction = 0.25;
    end

    drs_params = base;
    drs_params.aero.CL_A = drs_params.aero.CL_A * (1 - drs_fraction * 0.46);
    drs_params.aero.CD_A = drs_params.aero.CD_A * (1 - drs_fraction * 0.26);
    drs_params.mass.m_total  = drs_params.mass.m_total  + 2.5;
    drs_params.mass.m_sprung = drs_params.mass.m_sprung + 2.5;
    drs_params.drs = struct('enabled', true);

    drs_params_path = fullfile(runtime_output_dir, 'params_drs_run.json');
    drs_stage = struct( ...
        'name', 'runtime_params_drs_run', ...
        'optimizer', 'run_full_pipeline', ...
        'result', struct('track_type', track_type, 'drs_enabled', true));
    drs_artifact = build_vehicle_artifact(drs_params, ...
        'base_artifact', base_artifact, ...
        'stage_entry', drs_stage);
    save_vehicle_artifact(drs_params_path, drs_artifact);

    params_file_arg = '--params_file /workspace/output/runtime/params_drs_run.json';
    fprintf('DRS: 空力補正 CL×%.3f, CD×%.3f, 重量+2.5kg\n', ...
        1 - drs_fraction*0.46, 1 - drs_fraction*0.26);
    fprintf('DRS: パラメータファイル生成完了 (最適化・シミュレーション共通)\n\n');
elseif needs_generated_params
    params_run_path = fullfile(runtime_output_dir, 'params_run.json');
    run_stage = struct( ...
        'name', 'runtime_params_run', ...
        'optimizer', 'run_full_pipeline', ...
        'result', struct('track_type', track_type, 'drs_enabled', false));
    run_artifact = build_vehicle_artifact(base, ...
        'base_artifact', base_artifact, ...
        'stage_entry', run_stage);
    save_vehicle_artifact(params_run_path, run_artifact);

    params_file_arg = '--params_file /workspace/output/runtime/params_run.json';
    fprintf('パラメータファイル設定完了: %s\n\n', params_run_path);
end

%% Phase 0.5: POU Hardpoint最適化（オプション）
if opt.pou_optimize
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║ Phase 0.5: POU Hardpoint Optimization                        ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    % POU最適化ツールのパスを追加
    pou_dir = fullfile(pwd, 'pou_hardpoint_design');
    addpath(pou_dir);
    addpath(fullfile(pou_dir, 'geometry'));
    addpath(fullfile(pou_dir, 'optimization'));
    addpath(fullfile(pou_dir, 'visualization'));
    addpath(fullfile(pou_dir, 'export'));

    % パラメータファイルの決定
    pou_params_file = opt_file;

    if ~exist(pou_params_file, 'file')
        msg = sprintf('POU: パラメータファイル %s が見つかりません。', pou_params_file);
        if opt.allow_pou_failure
            warning('%s スキップします。', msg);
        else
            error('Pipeline:POUParamsFileMissing', '%s', msg);
        end
    else
        tic;
        pou_output_dir = fullfile(pwd, 'pou_output');

        pou_opts = struct();
        pou_opts.wheels = opt.pou_wheels;
        pou_opts.n_multistart = 5;
        pou_opts.output_dir = pou_output_dir;
        pou_opts.export_csv = true;
        pou_opts.export_json = false;
        pou_opts.export_report = true;
        pou_opts.update_params = true;  % 車両パラメータJSONにハードポイント情報を追記
        pou_opts.verbose = true;

        try
            pou_results = run_all_wheels(pou_params_file, pou_opts);
            pou_time = toc;
            fprintf('\nPOU最適化完了 (%.1f s)\n', pou_time);
            fprintf('出力: %s/\n\n', pou_output_dir);
        catch ME
            pou_time = toc;
            warning('POU最適化中にエラー: %s', ME.message);
            if ~opt.allow_pou_failure
                fprintf('POU最適化失敗 (%.1f s)。パイプラインを停止します。\n\n', pou_time);
                error('Pipeline:POUOptimizationFailed', ...
                    ['POU optimization failed and allow_pou_failure=false. ', ...
                     'Reason: %s'], ME.message);
            else
                fprintf('POU最適化失敗 (%.1f s)。allow_pou_failure=true のため続行します。\n\n', pou_time);
            end
        end
    end
else
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║ Phase 0.5: POU Hardpoint Optimization - SKIPPED              ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');
end

%% Phase 1: PID最適化（オプション）
gains_args = '';

if opt.optimize
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║ Phase 1: PID Optimization                                     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    tic;

    if strcmp(opt.method, 'bayesian')
        % ベイズ最適化
        fprintf('Running Bayesian Optimization (%d iterations)...\n', opt.n_iter);
        % Note: optimize_pid_gains takes (track_type, radius, sim_time, initial_speed)
        % Using global opt.n_iter inside optimize_pid_gains if strictly needed requires modification there,
        % but currently optimize_pid_gains hardcodes iterations to 50.
        % We pass opt.sim_time and opt.initial_speed as expected.
        % params_file_arg からファイルパスを抽出して最適化コンテナに渡す
        drs_container_path = '';
        if ~isempty(params_file_arg)
            tokens = regexp(params_file_arg, '--params_file\s+([^\s]+)', 'tokens');
            if ~isempty(tokens)
                drs_container_path = tokens{1}{1};
            end
        end

        if ~isempty(drs_container_path)
            optimize_pid_gains(track_type, radius, opt.sim_time, opt.initial_speed, opt.n_iter, drs_container_path);
        else
            optimize_pid_gains(track_type, radius, opt.sim_time, opt.initial_speed, opt.n_iter);
        end
    else
        % グリッドサーチ
        fprintf('Running Grid Search...\n');
        grid_search_pid(track_type, radius);
    end

    opt_time = toc;
    fprintf('\n✓ Optimization completed in %.1f seconds (%.1f minutes)\n', opt_time, opt_time/60);

    % 最適化結果を読み込み
    if strcmp(opt.method, 'bayesian')
        gains_file = sprintf('optimal_gains_%s.txt', track_type);
    else
        gains_file = sprintf('grid_search_results_%s.txt', track_type);
    end

    if exist(gains_file, 'file')
        % ベストコストを確認して最適化が成功したか判断
        gains_file_content = fileread(gains_file);
        best_cost_match = regexp(gains_file_content, 'Best cost:\s*([0-9.]+)', 'tokens');
        best_cost_val = Inf;
        if ~isempty(best_cost_match)
            best_cost_val = str2double(best_cost_match{1}{1});
        end

        if best_cost_val >= 5000.0
            % 全評価が失敗（全てNaN/DNF）→ 不正なゲインを使わずデフォルトへフォールバック
            warning('最適化が失敗しました (Best cost: %.1f >= 5000)。C++デフォルトゲインを使用します。', best_cost_val);
            gains_args = '';
        else
            fid = fopen(gains_file, 'r');
            gains_args = '';
            while ~feof(fid)
                line = fgetl(fid);
                if startsWith(strtrim(line), '--')
                    % Remove trailing backslash if present
                    line = strrep(line, '\', '');
                    gains_args = [gains_args ' ' strtrim(line)];
                end
            end
            fclose(fid);
            gains_args = strtrim(gains_args);
            fprintf('Using optimized gains (Best cost: %.4f): %s\n', best_cost_val, gains_args);
        end
    else
        warning('最適化結果ファイルが見つかりません。デフォルトゲインを使用します。');
    end

    fprintf('\n');
else
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║ Phase 1: PID Optimization - SKIPPED                           ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n');
    fprintf('Using default PID gains.\n\n');
end

%% Phase 2: シミュレーション実行
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║ Phase 2: Lap Time Simulation                                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

tic;

% Dockerコマンド構築
output_csv_name = sprintf('laptime_sim_%s.csv', track_type);
output_csv = fullfile('output', 'laptime', output_csv_name);
output_csv_host = fullfile(project_root, output_csv);
driver_args = '--driver pid';
precheck_cmd = '';

if strcmp(opt.driver_mode, 'rl')
    if isempty(opt.rl_actor_dir)
        opt.rl_actor_dir = resolve_default_rl_actor_export_dir(project_root);
        if isempty(opt.rl_actor_dir)
            opt.rl_actor_dir = fullfile('rl_driver', 'rl_actor_export');
        end
    end
    rl_actor_dir_host = opt.rl_actor_dir;
    if ~isfolder(rl_actor_dir_host)
        rl_actor_dir_host = fullfile(pwd, rl_actor_dir_host);
    end
    if ~isfolder(rl_actor_dir_host)
        error('RL actor directory not found: %s', opt.rl_actor_dir);
    end

    % 絶対パス化して /workspace 相対に変換
    if ~isfolder(rl_actor_dir_host)
        error('RL actor directory not found: %s', rl_actor_dir_host);
    end
    rl_actor_dir_host = char(java.io.File(rl_actor_dir_host).getCanonicalPath());
    project_root_abs = char(java.io.File(project_root).getCanonicalPath());
    if ~startsWith(rl_actor_dir_host, project_root_abs)
        error('rl_actor_dir must be under project root: %s', project_root_abs);
    end

    rel_actor_path = erase(rl_actor_dir_host, [project_root_abs filesep]);
    rel_actor_path = strrep(rel_actor_path, '\', '/');
    rl_actor_dir_container = ['/workspace/' rel_actor_path];

    required_files = {'rl_actor_weights.txt', 'rl_normalization_params.txt', 'rl_actor_test_data.csv'};
    for rf = 1:numel(required_files)
        if ~isfile(fullfile(rl_actor_dir_host, required_files{rf}))
            error('Missing RL export file: %s', fullfile(rl_actor_dir_host, required_files{rf}));
        end
    end

    fprintf('RL actor dir (host): %s\n', rl_actor_dir_host);
    fprintf('RL actor dir (container): %s\n', rl_actor_dir_container);

    driver_args = sprintf('--driver rl --rl-actor-dir ''%s''', rl_actor_dir_container);
    % rl_actor_check はLayerNorm未実装のため誤差が大きくなる場合があるのでスキップ
    precheck_cmd = '';
elseif strcmp(opt.driver_mode, 'lpv_lqr')
    driver_args = sprintf('--driver lpv_lqr --lpv-model %s', opt.lpv_model);
end

% トラックタイプに応じたファイルパス設定
if strcmp(track_type, 'ASE_autocross')
    track_file_arg = '/workspace/data/ASE_autocross_sim.csv';
else
    track_file_arg = track_type;
end

docker_cmd = sprintf(['docker compose -f %s/docker-compose.yml run --rm fsae-sim ' ...
    'bash -c "mkdir -p /workspace/output/laptime && cd build && %s./cpp/laptime_sim --track %s --radius %.4f --time %.4f --initial-speed %.4f %s %s %s ' ...
    '--output_json --output /workspace/%s"'], ...
    project_root, precheck_cmd, track_file_arg, radius, opt.sim_time, opt.initial_speed, driver_args, gains_args, params_file_arg, output_csv);

fprintf('Executing simulation...\n');
fprintf('Command: %s\n\n', docker_cmd);

    % Capture stdout and stderr
    docker_cmd = [docker_cmd, ' 2>&1'];
    [status, output] = system(docker_cmd);

sim_time = toc;
sim_result = [];
parse_error = [];
try
    sim_result = parse_laptime_json_output(output);
catch parse_exception
    parse_error = parse_exception;
end

sim_failed = status ~= 0;
if isempty(parse_error) && isstruct(sim_result)
    sim_failed = sim_failed || is_laptime_failed(sim_result);
end

if sim_failed
    fprintf('\n✗ Simulation FAILED\n');
    if isempty(parse_error) && isstruct(sim_result)
        if isfield(sim_result, 'failure_code') && ~isempty(sim_result.failure_code)
            fprintf('Failure code: %s\n', char(string(sim_result.failure_code)));
        end
        if isfield(sim_result, 'failure_message') && ~isempty(sim_result.failure_message)
            fprintf('Failure message: %s\n', char(string(sim_result.failure_message)));
        end
    elseif ~isempty(parse_error)
        fprintf('Failed to parse simulator JSON: %s\n', parse_error.message);
    end
    fprintf('Error output:\n%s\n', output);
    error('run_full_pipeline:laptime_failed', 'Lap time simulation failed.');
end

fprintf('\n✓ Simulation completed in %.1f seconds\n', sim_time);
fprintf('Output saved to: %s\n\n', output_csv_host);

% シミュレーション結果のサマリーを表示
fprintf('Simulation Summary:\n');
fprintf('─────────────────────────────────────────────────────────────────\n');
if isfield(sim_result, 'status')
    fprintf('Status: %s\n', char(string(sim_result.status)));
end
fprintf('Lap completed: %s\n', mat2str(isfield(sim_result, 'lap_completed') && sim_result.lap_completed));
if isfield(sim_result, 'distance_traveled')
    fprintf('Distance traveled: %.3f m\n', sim_result.distance_traveled);
end
if isfield(sim_result, 'max_speed')
    fprintf('Max speed: %.3f m/s\n', sim_result.max_speed);
end

if isfield(sim_result, 'lap_completed') && sim_result.lap_completed && isfield(sim_result, 'lap_time')
    lap_time_val = sim_result.lap_time;
    fprintf('\n');
    fprintf('╔════════════════════════════════════════╗\n');
    fprintf('║ LAP TIME: %7.3f seconds            ║\n', lap_time_val);
    fprintf('╚════════════════════════════════════════╝\n');
end
fprintf('\n');

%% Phase 3: 可視化
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║ Phase 3: Visualization                                        ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

if ~exist(output_csv_host, 'file')
    warning('シミュレーション結果ファイルが見つかりません: %s', output_csv_host);
    return;
end

if opt.visualize
    fprintf('Generating plots...\n');
    visualize_laptime_sim(output_csv_host, track_type, radius);
else
    fprintf('(可視化スキップ - EasyStartの[5]で表示できます)\n');
end

fprintf('\n✓ Visualization completed\n\n');

%% Phase 4: レポート生成（オプション）
if opt.save_report
    fprintf('╔════════════════════════════════════════════════════════════════╗\n');
    fprintf('║ Phase 4: Report Generation                                    ║\n');
    fprintf('╚════════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');

    report_file = fullfile(report_output_dir, sprintf('report_%s_%s.txt', track_type, datestr(now, 'yyyymmdd_HHMMSS')));

    fid = fopen(report_file, 'w');
    fprintf(fid, '╔════════════════════════════════════════════════════════════════╗\n');
    fprintf(fid, '║  FSAE Lap Time Simulation Report                              ║\n');
    fprintf(fid, '╚════════════════════════════════════════════════════════════════╝\n');
    fprintf(fid, '\n');
    fprintf(fid, 'Generated: %s\n', datestr(now));
    fprintf(fid, '\n');
    fprintf(fid, '== Configuration ==\n');
    fprintf(fid, 'Track Type:      %s\n', track_type);
    fprintf(fid, 'Radius:          %.1f m\n', radius);
    fprintf(fid, 'Sim Time:        %.3f s\n', opt.sim_time);
    fprintf(fid, 'Initial Speed:   %.3f m/s\n', opt.initial_speed);
    fprintf(fid, '\n');

    if opt.optimize
        fprintf(fid, '== PID Optimization ==\n');
        fprintf(fid, 'Method:          %s\n', opt.method);
        if strcmp(opt.method, 'bayesian')
            fprintf(fid, 'Iterations:      %d\n', opt.n_iter);
        end
        fprintf(fid, 'Time:            %.1f s (%.1f min)\n', opt_time, opt_time/60);
        fprintf(fid, 'Optimized Gains: %s\n', gains_args);
        fprintf(fid, '\n');
    end

    fprintf(fid, '== Simulation Results ==\n');
    fprintf(fid, '%s\n', output);
    fprintf(fid, '\n');
    fprintf(fid, '== Files Generated ==\n');
    fprintf(fid, '- CSV Data:      %s\n', output_csv_host);
    fprintf(fid, '- Figures:       (MATLAB figures)\n');
    fprintf(fid, '- Report:        %s\n', report_file);
    fprintf(fid, '\n');

    fclose(fid);

    fprintf('✓ Report saved to: %s\n\n', report_file);
end

%% 完了メッセージ
fprintf('╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║ Pipeline Completed Successfully!                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

if opt.optimize
    fprintf('Total time: %.1f seconds (%.1f minutes)\n', opt_time + sim_time, (opt_time + sim_time)/60);
else
    fprintf('Total time: %.1f seconds\n', sim_time);
end

fprintf('\n');
fprintf('Next steps:\n');
fprintf('  1. Review the generated figures\n');
fprintf('  2. Check the CSV data: %s\n', output_csv_host);
if opt.save_report
    fprintf('  3. Read the report: %s\n', report_file);
end
fprintf('\n');

end

function result = parse_laptime_json_output(output)
json_start = strfind(output, '{');
json_end = strfind(output, '}');
if isempty(json_start) || isempty(json_end)
    error('No JSON found in simulator output.');
end
json_content = output(json_start(1):json_end(end));
result = jsondecode(json_content);
end

function failed = is_laptime_failed(result)
failed = isstruct(result) && isfield(result, 'status') && ...
    strcmpi(char(string(result.status)), 'failed');
end
