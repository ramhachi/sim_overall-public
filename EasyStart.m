% EasyStart.m - FSAE Simulator Interactive Launcher
% 誰でも簡単にシミュレーションを実行できるスクリプト
% Run this file to start the interactive menu.

function EasyStart()

    start_script_path = fileparts(mfilename('fullpath'));
    addpath_if_missing(start_script_path);
    addpath_if_missing(fullfile(start_script_path, 'matlab'));
    addpath_if_missing(fullfile(start_script_path, 'matlab', 'design_sensitivity'));

% --- ここから: OS別のDockerパス自動設定 ---
    current_path = getenv('PATH');
    
    if ismac
        % Macの場合のDockerディレクトリ
        docker_dir = '/opt/homebrew/bin';
        % まだPATHに含まれていなければ追加（区切り文字はコロン）
        if ~contains(current_path, docker_dir)
            setenv('PATH', [docker_dir, ':', current_path]);
        end
        
    elseif ispc
        % Windowsの場合のDockerディレクトリ
        docker_dir = 'C:\Program Files\Docker\Docker\resources\bin';
        % まだPATHに含まれていなければ追加（区切り文字はセミコロン）
        if ~contains(current_path, docker_dir)
            setenv('PATH', [docker_dir, ';', current_path]);
        end
    end

% Clear command window for clean start
clc;

% ASCII Art Header
fprintf('=======================================================\n');
fprintf('   🏎️  FSAE Vehicle Dynamics Simulator - Easy Start  🏎️   \n');
fprintf('=======================================================\n');
fprintf('\n');
fprintf('ようこそ！ このスクリプトはシミュレーション環境の操作を案内します。\n');
fprintf('行いたい操作の番号を入力して Enter キーを押してください。\n');
fprintf('\n');

while true
    fprintf('\n-------------------------------------------------------\n');
    fprintf('  [1] 🛠️  環境構築・更新 (Dockerビルド) \n');
    fprintf('      -> 初回実行時や、C++コードを変更した後に実行してください\n');
    fprintf('\n');
    fprintf('  [2] 🏁  シミュレーション実行 (単走) \n');
    fprintf('      -> 現在の設定で1周のタイムアタックを行います\n');
    fprintf('\n');
    fprintf('  [3] 🧠  車両パラメータ最適化 (LPV-LQR GA)\n');
    fprintf('      -> 5時間・10並列の再開可能な高忠実度GAを管理します\n');
    fprintf('\n');
    fprintf('  [4] 📐  設計感度分析 (最小OAT)\n');
    fprintf('      -> 46次元設計ベクトルをautocross/PIDで感度評価します (baseline必須)\n');
    fprintf('\n');
    fprintf('  [5] 📊  結果の可視化 (グラフ表示) \n');
    fprintf('      -> 直近のシミュレーション結果を表示します\n');
    fprintf('\n');
    fprintf('  [6] 🛞  タイヤモデル学習 / フィッティング\n');
    fprintf('      -> Magic Formula / PINN の学習、モデル一覧確認、運用用モデル生成を行います\n');
    fprintf('\n');
    fprintf('  [7] 🔧  POU ハードポイント最適化 (サスペンション設計)\n');
    fprintf('      -> 多項式係数から物理的なハードポイント座標を逆算します\n');
    fprintf('\n');
    fprintf('  [8] 📐  サスペンション統合設計 (Front A-Arm → POU / Rear A-Arm)\n');
    fprintf('      -> JSON諸元からフロント(+POU)・リア(POU省略)の全3D座標を一貫生成します\n');
    fprintf('\n');
    fprintf('  [9] 🏗️  フレーム最適化 (3Dトラス & サスペンション統合)\n');
    fprintf('      -> 規定・サスペンションジオメトリを満たす最適なフレーム構造を探索します\n');
    fprintf('\n');
    fprintf('  [10] 🤖  RLドライバー管理 (強化学習)\n');
    fprintf('      -> 学習・評価・C++エクスポート・受け入れゲートを実行します\n');
    fprintf('\n');
    fprintf('  [11] 🎮  Assetto Corsa NK16 MOD 書き出し\n');
    fprintf('      -> 最新または指定した車両諸元JSONから、NK16_ICV差し替え用 data/ui を生成します\n');
    fprintf('\n');
    fprintf('  [0] ❌  終了\n');
    fprintf('-------------------------------------------------------\n');
    
    fprintf('番号を入力してください >> ');
    drawnow;
    choice = input('', 's');
    
    switch choice
        case '1'
            setup_environment();
        case '2'
            run_simulation();
        case '3'
            run_optimization();
        case '4'
            run_design_sensitivity_menu();
        case '5'
            visualize_results();
        case '6'
            run_fitting();
        case '7'
            run_pou_optimization();
        case '8'
            run_suspension_pipeline();
        case '9'
            run_frame_optimization_launcher();
        case '10'
            run_rl_driver_menu();
        case '11'
            run_assetto_corsa_export_menu();
        case '0'
            fprintf('\n👋 お疲れ様でした！\n');
            break;
        otherwise
            fprintf('\n⚠️  無効な番号です。もう一度試してください。\n');
    end
    
    fprintf('\nキーを押すとメニューに戻ります...\n');
    pause;
    clc;
end
end

%% --- Helper Functions ---

function setup_environment()
build_easystart_docker_environment();
end

function run_simulation()
fprintf('\n🏁  シミュレーションを実行します...\n');
fprintf('\n実行する競技を選択してください:\n');
fprintf('  [1] 🏎️  Circle (円形トラック, R=30m)\n');
fprintf('  [2] 🔄  Figure-8 (8の字, R=25m)\n');
fprintf('  [3] 📍  Skidpad (スキッドパッド, R=9.125m)\n');
fprintf('  [4] 🚀  Acceleration (加速, 75m)\n');
fprintf('  [5] 🎯  Autocross (オートクロス)\n');
fprintf('  [0] キャンセル\n');

fprintf('番号を選択 >> ');
drawnow;
track_choice = input('', 's');

% トラック設定の初期化
track_type = '';
radius = 0;

switch track_choice
    case '1'
        track_type = 'circle';
        radius = 30;
    case '2'
        track_type = 'figure_eight';
        radius = 25;
    case '3'
        track_type = 'skidpad';
        radius = 9.125;
    case '4'
        track_type = 'acceleration';
        radius = 75;
    case '5'
        track_type = 'ASE_autocross';
        radius = 0;
    case '0'
        fprintf('\nキャンセルしました。\n');
        return;
    otherwise
        fprintf('\n⚠️  無効な番号です。\n');
        return;
end

% 車両パラメータの選択
fprintf('\n車両パラメータを選択してください:\n');
fprintf('  [1] デフォルト値 (C++内蔵パラメータ)\n');
fprintf('  [2] JSONファイルから読み込み\n');

fprintf('番号を選択 >> ');
drawnow;
params_source_choice = input('', 's');

custom_params = '';
if strcmp(params_source_choice, '2')
    custom_params = select_params_json();
    if isempty(custom_params)
        fprintf('\nキャンセルしました。\n');
        return;
    end
    fprintf('選択: %s\n', custom_params);
end

start_script_path = fileparts(mfilename('fullpath'));
tire_model_family = 'pinn';
tire_model_name = '';

fprintf('\nタイヤモデルを選択してください (PINNのみ、本採用runtime):\n');
[tire_model_name, ok] = select_deployable_pinn_model(start_script_path);
if ~ok
    fprintf('\n⛔ deployable な PINN モデルがないため実行できません。\n');
    fprintf('  [6]-[2] で PINN 学習、または [6]-[3] でモデル一覧を確認してください。\n');
    return;
end
fprintf('選択したPINNモデル: %s\n', tire_model_name);

% ドライバーモード選択
fprintf('\nドライバーモードを選択してください:\n');
fprintf('  [1] PID (既存)\n');
fprintf('  [2] RL (学習済みActorを使用)\n');
fprintf('  [3] LPV-LQR steering6 (軽量・車両GAの標準)\n');

fprintf('番号を選択 >> ');
drawnow;
driver_choice = input('', 's');
driver_mode = 'pid';
rl_actor_dir = '';
if strcmp(driver_choice, '2')
    driver_mode = 'rl';
    [rl_actor_dir, ok] = select_rl_actor_dir(start_script_path);
    if ~ok
        fprintf('\nキャンセルしました。\n');
        return;
    end
elseif strcmp(driver_choice, '3')
    driver_mode = 'lpv_lqr';
end

should_optimize = false;
if strcmp(driver_mode, 'pid')
    % PID最適化実行の確認
    fprintf('\nPID最適化を実行しますか？\n');
    fprintf('  [1] はい（最適化してからシミュレーション実行）\n');
    fprintf('  [2] いいえ（デフォルトゲインでシミュレーション実行）\n');
    fprintf('番号を選択 >> ');
    drawnow;
    optimize_choice = input('', 's');
    should_optimize = strcmp(optimize_choice, '1');
else
    fprintf('\n%sモードのため、PID最適化はスキップします。\n', driver_mode);
end

drs_enabled = false;
if strcmp(driver_mode, 'lpv_lqr')
    fprintf('\nLPV-LQRの検証済み契約ではDRSを無効化します。\n');
else
    % DRS使用確認
    fprintf('\nDRS（ドラッグリダクションシステム）を使用しますか？\n');
    fprintf('  DRS ON: 直線でCL-46%%/CD-26%% の空力改善。ただし重量+2.5kg のペナルティあり\n');
    fprintf('  [1] はい (DRS あり)\n');
    fprintf('  [2] いいえ (DRS なし)\n');

    fprintf('番号を選択 >> ');
    drawnow;
    drs_choice = input('', 's');
    drs_enabled = strcmp(drs_choice, '1');
end

fprintf('\nPOU ハードポイント最適化を先に実行しますか？\n');
fprintf('  [1] はい (サスペンション artifact を更新してから走行)\n');
fprintf('  [2] いいえ\n');
fprintf('番号を選択 >> ');
drawnow;
pou_choice = input('', 's');
pou_optimize = strcmp(pou_choice, '1');

matlab_dir = fullfile(start_script_path, 'matlab');
current_dir = pwd;

% RLモードのとき: actor未エクスポートなら自動エクスポート
if strcmp(driver_mode, 'rl')
    if ~ensure_rl_actor_exported(start_script_path, rl_actor_dir)
        return;
    end
end

    % MATLABディレクトリで実行
try
    cd(matlab_dir);
    fprintf('\n');

    % トルクモード使用確認（JSONが未選択の場合のみ）
    if isempty(custom_params)
        fprintf('\nトルクモード（物理ステアリングモデル）を使用しますか？\n');
        fprintf('  [1] はい (Torque Mode - より現実に近いが制御難易度高)\n');
        fprintf('  [2] いいえ (Angle Mode - 安定)\n');

        fprintf('番号を選択 >> ');
        drawnow;
        torque_choice = input('', 's');
        if strcmp(torque_choice, '1')
            custom_params = fullfile(start_script_path, 'matlab', 'parameter_presets', 'params_flexible_torque.json');
        end
    end
    % run_full_pipelineを呼び出し（可視化はEasyStartの[5]で行うためスキップ）
    if strcmp(driver_mode, 'rl')
        fprintf('🤖 RLドライバーでシミュレーション実行...\n\n');
        run_full_pipeline(track_type, radius, 'drs_enabled', drs_enabled, ...
            'visualize', false, 'custom_params_file', custom_params, ...
            'driver_mode', 'rl', 'rl_actor_dir', rl_actor_dir, ...
            'tire_model_family', tire_model_family, 'tire_model_name', tire_model_name, ...
            'pou_optimize', pou_optimize);
    elseif strcmp(driver_mode, 'lpv_lqr')
        fprintf('🧭 LPV-LQR steering6でシミュレーション実行...\n\n');
        run_full_pipeline(track_type, radius, 'drs_enabled', false, ...
            'visualize', false, 'custom_params_file', custom_params, ...
            'driver_mode', 'lpv_lqr', 'lpv_model', 'steering6', ...
            'tire_model_family', tire_model_family, 'tire_model_name', tire_model_name, ...
            'pou_optimize', pou_optimize);
    elseif should_optimize
        fprintf('🧠 最適化を実行してからシミュレーション...\n\n');
        run_full_pipeline(track_type, radius, 'optimize', true, 'n_iter', 20, ...
            'drs_enabled', drs_enabled, 'visualize', false, ...
            'custom_params_file', custom_params, 'driver_mode', 'pid', ...
            'tire_model_family', tire_model_family, 'tire_model_name', tire_model_name, ...
            'pou_optimize', pou_optimize);
    else
        fprintf('⚡ デフォルトゲインでシミュレーション実行...\n\n');
        run_full_pipeline(track_type, radius, 'drs_enabled', drs_enabled, ...
            'visualize', false, 'custom_params_file', custom_params, 'driver_mode', 'pid', ...
            'tire_model_family', tire_model_family, 'tire_model_name', tire_model_name, ...
            'pou_optimize', pou_optimize);
    end

    cd(current_dir);
    fprintf('\n✅ シミュレーション完了！結果を見るにはメニューの[5]を選択してください。\n');

catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: シミュレーション実行中に問題が発生しました。\n');
    fprintf('詳細: %s\n', ME.message);
end
end

function run_design_sensitivity_menu()
% RUN_DESIGN_SENSITIVITY_MENU Minimal OAT design-sensitivity launcher.
fprintf('\n📐  設計感度分析 (最小OAT)\n');
fprintf('  baseline : matlab/parameter_presets/params_flexible_torque.json\n');
fprintf('  event    : autocross / LPVまたはPID / 180 s\n');
fprintf('  [1] pilot 8 variables\n');
fprintf('  [2] all 46 variables (長時間)\n');
fprintf('  [0] cancel\n');
fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');
if strcmp(choice, '0')
    return;
end
max_variables = 8;
if strcmp(choice, '2')
    max_variables = 0;
end
fprintf('\n評価ドライバー:\n');
fprintf('  [1] LPV (安定・スクリーニング推奨)\n');
fprintf('  [2] PID (GAと同じ条件)\n');
fprintf('番号を選択 >> ');
drawnow;
driver_choice = input('', 's');
driver_mode = 'lpv_lqr';
if strcmp(driver_choice, '2')
    driver_mode = 'pid';
end
try
    opts = struct('max_variables', max_variables, 'driver_mode', driver_mode);
    r = run_design_sensitivity(opts);
    fprintf('\n✅ 設計感度分析が完了しました。\n');
    fprintf('  samples : %s\n', fullfile(r.out_dir, 'samples.csv'));
    fprintf('  summary : %s\n', fullfile(r.out_dir, 'summary.csv'));
    fprintf('  freeze  : %s\n', fullfile(r.out_dir, 'freeze_candidates.txt'));
catch ME
    fprintf('\n⛔ 設計感度分析を中止しました: %s\n', ME.message);
end
end

function run_assetto_corsa_export_menu()
fprintf('\n🎮  Assetto Corsa NK16 MOD 向け書き出しを開始します\n');
fprintf('  sim_overall の車両諸元JSONから、NK16_ICVへ重ねる data/ui パッケージを生成します。\n\n');

start_script_path = fileparts(mfilename('fullpath'));

fprintf('使用する車両諸元JSONを選択してください:\n');
fprintf('  [1] 最新の最適化済みJSONを使用 (推奨)\n');
fprintf('  [2] 候補一覧から選択\n');
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');

vehicle_json_path = '';
vehicle_label = '';
switch choice
    case '1'
        candidates = find_vehicle_json_candidates(start_script_path, 'RequireExportReady', true);
        if isempty(candidates)
            fprintf('⚠️  利用可能な車両諸元JSONが見つかりません。\n');
            return;
        end
        optimizedIdx = find([candidates.is_optimized]);
        if isempty(optimizedIdx)
            candidate = candidates(1);
        else
            [~, newestIdxWithinOptimized] = max([candidates(optimizedIdx).datenum]);
            candidate = candidates(optimizedIdx(newestIdxWithinOptimized));
        end
        vehicle_json_path = candidate.path;
        vehicle_label = candidate.label;
        fprintf('既定選択: %s\n', vehicle_label);
    case '2'
        [vehicle_json_path, ok, selectedCandidate] = select_vehicle_json_candidate( ...
            'ProjectRoot', start_script_path, ...
            'Prompt', 'Assetto Corsa書き出しに使うJSONを選択してください:', ...
            'RequireExportReady', true);
        if ~ok
            fprintf('\nキャンセルしました。\n');
            return;
        end
        vehicle_label = selectedCandidate.label;
    case '0'
        fprintf('\nキャンセルしました。\n');
        return;
    otherwise
        fprintf('\n⚠️  無効な番号です。\n');
        return;
end

fprintf('\n書き出し先は output/assetto_corsa/NK16_ICV/ です。\n');
fprintf('必要なら、展開済みの NK16_ICV フォルダへそのまま上書きコピーできます。\n');
fprintf('展開済み NK16_ICV へ直接反映しますか？\n');
fprintf('  [1] いいえ (output/ にオーバーレイだけ生成)\n');
fprintf('  [2] はい (展開済み NK16_ICV フォルダへコピー)\n');
fprintf('番号を選択 >> ');
drawnow;
install_choice = input('', 's');

target_car_dir = '';
if strcmp(install_choice, '2')
    fprintf('展開済み NK16_ICV フォルダの絶対パスを入力 >> ');
    drawnow;
    target_car_dir = strtrim(input('', 's'));
    if isempty(target_car_dir)
        fprintf('\nキャンセルしました。\n');
        return;
    end
elseif ~strcmp(install_choice, '1')
    fprintf('\n⚠️  無効な番号です。\n');
    return;
end

fprintf('\n実行しますか？\n');
fprintf('  [1] はい\n');
fprintf('  [2] いいえ\n');
fprintf('番号を選択 >> ');
drawnow;
confirm = input('', 's');
if ~strcmp(confirm, '1')
    fprintf('\nキャンセルしました。\n');
    return;
end

try
    result = export_assetto_corsa_nk16(vehicle_json_path);
    if ~isempty(target_car_dir)
        installResult = install_assetto_corsa_nk16_export(result.output_car_dir, target_car_dir);
    else
        installResult = struct();
    end
    fprintf('\n✅ Assetto Corsa NK16 書き出し完了\n');
    fprintf('  Source JSON : %s\n', vehicle_label);
    fprintf('  Output dir  : %s\n', result.output_car_dir);
    if ~isempty(target_car_dir)
        fprintf('  Installed   : %s\n', installResult.target_car_dir);
        fprintf('  反映内容    : data/ と ui/ を直接上書きしました\n');
    else
        fprintf('  Overlay手順 : output/assetto_corsa/NK16_ICV/data と ui を NK16_ICV へコピー\n');
    end
catch ME
    fprintf('\n⛔ エラー: Assetto Corsa向け書き出しに失敗しました。\n');
    fprintf('詳細: %s\n', ME.message);
end
end

function run_optimization()
start_script_path = fileparts(mfilename('fullpath'));
config = vehicle_ga_default_config(start_script_path);
fprintf('\n🧠  LPV-LQR Full Vehicle GA\n');
fprintf('  [1] 5時間の新規Autocross探索 [推奨]\n');
fprintf('  [2] 1世代のクイックスモーク\n');
fprintf('  [3] 最新チェックポイントから再開\n');
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
mode_choice = input('', 's');

switch mode_choice
    case '1'
        % canonical defaults
    case '2'
        config.quick_mode = true;
        config.run_id = ['lpv_vehicle_ga_smoke_', ...
            char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))];
    case '3'
        checkpoints = dir(fullfile(config.output_root, '*', 'checkpoint', 'latest.mat'));
        if isempty(checkpoints)
            fprintf('\n⛔ 再開可能なチェックポイントがありません。\n');
            return;
        end
        [~, newest] = max([checkpoints.datenum]);
        config.resume_checkpoint = fullfile(checkpoints(newest).folder, checkpoints(newest).name);
    otherwise
        fprintf('\nキャンセルしました。\n');
        return;
end

fprintf('\n設定: Autocross / LPV-LQR steering6 / PINN固定 / DRS無効\n');
fprintf('Population=%d, MaxGenerations=%d, Workers=%d, GA上限=%.2f時間\n', ...
    config.population_size, config.max_generations, config.parallel_workers, ...
    config.ga_max_time_sec / 3600);
fprintf('過去解はdesign vectorのみseed化し、現物理で必ず再評価します。\n');
fprintf('Enterを押すと開始します...');
drawnow;
input('', 's');

try
    result = run_full_vehicle_ga(config);
    fprintf('\n✅  最適化が完了しました！\n');
    fprintf('Best cost: %.4f s\n', result.best_cost);
    fprintf('Artifact: %s\n', result.best_artifact);
catch ME
    fprintf('\n⛔ エラー: 最適化スクリプトの実行中に問題が発生しました。\n');
    fprintf('詳細: %s\n', ME.message);
end
end

function visualize_results()
fprintf('\n📊  データの可視化を準備中...\n');

start_script_path = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(start_script_path, 'matlab');
output_dir = fullfile(start_script_path, 'output', 'laptime');

% output/laptime ディレクトリからCSVを検索
files = dir(fullfile(output_dir, 'laptime_sim_*.csv'));

if isempty(files)
    fprintf('シミュレーション結果(.csv)が見つかりません。\n先にシミュレーションを実行してください。\n');
    return;
end

% 最終更新日でソート
[~, idx] = sort([files.datenum], 'descend');
latest_name = files(idx(1)).name;
latest_file = fullfile(files(idx(1)).folder, latest_name);

fprintf('最新のデータ: %s\n', latest_name);

% ファイル名からトラック種別を推定 (laptime_sim_<track>.csv)
[~, basename, ~] = fileparts(latest_name);
track_type = strrep(basename, 'laptime_sim_', '');

% トラック種別→デフォルト半径のマッピング
radius_map = containers.Map( ...
    {'circle','figure_eight','skidpad','acceleration','ASE_autocross'}, ...
    {30.0, 25.0, 9.125, 75.0, 0.0});
if isKey(radius_map, track_type)
    track_radius = radius_map(track_type);
else
    track_radius = 30.0;
end

current_dir = pwd;
try
    cd(matlab_dir);
    visualize_laptime_sim(latest_file, track_type, track_radius);
    cd(current_dir);
catch ME
    cd(current_dir);
    fprintf('可視化中にエラーが発生しました: %s\n', ME.message);
end
end

function run_fitting()
fprintf('\n🛞  タイヤモデル学習 / フィッティング\n');
fprintf('  [1] Magic Formula フィッティング\n');
fprintf('  [2] PINN 学習 + 運用モデル生成\n');
fprintf('  [3] PINN タイヤモデル一覧を表示\n');
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
fit_choice = input('', 's');

start_script_path = fileparts(mfilename('fullpath'));

switch fit_choice
    case '1'
        [tire_model_name, ok] = select_trainable_tire_dataset(start_script_path);
        if ~ok
            fprintf('\nキャンセルしました。\n');
            return;
        end

        fprintf('\nMagic Formula フィッティングを開始します...\n');
        fprintf('  対象タイヤ: %s\n', tire_model_name);
        fprintf('  TTCデータを使用してタイヤパラメータを同定します。\n');
        fprintf('  処理には数分かかる場合があります。中断するには Ctrl+C を押してください。\n');

        fprintf('Enterを押すと開始します...');
        drawnow;
        input('', 's');

        try
            assignin('base', 'run_from_launcher', true);
            assignin('base', 'tire_model', tire_model_name);
            script_path = fullfile(start_script_path, 'matlab', 'dl_tire_model', 'magic_formula_fitting.m');
            evalin('base', sprintf('run(''%s'')', script_path));

            fprintf('\n✅  Magic Formula フィッティングが完了しました！\n');
            fprintf('  結果は output/ ディレクトリに保存されました。\n');
        catch ME
            fprintf('\n⛔ エラー: Magic Formula フィッティング中に問題が発生しました。\n');
            fprintf('詳細: %s\n', ME.message);
        end

    case '2'
        [tire_model_name, ok] = select_trainable_tire_dataset(start_script_path);
        if ~ok
            fprintf('\nキャンセルしました。\n');
            return;
        end

        fprintf('\nPINN 学習対象: %s\n', tire_model_name);
        fprintf('学習後、物理妥当性ゲートを通過した場合のみ EasyStart から利用可能になります。\n');
        fprintf('Enterを押すと開始します...');
        drawnow;
        input('', 's');

        [status, output] = run_pinn_training(start_script_path, tire_model_name);
        fprintf('%s\n', output);
        validation_summary = load_pinn_validation_summary(start_script_path, tire_model_name);
        if status == 0
            if isfield(validation_summary, 'deployable') && validation_summary.deployable
                fprintf('\n✅  PINN 学習と運用モデル生成が完了しました！\n');
                fprintf('  EasyStart のシミュレーション実行で PINN を選択できます。\n');
            else
                fprintf('\n✅  PINN 学習は完了しました！\n');
                fprintf('  ただしこのモデルは non-deployable のため、EasyStart では参考表示のみ行います。\n');
                deployment_reasons = get_deployment_reasons(validation_summary);
                for ridx = 1:numel(deployment_reasons)
                    fprintf('    - %s\n', deployment_reasons{ridx});
                end
            end
        else
            fprintf('\n⛔ PINN 学習は完了しませんでした。\n');
            fprintf('  ログを確認し、必要ならデータや設定を見直してください。\n');
        end

    case '3'
        show_pinn_model_inventory(start_script_path);

    otherwise
        fprintf('\nキャンセルしました。\n');
        return;
end
end

function run_pou_optimization()
fprintf('\n🔧  POU ハードポイント最適化\n');
fprintf('  最適化された多項式係数（pou_jack_c1/c2, pou_camber_c1/c2）から\n');
fprintf('  物理的なハードポイント座標（M-pivot, bellcrank）を逆算します。\n\n');

% プロジェクトルート
start_script_path = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(start_script_path, 'matlab');
pou_dir = fullfile(matlab_dir, 'pou_hardpoint_design');

% POU ツールが存在するか確認
if ~exist(fullfile(pou_dir, 'main_pou_hardpoint_optimizer.m'), 'file')
    fprintf('⛔ エラー: POU最適化ツールが見つかりません。\n');
    fprintf('  期待されるパス: %s\n', pou_dir);
    return;
end

% パラメータファイル選択
[params_file, ok, selectedCandidate] = select_vehicle_json_candidate( ...
    'ProjectRoot', start_script_path, ...
    'Prompt', 'POU 最適化に使う車両 JSON を選択してください:', ...
    'RequireExportReady', true);
if ~ok
    fprintf('⚠️  利用可能な車両諸元JSONが見つかりません。\n');
    return;
end
fprintf('選択: %s\n\n', selectedCandidate.label);

% 最適化対象ホイール
fprintf('最適化する車輪を選択してください:\n');
fprintf('  [1] フロント両輪 (FL + FR) [推奨]\n');
fprintf('  [2] FL のみ\n');
fprintf('  [3] FR のみ\n');
fprintf('  [4] 全4輪 (FL + FR + RL + RR)\n');

fprintf('番号を選択 >> ');
drawnow;
wheel_choice = input('', 's');

switch wheel_choice
    case '1'
        wheels = [0, 1];
    case '2'
        wheels = [0];
    case '3'
        wheels = [1];
    case '4'
        wheels = [0, 1, 2, 3];
    otherwise
        wheels = [0, 1];
        fprintf('デフォルト: フロント両輪を選択\n');
end

% エクスポートオプション
fprintf('\n結果を車両パラメータJSONに書き戻しますか？\n');
fprintf('  [1] はい（ハードポイント座標を追記）\n');
fprintf('  [2] いいえ（CSV/JSONレポートのみ出力）\n');

fprintf('番号を選択 >> ');
drawnow;
update_choice = input('', 's');
update_params = strcmp(update_choice, '1');

fprintf('\n設定確認:\n');
wheel_names = {'FL', 'FR', 'RL', 'RR'};
wheel_str = strjoin(wheel_names(wheels + 1), ', ');
fprintf('  パラメータファイル: %s\n', selectedCandidate.label);
fprintf('  対象車輪: %s\n', wheel_str);
fprintf('  パラメータ書き戻し: %s\n', mat2str(update_params));
fprintf('\nEnterを押すと開始します...');
drawnow;
input('', 's');

% 実行
output_dir = fullfile(matlab_dir, 'pou_output');
current_dir = pwd;

try
    cd(matlab_dir);
    addpath(pou_dir);
    addpath(fullfile(pou_dir, 'geometry'));
    addpath(fullfile(pou_dir, 'optimization'));
    addpath(fullfile(pou_dir, 'visualization'));
    addpath(fullfile(pou_dir, 'export'));

    opts = struct();
    opts.wheels = wheels;
    opts.n_multistart = 5;
    opts.output_dir = output_dir;
    opts.export_csv = true;
    opts.export_json = ~update_params;
    opts.export_report = true;
    opts.update_params = update_params;
    opts.verbose = true;

    all_results = run_all_wheels(params_file, opts);

    cd(current_dir);

    fprintf('\n✅  POU ハードポイント最適化が完了しました！\n');
    fprintf('出力ファイル:\n');
    fprintf('  CSV (SolidWorks用): %s/pou_hardpoints_*.csv\n', output_dir);
    if update_params
        fprintf('  Artifact更新:        %s\n', params_file);
    else
        fprintf('  JSON:               %s/pou_hardpoints.json\n', output_dir);
    end
    fprintf('  レポート:           %s/pou_report_*.txt\n', output_dir);

    % 結果をプロットするか確認
    fprintf('\n結果のグラフを表示しますか？ [y/n] >> ');
    drawnow;
    plot_choice = input('', 's');
    if strcmpi(plot_choice, 'y')
        fields = fieldnames(all_results);
        for i = 1:length(fields)
            r = all_results.(fields{i});
            if isfield(r, 'delta_range_deg')
                try
                    plot_comparison(r);
                catch
                    % プロット失敗は無視
                end
            end
        end
    end

catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: POU最適化中に問題が発生しました。\n');
    fprintf('詳細: %s\n', ME.message);
end
end

function run_suspension_pipeline()
fprintf('\n📐  サスペンション統合設計 (A-Arm -> POU 自動連係)\n');
fprintf('  最適化された車両諸元（トレッド, キャンバー, バネ定数等）から、\n');
fprintf('  完全なサスペンションの3Dハードポイント座標を自動生成します。\n\n');

% プロジェクトルート
start_script_path = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(start_script_path, 'matlab');
a_arm_dir = fullfile(start_script_path, 'suspension_kinematics_design');
pou_dir = fullfile(matlab_dir, 'pou_hardpoint_design');

[params_file, ok, selectedCandidate] = select_vehicle_json_candidate( ...
    'ProjectRoot', start_script_path, ...
    'Prompt', 'サスペンション統合設計に使う車両 JSON を選択してください:', ...
    'RequireExportReady', true);
if ~ok
    fprintf('⚠️  利用可能な車両諸元JSONが見つかりません。\n');
    return;
end
fprintf('選択: %s\n\n', selectedCandidate.label);

fprintf('Phase 1 & 2: Aアーム幾何学の広域最適化を実行します。\n');
fprintf('Phase 1.5:   ステアリング（タイロッド）最適化を実行します。\n');
fprintf('Phase 2 & 3: POU（プッシュロッド）最適化を連携実行します。\n');
fprintf('\nEnterを押すと開始します (数分かかります)...');
drawnow;
input('', 's');

current_dir = pwd;
try
    % Phase 1 & 2: Front A-Arm Optimization
    fprintf('\n==========================================\n');
    fprintf(' Phase 1 & 2: Front A-Arm Optimization\n');
    fprintf('==========================================\n');
    cd(a_arm_dir);
    main_a_arm_optimizer(params_file);

    % Phase 1b: Rear A-Arm Optimization (POU省略)
    fprintf('\n==========================================\n');
    fprintf(' Phase 1b: Rear A-Arm Optimization (POU-less)\n');
    fprintf('==========================================\n');
    main_rear_a_arm_optimizer(params_file);
    
    % Phase 1.5: Steering Kinematics Optimization
    fprintf('\n==========================================\n');
    fprintf(' Phase 1.5: Steering Kinematics Optimization\n');
    fprintf('==========================================\n');
    steering_dir = fullfile(matlab_dir, 'steering_hardpoint_design');
    cd(steering_dir);
    main_steering_optimizer(params_file);
    
    % Phase 3: POU Linkage Optimization
    fprintf('\n==========================================\n');
    fprintf(' Phase 3: POU Linkage Optimization\n');
    fprintf('==========================================\n');
    cd(matlab_dir);
    addpath(pou_dir);
    addpath(fullfile(pou_dir, 'geometry'));
    addpath(fullfile(pou_dir, 'optimization'));
    addpath(fullfile(pou_dir, 'visualization'));
    addpath(fullfile(pou_dir, 'export'));
    
    output_dir = fullfile(matlab_dir, 'pou_output');
    opts = struct();
    opts.wheels = [0, 1]; % Front wheels FL / FR
    opts.n_multistart = 100;
    opts.output_dir = output_dir;
    opts.export_csv = true;
    opts.export_json = false;
    opts.export_report = true;
    opts.update_params = true;
    opts.verbose = true;
    
    all_results = run_all_wheels(params_file, opts);
    
    cd(current_dir);
    fprintf('\n✅  サスペンション統合設計パイプラインが完了しました！\n');
    fprintf('出力ファイル:\n');
    fprintf('  Front/Rear suspension artifact update: %s\n', params_file);
    fprintf('  POU (フロントのみ):       %s/pou_hardpoints_*.csv\n', output_dir);
    
    % 可視化オプション
    fprintf('\n最適化されたサスペンションの3Dジオメトリとグラフを表示しますか？ [y/n] >> ');
    drawnow;
    plot_choice = input('', 's');
    if strcmpi(plot_choice, 'y')
        try
            cd(matlab_dir);
            addpath(pou_dir);
            addpath(fullfile(pou_dir, 'visualization'));
            
            % 1. POU 2Dグラフの表示 (各ホイールごと)
            fields = fieldnames(all_results);
            for i = 1:length(fields)
                r = all_results.(fields{i});
                if isfield(r, 'delta_range_deg')
                    plot_comparison(r);
                end
            end
            
            % 2. 3Dサスペンション全体の表示
            visualize_3d_suspension(all_results, params_file);
            
            cd(current_dir);
        catch ME
            cd(current_dir);
            fprintf('3Dプロット中にエラーが発生しました: %s\n', ME.message);
        end
    end
    
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: サスペンション設計統合パイプライン中に問題が発生しました。\n');
    fprintf('詳細: %s\n', ME.message);
end

end

function run_frame_optimization_launcher()
fprintf('\n🏗️  フレーム最適化 (3Dトラス & サスペンション統合) を開始します\n');
fprintf('  この処理は遺伝的アルゴリズムを用いてフレーム構造を探索するため、\n');
fprintf('  数十分〜数時間かかる場合があります。\n\n');

% プロジェクトルートの取得
start_script_path = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(start_script_path, 'matlab');
frame_matlab_dir = fullfile(start_script_path, 'frame_optimize', 'matlab');

% 関数の存在確認
entrypoint_path = fullfile(frame_matlab_dir, 'run_canonical_frame_optimization.m');
if ~exist(entrypoint_path, 'file')
    fprintf('\n⛔ エラー: フレーム最適化関数が見つかりません。\n');
    fprintf('  期待されるパス: %s\n', entrypoint_path);
    return;
end
addpath(matlab_dir);
addpath(frame_matlab_dir);

% 車両パラメータファイルの選択
fprintf('剛性目標・車両諸元の読み込み元を選択してください:\n');
fprintf('  [1] 自動選択 (authoritative vehicle JSON を優先、無ければ安全に新規作成)\n');
fprintf('  [2] JSONファイルを明示選択 (この artifact を更新)\n');

fprintf('番号を選択 >> ');
drawnow;
params_choice = input('', 's');

vehicle_params_file = '';
if strcmp(params_choice, '2')
    [vehicle_params_file, ok, selectedCandidate] = select_vehicle_json_candidate( ...
        'ProjectRoot', start_script_path, ...
        'Prompt', 'フレーム最適化の基準に使う車両 JSON を選択してください:', ...
        'RequireExportReady', true, ...
        'RequireSuspensionHardpoints', true, ...
        'RequiredHardpointCorners', {'FL', 'RL'});
    if ok
        fprintf('選択: %s\n', selectedCandidate.label);
    else
        fprintf('自動選択で続行します。\n');
    end
end

frame_resolution = resolve_authoritative_vehicle_json(start_script_path, ...
    'ExplicitPath', vehicle_params_file, ...
    'RequireExportReady', true, ...
    'RequireSuspensionHardpoints', true, ...
    'RequiredHardpointCorners', {'FL', 'RL'}, ...
    'FallbackOutputPath', fullfile(start_script_path, 'output', 'runtime', 'frame_optimized_vehicle_artifact.json'));
if frame_resolution.exists
    fprintf('更新対象 artifact: %s\n', frame_resolution.path);
    vehicle_params_file = frame_resolution.path;
else
    fprintf('更新対象 artifact: %s (新規作成)\n', frame_resolution.path);
    vehicle_params_file = '';
end

fprintf('\n目標フレーム重量を入力してください (Enter でスキップ = ペナルティなし)\n');
mass_target_kg = prompt_optional_positive_scalar('目標重量 [kg]: ');

fprintf('\nSolidWorks VBAマクロを生成しますか？\n');
fprintf('  [1] はい (生成する)\n');
fprintf('  [2] いいえ (スキップ)\n');
fprintf('番号を選択 >> ');
drawnow;
sw_choice = input('', 's');
generate_solidworks_macro = strcmp(strtrim(sw_choice), '1');

fprintf('\nOnshape 3Dスケッチ用 FeatureScript を生成しますか？\n');
fprintf('  [1] はい (生成する)\n');
fprintf('  [2] いいえ (スキップ)\n');
fprintf('番号を選択 >> ');
drawnow;
onshape_choice = input('', 's');
generate_onshape_3d_sketch = strcmp(strtrim(onshape_choice), '1');

fprintf('\n実行しますか？\n');
fprintf('  [1] はい (実行する)\n');
fprintf('  [2] いいえ (キャンセル)\n');

fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');

if ~strcmp(choice, '1')
    fprintf('\nキャンセルしました。\n');
    return;
end

fprintf('\nEnterを押すと開始します...');
drawnow;
input('', 's');

try
    fprintf('\n==========================================\n');
    fprintf(' Frame Optimization\n');
    fprintf('==========================================\n');

    result = run_canonical_frame_optimization( ...
        'VehicleParamsFile', vehicle_params_file, ...
        'MassTargetKg', mass_target_kg, ...
        'GenerateSolidWorksMacro', generate_solidworks_macro, ...
        'GenerateOnshape3DSketch', generate_onshape_3d_sketch, ...
        'WaitForFigureClose', false, ...
        'DisplayProgress', true);

    fprintf('\n✅  フレーム最適化が完了しました！\n');
    if ~isempty(result.output_path)
        fprintf('更新された vehicle JSON: %s\n', result.output_path);
    end

catch ME
    fprintf('\n⛔ エラー: フレーム最適化中に問題が発生しました。\n');
    fprintf('詳細: %s\n', ME.message);
end

end

function value = prompt_optional_positive_scalar(prompt_text)
fprintf('%s', prompt_text);
drawnow;
raw_value = input('', 's');
if isempty(raw_value)
    value = [];
    return;
end

value = str2double(raw_value);
if isnan(value) || value <= 0
    warning('無効な入力。目標重量ペナルティを無効にします。');
    value = [];
end
end

function run_rl_driver_menu()
% RLドライバー管理サブメニュー
% 学習済みモデルの管理、新規学習、評価、C++エクスポートをまとめて実施

start_script_path = fileparts(mfilename('fullpath'));
rl_dir = fullfile(start_script_path, 'matlab', 'rl_driver');
rl_results_dir = fullfile(start_script_path, 'rl_results');  % プロジェクトルート直下
export_dir = fullfile(start_script_path, 'matlab', 'rl_driver', 'rl_actor_export');

ensure_rl_driver_paths(rl_dir);

while true
    fprintf('\n-------------------------------------------------------\n');
    fprintf('  🤖  RLドライバー管理\n');
    fprintf('-------------------------------------------------------\n');

    % 利用可能な最良エージェントを検出
    curriculum_agent = find_best_rl_agent(rl_results_dir);
    if ~isempty(curriculum_agent)
        info = dir(curriculum_agent);
        [~, rel] = fileparts2rel(curriculum_agent, start_script_path);
        fprintf('  📦 学習済みエージェント: %s\n', rel);
        fprintf('         (最終更新: %s)\n', string(datetime(info.datenum, 'ConvertFrom', 'datenum'), 'yyyy-MM-dd HH:mm'));
    else
        fprintf('  ⚠️  学習済みエージェントなし (rl_results/ 以下を検索)\n');
    end

    % エクスポート済みActorの確認
    actor_weights = fullfile(export_dir, 'rl_actor_weights.txt');
    if isfile(actor_weights)
        info2 = dir(actor_weights);
        fprintf('  📤 C++エクスポート済み: rl_actor_export/rl_actor_weights.txt\n');
        fprintf('         (最終更新: %s)\n', string(datetime(info2.datenum, 'ConvertFrom', 'datenum'), 'yyyy-MM-dd HH:mm'));
    else
        fprintf('  ⚠️  C++エクスポートなし (rl_actor_export/)\n');
    end

    fprintf('\n');
    fprintf('  [1] 🎓  カリキュラム学習開始 (straight→circle→skidpad→autocross)\n');
    fprintf('  [2] 🎯  単一トラック学習\n');
    fprintf('  [3] 📊  評価・可視化 (学習済みエージェントで走行)\n');
    fprintf('  [4] 📤  C++用Actorエクスポート\n');
    fprintf('  [5] ✅  受け入れゲート確認 (完走率・ラップタイム検証)\n');
    fprintf('  [6] ⚡  エースドライバー継続学習 (速度寄り phase3/4 続行)\n');
    fprintf('  [7] 🔁  ace final から phase4 追加学習\n');
    fprintf('  [8] ⏱️  RLラップタイムベンチ (autocross / skidpad)\n');
    fprintf('  [9] 🖥️  RLサーバー起動/再起動\n');
    fprintf('  [0] ◀   メインメニューへ戻る\n');
    fprintf('-------------------------------------------------------\n');

    fprintf('番号を選択 >> ');
    drawnow;
    rl_choice = input('', 's');

    switch rl_choice
        case '1'
            rl_start_curriculum(rl_dir, start_script_path);
        case '2'
            rl_start_single(rl_dir, start_script_path);
        case '3'
            rl_evaluate(rl_dir, start_script_path, curriculum_agent);
        case '4'
            rl_export(rl_dir, curriculum_agent, export_dir, rl_results_dir);
        case '5'
            rl_readiness_gate(rl_dir, curriculum_agent, export_dir, rl_results_dir);
        case '6'
            rl_start_ace_driver(rl_dir, start_script_path, curriculum_agent);
        case '7'
            rl_start_ace_phase4_extension(rl_dir, start_script_path);
        case '8'
            rl_benchmark_laptime(rl_dir, start_script_path);
        case '9'
            fprintf('\n🖥️  RLサーバーを起動します...\n');
            try
                startRLServer(start_script_path);
            catch ME
                fprintf('⛔ エラー: %s\n', ME.message);
            end
        case '0'
            return;
        otherwise
            fprintf('\n⚠️  無効な番号です。\n');
    end

    fprintf('\nキーを押すとRLメニューに戻ります...\n');
    pause;
end
end

% --- RL サブ関数 ---

function rl_start_curriculum(rl_dir, project_root)
fprintf('\n🎓  カリキュラム学習を開始します\n');
fprintf('  カリキュラム: straight → circle → skidpad → autocross\n');
fprintf('  車両諸元は 100 エピソードごとに更新されます\n');

fprintf('\nDomain Randomization（車両パラメータのランダム化）を使用しますか？\n');
fprintf('  [1] はい（推奨: 汎化性能が向上）\n');
fprintf('  [2] いいえ\n');
fprintf('番号を選択 >> ');
drawnow;
dr_choice = input('', 's');
use_dr = strcmp(dr_choice, '1');

fprintf('\n既存のエージェントから再開しますか？\n');
fprintf('  [1] いいえ（新規学習）\n');
fprintf('  [2] 最新のcurriculumエージェントから再開\n');

% 進捗ファイルがあれば完了フェーズを表示
progressFile = fullfile(project_root, 'rl_results', 'curriculum', 'progress.json');
if isfile(progressFile)
    try
        prog = jsondecode(fileread(progressFile));
        fprintf('      (完了済み: Phase %d / %s — %s)\n', ...
            prog.last_completed_phase, prog.track, prog.timestamp);
    catch
    end
end
fprintf('番号を選択 >> ');
drawnow;
resume_choice = input('', 's');

try
    current_dir = pwd;
    % プロジェクトルートから実行し、rl_driver関数を使えるようパスを追加
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    fprintf('\n🖥️  RLサーバーを起動します...\n');
    startRLServer(project_root);
    if strcmp(resume_choice, '2')
        best_agent = find_best_rl_resume_agent(fullfile(project_root, 'rl_results'));
        if ~isempty(best_agent)
            agent = loadRLAgentFromFile(best_agent);
            fprintf('✓ エージェント再読み込み: %s\n', best_agent);
            trainWithCurriculum('autocross', 'DomainRand', use_dr, 'RandInterval', 100, 'ResumeAgent', agent);
        else
            fprintf('⚠️  既存エージェントが見つかりません。新規学習します。\n');
            trainWithCurriculum('autocross', 'DomainRand', use_dr, 'RandInterval', 100);
        end
    else
        trainWithCurriculum('autocross', 'DomainRand', use_dr, 'RandInterval', 100);
    end
    cd(current_dir);
    fprintf('\n✅ カリキュラム学習完了！\n');
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_start_single(rl_dir, project_root)
fprintf('\n🎯  学習するトラックを選択してください:\n');
fprintf('  [1] straight  (直線加減速)\n');
fprintf('  [2] circle    (定円旋回)\n');
fprintf('  [3] skidpad   (スキッドパッド)\n');
fprintf('  [4] autocross (複合コース)\n');
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
track_choice = input('', 's');

tracks = {'straight', 'circle', 'skidpad', 'autocross'};
track_idx = str2double(track_choice);
if strcmp(track_choice, '0') || isnan(track_idx) || track_idx < 1 || track_idx > numel(tracks)
    fprintf('\nキャンセルしました。\n');
    return;
end
track_name = tracks{track_idx};

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    fprintf('\n🖥️  RLサーバーを起動します...\n');
    startRLServer(project_root);
    trainRLDriver(track_name);
    cd(current_dir);
    fprintf('\n✅ 学習完了！結果: rl_results/%s/\n', track_name);
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_evaluate(rl_dir, project_root, curriculum_agent)
fprintf('\n📊  評価に使用するエージェントを選択してください:\n');
if ~isempty(curriculum_agent)
    fprintf('  [1] 最良エージェント (%s)\n', curriculum_agent);
else
    fprintf('  [1] (利用可能なエージェントなし)\n');
end
fprintf('  [2] 他のファイルを指定\n');
fprintf('番号を選択 >> ');
drawnow;
agent_sel = input('', 's');

if strcmp(agent_sel, '2')
    fprintf('エージェントファイルのパスを入力 >> ');
    drawnow;
    agent_path = strtrim(input('', 's'));
elseif strcmp(agent_sel, '1')
    if isempty(curriculum_agent)
        fprintf('⛔ 利用可能なエージェントがありません。先に学習を実行してください。\n');
        return;
    end
    agent_path = curriculum_agent;
else
    fprintf('\nキャンセルしました。\n');
    return;
end

if ~isfile(agent_path)
    fprintf('⛔ ファイルが見つかりません: %s\n', agent_path);
    return;
end

fprintf('\n評価トラックを選択してください:\n');
fprintf('  [1] autocross\n');
fprintf('  [2] skidpad\n');
fprintf('  [3] circle\n');
fprintf('番号を選択 >> ');
drawnow;
t_choice = input('', 's');
tracks = {'autocross', 'skidpad', 'circle'};
t_idx = str2double(t_choice);
if isnan(t_idx) || t_idx < 1 || t_idx > numel(tracks)
    fprintf('\nキャンセルしました。\n');
    return;
end
eval_track = tracks{t_idx};

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    % サーバー起動（接続確認を兼ねる）
    fprintf('\n🖥️  RLサーバーを起動します...\n');
    startRLServer(project_root);
    % agent_pathを渡す（evaluateRLDriverはファイルパスまたはエージェントオブジェクトを受け取る）
    agent = loadRLAgentFromFile(agent_path);
    evaluateRLDriver(agent, eval_track);
    cd(current_dir);
    fprintf('\n✅ 評価完了！グラフを確認してください。\n');
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_export(rl_dir, curriculum_agent, export_dir, rl_results_dir)
fprintf('\n📤  C++用Actorエクスポートを開始します\n');

if isempty(curriculum_agent) || ~isfile(curriculum_agent)
    fprintf('⛔ 学習済みエージェントが見つかりません: %s\n', curriculum_agent);
    return;
end

published_agent = find_published_rl_agent(rl_results_dir);
if isempty(published_agent)
    fprintf(['⛔ default RL export path は未承認候補で上書きしません。\n' ...
        '   先に [5] 受け入れゲート確認を実行し、承認済み deployment candidate を作成してください。\n']);
    return;
end

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    agent = loadRLAgentFromFile(published_agent);
    exportVehicleParams = load_published_rl_vehicle_params(rl_results_dir);
    exportActorForCpp(agent, export_dir, exportVehicleParams);
    cd(current_dir);
    fprintf('\n✅ エクスポート完了: %s\n', export_dir);
    fprintf('  -> rl_actor_weights.txt\n');
    fprintf('  -> rl_normalization_params.txt\n');
    fprintf('  -> rl_actor_test_data.csv\n');
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_readiness_gate(rl_dir, curriculum_agent, export_dir, rl_results_dir)
fprintf('\n✅  受け入れゲート確認を開始します\n');
fprintf('  チェック内容: 完走率80%%以上 / コース逸脱10%%以下 / ラップタイム\n');

fprintf('\n受け入れゲートに使用するエージェントを選択してください:\n');
if ~isempty(curriculum_agent) && isfile(curriculum_agent)
    fprintf('  [1] 現在の候補 (%s)\n', curriculum_agent);
    fprintf('  [2] 他のファイルを指定\n');
else
    fprintf('  [1] 他のファイルを指定\n');
end
fprintf('番号を選択 >> ');
drawnow;
agent_sel = input('', 's');

if (~isempty(curriculum_agent) && isfile(curriculum_agent) && strcmp(agent_sel, '2')) || ...
        ((isempty(curriculum_agent) || ~isfile(curriculum_agent)) && strcmp(agent_sel, '1'))
    fprintf('エージェントファイルのパスを入力 >> ');
    drawnow;
    candidate_path = strtrim(input('', 's'));
elseif ~isempty(curriculum_agent) && isfile(curriculum_agent) && strcmp(agent_sel, '1')
    candidate_path = curriculum_agent;
else
    fprintf('\nキャンセルしました。\n');
    return;
end

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    report_dir = fullfile(rl_results_dir, 'curriculum');
    pidBaselines = getDefaultRLPIDBaselineLapTimes();
    if ~exist(report_dir, 'dir'), mkdir(report_dir); end
    result = runRLReadinessGate(candidate_path, ...
        'ActorExportDir', export_dir, ...
        'Tracks', {'autocross', 'skidpad', 'circle'}, ...
        'Episodes', 5, ...
        'SaveDir', report_dir, ...
        'PublishIfPass', true, ...
        'PIDBaselineLapTime', pidBaselines, ...
        'DeploymentDir', fullfile(rl_results_dir, 'deployment'));
    cd(current_dir);
    if result.summary.overall_pass
        fprintf('\n🎉 受け入れゲート: PASS！\n');
        if result.deployment_published
            fprintf('   承認済み deployment candidate を更新しました。\n');
        end
    else
        fprintf('\n⚠️  受け入れゲート: FAIL。詳細: %s\n', result.report_path);
    end
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_start_ace_driver(rl_dir, project_root, curriculum_agent)
fprintf('\n⚡  エースドライバー継続学習を開始します\n');
fprintf('  既定では phase34 recovery 完走結果から速度寄りの mixed-track 続行を行います。\n');
fprintf('  進捗が残っていれば ace-driver run 側の checkpoint から安全に再開します。\n');

defaultRecoveryFinal = fullfile(project_root, 'rl_results', ...
    'curriculum_torque_frame_fresh_v1', 'phase34_recovery_from_phase3_final', 'final_agent.mat');
if isfile(defaultRecoveryFinal)
    sourceAgent = defaultRecoveryFinal;
    fprintf('  既定の再開元: %s\n', sourceAgent);
elseif ~isempty(curriculum_agent) && isfile(curriculum_agent)
    sourceAgent = curriculum_agent;
    fprintf('  recovery final_agent が無いため、現在候補を再開元に使います: %s\n', sourceAgent);
else
    fprintf(['⛔ ace-driver 継続の再開元エージェントが見つかりません。\n' ...
        '   先に phase34 recovery を完走させるか、受け入れ候補を作成してください。\n']);
    return;
end

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    fprintf('\n🖥️  RLサーバーを起動します...\n');
    startRLServer(project_root);
    trainAceDriverFromRecovery('ResumeAgentPath', sourceAgent);
    cd(current_dir);
    fprintf('\n✅ エースドライバー継続学習を開始/再開しました。\n');
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_start_ace_phase4_extension(rl_dir, project_root)
fprintf('\n🔁  ace final から phase4 追加学習を開始します\n');
fprintf('  完了済み ace final_agent から、新しい run ディレクトリで phase4 を追加学習します。\n');
fprintf('  進捗が残っていれば extension run 側の checkpoint から安全に再開します。\n');

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    fprintf('\n🖥️  RLサーバーを起動します...\n');
    startRLServer(project_root);
    resumeAceDriverPhase4ExtensionAfterRestart();
    cd(current_dir);
    fprintf('\n✅ ace phase4 追加学習を開始/再開しました。\n');
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function rl_benchmark_laptime(rl_dir, project_root)
fprintf('\n⏱️  RL ラップタイムベンチを開始します\n');
fprintf('  autocross / skidpad の完走可否・ラップタイム・計算時間を計測します。\n');

[actor_dir, ok] = select_rl_actor_dir(project_root);
if ~ok
    fprintf('\nキャンセルしました。\n');
    return;
end

if ~ensure_rl_actor_exported(project_root, actor_dir)
    return;
end

try
    current_dir = pwd;
    ensure_rl_driver_paths(rl_dir);
    cd(project_root);
    results = benchmarkRLLaptimeSim('ActorDir', actor_dir, 'Tracks', {'autocross', 'skidpad'});
    cd(current_dir);
    fprintf('\n✅ RL ラップタイムベンチ完了\n');
    for i = 1:numel(results)
        fprintf('  [%s] lap_completed=%d, lap_time=%.3f s, wall_clock=%.3f s\n', ...
            results(i).track, results(i).lap_completed, results(i).lap_time, results(i).wall_clock_sec);
    end
catch ME
    cd(current_dir);
    fprintf('\n⛔ エラー: %s\n', ME.message);
end
end

function selected_path = select_params_json()
[selected_path, ok] = select_vehicle_json_candidate( ...
    'ProjectRoot', fileparts(mfilename('fullpath')), ...
    'Prompt', '利用可能な車両諸元JSONファイル:');
if ~ok
    selected_path = '';
end
end

function [selected_name, ok] = select_trainable_tire_dataset(project_root)
selected_name = '';
ok = false;

datasets = scan_trainable_tire_datasets(project_root);
if isempty(datasets)
    fprintf('⚠️  学習可能な TTC データセットが見つかりません。\n');
    return;
end

fprintf('\n学習対象のタイヤデータセットを選択してください:\n');
for i = 1:numel(datasets)
    fprintf('  [%d] %s\n', i, datasets{i});
end
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');

if strcmp(choice, '0')
    return;
end

idx = str2double(choice);
if isnan(idx) || idx < 1 || idx > numel(datasets)
    fprintf('⚠️  無効な番号です。\n');
    return;
end

selected_name = datasets{idx};
ok = true;
end

function datasets = scan_trainable_tire_datasets(project_root)
csv_dir = fullfile(project_root, 'data', 'tiredata', 'csv');
datasets = {};
if ~exist(csv_dir, 'dir')
    csv_dir = '';
end

files = [];
if ~isempty(csv_dir)
    files = dir(fullfile(csv_dir, '*.csv'));
end
names = {};
has_db = [];
has_cr = [];

for i = 1:length(files)
    tokens = regexp(files(i).name, '^(Db|Cr|Cmb)_(.+?)_Run\d+\.csv$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end
    test_type = tokens{1};
    tire_name = tokens{2};

    idx = find(strcmp(names, tire_name), 1);
    if isempty(idx)
        names{end+1} = tire_name; %#ok<AGROW>
        has_db(end+1) = false; %#ok<AGROW>
        has_cr(end+1) = false; %#ok<AGROW>
        idx = numel(names);
    end

    if strcmp(test_type, 'Db')
        has_db(idx) = true;
    elseif strcmp(test_type, 'Cr')
        has_cr(idx) = true;
    end
end

complete_names = names(has_db & has_cr);
raw_names = scan_raw_tire_datasets(project_root);
datasets = sort(unique([complete_names, raw_names]));
end

function datasets = scan_raw_tire_datasets(project_root)
data_dir = fullfile(project_root, 'data', 'tiredata');
datasets = {};
if ~exist(data_dir, 'dir')
    return;
end

manifest_entries = load_raw_tire_manifest_entries(project_root);
files = dir(fullfile(data_dir, '**', '*.dat'));
for i = 1:length(files)
    file_path = fullfile(files(i).folder, files(i).name);
    fid = fopen(file_path, 'r');
    if fid < 0
        continue;
    end
    header_line = fgetl(fid);
    fclose(fid);

    if ~ischar(header_line)
        continue;
    end

    manifest_tire_name = lookup_manifest_tire_name(files(i).name, manifest_entries);
    if ~isempty(manifest_tire_name)
        datasets{end+1} = normalize_tire_name_for_launcher(manifest_tire_name); %#ok<AGROW>
        continue;
    end

    tire_tokens = regexp(header_line, 'Tire_Name=([^;]+)', 'tokens', 'once');
    if ~isempty(tire_tokens)
        datasets{end+1} = normalize_tire_name_for_launcher(strtrim(tire_tokens{1})); %#ok<AGROW>
        continue;
    end

    project_tokens = regexp(header_line, 'Project\s+(\d+)', 'tokens', 'once');
    if ~isempty(project_tokens)
        datasets{end+1} = sprintf('Project%s_UnknownTire', project_tokens{1}); %#ok<AGROW>
    end
end

datasets = unique(datasets);
end

function manifest_entries = load_raw_tire_manifest_entries(project_root)
manifest_entries = struct('name', {}, 'tire_name', {});
manifest_path = fullfile(project_root, 'data', 'tiredata', 'raw_ttc_manifest.json');
if ~exist(manifest_path, 'file')
    return;
end

try
    manifest = jsondecode(fileread(manifest_path));
catch
    return;
end

if ~isfield(manifest, 'files') || isempty(manifest.files)
    return;
end

files_field = manifest.files;
if isstruct(files_field) && isfield(files_field, 'name')
    manifest_entries = files_field(:);
    return;
end

if ~isstruct(files_field)
    return;
end

file_names = fieldnames(files_field);
manifest_entries = repmat(struct('name', '', 'tire_name', ''), numel(file_names), 1);
for i = 1:numel(file_names)
    file_name = file_names{i};
    entry = files_field.(file_name);
    manifest_entries(i).name = file_name;
    if isstruct(entry) && isfield(entry, 'tire_name') && ~isempty(entry.tire_name)
        manifest_entries(i).tire_name = entry.tire_name;
    end
end
end

function tire_name = lookup_manifest_tire_name(file_name, manifest_entries)
tire_name = '';
if isempty(manifest_entries)
    return;
end

for i = 1:numel(manifest_entries)
    if isfield(manifest_entries(i), 'name') && strcmp(manifest_entries(i).name, file_name)
        if isfield(manifest_entries(i), 'tire_name') && ~isempty(manifest_entries(i).tire_name)
            tire_name = strtrim(manifest_entries(i).tire_name);
        end
        return;
    end
end
end

function normalized = normalize_tire_name_for_launcher(raw_name)
normalized = strtrim(raw_name);
normalized = strrep(normalized, ' ', '_');
normalized = strrep(normalized, ',', '');
normalized = strrep(normalized, '.', 'p');
normalized = strrep(normalized, '×', 'X');
end

function [selected_name, ok] = select_deployable_pinn_model(project_root)
selected_name = '';
ok = false;

[models, trained_only_models] = scan_pinn_model_inventory(project_root);
if isempty(models)
    fprintf('⚠️  deployable な PINN モデルが見つかりません。\n');
    if ~isempty(trained_only_models)
        fprintf('  学習済みだが運用不可のモデルはあります:\n');
        print_trained_only_pinn_models(trained_only_models);
    end
    fprintf('  [6]-[3] でモデル一覧を確認するか、[6]-[2] で PINN 学習を実行してください。\n');
    return;
end

fprintf('\n利用可能な PINN モデル:\n');
for i = 1:numel(models)
    fprintf('  [%d] %s\n', i, models{i});
end
if ~isempty(trained_only_models)
    fprintf('\n  --- 学習済みだが運用不可のモデル (参考表示) ---\n');
    print_trained_only_pinn_models(trained_only_models);
end
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');

if strcmp(choice, '0')
    return;
end

idx = str2double(choice);
if isnan(idx) || idx < 1 || idx > numel(models)
    fprintf('⚠️  無効な番号です。\n');
    return;
end

selected_name = models{idx};
ok = true;
end

function models = scan_deployable_pinn_models(project_root)
models = scan_pinn_model_inventory(project_root);
end

function [deployable_models, trained_only_models] = scan_pinn_model_inventory(project_root)
deployable_models = {};
trained_only_models = struct('name', {}, 'reasons', {});
model_root = fullfile(project_root, 'output', 'tire_models');
if ~exist(model_root, 'dir')
    return;
end

valid_datasets = scan_trainable_tire_datasets(project_root);

entries = dir(model_root);
for i = 1:length(entries)
    if ~entries(i).isdir || startsWith(entries(i).name, '.')
        continue;
    end

    model_name = entries(i).name;
    deployable = false;
    reasons = {};

    manifest_path = fullfile(entries(i).folder, entries(i).name, 'manifest.json');
    if isfile(manifest_path)
        try
            manifest = jsondecode(fileread(manifest_path));
            if isfield(manifest, 'tire_name') && ~isempty(manifest.tire_name)
                model_name = char(manifest.tire_name);
            end
            deployable = isfield(manifest, 'deployable') && manifest.deployable;
        catch
            % Ignore malformed manifests in the launcher.
        end
    end

    validation_summary = load_pinn_validation_summary(project_root, entries(i).name);
    if isfield(validation_summary, 'tire_name') && ~isempty(validation_summary.tire_name)
        model_name = char(validation_summary.tire_name);
    end
    if isfield(validation_summary, 'deployable') && ~validation_summary.deployable
        reasons = get_deployment_reasons(validation_summary);
    end

    if ~any(strcmp(valid_datasets, model_name))
        continue;
    end

    if deployable
        deployable_models{end+1} = model_name; %#ok<AGROW>
    elseif isfield(validation_summary, 'deployable')
        if ~any(strcmp({trained_only_models.name}, model_name))
            trained_only_models(end+1).name = model_name; %#ok<AGROW>
            trained_only_models(end).reasons = reasons;
        end
    end
end

deployable_models = sort(unique(deployable_models));
if ~isempty(trained_only_models)
    [~, order] = sort({trained_only_models.name});
    trained_only_models = trained_only_models(order);
end
end

function print_trained_only_pinn_models(trained_only_models)
for i = 1:numel(trained_only_models)
    fprintf('    - %s\n', trained_only_models(i).name);
    if ~isempty(trained_only_models(i).reasons)
        fprintf('      理由: %s\n', trained_only_models(i).reasons{1});
    end
end
end

function show_pinn_model_inventory(project_root)
[deployable_models, trained_only_models] = scan_pinn_model_inventory(project_root);

fprintf('\n📦  PINN タイヤモデル一覧\n');
fprintf('  deployable: %d モデル\n', numel(deployable_models));
for i = 1:numel(deployable_models)
    fprintf('    [利用可] %s\n', deployable_models{i});
end

fprintf('  学習済みだが運用不可: %d モデル\n', numel(trained_only_models));
print_trained_only_pinn_models(trained_only_models);

fprintf('\n  ※ [利用可] はシミュレーション/最適化で選択可能です。\n');
fprintf('  ※ 学習済みだが運用不可のモデルは参考表示のみです。\n');
end

function [rl_actor_dir, ok] = select_rl_actor_dir(start_script_path)
rl_actor_dir = '';
ok = false;
default_rl_dir = fullfile(start_script_path, 'matlab', 'rl_driver', 'rl_actor_export');

fprintf('\nRL Actorディレクトリを選択してください:\n');
fprintf('  [1] デフォルト (%s)\n', default_rl_dir);
fprintf('  [2] 手入力\n');
fprintf('  [0] キャンセル\n');
fprintf('番号を選択 >> ');
drawnow;
rl_dir_choice = input('', 's');

switch rl_dir_choice
    case '0'
        return;
    case '2'
        fprintf('ディレクトリパスを入力 >> ');
        drawnow;
        rl_actor_dir = normalize_project_path(start_script_path, strtrim(input('', 's')));
        if isempty(rl_actor_dir)
            fprintf('\n⚠️  RL Actorディレクトリが空です。\n');
            return;
        end
    otherwise
        rl_actor_dir = default_rl_dir;
end

ok = ~isempty(rl_actor_dir);
end

function ok = ensure_rl_actor_exported(start_script_path, rl_actor_dir)
ok = false;
project_root_abs = canonicalize_path(start_script_path);
rl_actor_dir_abs = canonicalize_path(rl_actor_dir);
if ~is_path_within_root(rl_actor_dir_abs, project_root_abs)
    fprintf('⛔ RL Actorディレクトリはプロジェクト配下である必要があります: %s\n', project_root_abs);
    return;
end

rl_results_dir = fullfile(start_script_path, 'rl_results');
default_rl_dir = fullfile(start_script_path, 'matlab', 'rl_driver', 'rl_actor_export');
published_agent = find_published_rl_agent(rl_results_dir);
is_default_dir = strcmp(rl_actor_dir_abs, canonicalize_path(default_rl_dir));

if is_default_dir && isempty(published_agent)
    fprintf(['⛔ default RL Actor path は承認済み deployment candidate 専用です。\n' ...
        '  先に [10] RLドライバー管理 → [5] 受け入れゲート確認 を実行してください。\n']);
    return;
end

missing_files = find_missing_rl_export_files(rl_actor_dir);
if isempty(missing_files)
    ok = true;
    return;
end

fprintf('\n📤 RL Actorエクスポートが不足しています。自動でエクスポートします...\n');
rl_dir_path = fullfile(start_script_path, 'matlab', 'rl_driver');
ensure_rl_driver_paths(rl_dir_path);
if is_default_dir
    best_agent = published_agent;
else
    best_agent = find_best_rl_agent(rl_results_dir);
end
if isempty(best_agent)
    if is_default_dir
        fprintf(['⛔ 承認済み deployment candidate が見つかりません。\n' ...
            '  先に [10] RLドライバー管理 → [5] 受け入れゲート確認 を実行してください。\n']);
    else
        fprintf('⛔ 学習済みエージェントが見つかりません。\n');
        fprintf('  先に [10] RLドライバー管理 → [1] カリキュラム学習 を実行してください。\n');
    end
    return;
end

fprintf('  エージェント: %s\n', best_agent);

if ~exist(rl_actor_dir, 'dir')
    mkdir(rl_actor_dir);
end

try
    if strcmp(best_agent, published_agent)
        exportVehicleParams = load_published_rl_vehicle_params(rl_results_dir);
    else
        exportVehicleParams = struct();
    end
    agent = loadRLAgentFromFile(best_agent);
    exportActorForCpp(agent, rl_actor_dir, exportVehicleParams);
catch ME_exp
    fprintf('⛔ エクスポートに失敗しました: %s\n', ME_exp.message);
    return;
end

missing_files = find_missing_rl_export_files(rl_actor_dir);
if ~isempty(missing_files)
    fprintf('⛔ RL Actorエクスポート後も必要ファイルが不足しています: %s\n', ...
        strjoin(missing_files, ', '));
    return;
end

fprintf('✅ エクスポート完了: %s\n', rl_actor_dir);
ok = true;
end

function normalized_path = normalize_project_path(project_root, input_path)
normalized_path = strtrim(char(input_path));
if isempty(normalized_path)
    return;
end

is_absolute = startsWith(normalized_path, filesep) || ...
    ~isempty(regexp(normalized_path, '^[A-Za-z]:[\\/]', 'once'));
if ~is_absolute
    normalized_path = fullfile(project_root, normalized_path);
end
end

function ensure_rl_driver_paths(rl_dir)
addpath_if_missing(fileparts(rl_dir));
addpath_if_missing(rl_dir);
end

function addpath_if_missing(dir_path)
dir_path = char(dir_path);
if isempty(dir_path) || ~exist(dir_path, 'dir')
    return;
end
if ~any(strcmp(strsplit(path, pathsep), dir_path))
    addpath(dir_path);
end
end

function canonical_path = canonicalize_path(path_value)
canonical_path = char(java.io.File(char(path_value)).getCanonicalPath());
end

function tf = is_path_within_root(path_value, root_value)
path_abs = canonicalize_path(path_value);
root_abs = canonicalize_path(root_value);
tf = strcmp(path_abs, root_abs) || startsWith(path_abs, [root_abs filesep]);
end

function missing_files = find_missing_rl_export_files(rl_actor_dir)
required_files = {'rl_actor_weights.txt', 'rl_normalization_params.txt', 'rl_actor_test_data.csv'};
missing_files = {};
for i = 1:numel(required_files)
    if ~isfile(fullfile(rl_actor_dir, required_files{i}))
        missing_files{end + 1} = required_files{i}; %#ok<AGROW>
    end
end
end

function validation_summary = load_pinn_validation_summary(project_root, tire_model_name)
validation_summary = struct();
summary_path = fullfile( ...
    project_root, ...
    'output', ...
    'tire_models', ...
    tire_model_name, ...
    'training_artifacts', ...
    'validation_summary.json');
if ~isfile(summary_path)
    return;
end

try
    validation_summary = jsondecode(fileread(summary_path));
catch
    validation_summary = struct();
end
end

function reasons = get_deployment_reasons(validation_summary)
reasons = {};
if ~isstruct(validation_summary) || ~isfield(validation_summary, 'deployment_reasons')
    return;
end

raw_reasons = validation_summary.deployment_reasons;
if iscell(raw_reasons)
    for i = 1:numel(raw_reasons)
        if ischar(raw_reasons{i}) || (isstring(raw_reasons{i}) && isscalar(raw_reasons{i}))
            reasons{end+1} = char(raw_reasons{i}); %#ok<AGROW>
        end
    end
elseif isstring(raw_reasons)
    reasons = cellstr(raw_reasons(:));
elseif ischar(raw_reasons)
    reasons = {raw_reasons};
end
end

function [status, output] = run_pinn_training(project_root, tire_model_name)
if ispc
    python_cmd = 'python';
    cmd = sprintf( ...
        'cd /d "%s" && %s tire_pinn\\main_pinn_train.py --tire-model "%s"', ...
        project_root, python_cmd, tire_model_name);
else
    python_cmd = 'python3';
    cmd = sprintf( ...
        'cd "%s" && %s tire_pinn/main_pinn_train.py --tire-model "%s"', ...
        project_root, python_cmd, tire_model_name);
end
[status, output] = system(cmd);
end

function best = find_best_rl_agent(rl_results_dir)
% 利用可能な最良RLエージェントを検索（優先順位順）
% 1. rl_results/deployment/deployment_agent.mat (明示的に昇格した運用候補)
% 2. rl_results/**/final_agent.mat (最新のカリキュラム/復旧ラン完成品)
% 3. rl_results/curriculum/phase4_*/agent.mat (フェーズ4完了) → 3 → 2 → 1
% 4. rl_results/**/agent.mat (最新のフェーズ完了品)
% 5. rl_results/*/trained_agent.mat (単一トラック学習の完成品) - 最新ファイル
% 6. rl_results/*/Agent*.mat - 最新チェックポイント
best = '';
if ~exist(rl_results_dir, 'dir'), return; end

% 優先1: final_agent
f0 = fullfile(rl_results_dir, 'deployment', 'deployment_agent.mat');
if isfile(f0), best = f0; return; end

% 優先2: any final_agent.mat (latest)
finalCandidates = dir(fullfile(rl_results_dir, '**', 'final_agent.mat'));
if ~isempty(finalCandidates)
    [~, idx] = max([finalCandidates.datenum]);
    best = fullfile(finalCandidates(idx).folder, finalCandidates(idx).name);
    return;
end

% 優先3: curriculum/phase*_*/agent.mat (高いフェーズを優先)
for phaseNum = 4:-1:1
    phaseDirs = dir(fullfile(rl_results_dir, 'curriculum', sprintf('phase%d_*', phaseNum)));
    phaseDirs = phaseDirs([phaseDirs.isdir]);
    if ~isempty(phaseDirs)
        [~, order] = sort([phaseDirs.datenum], 'descend');
        phaseDirs = phaseDirs(order);
        f = fullfile(phaseDirs(1).folder, phaseDirs(1).name, 'agent.mat');
        if isfile(f), best = f; return; end
    end
end

% 優先4: generic agent.mat (latest), excluding deployment_agent.mat
agentCandidates = dir(fullfile(rl_results_dir, '**', 'agent.mat'));
if ~isempty(agentCandidates)
    agentPaths = arrayfun(@(c) fullfile(c.folder, c.name), agentCandidates, 'UniformOutput', false);
    keepMask = ~contains(agentPaths, [filesep 'deployment' filesep]);
    agentCandidates = agentCandidates(keepMask);
    if ~isempty(agentCandidates)
        [~, idx] = max([agentCandidates.datenum]);
        best = fullfile(agentCandidates(idx).folder, agentCandidates(idx).name);
        return;
    end
end

% 優先5: trained_agent (最新)
candidates = dir(fullfile(rl_results_dir, '**', 'trained_agent.mat'));
if ~isempty(candidates)
    [~, idx] = max([candidates.datenum]);
    best = fullfile(candidates(idx).folder, candidates(idx).name);
    return;
end

% 優先6: Agent*.mat チェックポイント (最新)
candidates = dir(fullfile(rl_results_dir, '**', 'Agent*.mat'));
if ~isempty(candidates)
    [~, idx] = max([candidates.datenum]);
    best = fullfile(candidates(idx).folder, candidates(idx).name);
end
end

function best = find_best_rl_resume_agent(rl_results_dir)
% 学習再開用の候補を検索する。deployment は除外し、
% 直近のトレーニング成果物を優先する。
best = '';
if ~exist(rl_results_dir, 'dir'), return; end

progressFile = fullfile(rl_results_dir, 'curriculum', 'progress.json');
if isfile(progressFile)
    try
        prog = jsondecode(fileread(progressFile));
        if isfield(prog, 'last_completed_phase') && isnumeric(prog.last_completed_phase)
            phaseDirs = dir(fullfile(rl_results_dir, 'curriculum', sprintf('phase%d_*', prog.last_completed_phase)));
            phaseDirs = phaseDirs([phaseDirs.isdir]);
            if ~isempty(phaseDirs)
                [~, order] = sort([phaseDirs.datenum], 'descend');
                phaseDirs = phaseDirs(order);
                candidate = fullfile(phaseDirs(1).folder, phaseDirs(1).name, 'agent.mat');
                if isfile(candidate)
                    best = candidate;
                    return;
                end
            end
        end
    catch
    end
end

agentCandidates = dir(fullfile(rl_results_dir, '**', 'agent.mat'));
if ~isempty(agentCandidates)
    agentPaths = arrayfun(@(c) fullfile(c.folder, c.name), agentCandidates, 'UniformOutput', false);
    keepMask = ~contains(agentPaths, [filesep 'deployment' filesep]);
    agentCandidates = agentCandidates(keepMask);
    if ~isempty(agentCandidates)
        [~, idx] = max([agentCandidates.datenum]);
        best = fullfile(agentCandidates(idx).folder, agentCandidates(idx).name);
        return;
    end
end

finalCandidates = dir(fullfile(rl_results_dir, '**', 'final_agent.mat'));
if ~isempty(finalCandidates)
    [~, idx] = max([finalCandidates.datenum]);
    best = fullfile(finalCandidates(idx).folder, finalCandidates(idx).name);
    return;
end

candidates = dir(fullfile(rl_results_dir, '**', 'trained_agent.mat'));
if ~isempty(candidates)
    [~, idx] = max([candidates.datenum]);
    best = fullfile(candidates(idx).folder, candidates(idx).name);
    return;
end

candidates = dir(fullfile(rl_results_dir, '**', 'Agent*.mat'));
if ~isempty(candidates)
    [~, idx] = max([candidates.datenum]);
    best = fullfile(candidates(idx).folder, candidates(idx).name);
end
end

function published = find_published_rl_agent(rl_results_dir)
published = fullfile(rl_results_dir, 'deployment', 'deployment_agent.mat');
if ~isfile(published)
    published = '';
end
end

function vehicleParams = load_published_rl_vehicle_params(rl_results_dir)
vehicleParams = struct();
metadataPath = fullfile(rl_results_dir, 'deployment', 'deployment_metadata.json');
if ~isfile(metadataPath)
    return;
end

try
    metadata = jsondecode(fileread(metadataPath));
catch
    return;
end

if isstruct(metadata) && isfield(metadata, 'vehicle_params') && isstruct(metadata.vehicle_params)
    vehicleParams = metadata.vehicle_params;
end
end

function [dummy, rel] = fileparts2rel(full_path, base_path)
% フルパスからベースパスを除いた相対パスを返す
dummy = '';
rel = strrep(full_path, [base_path filesep], '');
end
