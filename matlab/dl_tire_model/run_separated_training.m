%% run_separated_training.m
% Cornering / DriveBrake データを分離して個別NNを学習
%
% 背景:
%   - Cornering試験: SAをスイープ (SR≈0) → Fy, Mz が主要出力
%   - DriveBrake試験: SRをスイープ (SA≈0) → Fx が主要出力
%   - 混合データで学習するとバイアスが生じる
%   → テスト種別ごとに専用モデルを学習する
%
% 使用方法:
%   1. tire_model にタイヤモデル名を設定（例: 'Hoosier_16p0X7p5-10'）
%   2. csv/ フォルダ内の Cr_* / Db_* ファイルを自動検索
%   3. MATLABで実行
%
% ファイル命名規則（analyze_and_rename_tire_data.m で生成）:
%   {Cr|Db}_{TireModel}[_R20][_C2000]_Run{XXX}.csv
%   例: Cr_Hoosier_16p0X7p5-10_R20_C2000_Run004.csv
%       Db_Goodyear_18p0X6p5-10_Run074.csv

clear; clc;

fprintf('\n');
fprintf('=================================================================\n');
fprintf(' Separated Tire Model Training\n');
fprintf(' Cornering / DriveBrake 分離学習\n');
fprintf('=================================================================\n\n');

%% ============================================================
%% ユーザ設定
%% ============================================================

% タイヤモデル名（csv/ フォルダ内のファイル名に含まれる部分）
%   例: 'Hoosier_16p0X7p5-10'
%       'Goodyear_18p0X6p5-10'
%       'MRF_18p0X6p0-10'
tire_model = 'Hoosier_18p0X6p0-10_R20';

% データディレクトリ（スクリプトからの相対パス）
data_dir = '../../data/tiredata/csv';

% 学習設定
split_ratio = 0.85;   % 訓練/検証の分割比

%% ============================================================
%% ファイル自動検索
%% ============================================================
fprintf('タイヤモデル: %s\n', tire_model);
fprintf('データ検索先: %s\n\n', data_dir);

% Cornering ファイル検索 (Cr_{tire_model}*.csv)
cr_pattern = fullfile(data_dir, ['Cr_' tire_model '*.csv']);
cr_found = dir(cr_pattern);
cornering_files = sort({cr_found.name}');

% DriveBrake ファイル検索 (Db_{tire_model}*.csv)
db_pattern = fullfile(data_dir, ['Db_' tire_model '*.csv']);
db_found = dir(db_pattern);
drivebrake_files = sort({db_found.name}');

% 検索結果表示
train_cornering = ~isempty(cornering_files);
train_drivebrake = ~isempty(drivebrake_files);

fprintf('--- 検出ファイル ---\n');
if train_cornering
    fprintf('  Cornering (%d ファイル):\n', length(cornering_files));
    for i = 1:length(cornering_files)
        fprintf('    %s\n', cornering_files{i});
    end
else
    fprintf('  Cornering: なし (Cr_%s*.csv が見つかりません)\n', tire_model);
end
fprintf('\n');

if train_drivebrake
    fprintf('  DriveBrake (%d ファイル):\n', length(drivebrake_files));
    for i = 1:length(drivebrake_files)
        fprintf('    %s\n', drivebrake_files{i});
    end
else
    fprintf('  DriveBrake: なし (Db_%s*.csv が見つかりません)\n', tire_model);
end
fprintf('\n');

if ~train_cornering && ~train_drivebrake
    fprintf('\n  ⚠ "%s" に一致するファイルが見つかりません。\n', tire_model);
    fprintf('  csv/ フォルダの内容を確認してください:\n');
    all_csv = dir(fullfile(data_dir, '*.csv'));
    % ユニークなタイヤモデル名を抽出して表示
    models = {};
    for i = 1:length(all_csv)
        name = all_csv(i).name;
        % Cr_ or Db_ を除去し、_RunXXX.csv を除去 → モデル名を抽出
        tok = regexp(name, '^(?:Cr|Db)_(.+)_Run\d+\.csv$', 'tokens');
        if ~isempty(tok)
            models{end+1} = tok{1}{1}; %#ok<SAGROW>
        end
    end
    models = unique(models);
    fprintf('  利用可能なタイヤモデル:\n');
    for i = 1:length(models)
        fprintf('    - %s\n', models{i});
    end
    fprintf('\n');
    return;
end

%% ============================================================
%% GPU チェック
%% ============================================================
fprintf('GPU Check:\n');
if gpuDeviceCount > 0
    gpu = gpuDevice();
    fprintf('  ✓ GPU: %s (%.1f GB)\n\n', gpu.Name, gpu.AvailableMemory / 1e9);
else
    fprintf('  ⚠ No GPU - using CPU\n\n');
end

%% ============================================================
%% Part 1: Cornering モデル学習
%% ============================================================
if train_cornering
    fprintf('=================================================================\n');
    fprintf(' Part 1: Cornering モデル学習\n');
    fprintf(' (SAスイープ → Fy, Mz が主要ターゲット)\n');
    fprintf('=================================================================\n\n');

    % --- データ読み込み ---
    fprintf('  [1/5] データ読み込み\n');
    cr_paths = cellfun(@(f) fullfile(data_dir, f), cornering_files, 'UniformOutput', false);
    cr_data = loadTireData(cr_paths);
    fprintf('    ✓ Cornering: %d サンプル\n', height(cr_data));

    % --- データ検証（SR≈0であることを確認） ---
    sr_std = std(cr_data.SR);
    sa_std = std(cr_data.SA);
    fprintf('    SR std: %.4f (≈0 expected)\n', sr_std);
    fprintf('    SA std: %.4f (大きいほど良い)\n', sa_std);
    if sr_std > 0.05
        fprintf('    ⚠ 警告: SR の分散が大きい → DriveBrakeデータが混入？\n');
    end
    fprintf('\n');

    % --- データ統計表示 ---
    fprintf('  [2/5] データ統計\n');
    fprintf('    SA  : [%.2f, %.2f] deg, mean=%.2f\n', min(cr_data.SA), max(cr_data.SA), mean(cr_data.SA));
    fprintf('    SR  : [%.4f, %.4f], mean=%.4f\n', min(cr_data.SR), max(cr_data.SR), mean(cr_data.SR));
    fprintf('    Fz  : [%.0f, %.0f] N, mean=%.0f\n', min(cr_data.Fz), max(cr_data.Fz), mean(cr_data.Fz));
    fprintf('    Fy  : [%.0f, %.0f] N, mean=%.0f\n', min(cr_data.Fy), max(cr_data.Fy), mean(cr_data.Fy));
    fprintf('    Mz  : [%.1f, %.1f] Nm, mean=%.1f\n', min(cr_data.Mz), max(cr_data.Mz), mean(cr_data.Mz));
    fprintf('\n');

    % --- 前処理 ---
    fprintf('  [3/5] 前処理\n');
    cr_config = struct('split_ratio', split_ratio);
    [cr_X_train, cr_Y_train, cr_X_val, cr_Y_val, cr_norm] = preprocessDataWrapper(cr_data, cr_config);
    fprintf('    ✓ Train: %d, Val: %d\n', size(cr_X_train,1), size(cr_X_val,1));
    fprintf('    入力: [SR, SA, Fz, IA, TSTC] (5次元)\n');
    fprintf('    出力: [Fx, Fy, Mx, Mz] (4次元)\n');
    fprintf('    ※ Corneringでは Fy, Mz が主要ターゲット\n\n');

    % --- モデル作成 ---
    fprintf('  [4/5] モデル作成 (Cornering専用)\n');
    cr_net = createCornering_model();
    fprintf('    ✓ Cornering model created\n');
    fprintf('    Architecture: 128-128-64-64-32 (5層, Fy/Mz重視)\n\n');

    % --- 学習 ---
    fprintf('  [5/5] 学習開始\n');
    cr_train_config = struct();
    cr_train_config.max_epochs = 500;
    cr_train_config.batch_size = 256;
    cr_train_config.learning_rate = 0.0001;
    cr_train_config.patience = 300;
    cr_train_config.val_frequency = 100;
    cr_train_config.random_seed = 42;

    tic;
    [cr_trained, cr_info] = trainTireModel(cr_net, cr_X_train, cr_Y_train, cr_X_val, cr_Y_val, cr_train_config);
    cr_time = toc / 60;
    fprintf('    ✓ 学習完了 (%.1f min)\n\n', cr_time);

    % --- 評価 ---
    fprintf('  Cornering モデル評価:\n');
    cr_metrics = evaluateModel(cr_trained, cr_X_val, cr_Y_val, cr_norm);
    fprintf('    Fy: RMSE=%.1fN, R²=%.4f ★主要\n', cr_metrics.Fy.RMSE, cr_metrics.Fy.R2);
    fprintf('    Mz: RMSE=%.1fNm, R²=%.4f ★主要\n', cr_metrics.Mz.RMSE, cr_metrics.Mz.R2);
    fprintf('    Fx: RMSE=%.1fN, R²=%.4f (参考)\n', cr_metrics.Fx.RMSE, cr_metrics.Fx.R2);
    fprintf('    Mx: RMSE=%.1fNm, R²=%.4f (参考)\n\n', cr_metrics.Mx.RMSE, cr_metrics.Mx.R2);

    % --- 保存 ---
    if ~exist('output', 'dir'), mkdir('output'); end
    cr_save_name = sprintf('output/cornering_%s.mat', tire_model);
    save(cr_save_name, 'cr_trained', 'cr_norm', 'cr_metrics', 'cr_info', 'tire_model');
    fprintf('    ✓ Saved: %s\n\n', cr_save_name);
end

%% ============================================================
%% Part 2: DriveBrake モデル学習
%% ============================================================
if train_drivebrake
    fprintf('=================================================================\n');
    fprintf(' Part 2: DriveBrake モデル学習\n');
    fprintf(' (SRスイープ → Fx が主要ターゲット)\n');
    fprintf('=================================================================\n\n');

    % --- データ読み込み ---
    fprintf('  [1/5] データ読み込み\n');
    db_paths = cellfun(@(f) fullfile(data_dir, f), drivebrake_files, 'UniformOutput', false);
    db_data = loadTireData(db_paths);
    fprintf('    ✓ DriveBrake: %d サンプル\n', height(db_data));

    % --- データ検証（SA≈0であることを確認） ---
    sr_std = std(db_data.SR);
    sa_std = std(db_data.SA);
    fprintf('    SR std: %.4f (大きいほど良い)\n', sr_std);
    fprintf('    SA std: %.4f (≈0 expected)\n', sa_std);
    if sa_std > 1.0
        fprintf('    ⚠ 警告: SA の分散が大きい → Corneringデータが混入？\n');
    end
    fprintf('\n');

    % --- データ統計表示 ---
    fprintf('  [2/5] データ統計\n');
    fprintf('    SR  : [%.4f, %.4f], mean=%.4f\n', min(db_data.SR), max(db_data.SR), mean(db_data.SR));
    fprintf('    SA  : [%.2f, %.2f] deg, mean=%.2f\n', min(db_data.SA), max(db_data.SA), mean(db_data.SA));
    fprintf('    Fz  : [%.0f, %.0f] N, mean=%.0f\n', min(db_data.Fz), max(db_data.Fz), mean(db_data.Fz));
    fprintf('    Fx  : [%.0f, %.0f] N, mean=%.0f\n', min(db_data.Fx), max(db_data.Fx), mean(db_data.Fx));
    fprintf('\n');

    % --- 前処理 ---
    fprintf('  [3/5] 前処理\n');
    db_config = struct('split_ratio', split_ratio);
    [db_X_train, db_Y_train, db_X_val, db_Y_val, db_norm] = preprocessDataWrapper(db_data, db_config);
    fprintf('    ✓ Train: %d, Val: %d\n', size(db_X_train,1), size(db_X_val,1));
    fprintf('    入力: [SR, SA, Fz, IA, TSTC] (5次元)\n');
    fprintf('    出力: [Fx, Fy, Mx, Mz] (4次元)\n');
    fprintf('    ※ DriveBrakeでは Fx が主要ターゲット\n\n');

    % --- モデル作成 ---
    fprintf('  [4/5] モデル作成 (DriveBrake専用)\n');
    db_net = createDriveBrake_model();
    fprintf('    ✓ DriveBrake model created\n');
    fprintf('    Architecture: 128-128-64-64-32 (5層, Fx重視)\n\n');

    % --- 学習 ---
    fprintf('  [5/5] 学習開始\n');
    db_train_config = struct();
    db_train_config.max_epochs = 500;
    db_train_config.batch_size = 256;
    db_train_config.learning_rate = 0.0001;
    db_train_config.patience = 300;
    db_train_config.val_frequency = 100;
    db_train_config.random_seed = 42;

    tic;
    [db_trained, db_info] = trainTireModel(db_net, db_X_train, db_Y_train, db_X_val, db_Y_val, db_train_config);
    db_time = toc / 60;
    fprintf('    ✓ 学習完了 (%.1f min)\n\n', db_time);

    % --- 評価 ---
    fprintf('  DriveBrake モデル評価:\n');
    db_metrics = evaluateModel(db_trained, db_X_val, db_Y_val, db_norm);
    fprintf('    Fx: RMSE=%.1fN, R²=%.4f ★主要\n', db_metrics.Fx.RMSE, db_metrics.Fx.R2);
    fprintf('    Fy: RMSE=%.1fN, R²=%.4f (参考)\n', db_metrics.Fy.RMSE, db_metrics.Fy.R2);
    fprintf('    Mx: RMSE=%.1fNm, R²=%.4f (参考)\n', db_metrics.Mx.RMSE, db_metrics.Mx.R2);
    fprintf('    Mz: RMSE=%.1fNm, R²=%.4f (参考)\n\n', db_metrics.Mz.RMSE, db_metrics.Mz.R2);

    % --- 保存 ---
    if ~exist('output', 'dir'), mkdir('output'); end
    db_save_name = sprintf('output/drivebrake_%s.mat', tire_model);
    save(db_save_name, 'db_trained', 'db_norm', 'db_metrics', 'db_info', 'tire_model');
    fprintf('    ✓ Saved: %s\n\n', db_save_name);
end

%% ============================================================
%% 最終サマリー
%% ============================================================
fprintf('=================================================================\n');
fprintf(' 最終サマリー: %s\n', tire_model);
fprintf('=================================================================\n\n');

if train_cornering
    fprintf('【Cornering モデル】（旋回性能: Fy, Mz）\n');
    fprintf('  データ: %d サンプル\n', height(cr_data));
    fprintf('  学習時間: %.1f min\n', cr_time);
    fprintf('  ★ Fy: RMSE=%.1fN, R²=%.4f\n', cr_metrics.Fy.RMSE, cr_metrics.Fy.R2);
    fprintf('  ★ Mz: RMSE=%.1fNm, R²=%.4f\n', cr_metrics.Mz.RMSE, cr_metrics.Mz.R2);
    fprintf('  用途: サスペンション設計、操安性解析、ラップタイムシミュ\n\n');
end

if train_drivebrake
    fprintf('【DriveBrake モデル】（加減速性能: Fx）\n');
    fprintf('  データ: %d サンプル\n', height(db_data));
    fprintf('  学習時間: %.1f min\n', db_time);
    fprintf('  ★ Fx: RMSE=%.1fN, R²=%.4f\n', db_metrics.Fx.RMSE, db_metrics.Fx.R2);
    fprintf('  用途: トラクション制御、ABS設計、加速/制動性能\n\n');
end

if train_cornering && train_drivebrake
    fprintf('【比較】混合学習 vs 分離学習\n');
    fprintf('  分離学習の利点:\n');
    fprintf('    - 各モデルが専門のデータのみで学習 → バイアス低減\n');
    fprintf('    - Cornering: SR≈0のデータのみ → Fy/Mzの精度向上\n');
    fprintf('    - DriveBrake: SA≈0のデータのみ → Fxの精度向上\n');
    fprintf('    - ゼロスリップでの異常出力を回避\n\n');
    
    fprintf('【複合領域（G-Gダイアグラム）が必要な場合】\n');
    fprintf('  → 両モデルを組み合わせるか、Combined試験データで追加学習\n\n');
end

fprintf('=================================================================\n');
fprintf(' 完了\n');
fprintf('=================================================================\n\n');


%% ============================================================
%% ローカル関数: モデルアーキテクチャ
%% ============================================================

function net = createCornering_model()
    % Cornering専用モデル（深層版）
    % SAが主要入力、Fy/Mzが主要出力
    % 8層: 256-256-128-128-64-64-32-32
    
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        % Block 1
        fullyConnectedLayer(256, 'Name', 'fc1')
        batchNormalizationLayer('Name', 'bn1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.15, 'Name', 'drop1')
        
        % Block 2
        fullyConnectedLayer(256, 'Name', 'fc2')
        batchNormalizationLayer('Name', 'bn2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.15, 'Name', 'drop2')
        
        % Block 3
        fullyConnectedLayer(128, 'Name', 'fc3')
        batchNormalizationLayer('Name', 'bn3')
        reluLayer('Name', 'relu3')
        dropoutLayer(0.12, 'Name', 'drop3')
        
        % Block 4
        fullyConnectedLayer(128, 'Name', 'fc4')
        batchNormalizationLayer('Name', 'bn4')
        reluLayer('Name', 'relu4')
        dropoutLayer(0.12, 'Name', 'drop4')
        
        % Block 5
        fullyConnectedLayer(64, 'Name', 'fc5')
        batchNormalizationLayer('Name', 'bn5')
        reluLayer('Name', 'relu5')
        dropoutLayer(0.1, 'Name', 'drop5')
        
        % Block 6
        fullyConnectedLayer(64, 'Name', 'fc6')
        batchNormalizationLayer('Name', 'bn6')
        reluLayer('Name', 'relu6')
        dropoutLayer(0.1, 'Name', 'drop6')
        
        % Block 7
        fullyConnectedLayer(32, 'Name', 'fc7')
        batchNormalizationLayer('Name', 'bn7')
        reluLayer('Name', 'relu7')
        
        % Block 8
        fullyConnectedLayer(32, 'Name', 'fc8')
        batchNormalizationLayer('Name', 'bn8')
        reluLayer('Name', 'relu8')
        
        % Output: [Fx, Fy, Mx, Mz]
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end

function net = createDriveBrake_model()
    % DriveBrake専用モデル（深層版）
    % SRが主要入力、Fxが主要出力
    % 8層: 256-256-128-128-64-64-32-32
    
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        % Block 1
        fullyConnectedLayer(256, 'Name', 'fc1')
        batchNormalizationLayer('Name', 'bn1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.15, 'Name', 'drop1')
        
        % Block 2
        fullyConnectedLayer(256, 'Name', 'fc2')
        batchNormalizationLayer('Name', 'bn2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.15, 'Name', 'drop2')
        
        % Block 3
        fullyConnectedLayer(128, 'Name', 'fc3')
        batchNormalizationLayer('Name', 'bn3')
        reluLayer('Name', 'relu3')
        dropoutLayer(0.12, 'Name', 'drop3')
        
        % Block 4
        fullyConnectedLayer(128, 'Name', 'fc4')
        batchNormalizationLayer('Name', 'bn4')
        reluLayer('Name', 'relu4')
        dropoutLayer(0.12, 'Name', 'drop4')
        
        % Block 5
        fullyConnectedLayer(64, 'Name', 'fc5')
        batchNormalizationLayer('Name', 'bn5')
        reluLayer('Name', 'relu5')
        dropoutLayer(0.1, 'Name', 'drop5')
        
        % Block 6
        fullyConnectedLayer(64, 'Name', 'fc6')
        batchNormalizationLayer('Name', 'bn6')
        reluLayer('Name', 'relu6')
        dropoutLayer(0.1, 'Name', 'drop6')
        
        % Block 7
        fullyConnectedLayer(32, 'Name', 'fc7')
        batchNormalizationLayer('Name', 'bn7')
        reluLayer('Name', 'relu7')
        
        % Block 8
        fullyConnectedLayer(32, 'Name', 'fc8')
        batchNormalizationLayer('Name', 'bn8')
        reluLayer('Name', 'relu8')
        
        % Output: [Fx, Fy, Mx, Mz]
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end