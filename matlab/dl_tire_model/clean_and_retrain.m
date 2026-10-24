%% clean_and_retrain.m
% データクリーニング → 再学習 (v2: 診断結果に基づく改善)
% 
% 診断結果 (diagnose_drivebrake_data.m):
%   1. Run068/071はSA sweep（SR固定 ≈-0.05）→ DriveBrake学習に不適切
%   2. Run069/070/072/073はSR sweep × 複数SA条件 → SA閾値フィルタ不要
%   3. corr(SR,Fx)≈0.85はSA閾値に依存しない → 全データ使用が最適
%   4. SR sweep制約(Fx=0)がSR感度を殺していた → 撤廃
%   5. CorneringのSR異常値（|SR|>1）は引き続き除去

clear; clc;

fprintf('\n');
fprintf('=================================================================\n');
fprintf(' Data Cleaning & Retraining\n');
fprintf('=================================================================\n\n');

%% ============================================================
%% ユーザ設定
%% ============================================================

tire_model = 'Hoosier_18p0X6p0-10_R20';
data_dir = '../../data/tiredata/csv';
split_ratio = 0.85;

% クリーニング設定
% DriveBrake: Run068/071はSA sweep（SR固定）→ 除外
%   Run069/070/072/073はSR sweep × 複数SA条件 → 全データ使用
%   （SA閾値フィルタは不要: corr(SR,Fx)≈0.85で閾値非依存）
db_exclude_runs = {'Run068', 'Run071'};  % SA sweep Runs（SR固定）
cr_sr_threshold = 0.05;   % Cornering:  |SR| < 0.05 のみ使用

%% ============================================================
%% Step 1: データ読み込み
%% ============================================================
fprintf('Step 1: データ読み込み\n\n');

% Cornering
cr_pattern = fullfile(data_dir, ['Cr_' tire_model '*.csv']);
cr_found = dir(cr_pattern);
cr_paths = cellfun(@(f) fullfile(data_dir, f), sort({cr_found.name}'), 'UniformOutput', false);
cr_raw = loadTireData(cr_paths);
fprintf('  Cornering 生データ: %d サンプル\n', height(cr_raw));

% DriveBrake（SA sweep RunはSR固定なので除外）
db_pattern = fullfile(data_dir, ['Db_' tire_model '*.csv']);
db_found = dir(db_pattern);
db_all_names = sort({db_found.name}');
db_keep_mask = true(size(db_all_names));
for i = 1:length(db_all_names)
    for j = 1:length(db_exclude_runs)
        if contains(db_all_names{i}, db_exclude_runs{j})
            db_keep_mask(i) = false;
            fprintf('  除外: %s (SA sweep, SR固定)\n', db_all_names{i});
        end
    end
end
db_names = db_all_names(db_keep_mask);
db_paths = cellfun(@(f) fullfile(data_dir, f), db_names, 'UniformOutput', false);
db_raw = loadTireData(db_paths);
fprintf('  DriveBrake 生データ: %d サンプル (%d/%d ファイル使用)\n\n', ...
    height(db_raw), length(db_names), length(db_all_names));

%% ============================================================
%% Step 2: データクリーニング
%% ============================================================
fprintf('Step 2: データクリーニング\n\n');

% --- Cornering: SR異常値を除去 ---
fprintf('  --- Cornering クリーニング ---\n');
fprintf('  フィルタ前: %d サンプル\n', height(cr_raw));
fprintf('  SR統計: mean=%.4f, std=%.4f, range=[%.2f, %.2f]\n', ...
    mean(cr_raw.SR), std(cr_raw.SR), min(cr_raw.SR), max(cr_raw.SR));

% |SR| < threshold のみ保持
cr_mask = abs(cr_raw.SR) < cr_sr_threshold;
cr_clean = cr_raw(cr_mask, :);

fprintf('  フィルタ後: %d サンプル (%.1f%% 残存)\n', ...
    height(cr_clean), 100 * height(cr_clean) / height(cr_raw));
fprintf('  除去理由: |SR| >= %.2f のサンプルを除去\n', cr_sr_threshold);
fprintf('  除去数: %d サンプル\n', height(cr_raw) - height(cr_clean));
fprintf('  フィルタ後SR: mean=%.6f, std=%.6f, range=[%.4f, %.4f]\n', ...
    mean(cr_clean.SR), std(cr_clean.SR), min(cr_clean.SR), max(cr_clean.SR));
fprintf('  フィルタ後SA: mean=%.2f, std=%.2f, range=[%.2f, %.2f]\n', ...
    mean(cr_clean.SA), std(cr_clean.SA), min(cr_clean.SA), max(cr_clean.SA));
fprintf('\n');

% --- DriveBrake: Run選別済み（SA閾値フィルタなし）---
fprintf('  --- DriveBrake データ概要 ---\n');
fprintf('  サンプル数: %d (Run選別済み)\n', height(db_raw));
fprintf('  SA統計: mean=%.2f deg, std=%.2f deg, range=[%.2f, %.2f]\n', ...
    mean(db_raw.SA), std(db_raw.SA), min(db_raw.SA), max(db_raw.SA));
fprintf('  SR統計: mean=%.4f, std=%.4f, range=[%.4f, %.4f]\n', ...
    mean(db_raw.SR), std(db_raw.SR), min(db_raw.SR), max(db_raw.SR));
fprintf('  corr(SR, Fx) = %.4f\n', corr(db_raw.SR, db_raw.Fx));
fprintf('  ※ SA閾値フィルタなし（複合スリップデータもSR-Fx学習に有用）\n');

% Run選別済みなのでそのまま使用
db_clean = db_raw;
fprintf('\n');

% --- クリーニング後の物理整合性確認 ---
fprintf('  --- クリーニング後の物理整合性 ---\n');
fprintf('  DriveBrake:\n');
fprintf('    corr(SR, Fx) = %.4f (期待: 高い)\n', corr(db_clean.SR, db_clean.Fx));
fprintf('    corr(SR, Fy) = %.4f (期待: 低い)\n', corr(db_clean.SR, db_clean.Fy));
fprintf('    mean(Fy) = %.2f N (期待: ≈0)\n', mean(db_clean.Fy));
fprintf('    std(Fy)  = %.2f N (期待: 小さい)\n', std(db_clean.Fy));
fprintf('  Cornering:\n');
fprintf('    corr(SA, Fy) = %.4f (期待: 高い)\n', corr(cr_clean.SA, cr_clean.Fy));
fprintf('    corr(SA, Fx) = %.4f (期待: 低い)\n', corr(cr_clean.SA, cr_clean.Fx));
fprintf('    mean(Fx) = %.2f N (期待: ≈0)\n', mean(cr_clean.Fx));
fprintf('    std(Fx)  = %.2f N (期待: 小さい)\n', std(cr_clean.Fx));
fprintf('\n');

%% ============================================================
%% Step 3: ゼロスリップデータの追加（データ拡張）
%% ============================================================
fprintf('Step 3: ゼロスリップデータの追加\n\n');

% SA=0, SR=0 → Fx=0, Fy=0 の物理制約をデータとして追加
n_zero = 500;  % 追加するゼロスリップサンプル数

% Fz, IA, TSTC の範囲を既存データから取得
Fz_range = [min(db_clean.Fz), max(db_clean.Fz)];
IA_range = [min(db_clean.IA), max(db_clean.IA)];
TSTC_range = [min(db_clean.TSTC), max(db_clean.TSTC)];

rng(42);
zero_Fz = Fz_range(1) + (Fz_range(2) - Fz_range(1)) * rand(n_zero, 1);
zero_IA = IA_range(1) + (IA_range(2) - IA_range(1)) * rand(n_zero, 1);
zero_TSTC = TSTC_range(1) + (TSTC_range(2) - TSTC_range(1)) * rand(n_zero, 1);

zero_data = table( ...
    zeros(n_zero, 1), ...  % SR = 0
    zeros(n_zero, 1), ...  % SA = 0
    zero_Fz, ...           % Fz (ランダム)
    zero_IA, ...           % IA (ランダム)
    zero_TSTC, ...         % TSTC (ランダム)
    zeros(n_zero, 1), ...  % Fx = 0
    zeros(n_zero, 1), ...  % Fy = 0
    zeros(n_zero, 1), ...  % Mx = 0
    zeros(n_zero, 1), ...  % Mz = 0
    'VariableNames', cr_clean.Properties.VariableNames);

% DriveBrakeにゼロスリップデータを追加
db_augmented = [db_clean; zero_data];
fprintf('  DriveBrake: %d → %d サンプル (+%d ゼロスリップ)\n', ...
    height(db_clean), height(db_augmented), n_zero);

% Corneringにもゼロスリップデータを追加
cr_augmented = [cr_clean; zero_data];
fprintf('  Cornering:  %d → %d サンプル (+%d ゼロスリップ)\n', ...
    height(cr_clean), height(cr_augmented), n_zero);

% ※ SR sweep制約データ（Fx=0強制）は撤廃
%   理由: SA=0, SR≠0ではFxはSRに比例する大きな値を取る（0ではない）
%   Fx=0制約がNNのSR感度を殺していた（SR sweepでFx range=7.6N問題の主犯）
%   実データ（Run069/070/072/073）にSA≈0のSR sweepが含まれるため不要

fprintf('  最終DriveBrake: %d サンプル\n', height(db_augmented));
fprintf('\n');

%% ============================================================
%% Step 4: 前処理 & 学習
%% ============================================================
fprintf('Step 4: 前処理 & 学習\n\n');

% GPU チェック
if gpuDeviceCount > 0
    gpu = gpuDevice();
    fprintf('  GPU: %s (%.1f GB)\n\n', gpu.Name, gpu.AvailableMemory / 1e9);
else
    fprintf('  CPU モード\n\n');
end

% --- Cornering ---
fprintf('  === Cornering モデル ===\n');
cr_config = struct('split_ratio', split_ratio);
[cr_X_train, cr_Y_train, cr_X_val, cr_Y_val, cr_norm] = preprocessDataWrapper(cr_augmented, cr_config);
fprintf('  Train: %d, Val: %d\n', size(cr_X_train,1), size(cr_X_val,1));

cr_net = createModel();

cr_train_config = struct();
cr_train_config.max_epochs = 500;
cr_train_config.batch_size = 256;
cr_train_config.learning_rate = 0.0005;
cr_train_config.patience = 300;
cr_train_config.val_frequency = 100;
cr_train_config.random_seed = 42;

tic;
[cr_trained, cr_info] = trainTireModel(cr_net, cr_X_train, cr_Y_train, cr_X_val, cr_Y_val, cr_train_config);
cr_time = toc / 60;
fprintf('  ✓ 学習完了 (%.1f min)\n', cr_time);

cr_metrics = evaluateModel(cr_trained, cr_X_val, cr_Y_val, cr_norm);
fprintf('  Fy: RMSE=%.1fN, R²=%.4f\n', cr_metrics.Fy.RMSE, cr_metrics.Fy.R2);
fprintf('  Mz: RMSE=%.1fNm, R²=%.4f\n\n', cr_metrics.Mz.RMSE, cr_metrics.Mz.R2);

% --- DriveBrake ---
fprintf('  === DriveBrake モデル ===\n');
db_config = struct('split_ratio', split_ratio);
[db_X_train, db_Y_train, db_X_val, db_Y_val, db_norm] = preprocessDataWrapper(db_augmented, db_config);
fprintf('  Train: %d, Val: %d\n', size(db_X_train,1), size(db_X_val,1));

db_net = createModel();

db_train_config = struct();
db_train_config.max_epochs = 500;
db_train_config.batch_size = 256;
db_train_config.learning_rate = 0.0005;
db_train_config.patience = 300;
db_train_config.val_frequency = 100;
db_train_config.random_seed = 42;

tic;
[db_trained, db_info] = trainTireModel(db_net, db_X_train, db_Y_train, db_X_val, db_Y_val, db_train_config);
db_time = toc / 60;
fprintf('  ✓ 学習完了 (%.1f min)\n', db_time);

db_metrics = evaluateModel(db_trained, db_X_val, db_Y_val, db_norm);
fprintf('  Fx: RMSE=%.1fN, R²=%.4f\n\n', db_metrics.Fx.RMSE, db_metrics.Fx.R2);

%% ============================================================
%% Step 5: ゼロスリップ検証
%% ============================================================
fprintf('Step 5: ゼロスリップ検証\n\n');

Fz_test = [200, 500, 800, 1100, 1400];
fprintf('  Fz [N] | Cr:Fx [N] | Cr:Fy [N] | Db:Fx [N] | Db:Fy [N]\n');
fprintf('  -------|-----------|-----------|-----------|----------\n');

for fz = Fz_test
    % 入力: [SR=0, SA=0, Fz, IA=0, TSTC=40]
    X_test = [0, 0, fz, 0, 40];
    
    % 正規化
    X_cr_norm = (X_test - cr_norm.input_mean) ./ cr_norm.input_std;
    X_db_norm = (X_test - db_norm.input_mean) ./ db_norm.input_std;
    
    % 推論
    Y_cr_norm = predict(cr_trained, X_cr_norm);
    Y_db_norm = predict(db_trained, X_db_norm);
    
    % 逆正規化
    Y_cr = double(Y_cr_norm) .* cr_norm.output_std + cr_norm.output_mean;
    Y_db = double(Y_db_norm) .* db_norm.output_std + db_norm.output_mean;
    
    fprintf('  %5.0f | %9.2f | %9.2f | %9.2f | %9.2f\n', ...
        fz, Y_cr(1), Y_cr(2), Y_db(1), Y_db(2));
end

fprintf('\n');

% --- SR sweep (SA=0) での DriveBrake 検証 ---
fprintf('  SR sweep (SA=0, Fz=800N):\n');
fprintf('  SR      | Db:Fx [N] | Db:Fy [N]\n');
fprintf('  --------|-----------|----------\n');

sr_test = -0.2:0.04:0.2;
db_fx_vals = zeros(size(sr_test));
db_fy_vals = zeros(size(sr_test));

for i = 1:length(sr_test)
    X_test = [sr_test(i), 0, 800, 0, 40];
    X_norm = (X_test - db_norm.input_mean) ./ db_norm.input_std;
    Y_norm = predict(db_trained, X_norm);
    Y = double(Y_norm) .* db_norm.output_std + db_norm.output_mean;
    db_fx_vals(i) = Y(1);
    db_fy_vals(i) = Y(2);
    fprintf('  %7.3f | %9.2f | %9.2f\n', sr_test(i), Y(1), Y(2));
end

fprintf('\n  Fx range: %.1f N (期待: 大きい)\n', max(db_fx_vals) - min(db_fx_vals));
fprintf('  Fy range: %.1f N (期待: ≈0)\n', max(db_fy_vals) - min(db_fy_vals));
fprintf('  |Fy| mean: %.1f N (期待: <50N)\n\n', mean(abs(db_fy_vals)));

%% ============================================================
%% Step 6: 保存
%% ============================================================
fprintf('Step 6: 保存\n\n');

if ~exist('output', 'dir'), mkdir('output'); end

cr_save = sprintf('output/cornering_%s.mat', tire_model);
save(cr_save, 'cr_trained', 'cr_norm', 'cr_metrics', 'cr_info', 'tire_model');
fprintf('  ✓ %s\n', cr_save);

db_save = sprintf('output/drivebrake_%s.mat', tire_model);
save(db_save, 'db_trained', 'db_norm', 'db_metrics', 'db_info', 'tire_model');
fprintf('  ✓ %s\n\n', db_save);

%% ============================================================
%% Step 7: クリーニング前後の比較サマリー
%% ============================================================
fprintf('=================================================================\n');
fprintf(' クリーニング前後の比較\n');
fprintf('=================================================================\n\n');

fprintf('  --- データ量 ---\n');
fprintf('  Cornering:  %d → %d (%.1f%%除去, SR異常値)\n', ...
    height(cr_raw), height(cr_clean), 100*(1-height(cr_clean)/height(cr_raw)));
fprintf('  DriveBrake: %d サンプル (Run選別: 除外=%s)\n', ...
    height(db_clean), strjoin(db_exclude_runs, ', '));
fprintf('\n');

fprintf('  --- 精度 ---\n');
fprintf('  Cornering Fy:  RMSE=%.1fN, R²=%.4f\n', cr_metrics.Fy.RMSE, cr_metrics.Fy.R2);
fprintf('  Cornering Mz:  RMSE=%.1fNm, R²=%.4f\n', cr_metrics.Mz.RMSE, cr_metrics.Mz.R2);
fprintf('  DriveBrake Fx: RMSE=%.1fN, R²=%.4f\n', db_metrics.Fx.RMSE, db_metrics.Fx.R2);
fprintf('\n');

fprintf('=================================================================\n');
fprintf(' 完了 - validate_zero_slip_behavior.m で再検証してください\n');
fprintf('=================================================================\n');


%% ============================================================
%% ローカル関数
%% ============================================================

function net = createModel()
    % 8層モデル: 256-256-128-128-64-64-32-32
    layers = [
        featureInputLayer(5, 'Name', 'input', 'Normalization', 'none')
        
        fullyConnectedLayer(256, 'Name', 'fc1')
        batchNormalizationLayer('Name', 'bn1')
        reluLayer('Name', 'relu1')
        dropoutLayer(0.15, 'Name', 'drop1')
        
        fullyConnectedLayer(256, 'Name', 'fc2')
        batchNormalizationLayer('Name', 'bn2')
        reluLayer('Name', 'relu2')
        dropoutLayer(0.15, 'Name', 'drop2')
        
        fullyConnectedLayer(128, 'Name', 'fc3')
        batchNormalizationLayer('Name', 'bn3')
        reluLayer('Name', 'relu3')
        dropoutLayer(0.12, 'Name', 'drop3')
        
        fullyConnectedLayer(128, 'Name', 'fc4')
        batchNormalizationLayer('Name', 'bn4')
        reluLayer('Name', 'relu4')
        dropoutLayer(0.12, 'Name', 'drop4')
        
        fullyConnectedLayer(64, 'Name', 'fc5')
        batchNormalizationLayer('Name', 'bn5')
        reluLayer('Name', 'relu5')
        dropoutLayer(0.1, 'Name', 'drop5')
        
        fullyConnectedLayer(64, 'Name', 'fc6')
        batchNormalizationLayer('Name', 'bn6')
        reluLayer('Name', 'relu6')
        dropoutLayer(0.1, 'Name', 'drop6')
        
        fullyConnectedLayer(32, 'Name', 'fc7')
        batchNormalizationLayer('Name', 'bn7')
        reluLayer('Name', 'relu7')
        
        fullyConnectedLayer(32, 'Name', 'fc8')
        batchNormalizationLayer('Name', 'bn8')
        reluLayer('Name', 'relu8')
        
        fullyConnectedLayer(4, 'Name', 'output')
        regressionLayer('Name', 'regression')
    ];
    
    net = layerGraph(layers);
end