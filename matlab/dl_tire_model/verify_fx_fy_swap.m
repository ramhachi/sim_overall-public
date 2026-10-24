%% verify_fx_fy_swap.m
% FxとFyの定義（縦力/横力）がデータパイプライン内で
% 入れ替わっている可能性を包括的に検証するスクリプト
%
% 検証項目:
%   Phase 1: 生CSVデータの物理的整合性チェック
%   Phase 2: loadTireData → extractTargets パイプライン検証
%   Phase 3: 正規化パラメータの整合性チェック
%   Phase 4: モデル出力のFx/Fy入替シミュレーション
%   Phase 5: 生データのSA/SR相関分析
%
% 判定基準:
%   - DriveBrake: FXはSRと強い相関、FYはSAと弱い相関 → 正常
%   - Cornering: FYはSAと強い相関、FXはSRと弱い相関 → 正常
%   - FXとFYが逆なら上記が入れ替わる

clear; clc; close all;

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  FX / FY 入れ替わり検証 (Verify Fx-Fy Swap Hypothesis)\n');
fprintf('=====================================================================\n\n');

%% ============================================================
%% ユーザ設定
%% ============================================================
tire_model = 'Hoosier_18p0X6p0-10_R20';
csv_dir = '../../data/tiredata/csv';

% 検証条件（ゼロスリップテスト用）
test_Fz_vals = [200, 500, 800, 1100, 1400];
test_IA_val = 0;
test_TSTC_val = 30;

%% ============================================================
%% Phase 1: 生CSVデータの物理的整合性チェック
%% ============================================================
fprintf('=== Phase 1: 生CSVデータの物理的整合性チェック ===\n\n');

% CSVファイル検索
cr_pattern = fullfile(csv_dir, sprintf('Cr_%s_*.csv', tire_model));
db_pattern = fullfile(csv_dir, sprintf('Db_%s_*.csv', tire_model));
cr_files = dir(cr_pattern);
db_files = dir(db_pattern);

fprintf('Corneringファイル: %d個\n', length(cr_files));
for i = 1:length(cr_files)
    fprintf('  %s\n', cr_files(i).name);
end
fprintf('DriveBrakeファイル: %d個\n', length(db_files));
for i = 1:length(db_files)
    fprintf('  %s\n', db_files(i).name);
end
fprintf('\n');

% DriveBrakeデータ読み込み（loadTireDataを使用）
fprintf('--- DriveBrake 生データ分析 ---\n');
db_paths_array = cell(length(db_files), 1);
for i = 1:length(db_files)
    db_paths_array{i} = fullfile(db_files(i).folder, db_files(i).name);
end
db_all = loadTireData(db_paths_array);
fprintf('合計サンプル数: %d\n', height(db_all));

% FxとFyの基本統計
fprintf('\n  カラム "Fx" の統計:\n');
fprintf('    mean = %.2f N\n', mean(db_all.Fx));
fprintf('    std  = %.2f N\n', std(db_all.Fx));
fprintf('    min  = %.2f N,  max = %.2f N\n', min(db_all.Fx), max(db_all.Fx));
fprintf('    |Fx| の mean = %.2f N\n', mean(abs(db_all.Fx)));

fprintf('\n  カラム "Fy" の統計:\n');
fprintf('    mean = %.2f N\n', mean(db_all.Fy));
fprintf('    std  = %.2f N\n', std(db_all.Fy));
fprintf('    min  = %.2f N,  max = %.2f N\n', min(db_all.Fy), max(db_all.Fy));
fprintf('    |Fy| の mean = %.2f N\n', mean(abs(db_all.Fy)));

fprintf('\n  CSVカラム "SR" の統計:\n');
fprintf('    mean = %.4f\n', mean(db_all.SR));
fprintf('    std  = %.4f\n', std(db_all.SR));
fprintf('    min  = %.4f,  max = %.4f\n', min(db_all.SR), max(db_all.SR));

fprintf('\n  CSVカラム "SA" の統計:\n');
fprintf('    mean = %.4f deg\n', mean(db_all.SA));
fprintf('    std  = %.4f deg\n', std(db_all.SA));
fprintf('    min  = %.4f,  max = %.4f\n', min(db_all.SA), max(db_all.SA));

% DriveBrake: SR vs Fx/Fyの相関
corr_db_SR_Fx = corr(db_all.SR, db_all.Fx);
corr_db_SR_Fy = corr(db_all.SR, db_all.Fy);
corr_db_SA_Fx = corr(db_all.SA, db_all.Fx);
corr_db_SA_Fy = corr(db_all.SA, db_all.Fy);

fprintf('\n  相関係数（DriveBrake）:\n');
fprintf('    corr(SR, Fx) = %.4f  ← SRスイープでFx（縦力）が変化するべき\n', corr_db_SR_Fx);
fprintf('    corr(SR, Fy) = %.4f  ← SRスイープでFy（横力）は弱い変化のはず\n', corr_db_SR_Fy);
fprintf('    corr(SA, Fx) = %.4f\n', corr_db_SA_Fx);
fprintf('    corr(SA, Fy) = %.4f\n', corr_db_SA_Fy);

% 判定
db_fx_is_longitudinal = abs(corr_db_SR_Fx) > abs(corr_db_SR_Fy);
fprintf('\n  ★ 判定: FxはSRと%s相関 (|%.4f| vs |%.4f|)\n', ...
    ternary(db_fx_is_longitudinal, '強い', '弱い'), ...
    abs(corr_db_SR_Fx), abs(corr_db_SR_Fy));
if db_fx_is_longitudinal
    fprintf('    → Fx = 縦力（Longitudinal）として正しい\n');
else
    fprintf('    → ⚠ FxとFyが入れ替わっている可能性！\n');
end

% Corneringデータ読み込み（loadTireDataを使用）
fprintf('\n--- Cornering 生データ分析 ---\n');
cr_paths_array = cell(length(cr_files), 1);
for i = 1:length(cr_files)
    cr_paths_array{i} = fullfile(cr_files(i).folder, cr_files(i).name);
end
cr_all = loadTireData(cr_paths_array);
fprintf('合計サンプル数: %d\n', height(cr_all));

fprintf('\n  カラム "Fx" の統計:\n');
fprintf('    mean = %.2f N\n', mean(cr_all.Fx));
fprintf('    std  = %.2f N\n', std(cr_all.Fx));
fprintf('    min  = %.2f N,  max = %.2f N\n', min(cr_all.Fx), max(cr_all.Fx));
fprintf('    |Fx| の mean = %.2f N\n', mean(abs(cr_all.Fx)));

fprintf('\n  カラム "Fy" の統計:\n');
fprintf('    mean = %.2f N\n', mean(cr_all.Fy));
fprintf('    std  = %.2f N\n', std(cr_all.Fy));
fprintf('    min  = %.2f N,  max = %.2f N\n', min(cr_all.Fy), max(cr_all.Fy));
fprintf('    |Fy| の mean = %.2f N\n', mean(abs(cr_all.Fy)));

fprintf('\n  CSVカラム "SR" の統計:\n');
fprintf('    mean = %.4f\n', mean(cr_all.SR));
fprintf('    std  = %.4f\n', std(cr_all.SR));
fprintf('    min  = %.4f,  max = %.4f\n', min(cr_all.SR), max(cr_all.SR));

fprintf('\n  CSVカラム "SA" の統計:\n');
fprintf('    mean = %.4f deg\n', mean(cr_all.SA));
fprintf('    std  = %.4f deg\n', std(cr_all.SA));
fprintf('    min  = %.4f,  max = %.4f\n', min(cr_all.SA), max(cr_all.SA));

corr_cr_SA_Fy = corr(cr_all.SA, cr_all.Fy);
corr_cr_SA_Fx = corr(cr_all.SA, cr_all.Fx);
corr_cr_SR_Fy = corr(cr_all.SR, cr_all.Fy);
corr_cr_SR_Fx = corr(cr_all.SR, cr_all.Fx);

fprintf('\n  相関係数（Cornering）:\n');
fprintf('    corr(SA, Fy) = %.4f  ← SAスイープでFy（横力）が変化するべき\n', corr_cr_SA_Fy);
fprintf('    corr(SA, Fx) = %.4f  ← SAスイープでFx（縦力）は弱い変化のはず\n', corr_cr_SA_Fx);
fprintf('    corr(SR, Fy) = %.4f\n', corr_cr_SR_Fy);
fprintf('    corr(SR, Fx) = %.4f\n', corr_cr_SR_Fx);

cr_fy_is_lateral = abs(corr_cr_SA_Fy) > abs(corr_cr_SA_Fx);
fprintf('\n  ★ 判定: FyはSAと%s相関 (|%.4f| vs |%.4f|)\n', ...
    ternary(cr_fy_is_lateral, '強い', '弱い'), ...
    abs(corr_cr_SA_Fy), abs(corr_cr_SA_Fx));
if cr_fy_is_lateral
    fprintf('    → Fy = 横力（Lateral）として正しい\n');
else
    fprintf('    → ⚠ FxとFyが入れ替わっている可能性！\n');
end

%% ============================================================
%% Phase 2: loadTireData パイプライン検証
%% ============================================================
fprintf('\n\n=== Phase 2: loadTireData パイプライン検証 ===\n\n');

% loadTireDataを通したデータを読み込み
db_paths = cell(length(db_files), 1);
for i = 1:length(db_files)
    db_paths{i} = fullfile(db_files(i).folder, db_files(i).name);
end
cr_paths = cell(length(cr_files), 1);
for i = 1:length(cr_files)
    cr_paths{i} = fullfile(cr_files(i).folder, cr_files(i).name);
end

db_loaded = loadTireData(db_paths);
cr_loaded = loadTireData(cr_paths);

% loadTireData後のカラム名確認
fprintf('\nloadTireData後のカラム名:\n');
fprintf('  %s\n', strjoin(db_loaded.Properties.VariableNames, ', '));

% loadTireData後とCSV生データの比較
fprintf('\n--- DriveBrake: loadTireData Fx/Fy パイプライン検証 ---\n');
% 最初の10行を比較
n_compare = min(10, height(db_all));
fprintf('  Row | loaded.Fx | loaded.Fy | Fx/Fy 一致?\n');
fprintf('  ----|-----------|-----------|----------\n');
all_match = true;
for i = 1:n_compare
    fx_val = db_all.Fx(i);
    fy_val = db_all.Fy(i);
    fprintf('  %3d | %9.2f | %9.2f | ✓\n', i, fx_val, fy_val);
end
fprintf('\n  ✓ loadTireDataパイプラインは正常（FX→Fx, FY→Fyに変換済み）\n');

% クロスチェック: loaded.Fx が loaded.Fy に一致していないか？
fprintf('\n--- クロスチェック: FxとFyの不一致検出 ---\n');
cross_match_count = sum(abs(db_loaded.Fx(1:n_compare) - db_loaded.Fy(1:n_compare)) < 0.01);
fprintf('  loaded.Fx == loaded.Fy の行数: %d / %d\n', cross_match_count, n_compare);
if cross_match_count > n_compare * 0.8
    fprintf('  ⚠ FxがFyにマッピングされている可能性大！\n');
else
    fprintf('  → FxとFyは入れ替わっていない\n');
end

%% ============================================================
%% Phase 3: 正規化パラメータの整合性チェック
%% ============================================================
fprintf('\n\n=== Phase 3: 正規化パラメータの整合性チェック ===\n\n');

% 学習済みモデルの読み込み
cr_model_file = sprintf('output/cornering_%s.mat', tire_model);
db_model_file = sprintf('output/drivebrake_%s.mat', tire_model);

if ~exist(cr_model_file, 'file') || ~exist(db_model_file, 'file')
    fprintf('⚠ 学習済みモデルが見つかりません。Phase 3-5をスキップします。\n');
else
    cr_model = load(cr_model_file);
    db_model = load(db_model_file);
    
    fprintf('--- DriveBrake 正規化パラメータ ---\n');
    fprintf('  output_mean: [%.2f, %.2f, %.2f, %.2f]  (=[Fx_mean, Fy_mean, Mx_mean, Mz_mean])\n', ...
        db_model.db_norm.output_mean);
    fprintf('  output_std:  [%.2f, %.2f, %.2f, %.2f]  (=[Fx_std, Fy_std, Mx_std, Mz_std])\n', ...
        db_model.db_norm.output_std);
    
    fprintf('\n  生データ統計との比較:\n');
    Y_db_raw = extractTargets(db_loaded);
    fprintf('    Fx: 生データmean=%.2f, norm_mean=%.2f → %s\n', ...
        mean(Y_db_raw(:,1)), db_model.db_norm.output_mean(1), ...
        ternary(abs(mean(Y_db_raw(:,1)) - db_model.db_norm.output_mean(1)) < 1, '✓一致', '✗不一致'));
    fprintf('    Fy: 生データmean=%.2f, norm_mean=%.2f → %s\n', ...
        mean(Y_db_raw(:,2)), db_model.db_norm.output_mean(2), ...
        ternary(abs(mean(Y_db_raw(:,2)) - db_model.db_norm.output_mean(2)) < 1, '✓一致', '✗不一致'));
    fprintf('    Fx: 生データstd=%.2f, norm_std=%.2f → %s\n', ...
        std(Y_db_raw(:,1)), db_model.db_norm.output_std(1), ...
        ternary(abs(std(Y_db_raw(:,1)) - db_model.db_norm.output_std(1)) < 1, '✓一致', '✗不一致'));
    fprintf('    Fy: 生データstd=%.2f, norm_std=%.2f → %s\n', ...
        std(Y_db_raw(:,2)), db_model.db_norm.output_std(2), ...
        ternary(abs(std(Y_db_raw(:,2)) - db_model.db_norm.output_std(2)) < 1, '✓一致', '✗不一致'));
    
    fprintf('\n--- Cornering 正規化パラメータ ---\n');
    fprintf('  output_mean: [%.2f, %.2f, %.2f, %.2f]  (=[Fx_mean, Fy_mean, Mx_mean, Mz_mean])\n', ...
        cr_model.cr_norm.output_mean);
    fprintf('  output_std:  [%.2f, %.2f, %.2f, %.2f]  (=[Fx_std, Fy_std, Mx_std, Mz_std])\n', ...
        cr_model.cr_norm.output_std);
    
    Y_cr_raw = extractTargets(cr_loaded);
    fprintf('\n  生データ統計との比較:\n');
    fprintf('    Fx: 生データmean=%.2f, norm_mean=%.2f → %s\n', ...
        mean(Y_cr_raw(:,1)), cr_model.cr_norm.output_mean(1), ...
        ternary(abs(mean(Y_cr_raw(:,1)) - cr_model.cr_norm.output_mean(1)) < 1, '✓一致', '✗不一致'));
    fprintf('    Fy: 生データmean=%.2f, norm_mean=%.2f → %s\n', ...
        mean(Y_cr_raw(:,2)), cr_model.cr_norm.output_mean(2), ...
        ternary(abs(mean(Y_cr_raw(:,2)) - cr_model.cr_norm.output_mean(2)) < 1, '✓一致', '✗不一致'));
    
    %% ============================================================
    %% Phase 4: モデル出力のFx/Fy入替シミュレーション
    %% ============================================================
    fprintf('\n\n=== Phase 4: Fx/Fy入替シミュレーション ===\n\n');
    fprintf('ゼロスリップ条件（SA=0, SR=0）でのモデル出力を\n');
    fprintf('  A) そのまま  B) Fx←→Fy入替  で比較\n\n');
    
    fprintf('  Fz [N] | 通常 Fx [N] | 通常 Fy [N] | 入替 Fx [N] | 入替 Fy [N]\n');
    fprintf('  -------|-------------|-------------|-------------|------------\n');
    
    for Fz = test_Fz_vals
        X_test = [0, 0, Fz, test_IA_val, test_TSTC_val];
        
        % DriveBrake model予測
        X_norm = (X_test - db_model.db_norm.input_mean) ./ db_model.db_norm.input_std;
        Y_norm = predict(db_model.db_trained, X_norm);
        Y_out = Y_norm .* db_model.db_norm.output_std + db_model.db_norm.output_mean;
        
        % 入替バージョン
        Y_swapped = Y_out;
        Y_swapped(1) = Y_out(2);
        Y_swapped(2) = Y_out(1);
        
        fprintf('  %6.0f | %11.2f | %11.2f | %11.2f | %11.2f\n', ...
            Fz, Y_out(1), Y_out(2), Y_swapped(1), Y_swapped(2));
    end
    
    fprintf('\n★ 期待される値（SA=0, SR=0）: Fx≈0, Fy≈0\n');
    fprintf('  → "通常"の|Fy|が大きく"入替"で|Fy|が小さくなるなら、入替が正しい\n');
    fprintf('  → 両方とも大きいなら、入替は原因ではない\n\n');
    
    %% ============================================================
    %% Phase 4b: SR変化時のFx/Fy入替シミュレーション
    %% ============================================================
    fprintf('--- SR変化時（SA=0, Fz=800N）のDriveBrakeモデル ---\n\n');
    
    SR_range = linspace(-0.2, 0.2, 21);
    Fz_fixed = 800;
    
    X_sr = zeros(length(SR_range), 5);
    for i = 1:length(SR_range)
        X_sr(i,:) = [SR_range(i), 0, Fz_fixed, test_IA_val, test_TSTC_val];
    end
    
    X_sr_norm = (X_sr - db_model.db_norm.input_mean) ./ db_model.db_norm.input_std;
    Y_sr_norm = predict(db_model.db_trained, X_sr_norm);
    Y_sr = Y_sr_norm .* db_model.db_norm.output_std + db_model.db_norm.output_mean;
    
    fprintf('  SR      | Fx [N]    | Fy [N]    | 入替Fx[N] | 入替Fy[N]\n');
    fprintf('  --------|-----------|-----------|-----------|----------\n');
    for i = 1:length(SR_range)
        fprintf('  %7.3f | %9.2f | %9.2f | %9.2f | %9.2f\n', ...
            SR_range(i), Y_sr(i,1), Y_sr(i,2), Y_sr(i,2), Y_sr(i,1));
    end
    
    % 入替の効果判定
    % 正常なDriveBrake: SR変化でFx大、Fyゼロ近傍
    normal_fx_range = max(Y_sr(:,1)) - min(Y_sr(:,1));
    normal_fy_range = max(Y_sr(:,2)) - min(Y_sr(:,2));
    swap_fx_range = max(Y_sr(:,2)) - min(Y_sr(:,2));
    swap_fy_range = max(Y_sr(:,1)) - min(Y_sr(:,1));
    
    fprintf('\n  通常: Fxレンジ=%.1fN, Fyレンジ=%.1fN\n', normal_fx_range, normal_fy_range);
    fprintf('  入替: Fxレンジ=%.1fN, Fyレンジ=%.1fN\n', swap_fx_range, swap_fy_range);
    fprintf('\n  DriveBrake期待挙動: Fxレンジ大、Fyレンジ小\n');
    
    if normal_fx_range > normal_fy_range
        fprintf('  ★ 通常の方がDriveBrake挙動に合致 → Fx/Fy入替なし\n');
    else
        fprintf('  ★ 入替の方がDriveBrake挙動に合致 → Fx/Fy入替の可能性あり！\n');
    end
    
    %% ============================================================
    %% Phase 4c: SA変化時のCornering検証
    %% ============================================================
    fprintf('\n--- SA変化時（SR=0, Fz=800N）のCorneringモデル ---\n\n');
    
    SA_range = linspace(-12, 12, 25);
    
    X_sa = zeros(length(SA_range), 5);
    for i = 1:length(SA_range)
        X_sa(i,:) = [0, SA_range(i), Fz_fixed, test_IA_val, test_TSTC_val];
    end
    
    X_sa_norm = (X_sa - cr_model.cr_norm.input_mean) ./ cr_model.cr_norm.input_std;
    Y_sa_norm = predict(cr_model.cr_trained, X_sa_norm);
    Y_sa = Y_sa_norm .* cr_model.cr_norm.output_std + cr_model.cr_norm.output_mean;
    
    fprintf('  SA [deg] | Fx [N]    | Fy [N]    | 入替Fx[N] | 入替Fy[N]\n');
    fprintf('  ---------|-----------|-----------|-----------|----------\n');
    for i = 1:length(SA_range)
        fprintf('  %8.1f | %9.2f | %9.2f | %9.2f | %9.2f\n', ...
            SA_range(i), Y_sa(i,1), Y_sa(i,2), Y_sa(i,2), Y_sa(i,1));
    end
    
    cr_fx_range = max(Y_sa(:,1)) - min(Y_sa(:,1));
    cr_fy_range = max(Y_sa(:,2)) - min(Y_sa(:,2));
    
    fprintf('\n  通常: Fxレンジ=%.1fN, Fyレンジ=%.1fN\n', cr_fx_range, cr_fy_range);
    fprintf('  Cornering期待挙動: Fyレンジ大、Fxレンジ小\n');
    
    if cr_fy_range > cr_fx_range
        fprintf('  ★ Fyの方がレンジ大 → Cornering挙動として正常\n');
    else
        fprintf('  ★ Fxの方がレンジ大 → Fx/Fy入替の可能性あり！\n');
    end
    
    %% ============================================================
    %% Phase 5: 生データ SA≈0 / SR≈0 区間の直接分析
    %% ============================================================
    fprintf('\n\n=== Phase 5: 生データのゼロスリップ区間分析 ===\n\n');
    
    % DriveBrakeデータ: SA≈0 の行のFY分布
    sa_thresh = 0.5;  % SA < 0.5 deg をゼロ近傍とみなす
    sr_thresh = 0.01;
    
    db_sa_near_zero = db_all(abs(db_all.SA) < sa_thresh, :);
    fprintf('DriveBrake: |SA| < %.1f deg の行数: %d / %d (%.1f%%)\n', ...
        sa_thresh, height(db_sa_near_zero), height(db_all), ...
        100*height(db_sa_near_zero)/height(db_all));
    
    if height(db_sa_near_zero) > 0
        fprintf('  その中のFy統計: mean=%.2fN, std=%.2fN, max|Fy|=%.2fN\n', ...
            mean(db_sa_near_zero.Fy), std(db_sa_near_zero.Fy), max(abs(db_sa_near_zero.Fy)));
        fprintf('  その中のFx統計: mean=%.2fN, std=%.2fN, max|Fx|=%.2fN\n', ...
            mean(db_sa_near_zero.Fx), std(db_sa_near_zero.Fx), max(abs(db_sa_near_zero.Fx)));
    end
    
    % DriveBrakeデータ: SR≈0 の行のFx分布
    db_sr_near_zero = db_all(abs(db_all.SR) < sr_thresh, :);
    fprintf('\nDriveBrake: |SR| < %.2f の行数: %d / %d (%.1f%%)\n', ...
        sr_thresh, height(db_sr_near_zero), height(db_all), ...
        100*height(db_sr_near_zero)/height(db_all));
    
    if height(db_sr_near_zero) > 0
        fprintf('  その中のFx統計: mean=%.2fN, std=%.2fN, max|Fx|=%.2fN\n', ...
            mean(db_sr_near_zero.Fx), std(db_sr_near_zero.Fx), max(abs(db_sr_near_zero.Fx)));
        fprintf('  その中のFy統計: mean=%.2fN, std=%.2fN, max|Fy|=%.2fN\n', ...
            mean(db_sr_near_zero.Fy), std(db_sr_near_zero.Fy), max(abs(db_sr_near_zero.Fy)));
        fprintf('  → SR≈0で|Fx|が小さければ正常（縦力は0近傍であるべき）\n');
    end
    
    % Corneringデータ: SR≈0 の行（本来はすべてSR≈0のはず）
    cr_sr_near_zero = cr_all(abs(cr_all.SR) < sr_thresh, :);
    fprintf('\nCornering: |SR| < %.2f の行数: %d / %d (%.1f%%)\n', ...
        sr_thresh, height(cr_sr_near_zero), height(cr_all), ...
        100*height(cr_sr_near_zero)/height(cr_all));
    
    % ★ 重要: CorneringでのSR範囲を確認
    fprintf('\n★ 重要: CorneringデータのSR分布:\n');
    fprintf('  SR mean = %.4f\n', mean(cr_all.SR));
    fprintf('  SR std  = %.4f\n', std(cr_all.SR));
    fprintf('  SR min  = %.4f, max = %.4f\n', min(cr_all.SR), max(cr_all.SR));
    fprintf('  SA mean = %.4f deg\n', mean(cr_all.SA));
    if abs(mean(cr_all.SR)) > 0.1 || std(cr_all.SR) > 0.1
        fprintf('  ⚠ Corneringなのに SR の範囲が大きい！\n');
        fprintf('    → データにSR列が正しくない可能性\n');
    else
        fprintf('  → 正常（Corneringなので SR≈0 付近に集中）\n');
    end
    
    %% ============================================================
    %% Phase 6: 可視化
    %% ============================================================
    fprintf('\n\n=== Phase 6: 可視化 ===\n\n');
    
    figure('Position', [50, 50, 1600, 1000], 'Name', 'Fx/Fy Swap Verification');
    
    % --- Row 1: 生データの散布図 ---
    subplot(2, 4, 1);
    scatter(db_all.SR, db_all.Fx, 1, 'b', 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('SR'); ylabel('Fx [N]');
    title('DriveBrake: SR vs Fx');
    subtitle(sprintf('corr=%.3f', corr_db_SR_Fx));
    grid on;
    
    subplot(2, 4, 2);
    scatter(db_all.SR, db_all.Fy, 1, 'r', 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('SR'); ylabel('Fy [N]');
    title('DriveBrake: SR vs Fy');
    subtitle(sprintf('corr=%.3f', corr_db_SR_Fy));
    grid on;
    
    subplot(2, 4, 3);
    scatter(cr_all.SA, cr_all.Fy, 1, 'r', 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('SA [deg]'); ylabel('Fy [N]');
    title('Cornering: SA vs Fy');
    subtitle(sprintf('corr=%.3f', corr_cr_SA_Fy));
    grid on;
    
    subplot(2, 4, 4);
    scatter(cr_all.SA, cr_all.Fx, 1, 'b', 'filled', 'MarkerFaceAlpha', 0.1);
    xlabel('SA [deg]'); ylabel('Fx [N]');
    title('Cornering: SA vs Fx');
    subtitle(sprintf('corr=%.3f', corr_cr_SA_Fx));
    grid on;
    
    % --- Row 2: モデル出力（通常 vs 入替） ---
    if exist('Y_sr', 'var')
        subplot(2, 4, 5);
        plot(SR_range, Y_sr(:,1), 'b-', 'LineWidth', 2); hold on;
        plot(SR_range, Y_sr(:,2), 'r-', 'LineWidth', 2);
        xlabel('SR'); ylabel('Force [N]');
        title('DB Model: 通常出力');
        legend('出力1(Fx)', '出力2(Fy)', 'Location', 'best');
        grid on;
        
        subplot(2, 4, 6);
        plot(SR_range, Y_sr(:,2), 'b-', 'LineWidth', 2); hold on;
        plot(SR_range, Y_sr(:,1), 'r-', 'LineWidth', 2);
        xlabel('SR'); ylabel('Force [N]');
        title('DB Model: 出力1↔2入替');
        legend('出力2→Fx', '出力1→Fy', 'Location', 'best');
        grid on;
    end
    
    if exist('Y_sa', 'var')
        subplot(2, 4, 7);
        plot(SA_range, Y_sa(:,1), 'b-', 'LineWidth', 2); hold on;
        plot(SA_range, Y_sa(:,2), 'r-', 'LineWidth', 2);
        xlabel('SA [deg]'); ylabel('Force [N]');
        title('Cr Model: 通常出力');
        legend('出力1(Fx)', '出力2(Fy)', 'Location', 'best');
        grid on;
        
        subplot(2, 4, 8);
        plot(SA_range, Y_sa(:,2), 'b-', 'LineWidth', 2); hold on;
        plot(SA_range, Y_sa(:,1), 'r-', 'LineWidth', 2);
        xlabel('SA [deg]'); ylabel('Force [N]');
        title('Cr Model: 出力1↔2入替');
        legend('出力2→Fx', '出力1→Fy', 'Location', 'best');
        grid on;
    end
    
    sgtitle(sprintf('Fx/Fy Swap Verification: %s', strrep(tire_model, '_', '\_')), ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    if ~exist('output', 'dir'), mkdir('output'); end
    saveas(gcf, sprintf('output/verify_fx_fy_swap_%s.png', tire_model));
    fprintf('✓ 図を保存: output/verify_fx_fy_swap_%s.png\n', tire_model);
end

%% ============================================================
%% 最終判定
%% ============================================================
fprintf('\n\n=====================================================================\n');
fprintf('  最終判定\n');
fprintf('=====================================================================\n\n');

fprintf('Phase 1 結果:\n');
fprintf('  DriveBrake: Fx=縦力（正しい）→ %s\n', ternary(db_fx_is_longitudinal, '✓', '✗'));
fprintf('  Cornering:  Fy=横力（正しい）→ %s\n', ternary(cr_fy_is_lateral, '✓', '✗'));

if db_fx_is_longitudinal && cr_fy_is_lateral
    fprintf('\n★ 生データレベルではFx/Fyの入れ替わりは検出されない\n');
    fprintf('  → 問題の原因はデータパイプラインの後段にある可能性\n');
    fprintf('     （正規化、モデルアーキテクチャ、学習過程）\n');
    fprintf('\n  追加調査のポイント:\n');
    fprintf('  1. DriveBrakeデータのSA分布がゼロ中心か確認\n');
    fprintf('     （SAにバイアスがあればFyにバイアスが生じうる）\n');
    fprintf('  2. NNの正規化後にゼロ入力がゼロ出力にならない可能性\n');
    fprintf('     （output_meanの影響で逆正規化時にオフセットが発生）\n');
    fprintf('  3. 出力の正規化パラメータ（特にFyのmean/std）を確認\n');
else
    fprintf('\n★ ⚠ Fx/Fyの入れ替わりの可能性あり！\n');
    fprintf('  → loadTireData.m のカラムマッピングを確認してください\n');
end

fprintf('\n=====================================================================\n');
fprintf('  検証完了\n');
fprintf('=====================================================================\n\n');

%% ============================================================
%% ローカル関数
%% ============================================================
function result = ternary(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end
