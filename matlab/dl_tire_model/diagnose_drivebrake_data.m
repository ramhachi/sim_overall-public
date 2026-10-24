%% diagnose_drivebrake_data.m
% DriveBrakeデータの詳細分析
%
% 目的:
%   - SA分布の実態を把握（なぜ67%が除去されたか）
%   - Run毎のSA/SR分布を確認（複合スリップRunの特定）
%   - SA閾値ごとのデータ残存量と物理整合性を比較
%   - 最適なフィルタリング戦略を決定
%
% clean_and_retrain.m での問題:
%   - db_sa_threshold=1.0 → 67%除去 → SR感度が喪失（Fx range=7.6N）

clear; clc; close all;

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  DriveBrake データ診断 (Diagnose DriveBrake Data)\n');
fprintf('=====================================================================\n\n');

%% ============================================================
%% データ読み込み
%% ============================================================
tire_model = 'Hoosier_18p0X6p0-10_R20';
data_dir = '../../data/tiredata/csv';

db_pattern = fullfile(data_dir, ['Db_' tire_model '*.csv']);
db_found = dir(db_pattern);
db_names = sort({db_found.name}');

fprintf('DriveBrakeファイル: %d個\n\n', length(db_names));

%% ============================================================
%% Phase 1: Run毎の分析
%% ============================================================
fprintf('=== Phase 1: Run毎のSA/SR分布分析 ===\n\n');

fprintf('  Run名                                       | サンプル数 | SA mean | SA std | SA range          | SR mean | SR std | SR range\n');
fprintf('  ---------------------------------------------|-----------|---------|--------|-------------------|---------|--------|------------------\n');

run_data = {};
run_labels = {};
for i = 1:length(db_names)
    fpath = fullfile(data_dir, db_names{i});
    data = loadTireData({fpath});
    run_data{i} = data;
    run_labels{i} = db_names{i};
    
    fprintf('  %-44s | %9d | %7.2f | %6.2f | [%6.2f, %6.2f] | %7.4f | %6.4f | [%7.4f, %7.4f]\n', ...
        db_names{i}, height(data), ...
        mean(data.SA), std(data.SA), min(data.SA), max(data.SA), ...
        mean(data.SR), std(data.SR), min(data.SR), max(data.SR));
end
fprintf('\n');

%% ============================================================
%% Phase 2: SA分布のヒストグラム（全体 + Run毎）
%% ============================================================
fprintf('=== Phase 2: SA分布の詳細分析 ===\n\n');

% 全データ結合
db_all_paths = cellfun(@(f) fullfile(data_dir, f), db_names, 'UniformOutput', false);
db_all = loadTireData(db_all_paths);

% SA分布の分位点
sa_vals = db_all.SA;
fprintf('  SA分布:\n');
fprintf('    サンプル数: %d\n', length(sa_vals));
fprintf('    mean = %.4f deg\n', mean(sa_vals));
fprintf('    std  = %.4f deg\n', std(sa_vals));
fprintf('    中央値 = %.4f deg\n', median(sa_vals));
fprintf('    5%%ile = %.2f, 25%%ile = %.2f, 75%%ile = %.2f, 95%%ile = %.2f\n', ...
    prctile(sa_vals, 5), prctile(sa_vals, 25), prctile(sa_vals, 75), prctile(sa_vals, 95));
fprintf('\n');

% SAのユニーク値（離散的か連続的か）
unique_sa = unique(round(sa_vals, 1));
fprintf('  SA（0.1度刻み）のユニーク値数: %d\n', length(unique_sa));
if length(unique_sa) < 50
    fprintf('  SA値一覧: ');
    fprintf('%.1f ', unique_sa);
    fprintf('\n');
end
fprintf('\n');

% SAの離散分布（DriveBrakeのSAはステップ設定の場合が多い）
fprintf('  SA区間別サンプル数:\n');
sa_bins = [-15, -10, -7, -5, -3, -1, -0.5, 0.5, 1, 3, 5, 7, 10, 15];
for j = 1:length(sa_bins)-1
    mask = sa_vals >= sa_bins(j) & sa_vals < sa_bins(j+1);
    n_in_bin = sum(mask);
    if n_in_bin > 0
        fprintf('    SA ∈ [%5.1f, %5.1f): %6d サンプル (%.1f%%)', ...
            sa_bins(j), sa_bins(j+1), n_in_bin, 100*n_in_bin/length(sa_vals));
        % この区間のFx統計
        fx_in_bin = db_all.Fx(mask);
        sr_in_bin = db_all.SR(mask);
        fprintf('  Fx: mean=%.0f, std=%.0f, |SR|mean=%.3f\n', ...
            mean(fx_in_bin), std(fx_in_bin), mean(abs(sr_in_bin)));
    end
end
fprintf('\n');

%% ============================================================
%% Phase 3: SA閾値ごとの残存データ量と物理整合性
%% ============================================================
fprintf('=== Phase 3: SA閾値別のデータ量と品質 ===\n\n');

thresholds = [0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0];

fprintf('  |SA| < th | 残存数   | 残存%%  | corr(SR,Fx) | Fx mean | Fx std  | Fy mean | Fy std  | SR std  | SR range\n');
fprintf('  ---------|----------|--------|-------------|---------|---------|---------|---------|---------|----------\n');

for th = thresholds
    mask = abs(db_all.SA) < th;
    d = db_all(mask, :);
    n = height(d);
    if n > 10
        fprintf('  %7.1f° | %8d | %5.1f%% | %11.4f | %7.1f | %7.1f | %7.1f | %7.1f | %7.4f | [%.3f, %.3f]\n', ...
            th, n, 100*n/height(db_all), ...
            corr(d.SR, d.Fx), mean(d.Fx), std(d.Fx), ...
            mean(d.Fy), std(d.Fy), std(d.SR), min(d.SR), max(d.SR));
    end
end
fprintf('\n');

%% ============================================================
%% Phase 4: フィルタなし(全データ)のSR vs Fx特性
%% ============================================================
fprintf('=== Phase 4: SR vs Fx 特性（フィルタなし vs あり） ===\n\n');

% 全データ
fprintf('  全データ (N=%d):\n', height(db_all));
fprintf('    corr(SR, Fx) = %.4f\n', corr(db_all.SR, db_all.Fx));
fprintf('    Fx: mean=%.1f, std=%.1f, range=[%.1f, %.1f]\n', ...
    mean(db_all.Fx), std(db_all.Fx), min(db_all.Fx), max(db_all.Fx));

% SA<1°
mask1 = abs(db_all.SA) < 1.0;
d1 = db_all(mask1, :);
fprintf('\n  |SA|<1° (N=%d):\n', height(d1));
fprintf('    corr(SR, Fx) = %.4f\n', corr(d1.SR, d1.Fx));
fprintf('    Fx: mean=%.1f, std=%.1f, range=[%.1f, %.1f]\n', ...
    mean(d1.Fx), std(d1.Fx), min(d1.Fx), max(d1.Fx));
fprintf('    SR: mean=%.4f, std=%.4f, range=[%.4f, %.4f]\n', ...
    mean(d1.SR), std(d1.SR), min(d1.SR), max(d1.SR));

% SA<3°
mask3 = abs(db_all.SA) < 3.0;
d3 = db_all(mask3, :);
fprintf('\n  |SA|<3° (N=%d):\n', height(d3));
fprintf('    corr(SR, Fx) = %.4f\n', corr(d3.SR, d3.Fx));
fprintf('    Fx: mean=%.1f, std=%.1f, range=[%.1f, %.1f]\n', ...
    mean(d3.Fx), std(d3.Fx), min(d3.Fx), max(d3.Fx));
fprintf('    SR: mean=%.4f, std=%.4f, range=[%.4f, %.4f]\n', ...
    mean(d3.SR), std(d3.SR), min(d3.SR), max(d3.SR));

% SA<5°
mask5 = abs(db_all.SA) < 5.0;
d5 = db_all(mask5, :);
fprintf('\n  |SA|<5° (N=%d):\n', height(d5));
fprintf('    corr(SR, Fx) = %.4f\n', corr(d5.SR, d5.Fx));
fprintf('    Fx: mean=%.1f, std=%.1f, range=[%.1f, %.1f]\n', ...
    mean(d5.Fx), std(d5.Fx), min(d5.Fx), max(d5.Fx));
fprintf('    SR: mean=%.4f, std=%.4f, range=[%.4f, %.4f]\n', ...
    mean(d5.SR), std(d5.SR), min(d5.SR), max(d5.SR));

fprintf('\n');

%% ============================================================
%% Phase 5: Fz条件別のSR vs Fx（各SA閾値で）
%% ============================================================
fprintf('=== Phase 5: Fz×SR条件別のFx範囲 ===\n\n');

Fz_bins = [0, 300, 600, 900, 1200, 1500, 2000];
sa_th = 1.0;
d_filtered = db_all(abs(db_all.SA) < sa_th, :);

fprintf('  |SA| < %.1f° フィルタ後 (N=%d):\n\n', sa_th, height(d_filtered));
fprintf('  Fz区間       | サンプル数 | SR range         | Fx range         | Fx std\n');
fprintf('  -------------|-----------|------------------|------------------|-------\n');

for j = 1:length(Fz_bins)-1
    mask = d_filtered.Fz >= Fz_bins(j) & d_filtered.Fz < Fz_bins(j+1);
    dd = d_filtered(mask, :);
    if height(dd) > 0
        fprintf('  [%4d, %4d) | %9d | [%6.3f, %6.3f] | [%7.1f, %7.1f] | %7.1f\n', ...
            Fz_bins(j), Fz_bins(j+1), height(dd), ...
            min(dd.SR), max(dd.SR), min(dd.Fx), max(dd.Fx), std(dd.Fx));
    end
end
fprintf('\n');

%% ============================================================
%% Phase 6: 可視化
%% ============================================================
fprintf('=== Phase 6: 可視化 ===\n\n');

figure('Position', [50, 50, 1800, 1200], 'Name', 'DriveBrake Data Diagnosis');

% (1) SA分布ヒストグラム
subplot(3, 4, 1);
histogram(db_all.SA, 100);
xlabel('SA [deg]'); ylabel('Count');
title('全データ: SA分布');
xline([-1 1], 'r--', 'LineWidth', 1.5);
xline([-3 3], 'g--', 'LineWidth', 1);

% (2) SR分布ヒストグラム
subplot(3, 4, 2);
histogram(db_all.SR, 100);
xlabel('SR'); ylabel('Count');
title('全データ: SR分布');

% (3) SA vs SR 散布図
subplot(3, 4, 3);
scatter(db_all.SA, db_all.SR, 1, db_all.Fz, 'filled', 'MarkerFaceAlpha', 0.1);
xlabel('SA [deg]'); ylabel('SR');
title('SA vs SR (color=Fz)');
colorbar;

% (4) SR vs Fx 散布図（全データ）
subplot(3, 4, 4);
scatter(db_all.SR, db_all.Fx, 1, abs(db_all.SA), 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('SR'); ylabel('Fx [N]');
title('全データ: SR vs Fx (color=|SA|)');
colorbar;
clim([0 12]);

% (5) SR vs Fx（|SA|<1°）
subplot(3, 4, 5);
scatter(d1.SR, d1.Fx, 1, d1.Fz, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('SR'); ylabel('Fx [N]');
title('|SA|<1°: SR vs Fx (color=Fz)');
colorbar;

% (6) SR vs Fx（|SA|<3°）
subplot(3, 4, 6);
scatter(d3.SR, d3.Fx, 1, d3.Fz, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('SR'); ylabel('Fx [N]');
title('|SA|<3°: SR vs Fx (color=Fz)');
colorbar;

% (7) SR vs Fx（|SA|<5°）
subplot(3, 4, 7);
scatter(d5.SR, d5.Fx, 1, d5.Fz, 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('SR'); ylabel('Fx [N]');
title('|SA|<5°: SR vs Fx (color=Fz)');
colorbar;

% (8) SR vs Fx（フィルタなし、Fz色分け）
subplot(3, 4, 8);
scatter(db_all.SR, db_all.Fx, 1, db_all.Fz, 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('SR'); ylabel('Fx [N]');
title('全データ: SR vs Fx (color=Fz)');
colorbar;

% (9) SR vs Fy（全データ）
subplot(3, 4, 9);
scatter(db_all.SR, db_all.Fy, 1, abs(db_all.SA), 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('SR'); ylabel('Fy [N]');
title('全データ: SR vs Fy (color=|SA|)');
colorbar;
clim([0 12]);

% (10) SR vs Fy（|SA|<1°）
subplot(3, 4, 10);
scatter(d1.SR, d1.Fy, 1, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
xlabel('SR'); ylabel('Fy [N]');
title('|SA|<1°: SR vs Fy');
grid on;

% (11) Run毎のSA分布（Box Plot）
subplot(3, 4, 11);
sa_by_run = [];
run_group = [];
for i = 1:length(run_data)
    sa_by_run = [sa_by_run; run_data{i}.SA];
    run_group = [run_group; i * ones(height(run_data{i}), 1)];
end
boxplot(sa_by_run, run_group);
xlabel('Run'); ylabel('SA [deg]');
title('Run毎のSA分布');
xticklabels(cellfun(@(s) s(end-5:end-4), run_labels, 'UniformOutput', false));

% (12) Fz分布
subplot(3, 4, 12);
histogram(db_all.Fz, 50);
xlabel('Fz [N]'); ylabel('Count');
title('全データ: Fz分布');

sgtitle(sprintf('DriveBrake Data Diagnosis: %s', strrep(tire_model, '_', '\_')), ...
    'FontSize', 14, 'FontWeight', 'bold');

if ~exist('output', 'dir'), mkdir('output'); end
saveas(gcf, sprintf('output/diagnose_drivebrake_%s.png', tire_model));
fprintf('✓ 図を保存: output/diagnose_drivebrake_%s.png\n\n', tire_model);

%% ============================================================
%% Phase 7: 推奨事項
%% ============================================================
fprintf('=====================================================================\n');
fprintf('  推奨事項\n');
fprintf('=====================================================================\n\n');

% 各閾値で corr(SR, Fx) が最も高い閾値を推奨
best_th = 0;
best_corr = 0;
best_n = 0;
for th = thresholds
    mask = abs(db_all.SA) < th;
    d = db_all(mask, :);
    if height(d) > 1000
        c = corr(d.SR, d.Fx);
        % corr * sqrt(N) をスコアとして使用（精度とデータ量のバランス）
        score = abs(c) * sqrt(height(d));
        if score > best_corr * sqrt(best_n) || best_n == 0
            best_th = th;
            best_corr = c;
            best_n = height(d);
        end
    end
end

fprintf('  推奨SA閾値: |SA| < %.1f°\n', best_th);
fprintf('    → データ数: %d (%.1f%%残存)\n', best_n, 100*best_n/height(db_all));
fprintf('    → corr(SR, Fx) = %.4f\n', best_corr);
fprintf('\n');

% |SA|<1 のSR分布問題をチェック
if height(d1) > 0 && (max(d1.SR) - min(d1.SR)) < 0.1
    fprintf('  ⚠ |SA|<1° のSR範囲が狭い (%.4f)。\n', max(d1.SR) - min(d1.SR));
    fprintf('    → |SA|<1° のデータはSRスイープの初期区間のみの可能性\n');
    fprintf('    → SA閾値を緩和してデータ量を確保すべき\n\n');
end

fprintf('=====================================================================\n');
fprintf('  診断完了\n');
fprintf('=====================================================================\n');
