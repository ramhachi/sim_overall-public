% EXAMPLE_USAGE_JP - POUハードポイント最適化ワークフローのデモ（日本語版）
%
% このスクリプトは、最適化された多項式係数を物理的な座標に変換する
% POUハードポイント逆運動学システムの使用方法を示します。

clear; clc; close all;

fprintf('=== POUハードポイント最適化 - 使用例デモ ===\n\n');

% 必要なパスを追加
addpath('geometry');
addpath('optimization');
addpath('visualization');

%% ステップ1: 車両パラメータの読み込み
fprintf('ステップ1: 最適化済み車両パラメータを読み込んでいます...\n');

% 最適化パラメータJSONファイルへのパス（車両最適化から）
params_file = '../optimized_autocross.json';

if ~exist(params_file, 'file')
    error('最適化パラメータファイルが見つかりません: %s', params_file);
end

fprintf('  ✓ 見つかりました: %s\n\n', params_file);

%% ステップ2: 最適化オプションの設定
fprintf('ステップ2: 最適化を設定しています...\n');

options = struct();
options.wheel_index = 0;           % 0=FL, 1=FR, 2=RL, 3=RR
options.n_multistart = 5;          % ランダム初期値の数
options.delta_range_deg = [-30, 30];  % ステアリング範囲 [deg]
options.n_delta_points = 61;       % スイープ点数
options.verbose = true;            % 進捗を表示

wheel_names = {'FL', 'FR', 'RL', 'RR'};
fprintf('  ホイール: %s\n', wheel_names{options.wheel_index + 1});
fprintf('  マルチスタート: %d 回の初期値\n', options.n_multistart);
fprintf('  ステアリング範囲: [%.0f, %.0f] deg\n', options.delta_range_deg);
fprintf('\n');

%% ステップ3: 最適化の実行
fprintf('ステップ3: 逆運動学最適化を実行しています...\n');
fprintf('  （複雑さによって10-30秒かかる場合があります）\n\n');

tic;
results = main_pou_hardpoint_optimizer(params_file, options);
total_time = toc;

%% ステップ4: 結果の分析
fprintf('\n');
fprintf('========================================\n');
fprintf('最適化完了\n');
fprintf('========================================\n');
fprintf('合計時間: %.1f 秒\n\n', total_time);

% 成功チェック
if results.optimization_exitflag >= 1
    fprintf('ステータス: ✓ 収束\n');
else
    fprintf('ステータス: ✗ 失敗 (exitflag=%d)\n', results.optimization_exitflag);
end

% ハードポイント座標
fprintf('\n--- ハードポイント座標 ---\n');
fprintf('M-pivot (アップライトフレーム): [%.4f, %.4f, %.4f] m\n', results.M_pivot_upright);
fprintf('M-pivot (シャシーフレーム):      [%.4f, %.4f, %.4f] m\n', results.M_pivot_chassis);
fprintf('ベルクランクピボット:            [%.4f, %.4f, %.4f] m\n', results.bellcrank_pivot);

% 性能指標
fprintf('\n--- 性能指標 ---\n');
fprintf('多項式フィット誤差 (RMS): %.2f mm', results.polynomial_error_mm);
if results.polynomial_error_mm < 2.0
    fprintf(' ✓ 合格 (< 2mm)\n');
else
    fprintf(' ✗ 不合格 (> 2mm)\n');
end

fprintf('最大ジャッキアップ誤差:   %.2f mm\n', results.max_jack_error_mm);
fprintf('最大キャンバー誤差:       %.2f deg\n', results.max_camber_error_deg);

fprintf('最大ロッドエンド角:       %.2f deg', results.max_rod_angle_deg);
if results.max_rod_angle_deg < 15.0
    fprintf(' ✓ 合格 (< 15°)\n');
else
    fprintf(' ⚠ 警告 (> 15°、高角度対応ロッドエンドを使用)\n');
end

fprintf('インストレーション比:     %.3f', results.installation_ratio);
if results.installation_ratio >= 0.10 && results.installation_ratio <= 0.30
    fprintf(' ✓ 合格 (0.10-0.30)\n');
else
    fprintf(' ⚠ 範囲外 (推奨: 0.10-0.30)\n');
end

fprintf('プッシュロッド長 (公称):  %.3f m\n', results.pushrod_length_nominal_m);

% パッケージング検証
fprintf('\n--- パッケージング検証 ---\n');
if results.packaging_valid
    fprintf('ステータス: ✓ 合格 (すべての制約を満足)\n');
else
    fprintf('ステータス: ✗ 不合格 (%d 件の違反)\n', length(results.violations));
    fprintf('違反内容:\n');
    for i = 1:length(results.violations)
        fprintf('  %d. %s\n', i, results.violations{i});
    end
end

fprintf('========================================\n\n');

%% ステップ5: 結果の可視化
fprintf('ステップ5: 比較プロットを生成しています...\n');

try
    fig = plot_comparison(results);
    fprintf('  ✓ プロット生成成功\n');

    % オプション: 図を保存
    % saveas(fig, 'pou_hardpoint_optimization_results.png');
    % fprintf('  ✓ プロット保存: pou_hardpoint_optimization_results.png\n');
catch ME
    fprintf('  ✗ プロット生成失敗: %s\n', ME.message);
end

%% ステップ6: SolidWorks用エクスポート（Phase 3 - 未実装）
fprintf('\nステップ6: SolidWorks用エクスポート...\n');
fprintf('  ⏳ 未実装 (Phase 3)\n');
fprintf('  将来: export_to_solidworks(results, ''pou_hardpoints_FL.csv'')\n');

%% まとめ
fprintf('\n=== デモ完了 ===\n');
fprintf('最適化されたハードポイント座標をCAD設計に使用できます。\n');
fprintf('次のステップ:\n');
fprintf('  1. プロットを確認して多項式フィット品質を検証\n');
fprintf('  2. パッケージング検証結果を確認\n');
fprintf('  3. SolidWorksインポート用に座標をエクスポート (Phase 3)\n');
fprintf('  4. 必要に応じて他のホイール (FR, RL, RR) でも繰り返し\n\n');

%% 追加例: 全4輪の最適化
fprintf('=== 追加例: 全4輪の最適化 ===\n');
fprintf('以下のコードで全ホイールを順番に最適化できます:\n\n');

fprintf('wheel_names = {''FL'', ''FR'', ''RL'', ''RR''};\n');
fprintf('all_results = cell(4, 1);\n\n');

fprintf('for i = 0:3\n');
fprintf('    fprintf(''\\n=== 最適化中: %%s ホイール ===\\n'', wheel_names{i+1});\n');
fprintf('    all_results{i+1} = main_pou_hardpoint_optimizer(params_file, ...\n');
fprintf('        struct(''wheel_index'', i, ''verbose'', false));\n');
fprintf('    fprintf(''完了: 誤差 %%.2f mm, パッケージング %%s\\n'', ...\n');
fprintf('        all_results{i+1}.polynomial_error_mm, ...\n');
fprintf('        all_results{i+1}.packaging_valid ? ''合格'' : ''不合格'');\n');
fprintf('end\n\n');

fprintf('実行しますか？ (y/n): ');
user_input = input('', 's');

if strcmpi(user_input, 'y')
    fprintf('\n全4輪の最適化を開始...\n');

    all_results = cell(4, 1);
    for i = 0:3
        fprintf('\n=== 最適化中: %s ホイール ===\n', wheel_names{i+1});

        all_results{i+1} = main_pou_hardpoint_optimizer(params_file, ...
            struct('wheel_index', i, 'verbose', false));

        if all_results{i+1}.packaging_valid
            status_str = '✓ 合格';
        else
            status_str = '✗ 不合格';
        end

        fprintf('完了: 誤差 %.2f mm, パッケージング %s\n', ...
            all_results{i+1}.polynomial_error_mm, status_str);
    end

    % 結果サマリー
    fprintf('\n=== 全ホイール結果サマリー ===\n');
    fprintf('ホイール | 誤差[mm] | ロッド角[deg] | IR    | パッケージング\n');
    fprintf('---------|----------|---------------|-------|---------------\n');
    for i = 1:4
        if all_results{i}.packaging_valid
            pkg_str = '✓ 合格';
        else
            pkg_str = '✗ 不合格';
        end

        fprintf('%-8s | %8.2f | %13.1f | %.3f | %s\n', ...
            wheel_names{i}, ...
            all_results{i}.polynomial_error_mm, ...
            all_results{i}.max_rod_angle_deg, ...
            all_results{i}.installation_ratio, ...
            pkg_str);
    end
else
    fprintf('スキップしました。\n');
end

fprintf('\n=== すべて完了 ===\n');
