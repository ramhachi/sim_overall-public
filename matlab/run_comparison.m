%% デフォルト vs 最適化 結果比較
% クイック実行スクリプト

clear; close all; clc;

fprintf('=== PID Optimization Results Comparison ===\n\n');

% ファイル存在確認
if ~isfile('default_sim.csv')
    error('default_sim.csv が見つかりません');
end
if ~isfile('optimized_sim.csv')
    error('optimized_sim.csv が見つかりません');
end

% 比較実行
fprintf('比較を開始します...\n\n');
compare_results('default_sim.csv', 'optimized_sim.csv', 'figure_eight', 25);

fprintf('\n✅ 比較完了！\n');
fprintf('2つのフィギュアが開きました：\n');
fprintf('  - Figure 1: 軌跡・誤差・制御入力の比較\n');
fprintf('  - Figure 2: 誤差分布のヒストグラム\n\n');
