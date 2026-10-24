%% 8の字トラックシミュレーション結果の可視化
% クイック実行スクリプト
%
% 使い方：
%   MATLABで matlab/ ディレクトリを開いて、このスクリプトを実行

clear; close all; clc;

fprintf('=== 8の字トラック シミュレーション可視化 ===\n\n');

% CSVファイルのチェック
csv_file = 'laptime_sim_figure_eight.csv';
if ~isfile(csv_file)
    error('CSVファイルが見つかりません: %s\nシミュレーションを先に実行してください。', csv_file);
end

% 可視化実行
fprintf('可視化を開始します...\n');
visualize_laptime_sim(csv_file, 'figure_eight', 25);

fprintf('\n✅ 可視化完了！\n');
fprintf('3つのフィギュアウィンドウが開きました：\n');
fprintf('  - Figure 1: メイン結果（軌跡、速度、誤差、制御入力）\n');
fprintf('  - Figure 2: 詳細時系列データ\n');
fprintf('  - Figure 3: g-gダイアグラム\n\n');
