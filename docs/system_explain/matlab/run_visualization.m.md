# `matlab/run_visualization.m`

## 1. ファイルの目的

8 の字トラック結果 CSV の存在確認を行い、`visualize_laptime_sim()` を既定条件で起動する補助スクリプトです。

## 2. 主な入出力または依存関係

- 入力: `laptime_sim_figure_eight.csv`
- 出力: 可視化図と標準出力
- 依存関係: `visualize_laptime_sim`

## 3. 処理の流れ / アルゴリズム

1. ワークスペースを初期化します。
2. 対象 CSV の存在を確認し、なければエラーにします。
3. `visualize_laptime_sim(csv_file, 'figure_eight', 25)` を実行します。
4. 開いた図の意味を説明して終了します。

## 4. ファイル内の定義

### スクリプト本体

- 既定ファイル名・既定トラック条件で可視化関数を呼ぶだけのランチャーです。

## 5. 注意点・前提条件

- 実行ディレクトリに `laptime_sim_figure_eight.csv` が必要です。
- 図の詳細な生成ロジックは `visualize_laptime_sim.m` 側にあります。
