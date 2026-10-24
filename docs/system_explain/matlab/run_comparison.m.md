# `matlab/run_comparison.m`

## 1. ファイルの目的

比較用 CSV の存在確認をして `compare_results()` を呼ぶだけのクイック実行スクリプトです。

## 2. 主な入出力または依存関係

- 入力: `default_sim.csv`, `optimized_sim.csv`
- 出力: 比較図と標準出力
- 依存関係: `compare_results`

## 3. 処理の流れ / アルゴリズム

1. ワークスペースを初期化します。
2. 2 つの CSV が存在するか確認します。
3. `compare_results('default_sim.csv', 'optimized_sim.csv', 'figure_eight', 25)` を呼びます。
4. 完了メッセージを表示します。

## 4. ファイル内の定義

### スクリプト本体

- ユーザーが手で引数を与えずに比較を起動するためのランチャーです。
- 本質的な比較処理は `compare_results.m` に委譲しています。

## 5. 注意点・前提条件

- 実行ディレクトリに `default_sim.csv` と `optimized_sim.csv` がある前提です。
- トラック条件は `figure_eight`, 半径 25m に固定です。
