# `matlab/dl_tire_model/main_prediction_workflow.m`

## 1. 概要

`main_prediction_workflow.m` は、保存済みタイヤモデルの読込、サンプル入力作成、推論、結果保存、簡易可視化までを順に実行する推論ワークフロースクリプトです。

## 2. ファイルの役割

- `output/tire_model_latest.mat` を読込む
- サンプル入力または CSV 入力を準備する
- `predictTireForces` で推論する
- 結果を表示・CSV 保存・MAT 保存・散布図保存する

## 3. インターフェース

### スクリプトの主要変数・入出力

- `model_path`
  - 既定で `'output/tire_model_latest.mat'`。
- `X_raw`
  - 推論対象の `N×5` 入力行列。
- `Y_pred`
  - 推論結果 `N×4` 行列。
- `output_dir`
  - 結果保存先。`'output/predictions'`。
- `timestamp`
  - 保存ファイル名に使う時刻文字列。
- 主な保存物
  - `predictions_<timestamp>.csv`
  - `predictions_<timestamp>.mat`
  - `predictions_plot_<timestamp>.png`

## 4. 依存関係

- `predictTireForces`
- MATLAB 標準関数
  - `isfile`, `load`, `table`, `writetable`, `save`, `scatter`, `saveas`, `mkdir`, `fprintf`

## 5. 処理の流れ

### セクション: CONFIGURATION

- ワークスペースを初期化し、`model_path` を設定します。
- モデルファイルが無ければ `main_training_workflow.m` を案内するエラーを出します。

### セクション: STEP 1 LOAD TRAINED MODEL

- `load(model_path)` でモデルを読み込み、`trained_net` と `norm_params` を取り出します。

### セクション: STEP 2 PREPARE INPUT DATA

- 5 個の手動サンプル `input_1`〜`input_5` を定義します。
- それらを `X_raw` にまとめて表示します。
- 下部コメントに CSV 読込版の代替例もあります。

### セクション: STEP 3 MAKE PREDICTIONS

- `predictTireForces(trained_net, X_raw, norm_params)` を呼びます。

### セクション: STEP 4 DISPLAY RESULTS

- 表形式の一覧表示と、サンプルごとの詳細表示を行います。

### セクション: STEP 5 SAVE RESULTS

- `output/predictions` を作成します。
- 入力と出力をまとめた `results_table` を CSV 保存します。
- `X_raw`, `Y_pred`, `timestamp` を MAT 保存します。

### セクション: STEP 6 VISUALIZE PREDICTIONS

- `Fx vs SR`, `Fy vs SA`, `Mx vs IA`, `Mz vs SA` の散布図を 2×2 で作り、PNG 保存します。

### セクション: WORKFLOW COMPLETE

- 使用モデル、件数、保存先を表示し、次の作業を案内します。

### セクション: HELPER FUNCTION: BATCH PREDICTION FROM FILE

- 実行コードではなく、コメントアウトされた補助例です。
- CSV 読込 → 推論 → 結果 CSV 出力の流れが記述されています。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 推論ロジック本体は `predictTireForces` 呼び出しです。
- 手動入力例は 5 ケース固定で、ブレーキ、コーナリング、複合スリップなどをコメントで区別しています。

### コメントアウトされた補助関数例 `batch_predict_from_file`

- `model_path`, `input_csv`, `output_csv` を受け取る想定の例です。
- モデル読込、`readtable`、`predictTireForces`、`writetable` を行う手順だけがコメントで示されています。
- 実際に使うにはコメント解除が必要です。

## 7. 注意点

- モデル読込時に `trained_net` と `norm_params` の存在確認はしていません。
- CSV 入力ルートはコメントアウトされており、既定では手動サンプルのみ使います。
- 補助関数 `batch_predict_from_file` はコメントのため、スクリプト実行では定義されません。
