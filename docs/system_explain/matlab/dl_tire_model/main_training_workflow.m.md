# `matlab/dl_tire_model/main_training_workflow.m`

## 1. 概要

`main_training_workflow.m` は、データ前処理からモデル生成、学習、評価、可視化、エクスポート、サマリ保存までを一括実行する学習用スクリプトです。

## 2. ファイルの役割

- 学習条件と入力データを設定する
- `preprocessData` で学習・検証データを作る
- モデル作成、学習、評価、可視化、エクスポートを順次実行する
- テキスト形式の学習サマリを出力する

## 3. インターフェース

### スクリプトの主要変数・入出力

- `data_paths`
  - 2 つの TTC CSV パス。
- `output_dir`
  - 既定で `'output'`。
- `config`
  - 学習設定構造体。
  - `split_ratio`, `learning_rate`, `batch_size`, `max_epochs`, `patience`, `val_frequency`, `random_seed`, `model_type`
- 主な生成物
  - `trained_net`, `training_info`, `metrics`, `norm_params`
  - `output/tire_model_<timestamp>.mat`
  - `output/tire_model_latest.mat`
  - `output/visualizations/*.png`
  - `output/training_summary_<timestamp>.txt`

## 4. 依存関係

- `preprocessData`
- `createModelFromConfig`
- `trainTireModel`
- `evaluateModel`
- `denormalizeData`
- `generateAllVisualizations`
- `exportModel`
- MATLAB 標準関数
  - `predict`, `mkdir`, `fopen`, `fprintf`, `fclose`, `datetime`

## 5. 処理の流れ

### セクション: CONFIGURATION

- ワークスペース初期化、開始時刻表示、データパス設定、`output_dir` 作成、`config` 初期化を行います。

### セクション: STEP 1 LOAD AND PREPROCESS DATA

- `preprocessData(data_paths, config)` を実行し、`X_train`, `Y_train`, `X_val`, `Y_val`, `norm_params` を取得します。
- 失敗時は詳細を表示して処理を停止します。

### セクション: STEP 2 CREATE MODEL

- `createModelFromConfig(config)` でモデルアーキテクチャを生成します。

### セクション: STEP 3 TRAIN MODEL

- `trainTireModel(net, X_train, Y_train, X_val, Y_val, config)` を呼びます。

### セクション: STEP 4 EVALUATE MODEL

- `evaluateModel(trained_net, X_val, Y_val, norm_params)` を実行します。
- 評価失敗時は `metrics.meets_requirements = false` だけを持つ構造体に置き換えて継続します。

### セクション: STEP 5 GENERATE VISUALIZATIONS

- `predict(trained_net, X_val)` で正規化空間の予測を取り、`denormalizeData` で入出力を元スケールへ戻します。
- `generateAllVisualizations(X_val_denorm, Y_actual, Y_pred, viz_dir)` を実行します。

### セクション: STEP 6 EXPORT MODEL

- 時刻付きファイル名 `tire_model_<timestamp>.mat` を作成し、`exportModel` で保存します。
- 併せて `tire_model_latest.mat` も保存します。

### セクション: STEP 7 SAVE TRAINING SUMMARY

- 学習件数、設定、学習時間、最終損失、性能指標、出力ファイル一覧をテキストに保存します。

### セクション: WORKFLOW COMPLETE

- 出力物を再表示し、`metrics.meets_requirements` の真偽で完了メッセージを分岐します。

## 6. 関数・ローカル関数詳細

### スクリプト内の関数定義

- このファイルにローカル関数は定義されていません。
- 全処理は外部関数の組合せで構成されています。

## 7. 注意点

- 可視化失敗や評価失敗は警告扱いで継続します。
- `timestamp`, `model_path`, `viz_dir`, `summary_path` は前段の成功を前提に後半で再利用されるため、失敗経路では未定義になる可能性があります。
- `config` に `hidden_sizes` や `dropout_rate` は含まれておらず、必要なら `createModelFromConfig` 側の既定値に依存します。
