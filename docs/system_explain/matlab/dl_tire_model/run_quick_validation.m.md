# `matlab/dl_tire_model/run_quick_validation.m`

## 1. 概要

サンプルデータで短時間検証を行う簡易バリデーションスクリプトです。

## 2. ファイルの役割

四分の一サンプル CSV を使い、前処理、モデル生成、50 エポック上限の学習、評価、速度測定、保存、要件判定までを短い設定で確認する役割です。動作確認や初期検証を想定した構成になっています。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `data_path = '../../data/tiredata/DriveBrake_sampled_quarter.csv'`
  - `config.split_ratio = 0.8`
  - `train_config`
    - `max_epochs = 50`
    - `batch_size = 128`
    - `learning_rate = 0.001`
    - `patience = 5`
    - `val_frequency = 20`
    - `random_seed = 42`
  - `trained_net`, `training_info`, `metrics`, `time_per_sample_ms`
- 出力:
  - 画面ログ
  - `output/quick_validation_model.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `preprocessDataWrapper`
  - `createTireModel`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB 組み込み:
  - `addpath`
  - `pwd`
  - `gpuDeviceCount`
  - `gpuDevice`
  - `isfile`
  - `error`
  - `randn`
  - `predict`
  - `tic` / `toc`
  - `exist`
  - `mkdir`
  - `save`
  - `rethrow`

## 5. 処理の流れ

1. `clear; clc;` 後に `addpath(pwd)` を実行します。
2. GPU 有無を表示します。
3. サンプル CSV の存在を `isfile` で確認し、なければ停止します。
4. `loadTireData` でデータを読み込みます。
5. `preprocessDataWrapper` を `split_ratio=0.8` で実行します。
6. `createTireModel()` でモデルを生成します。
7. 50 エポック上限の `train_config` を構築して学習します。
8. `evaluateModel` で指標を算出します。
9. `randn(1000,5)` に対して `predict` を実行し、サンプル当たり推論時間を計算します。
10. `output/quick_validation_model.mat` へ保存します。
11. RMSE、R²、速度要件を判定し、`PASSED` か `COMPLETED WITH WARNINGS` を表示します。
12. 例外時は全スタックを表示して再送出します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- `Step 1` から `Step 9` まで順序立てて実行する構成です。
- 要件判定は `Fx/Fy RMSE < 100`, `Fx/Fy/Mx/Mz の R² > 0.95`, `Prediction < 1ms` です。
- 保存対象には `training_info` も含まれます。

### ローカル関数

- ありません。

## 7. 注意点

- コメントでは「sampled data 用」とされており、読み込むデータも `DriveBrake_sampled_quarter.csv` 固定です。
- 速度計測は無作為データに対する生 `predict` で行っており、`predictTireForces` のような正規化込み経路ではありません。
- 失敗時は `ME.stack` 全件を表示するため、他の検証スクリプトよりログが長くなります。
