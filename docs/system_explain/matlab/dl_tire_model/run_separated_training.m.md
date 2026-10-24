# `matlab/dl_tire_model/run_separated_training.m`

## 1. 概要

Cornering 用と DriveBrake 用のデータを分離し、それぞれ専用ネットワークを学習するスクリプトです。

## 2. ファイルの役割

ファイル名規則に基づいて `Cr_*` と `Db_*` の CSV を自動探索し、旋回系と加減速系を別モデルで学習します。混合学習によるバイアスを避けることを意図しており、各データ群ごとに統計表示、前処理、学習、評価、保存を行います。

## 3. インターフェース

- 種別: スクリプト
- ユーザ設定変数:
  - `tire_model = 'Hoosier_18p0X6p0-10_R20'`
  - `data_dir = '../../data/tiredata/csv'`
  - `split_ratio = 0.85`
- 中間・出力変数:
  - `cornering_files`, `drivebrake_files`
  - `train_cornering`, `train_drivebrake`
  - `cr_*` 系変数群
  - `db_*` 系変数群
- 出力:
  - 画面ログ
  - `output/cornering_<tire_model>.mat`
  - `output/drivebrake_<tire_model>.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `preprocessDataWrapper`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB Deep Learning Toolbox 関数:
  - `featureInputLayer`
  - `fullyConnectedLayer`
  - `batchNormalizationLayer`
  - `reluLayer`
  - `dropoutLayer`
  - `regressionLayer`
  - `layerGraph`
- MATLAB 組み込み:
  - `dir`
  - `fullfile`
  - `sort`
  - `isempty`
  - `regexp`
  - `unique`
  - `return`
  - `gpuDeviceCount`
  - `gpuDevice`
  - `std`
  - `min`
  - `max`
  - `mean`
  - `tic` / `toc`
  - `exist`
  - `mkdir`
  - `save`

## 5. 処理の流れ

1. `tire_model` と `data_dir` を設定します。
2. `Cr_<tire_model>*.csv` と `Db_<tire_model>*.csv` を `dir` で検索します。
3. 検出ファイル一覧を表示し、両方とも見つからなければ利用可能モデル一覧を抽出して `return` します。
4. GPU の有無を表示します。
5. Cornering ファイルがある場合:
   1. ファイル群を読み込みます。
   2. `SR` と `SA` の標準偏差を表示し、`SR` 分散が大きいと警告します。
   3. `SA`, `SR`, `Fz`, `Fy`, `Mz` の統計を表示します。
   4. `preprocessDataWrapper` で前処理します。
   5. `createCornering_model()` で専用モデルを作ります。
   6. 500 エポック上限、学習率 `0.0001` の設定で学習します。
   7. `evaluateModel` で評価し、`output/cornering_<tire_model>.mat` に保存します。
6. DriveBrake ファイルがある場合:
   1. ファイル群を読み込みます。
   2. `SR` と `SA` の標準偏差を表示し、`SA` 分散が大きいと警告します。
   3. `SR`, `SA`, `Fz`, `Fx` の統計を表示します。
   4. `preprocessDataWrapper` で前処理します。
   5. `createDriveBrake_model()` で専用モデルを作ります。
   6. Cornering 側と同設定で学習します。
   7. `evaluateModel` で評価し、`output/drivebrake_<tire_model>.mat` に保存します。
7. 最後に各モデルの要約と、分離学習の利点、複合領域への補足を表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- セクションは「ユーザ設定」「ファイル自動検索」「GPU チェック」「Part 1: Cornering」「Part 2: DriveBrake」「最終サマリー」に分かれています。
- 学習対象データの存在に応じて Cornering 部と DriveBrake 部は独立に実行されます。
- ログでは Cornering 側で `Fy, Mz`、DriveBrake 側で `Fx` を主要ターゲットとして扱っています。

### `createCornering_model()`

- Cornering 専用のローカル関数です。
- 入力は 5 次元、出力は 4 次元です。
- 隠れ層は `256-256-128-128-64-64-32-32` の 8 ブロック構成で、各ブロックに Batch Normalization、ReLU、前半 6 ブロックには Dropout が入ります。
- 最後に `fullyConnectedLayer(4)` と `regressionLayer` を接続し、`layerGraph` を返します。

### `createDriveBrake_model()`

- DriveBrake 専用のローカル関数です。
- 層構成は `createCornering_model()` と同一で、入力 5・出力 4 の回帰ネットワークです。
- コメントでは SR を主要入力、Fx を主要出力と説明しています。

## 7. 注意点

- `tire_model` はコード内固定値で、引数化されていません。
- ログ文には `Architecture: 128-128-64-64-32 (5層)` とありますが、実際のローカル関数実装は `256-256-128-128-64-64-32-32` の 8 ブロックです。
- 出力は 4 成分すべてを学習対象にしていますが、ログでは用途に応じて主要指標のみ強調しています。
