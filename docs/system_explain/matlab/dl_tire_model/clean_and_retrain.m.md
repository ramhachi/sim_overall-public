# `matlab/dl_tire_model/clean_and_retrain.m`

## 1. 概要

`clean_and_retrain.m` は、Cornering/DriveBrake データの選別とクリーニングを行い、ゼロスリップ点を追加した上で 2 つのネットワークを再学習し、簡易検証と MAT 保存まで実行する一括スクリプトです。

## 2. ファイルの役割

- Cornering では `|SR| < 0.05` の条件で異常値を除去します。
- DriveBrake では `Run068`, `Run071` をファイル名ベースで除外し、それ以外は SA 閾値をかけずに使います。
- Cornering/DriveBrake の両方にゼロスリップ点 500 件を追加します。
- `preprocessDataWrapper` → `trainTireModel` → `evaluateModel` の流れで再学習します。
- ゼロスリップ入力と SR sweep 入力で再学習モデルを点検し、`output/*.mat` に保存します。

## 3. インターフェース

### 3.1 種別

スクリプトです。引数や戻り値はありません。

### 3.2 主な設定変数

- `tire_model = 'Hoosier_18p0X6p0-10_R20'`
- `data_dir = '../../data/tiredata/csv'`
- `split_ratio = 0.85`
- `db_exclude_runs = {'Run068', 'Run071'}`
- `cr_sr_threshold = 0.05`
- `n_zero = 500`

### 3.3 主な生成物

- 学習データ
  - `cr_clean`, `db_clean`
  - `cr_augmented`, `db_augmented`
- 学習結果
  - `cr_trained`, `db_trained`
  - `cr_norm`, `db_norm`
  - `cr_metrics`, `db_metrics`
  - `cr_info`, `db_info`
- 保存ファイル
  - `output/cornering_<tire_model>.mat`
  - `output/drivebrake_<tire_model>.mat`

## 4. 依存関係

- プロジェクト関数
  - `loadTireData`
  - `preprocessDataWrapper`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB/Deep Learning Toolbox 関数
  - `gpuDeviceCount`, `gpuDevice`, `predict`, `save`, `mkdir`, `corr`, `rng`
  - ネットワーク定義用に `featureInputLayer`, `fullyConnectedLayer`, `batchNormalizationLayer`, `reluLayer`, `dropoutLayer`, `regressionLayer`, `layerGraph`

## 5. 処理の流れ

1. ユーザ設定部で対象タイヤ、データディレクトリ、除外 Run、分割率を定義します。
2. Cornering と DriveBrake の CSV 群を列挙し、`loadTireData` で読み込みます。DriveBrake は `Run068/071` を除外します。
3. Cornering は `|SR| < 0.05` で `cr_clean` に絞り、DriveBrake は選別済みデータをそのまま `db_clean` とします。
4. 既存データの `Fz/IA/TSTC` レンジから乱数で 500 件のゼロスリップデータを作り、Cornering/DriveBrake の両方に追加します。
5. Cornering モデルと DriveBrake モデルをそれぞれ前処理・学習・評価します。どちらも同じローカル関数 `createModel()` を使います。
6. `SR=0, SA=0, IA=0, TSTC=40` のゼロスリップ入力を複数 `Fz` で評価し、出力を表示します。
7. `SA=0, Fz=800, IA=0, TSTC=40` で `SR=-0.2:0.04:0.2` の sweep を行い、DriveBrake 出力を確認します。
8. 学習済みモデルと正規化パラメータ、評価結果、学習情報を `output/*.mat` に保存します。
9. 最後にデータ量と精度の比較サマリーを表示します。

## 6. 関数・ローカル関数詳細

### 6.1 スクリプト各セクション

#### ユーザ設定

- タイヤ名・入力ディレクトリ・分割率・除外 Run・Cornering の SR 閾値を固定します。

#### Step 1: データ読み込み

- `Cr_<tire_model>*.csv` と `Db_<tire_model>*.csv` を列挙します。
- DriveBrake 側はファイル名に `Run068` または `Run071` を含むものを除外します。

#### Step 2: データクリーニング

- Cornering は `abs(cr_raw.SR) < cr_sr_threshold` の行だけを採用します。
- DriveBrake は SA 閾値を使わず、Run 選別済みデータを丸ごと使います。
- 物理整合性の目安として `corr` や `mean/std` を表示します。

#### Step 3: ゼロスリップデータ追加

- `Fz`, `IA`, `TSTC` は既存データの最小値〜最大値の一様乱数です。
- `SR`, `SA`, `Fx`, `Fy`, `Mx`, `Mz` はすべて 0 に固定されます。
- `cr_clean.Properties.VariableNames` を使って列順を既存データに合わせています。

#### Step 4: 前処理 & 学習

- Cornering/DriveBrake それぞれで `preprocessDataWrapper` を呼びます。
- 学習設定は両モデルとも `max_epochs=500`, `batch_size=256`, `learning_rate=0.0005`, `patience=300`, `val_frequency=100`, `random_seed=42` です。
- 評価結果から Cornering は `Fy`, `Mz`、DriveBrake は `Fx` を表示します。

#### Step 5: ゼロスリップ検証

- `Fz_test = [200, 500, 800, 1100, 1400]` でゼロスリップ入力を評価します。
- さらに `sr_test = -0.2:0.04:0.2` の sweep で DriveBrake の `Fx/Fy` レンジを表示します。

#### Step 6〜7: 保存と比較サマリー

- `output` ディレクトリがなければ作成します。
- 保存ファイルには学習済みネットワーク、正規化パラメータ、評価指標、学習情報、`tire_model` を含めます。

### 6.2 ローカル関数 `createModel`

```matlab
net = createModel()
```

- 5 入力・4 出力の全結合回帰ネットワークを返します。
- 隠れ層構成は `256-256-128-128-64-64-32-32` です。
- 各ブロックに `batchNormalizationLayer` と `reluLayer` を置き、前半 6 ブロックに `dropoutLayer` を入れています。
- ドロップアウト率は順に `0.15, 0.15, 0.12, 0.12, 0.1, 0.1` です。
- 出力は `fullyConnectedLayer(4)` と `regressionLayer` です。

## 7. 注意点

- スクリプト内で GPU 確認に `gpuDeviceCount` を直接使っており、`checkGPUAvailability` は呼びません。
- ゼロスリップ追加は Cornering/DriveBrake の両方に同一生成ロジックを適用します。
- 保存先は固定で `output/` 配下です。
- 評価表示は一部出力成分だけを抜粋しており、`evaluateModel` が返す全指標を表示するわけではありません。
