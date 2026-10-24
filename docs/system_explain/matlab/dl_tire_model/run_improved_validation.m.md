# `matlab/dl_tire_model/run_improved_validation.m`

## 1. 概要

深いモデルと長めの学習設定で検証を行うスクリプトです。学習・評価・速度測定・要件判定・保存を一括実行します。

## 2. ファイルの役割

単純な検証よりも高精度を狙う実験用スクリプトです。`createDeepTireModel` で深層モデルを作成し、300 エポック上限の学習設定で `trainTireModel` を実行し、評価指標と要件充足状況をレポートします。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `data_path = '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'`
  - `config.split_ratio = 0.8`
  - `train_config`
    - `max_epochs = 300`
    - `batch_size = 256`
    - `learning_rate = 0.001`
    - `patience = 20`
    - `val_frequency = 50`
    - `random_seed = 42`
  - `trained_net`, `training_info`, `metrics`, `train_time`, `pred_time`
- 出力:
  - 画面ログ
  - `output/improved_validation_model.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `preprocessDataWrapper`
  - `createDeepTireModel`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB 組み込み:
  - `gpuDeviceCount`
  - `gpuDevice`
  - `fprintf`
  - `height`
  - `size`
  - `randn`
  - `predict`
  - `tic` / `toc`
  - `exist`
  - `mkdir`
  - `save`
  - `rethrow`

## 5. 処理の流れ

1. GPU の有無を表示します。
2. 固定 CSV を `loadTireData` で読み込みます。
3. `preprocessDataWrapper` を `split_ratio=0.8` で実行します。
4. `createDeepTireModel()` で深層モデルを作ります。
5. `train_config` を構築して `trainTireModel` を実行し、学習時間を分単位で記録します。
6. `evaluateModel` で検証データに対する `metrics` を計算します。
7. `randn(1000, 5)` を使って推論速度を測り、1 サンプル当たり ms に換算します。
8. `output` がなければ `mkdir` し、モデル・正規化係数・評価・学習情報を保存します。
9. Fx/Fy/Mx/Mz の RMSE・R²・TIC を表示します。
10. RMSE、R²、速度に関する条件を判定し、すべて満たすか確認します。
11. 条件未達時は改善提案を表示します。
12. 最後に `training_info.TrainingLoss` と `ValidationLoss` の最終値、エポック数を表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- スクリプトは `Step 1` から `Step 8` までの順で構成されています。
- 判定基準は `Fx/Fy RMSE < 100`, `Fx/Fy R² > 0.95`, `Mx/Mz R² > 0.85`, `Speed < 1ms` です。
- 例外時は最上位スタック位置を表示して再送出します。

### `bool_str(val)`

- 真偽値を `PASS ✓` または `FAIL ✗` に変換するローカル関数です。
- 要件判定の各表示で使われます。

## 7. 注意点

- 入力データは DriveBrake の単一 CSV に固定されています。
- 速度測定は正規化済みである保証のない `randn(1000,5)` を直接 `predict` へ渡しています。
- 学習時間や精度に関するコメント文はコード内ログであり、実際の性能は実行環境依存です。
