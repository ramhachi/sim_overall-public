# `matlab/dl_tire_model/run_simple_validation.m`

## 1. 概要

比較的単純な構成で必須検証を行うスクリプトです。学習・評価・速度測定・要件判定を最小限の制御で実行します。

## 2. ファイルの役割

複雑な検証オーケストレーションを避けた「Minimal and Robust」版として、固定データセット、`createTireModel`、300 エポック上限の学習設定を使ってモデル品質を確認します。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `data_path = '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'`
  - `config.split_ratio = 0.9`
  - `train_config`
    - `max_epochs = 300`
    - `batch_size = 512`
    - `learning_rate = 0.0015`
    - `patience = 25`
    - `val_frequency = 50`
    - `random_seed = 42`
  - `trained_net`, `training_info`, `metrics`, `pred_time`
- 出力:
  - 画面ログ
  - `output/simple_validation_model.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `preprocessDataWrapper`
  - `createTireModel`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB 組み込み:
  - `gpuDeviceCount`
  - `gpuDevice`
  - `randn`
  - `predict`
  - `tic` / `toc`
  - `exist`
  - `mkdir`
  - `save`
  - `rethrow`

## 5. 処理の流れ

1. GPU の有無を表示します。
2. 固定 CSV を読み込みます。
3. `split_ratio=0.9` で前処理を実行し、学習割合を 90% にします。
4. `createTireModel()` でモデルを作成します。
5. 300 エポック上限、バッチサイズ 512 の設定で学習します。
6. `evaluateModel` で性能評価を実行します。
7. `randn(1000,5)` に対する `predict` で推論速度を測ります。
8. `output/simple_validation_model.mat` に `trained_net`, `norm_params`, `metrics` を保存します。
9. `Fx/Fy RMSE < 100`, `Fx/Fy R² > 0.90`, `Speed < 1ms` を判定します。
10. 全条件を満たせば `VALIDATION PASSED`、そうでなければ警告付き完了を表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- `Step 1` から `Step 8` の順に構成されています。
- `metrics` の表示対象は `Fx`, `Fy`, `Mx`, `Mz` の 4 成分ですが、要件判定に使う R² は `Fx` と `Fy` だけです。
- 保存対象に `training_info` は含まれていません。

### `bool_str(val)`

- 真偽値を `PASS ✓` または `FAIL ✗` に変換するローカル関数です。
- 要件一覧の 3 行出力で使われます。

## 7. 注意点

- データセットは単一の DriveBrake CSV に固定です。
- `metrics.Mx` と `metrics.Mz` は表示されますが、要件判定の R² 条件には使われていません。
- コメントに「sampled data 用に緩和」とありますが、読み込むファイル名自体は sampled 版ではありません。
