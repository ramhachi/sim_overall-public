# `matlab/dl_tire_model/run_ultra_deep_validation.m`

## 1. 概要

非常に深いネットワーク構成で検証するスクリプトです。8 隠れ層モデルの学習と評価を行います。

## 2. ファイルの役割

「Maximum depth with 8 hidden layers」という方針で、`createVeryDeepTireModel` を使った検証を実施します。`run_simple_validation.m` と比較する意図もログに含まれており、精度と推論速度をまとめて確認する実験スクリプトです。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `data_path = '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'`
  - `config.split_ratio = 0.9`
  - `train_config`
    - `max_epochs = 400`
    - `batch_size = 512`
    - `learning_rate = 0.002`
    - `patience = 300`
    - `val_frequency = 50`
    - `random_seed = 12`
  - `trained_net`, `training_info`, `metrics`, `pred_time`
- 出力:
  - 画面ログ
  - `output/ultra_deep_validation_model.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `preprocessDataWrapper`
  - `createVeryDeepTireModel`
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
2. DriveBrake CSV を読み込みます。
3. `split_ratio=0.9` で前処理し、学習 90%、検証 10% に分割します。
4. `createVeryDeepTireModel()` で超深層モデルを作成します。
5. 400 エポック上限、学習率 `0.002`、`patience=300` の設定で学習します。
6. `evaluateModel` で RMSE・R²・TIC を取得します。
7. ランダム 1000 サンプルで推論速度を計測します。
8. `output/ultra_deep_validation_model.mat` に結果一式を保存します。
9. Fx/Fy の RMSE、Fx/Fy/Mx/Mz の R²、速度の各条件を判定します。
10. 学習損失、検証損失、エポック数、モデル深さ、概算パラメータ数を表示します。
11. 最後に `run_simple_validation` との比較を促す注意書きを表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- `Step 1` から `Step 8` までの構成です。
- 判定基準は `run_improved_validation.m` と同様に、`Fx/Fy RMSE < 100`, `Fx/Fy R² > 0.95`, `Mx/Mz R² > 0.85`, `Speed < 1ms` です。
- `patience` の値は 300 で、ログ文の「Patience: 30」とは一致していません。

### `bool_str(val)`

- 要件判定表示用のローカル関数です。
- `true` のとき `PASS ✓`、`false` のとき `FAIL ✗` を返します。

## 7. 注意点

- コメント上は「8 hidden layers」とされ、ログでも同方針を明示しています。
- 概算パラメータ数 `~200k` は計算式ではなく固定文字列の表示です。
- 学習設定の表示文と実際の `train_config.patience` 値が一致していません。
