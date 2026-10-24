# `matlab/dl_tire_model/logExperiment.m`

## 1. 概要

`logExperiment` は、ハイパーパラメータ設定、性能指標、学習情報を実験 ID 付きで保存する関数です。`.mat` とテキストの 2 形式でログを残します。

## 2. ファイルの役割

- タイムスタンプベースの実験 ID を作る
- 実験ログ構造体を構築する
- `.mat` 形式で保存する
- 人間が読みやすい `.txt` サマリも出力する

## 3. インターフェース

### 関数

`experiment_id = logExperiment(config, metrics, training_info, log_dir)`

### 引数

- `config`
  - ハイパーパラメータ設定構造体。
- `metrics`
  - `evaluateModel` 由来を想定した性能指標構造体。
- `training_info`
  - `trainTireModel` 由来を想定した学習情報構造体。
- `log_dir`
  - 保存ディレクトリ。省略時は `'experiments/logs'`。

### 戻り値

- `experiment_id`
  - `exp_yyyymmdd_HHMMSS` 形式の ID。

## 4. 依存関係

- MATLAB 標準関数
  - `exist`, `mkdir`, `datestr`, `now`, `datetime`, `fullfile`, `save`, `fopen`, `fprintf`, `fclose`, `mat2str`, `num2str`

## 5. 処理の流れ

1. `log_dir` の既定値を設定する。
2. ディレクトリが無ければ作成する。
3. 現在時刻から `experiment_id` を生成する。
4. `experiment_log` 構造体へ `experiment_id`, `timestamp`, `config`, `metrics`, `training_info` を格納する。
5. `<experiment_id>.mat` として保存する。
6. `<experiment_id>.txt` を開き、設定・学習情報・性能指標・要求達成可否を書き出す。
7. 保存先と実験 ID をコンソール表示する。

## 6. 関数・ローカル関数詳細

### `logExperiment`

- テキストログでは `config` に以下のフィールドがある前提です。
  - `learning_rate`, `batch_size`, `max_epochs`, `patience`, `val_frequency`, `random_seed`, `model_type`, `hidden_sizes`, `dropout_rate`, `split_ratio`
- `training_info` には少なくとも
  - `TrainingTime`, `FinalValidationLoss`, `TrainingLoss`
  を期待しています。
- `metrics` には `Fx/Fy/Mx/Mz` 各構造体の `RMSE`, `R2`, `TIC`, `MAE` と、全体の `meets_requirements` を期待しています。

### ローカル関数

- 定義されていません。

## 7. 注意点

- 入力構造体の必要フィールド検証はありません。欠損すると `fprintf` 時点で失敗します。
- テキストログの生成中に例外が起きた場合は `catch` で `fclose(fid)` 後に再送出します。
- 実験 ID は秒単位タイムスタンプなので、同一秒内の複数呼び出しでは重複しうります。
