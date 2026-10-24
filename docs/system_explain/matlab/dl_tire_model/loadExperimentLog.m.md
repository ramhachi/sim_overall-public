# `matlab/dl_tire_model/loadExperimentLog.m`

## 1. 概要

`loadExperimentLog` は、実験 ID から `.mat` 形式の実験ログを読み込み、`experiment_log` 構造体を返す関数です。

## 2. ファイルの役割

- ログディレクトリの既定値を与える
- 実験 ID からログファイル名を組み立てる
- `experiment_log` 変数をロードして返す

## 3. インターフェース

### 関数

`experiment_log = loadExperimentLog(experiment_id, log_dir)`

### 引数

- `experiment_id`
  - 読み込みたい実験 ID。
- `log_dir`
  - ログ保存ディレクトリ。省略時は `'experiments/logs'`。

### 戻り値

- `experiment_log`
  - 読み込んだ実験ログ構造体。

## 4. 依存関係

- MATLAB 標準関数
  - `fullfile`, `isfile`, `load`, `fprintf`, `error`

## 5. 処理の流れ

1. `log_dir` が省略された場合は `'experiments/logs'` を設定する。
2. `fullfile(log_dir, [experiment_id, '.mat'])` で対象パスを作る。
3. ファイル存在を確認する。
4. `load` で読み込み、`loaded.experiment_log` を戻り値へ代入する。
5. 読み込んだ ID を `fprintf` で表示する。

## 6. 関数・ローカル関数詳細

### `loadExperimentLog`

- 返すのはファイル全体ではなく、その中の `experiment_log` 変数です。
- ロード後に中身のフィールド妥当性は検査しません。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `.mat` 内に `experiment_log` 変数が無い場合の防御処理はありません。
- ファイル名規約は `<experiment_id>.mat` 固定です。
