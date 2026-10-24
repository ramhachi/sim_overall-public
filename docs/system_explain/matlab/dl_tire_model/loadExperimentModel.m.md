# `matlab/dl_tire_model/loadExperimentModel.m`

## 1. 概要

`loadExperimentModel` は、実験 ID に対応する保存済みモデルを読み込み、学習済みネットワークと正規化パラメータを返す関数です。

## 2. ファイルの役割

- 既定モデルディレクトリを与える
- 実験 ID からモデルファイル名を組み立てる
- `trained_net` と `norm_params` をロードして返す

## 3. インターフェース

### 関数

`[trained_net, norm_params] = loadExperimentModel(experiment_id, model_dir)`

### 引数

- `experiment_id`
  - 実験 ID。
- `model_dir`
  - モデル保存ディレクトリ。省略時は `'experiments/models'`。

### 戻り値

- `trained_net`
  - 学習済みネットワーク。
- `norm_params`
  - 正規化パラメータ構造体。

## 4. 依存関係

- MATLAB 標準関数
  - `sprintf`, `fullfile`, `isfile`, `load`, `fprintf`, `error`

## 5. 処理の流れ

1. `model_dir` が省略された場合は `'experiments/models'` を使う。
2. ファイル名を `sprintf('%s_model.mat', experiment_id)` で作る。
3. `fullfile` で完全パスを作る。
4. ファイル存在を確認する。
5. `load(model_path)` で読み込む。
6. `loaded.trained_net` と `loaded.norm_params` を返す。
7. 読み込んだ実験 ID を表示する。

## 6. 関数・ローカル関数詳細

### `loadExperimentModel`

- 実験 ID に `_model.mat` を付けたファイル名規約に依存します。
- 返す変数名はファイル内の `trained_net`, `norm_params` 固定です。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `.mat` 内に期待変数が存在するかの検証はありません。
- `model_dir` が異なる環境では明示指定が必要です。
