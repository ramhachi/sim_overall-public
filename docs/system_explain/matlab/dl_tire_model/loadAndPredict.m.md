# `matlab/dl_tire_model/loadAndPredict.m`

## 1. 概要

`loadAndPredict` は、保存済みモデルファイルを読み込み、その場で `predictTireForces` を実行する簡易ラッパー関数です。

## 2. ファイルの役割

- モデル `.mat` の存在確認
- `trained_net` と `norm_params` の取得
- 生入力 `X_raw` に対する推論実行

## 3. インターフェース

### 関数

`Y = loadAndPredict(model_path, X_raw)`

### 引数

- `model_path`
  - エクスポート済み `.mat` ファイルのパス。
- `X_raw`
  - `N×5` の生入力行列。コメントでは `[SR, SA, Fz, IA, TSTC]`。

### 戻り値

- `Y`
  - `N×4` の推論結果。コメントでは `[Fx, Fy, Mx, Mz]`。

## 4. 依存関係

- `predictTireForces`
- MATLAB 標準関数
  - `isfile`, `load`, `isfield`, `error`

## 5. 処理の流れ

1. `model_path` の存在を確認する。
2. `load(model_path)` でモデル構造体を読み込む。
3. 読込結果に `trained_net` があることを確認する。
4. 読込結果に `norm_params` があることを確認する。
5. `predictTireForces(model.trained_net, X_raw, model.norm_params)` を呼ぶ。
6. 推論結果 `Y` を返す。

## 6. 関数・ローカル関数詳細

### `loadAndPredict`

- モデルのロードと推論を 1 回の関数呼び出しにまとめています。
- モデルファイルは MATLAB 変数名として `trained_net` と `norm_params` を含む前提です。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `X_raw` の列数や型はこの関数では検証していません。
- モデルファイルに期待名の変数が無い場合は `error` で停止します。
- 推論ロジック自体は `predictTireForces` に委譲されています。
