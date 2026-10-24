# `matlab/dl_tire_model/generateLookupTable.m`

## 1. 概要

`generateLookupTable` は、学習済みネットワークを 5 次元グリッド上で評価し、C++ 連携を想定したルックアップテーブルを `.mat` として保存する関数です。

## 2. ファイルの役割

- 5 入力軸のグリッドを生成する
- 全格子点で `predictTireForces` を呼んで 4 出力を計算する
- 補間に使える配列群とメタデータを `lookup_table` 構造体へまとめて保存する

## 3. インターフェース

### 関数

`generateLookupTable(trained_net, norm_params, output_path, grid_config)`

### 引数

- `trained_net`
  - 学習済みニューラルネットワーク。
- `norm_params`
  - 正規化パラメータ構造体。
- `output_path`
  - 保存先 `.mat` パス。
- `grid_config`
  - 任意の構造体。各フィールドは `[min, max, n_points]`。
  - `SR_range`, `SA_range`, `Fz_range`, `IA_range`, `TSTC_range`
  - 未指定時は関数内の既定値を使います。

### 戻り値

- なし
- 副作用として `output_path` に `lookup_table` を保存します。

## 4. 依存関係

- `predictTireForces`
- MATLAB 標準関数
  - `linspace`, `ndgrid`, `reshape`, `save`, `dir`, `fileparts`, `mkdir`, `tic`, `toc`

## 5. 処理の流れ

1. `grid_config` が無い場合、5 軸の既定レンジを設定する。
2. 各レンジから `min/max/n` を取り出す。
3. `linspace` で 1 次元配列を作る。
4. 総格子点数 `SR_n * SA_n * Fz_n * IA_n * TSTC_n` を計算し、100 万点超なら警告する。
5. `ndgrid` で 5 次元グリッドを作る。
6. グリッドを `N×5` の `X_grid` に平坦化する。
7. `predictTireForces(trained_net, X_grid, norm_params)` で `Y_grid` を計算する。
8. `Y_grid` の 4 列をそれぞれ 5 次元配列へ `reshape` する。
9. 軸配列、出力グリッド、次元情報、入出力名、単位を `lookup_table` 構造体へ格納する。
10. 保存先ディレクトリを必要に応じて作成し、`-v7.3` で保存する。
11. ファイルサイズと `interpn` を使う簡易利用例を表示する。

## 6. 関数・ローカル関数詳細

### `generateLookupTable`

- 既定グリッドは
  - `SR: [-0.2, 0.2, 20]`
  - `SA: [-15, 15, 30]`
  - `Fz: [500, 2500, 20]`
  - `IA: [-5, 5, 10]`
  - `TSTC: [20, 80, 10]`
  です。
- 出力は `Fx_grid`, `Fy_grid`, `Mx_grid`, `Mz_grid` の 4 つの 5 次元配列です。
- 保存構造体には `input_names`, `output_names`, `input_units`, `output_units` も追加されます。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `grid_config` を渡す場合、不足フィールドを既定値で補完する処理はありません。
- 全格子点を一括で推論するため、分解能を上げるとメモリ使用量と計算時間が急増します。
- コメントでは C++ 統合向けとされていますが、この関数自身は MATLAB の `.mat` 保存までしか行いません。
