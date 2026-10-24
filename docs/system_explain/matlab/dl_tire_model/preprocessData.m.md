# `matlab/dl_tire_model/preprocessData.m`

## 1. 概要

CSV 群の読み込みから特徴量抽出、欠損除去、正規化、学習用/検証用分割までを一括実行する前処理関数です。

## 2. ファイルの役割

学習前処理の標準パイプラインを 1 本化した関数です。`loadTireData` でデータを読み込んだ後、`extractFeatures` と `extractTargets` で入力・教師を作成し、`removeMissingData`、`normalizeData`、`splitData` を順番に適用します。

## 3. インターフェース

- 形式: `[X_train, Y_train, X_val, Y_val, norm_params] = preprocessData(data_paths, config)`
- 引数:
  - `data_paths`: CSV ファイルパスのセル配列
  - `config`: 設定構造体
    - `split_ratio`（省略時 0.8）
- 戻り値:
  - `X_train`: 正規化済み学習入力
  - `Y_train`: 正規化済み学習教師
  - `X_val`: 正規化済み検証入力
  - `Y_val`: 正規化済み検証教師
  - `norm_params`: 正規化係数構造体
    - `input_mean`
    - `input_std`
    - `output_mean`
    - `output_std`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `extractFeatures`
  - `extractTargets`
  - `removeMissingData`
  - `normalizeData`
  - `splitData`
- MATLAB 組み込み:
  - `fprintf`
  - `nargin`
  - `isfield`
  - `size`
  - `struct`

## 5. 処理の流れ

1. 開始バナーを表示します。
2. `config` 未指定時は空構造体を作り、`split_ratio` 未指定時は `0.8` を設定します。
3. `loadTireData(data_paths)` でデータを読み込みます。
4. `extractFeatures(data)` と `extractTargets(data)` で 5 入力・4 出力を抽出します。
5. `removeMissingData(X, Y)` で NaN/Inf 行を除去します。
6. `normalizeData` を入力と出力にそれぞれ適用し、平均と標準偏差を保存します。
7. `norm_params` 構造体へ正規化係数を格納します。
8. `splitData(X_norm, Y_norm, config.split_ratio)` で学習用と検証用に分割します。
9. 学習件数、検証件数、入力名、出力名を表示して終了します。

## 6. 関数・ローカル関数詳細

### `preprocessData(data_paths, config)`

- 本ファイル唯一の公開関数です。
- データ読み込みを内部に含むため、呼び出し側は CSV パス集合を渡すだけで前処理全体を実行できます。
- 返却される `norm_params` は後段の推論関数や評価関数で再利用する前提です。

## 7. 注意点

- 入力データがテーブル済みの場合には使えず、ファイルパス前提です。その用途は `preprocessDataWrapper` が担います。
- 正規化は欠損除去後の全データに対して行われ、その後で分割します。
- 乱数シード設定はこの関数では行っておらず、分割の再現性は `splitData` 実装に依存します。
