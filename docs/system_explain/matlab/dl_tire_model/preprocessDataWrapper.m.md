# `matlab/dl_tire_model/preprocessDataWrapper.m`

## 1. 概要

テーブル入力とファイルパス入力の両方を受け付ける前処理ラッパー関数です。

## 2. ファイルの役割

`preprocessData` とほぼ同じ前処理手順を持ちつつ、入力型を柔軟化する役割を担います。既に読み込み済みのテーブルをそのまま処理する経路と、セル配列から `loadTireData` を呼ぶ経路を切り替えます。

## 3. インターフェース

- 形式: `[X_train, Y_train, X_val, Y_val, norm_params] = preprocessDataWrapper(data_input, config)`
- 引数:
  - `data_input`: テーブル、またはファイルパスのセル配列
  - `config`: 設定構造体
    - `split_ratio`（省略時 0.8）
- 戻り値:
  - `X_train`, `Y_train`, `X_val`, `Y_val`
  - `norm_params`
- エラー条件:
  - `data_input` がテーブルでもセル配列でもない

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `loadTireData`
  - `extractFeatures`
  - `extractTargets`
  - `removeMissingData`
  - `normalizeData`
  - `splitData`
- MATLAB 組み込み:
  - `nargin`
  - `isfield`
  - `istable`
  - `iscell`
  - `fprintf`
  - `error`
  - `size`

## 5. 処理の流れ

1. `config` がなければ空構造体を作成し、`split_ratio` 未指定時は `0.8` を設定します。
2. `data_input` がテーブルならそのまま `data` に代入します。
3. `data_input` がセル配列なら `loadTireData(data_input)` で読み込みます。
4. どちらでもなければ `error` を送出します。
5. 以降は `preprocessData` と同じ順序で、特徴量抽出→欠損除去→正規化→分割を実行します。
6. 最後に件数と入出力次元の要約を表示します。

## 6. 関数・ローカル関数詳細

### `preprocessDataWrapper(data_input, config)`

- 本ファイル唯一の公開関数です。
- ラッパーという名称ですが、`preprocessData` を直接呼ばず、内部で同等の処理を展開しています。
- 前処理のログ文言は `preprocessData` に近いですが、読み込み段階が入力型分岐に置き換わっています。

## 7. 注意点

- テーブル入力時は `loadTireData` を通らないため、列構成は `extractFeatures` / `extractTargets` が受け付ける形である必要があります。
- `stats` は `removeMissingData` から受け取っていますが、表示以外には返していません。
- `preprocessData` と機能が重複しているため、変更時に挙動差が生じやすい構成です。
