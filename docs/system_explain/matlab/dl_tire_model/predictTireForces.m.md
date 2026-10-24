# `matlab/dl_tire_model/predictTireForces.m`

## 1. 概要

生のタイヤ状態量を受け取り、学習済みネットワークで推論した後、出力を物理量へ戻して返す関数です。

## 2. ファイルの役割

この関数は、外部呼び出し側が正規化・逆正規化を意識せずに `predict()` を使えるようにする推論ラッパーです。入力列数と `norm_params` の必須フィールドを検証し、入力正規化→ネットワーク推論→出力逆正規化の 3 段階を一貫して実行します。

## 3. インターフェース

- 形式: `Y = predictTireForces(net, X_raw, norm_params)`
- 引数:
  - `net`: 学習済みニューラルネットワーク
  - `X_raw`: `N×5` の生入力行列。列順は `[SR, SA, Fz, IA, TSTC]`
  - `norm_params`: 正規化係数構造体
    - `input_mean`
    - `input_std`
    - `output_mean`
    - `output_std`
- 戻り値:
  - `Y`: `N×4` の推論結果。列順は `[Fx, Fy, Mx, Mz]`
- エラー条件:
  - 引数が 3 個未満
  - `X_raw` の列数が 5 でない
  - `norm_params` に必須フィールドが欠けている

## 4. 依存関係

- MATLAB 組み込み:
  - `nargin`
  - `size`
  - `isfield`
  - `error`
  - `predict`
- 外部ファイル依存:
  - なし。ただし `net` と `norm_params` は呼び出し元で準備済みである必要があります。

## 5. 処理の流れ

1. `nargin < 3` を確認し、不足時は `error` で停止します。
2. `size(X_raw, 2)` が 5 であることを確認します。
3. `required_fields = {'input_mean','input_std','output_mean','output_std'}` を順に検査します。
4. `X_norm = (X_raw - norm_params.input_mean) ./ norm_params.input_std` で入力を正規化します。
5. `Y_norm = predict(net, X_norm)` で推論します。
6. `Y = Y_norm .* norm_params.output_std + norm_params.output_mean` で物理量へ戻します。

## 6. 関数・ローカル関数詳細

### `predictTireForces(net, X_raw, norm_params)`

- 本ファイル唯一の公開関数です。
- 入力特徴量と出力物理量の列順をコメントで明示しています。
- 正規化係数の有無をフィールド単位でチェックするため、構造体の一部欠落も検出できます。
- 推論そのものは `predict(net, X_norm)` に委譲しています。

## 7. 注意点

- `input_std` や `output_std` が 0 を含む場合の防御は入っていません。
- `X_raw` の行数と `norm_params` 各ベクトルの長さ整合性は明示検証していません。
- 出力単位はコメント上で `Fx/Fy: N`, `Mx/Mz: Nm` とされています。
