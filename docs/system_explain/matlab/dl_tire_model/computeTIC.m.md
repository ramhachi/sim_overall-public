# `matlab/dl_tire_model/computeTIC.m`

## 1. 概要

`computeTIC` は、実測値と予測値の各列について Theil Inequality Coefficient (TIC) を計算する関数です。

## 2. ファイルの役割

- `RMSE / (RMS(actual) + RMS(pred))` を列ごとに計算します。
- 出力スケールが異なる複数成分の比較指標を返します。
- 分母が 0 の場合は `NaN` を返します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
tic_value = computeTIC(y_actual, y_pred)
```

### 3.2 引数

- `y_actual`: `N×D` の実測値行列
- `y_pred`: `N×D` の予測値行列

### 3.3 戻り値

- `tic_value`: `1×D` の TIC ベクトル

## 4. 依存関係

- MATLAB 標準関数: `size`, `mean`, `sqrt`, `zeros`, `error`

## 5. 処理の流れ

1. 入力の行数・列数一致を確認します。
2. 出力列数分だけ結果ベクトルを確保します。
3. 各列について RMSE を計算します。
4. 同じ列について `RMS(actual)` と `RMS(pred)` を計算します。
5. 分母が 0 なら `NaN`、それ以外は TIC を格納します。

## 6. 関数・ローカル関数詳細

### 6.1 `computeTIC`

- RMSE 計算はこの関数内で完結しており、`computeRMSE` は呼びません。
- TIC は列ごとに独立計算されます。

## 7. 注意点

- コメントでは TIC の範囲を 0〜1 と説明していますが、コード側は範囲制約を行いません。
- 実測値と予測値が両方とも全ゼロの列は `NaN` になります。
