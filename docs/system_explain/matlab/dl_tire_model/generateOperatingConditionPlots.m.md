# `matlab/dl_tire_model/generateOperatingConditionPlots.m`

## 1. 概要

`generateOperatingConditionPlots` は、入力特徴量のうち `SR`、`SA`、`Fz` をビン分割し、各ビンごとの RMSE と R² を `Fx/Fy/Mx/Mz` について可視化する関数です。

## 2. ファイルの役割

- 運転条件ごとの予測性能を集計する
- 条件別 RMSE 棒グラフを保存する
- 条件別 R² 棒グラフを保存する
- まとめ用ヒートマップを保存する

## 3. インターフェース

### 関数

`generateOperatingConditionPlots(X_features, Y_actual, Y_pred, output_dir, varargin)`

### 引数

- `X_features`
  - `N×5` 行列。列順は `[SR, SA, Fz, IA, TSTC]`。
- `Y_actual`
  - `N×4` 行列。実測 `[Fx, Fy, Mx, Mz]`。
- `Y_pred`
  - `N×4` 行列。予測 `[Fx, Fy, Mx, Mz]`。
- `output_dir`
  - 図の保存先。
- `varargin`
  - `inputParser` で解釈される任意引数。
  - `NumBins` : ビン数。既定値は `5`。

### 戻り値

- なし

## 4. 依存関係

- MATLAB 標準関数
  - `inputParser`, `histcounts`, `mean`, `sum`, `sqrt`, `bar`, `heatmap`, `saveas`, `mkdir`

## 5. 処理の流れ

1. `inputParser` で `NumBins` を読む。
2. `X_features` が 5 列、`Y_actual` と `Y_pred` が 4 列で、行数が一致することを検証する。
3. 出力ディレクトリを必要に応じて作成する。
4. `feature_names` などの表示用ラベルを定義する。
5. 解析対象を `key_features = [1, 2, 3]`、すなわち `SR/SA/Fz` に限定する。
6. 各特徴量について
   - `histcounts` でビン分けする
   - 各ビン・各出力の残差から RMSE を計算する
   - 各ビン・各出力の R² を計算する
   - RMSE 図を保存する
   - R² 図を保存する
7. 最後に `heatmap` で `feature × output` のサマリ図を保存する。

## 6. 関数・ローカル関数詳細

### `generateOperatingConditionPlots`

- RMSE は `sqrt(mean((Y_pred - Y_actual).^2))` をビン単位で計算します。
- R² は `1 - SS_res / SS_tot` で計算し、`SS_tot == 0` の場合は `NaN` を入れます。
- サンプル数 `n` は各棒の上に `text` で注記されます。
- 保存ファイル名は主に以下です。
  - `rmse_by_SR.png`
  - `rmse_by_SA.png`
  - `rmse_by_Fz.png`
  - `r2_by_SR.png`
  - `r2_by_SA.png`
  - `r2_by_Fz.png`
  - `rmse_heatmap_summary.png`

### ローカル関数

- 定義されていません。

## 7. 注意点

- ビン解析の対象は `SR/SA/Fz` の 3 特徴量のみで、`IA` と `TSTC` は使いません。
- R² 図は `ylim([0, 1])` に固定されているため、負の R² は表示範囲外になります。
- サマリヒートマップの `heatmap_data` は、実装上は各特徴量ごとに同じ全サンプル残差 RMSE を再計算しており、特徴量別の差は反映されません。
