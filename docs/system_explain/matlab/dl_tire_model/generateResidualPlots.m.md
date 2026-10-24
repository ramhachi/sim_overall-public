# `matlab/dl_tire_model/generateResidualPlots.m`

## 1. 概要

`generateResidualPlots` は、`Fx/Fy/Mx/Mz` の各出力について、実測値に対する残差 `Predicted - Actual` を散布図で保存する関数です。

## 2. ファイルの役割

- 残差分布を出力ごとに可視化する
- 平均、標準偏差、RMSE を図中へ書き込む
- 各出力を個別 PNG として保存する

## 3. インターフェース

### 関数

`generateResidualPlots(Y_actual, Y_pred, output_dir)`

### 引数

- `Y_actual`
  - `N×4` 行列。実測 `[Fx, Fy, Mx, Mz]`。
- `Y_pred`
  - `N×4` 行列。予測 `[Fx, Fy, Mx, Mz]`。
- `output_dir`
  - 保存先ディレクトリ。

### 戻り値

- なし

## 4. 依存関係

- MATLAB 標準関数
  - `figure`, `scatter`, `plot`, `mean`, `std`, `sqrt`, `saveas`, `close`, `mkdir`

## 5. 処理の流れ

1. `Y_actual` と `Y_pred` が 4 列であることを確認する。
2. 行数一致を確認する。
3. 出力ディレクトリを必要に応じて作成する。
4. `Fx/Fy/Mx/Mz` ごとに
   - 残差を計算する
   - 実測値 vs 残差の散布図を描く
   - `y=0` の基準線を描く
   - 平均、標準偏差、RMSE を計算し注記する
   - `residual_<output>.png` で保存して図を閉じる
5. 完了メッセージを表示する。

## 6. 関数・ローカル関数詳細

### `generateResidualPlots`

- 出力名は `{'Fx','Fy','Mx','Mz'}`、単位は `{'N','N','Nm','Nm'}` に固定されています。
- 残差は `Y_pred(:,i) - Y_actual(:,i)` で定義されています。
- 各図は個別ウィンドウで生成し、保存後に `close(gcf)` で明示的に閉じます。

### ローカル関数

- 定義されていません。

## 7. 注意点

- 残差の定義は「予測−実測」です。
- 軸範囲はデータに応じて自動決定されます。
- 入力の内容妥当性は列数と行数のみを確認し、値域や `NaN` は検査しません。
