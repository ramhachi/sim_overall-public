# `matlab/dl_tire_model/generateScatterPlots.m`

## 1. 概要

`generateScatterPlots` は、各出力について「実測値 vs 予測値」の散布図を作成し、完全一致線と R² 注記を付けて保存する関数です。

## 2. ファイルの役割

- `Fx/Fy/Mx/Mz` の予測精度を対角線比較で可視化する
- 各出力ごとに PNG を保存する
- 各図に R² を表示する

## 3. インターフェース

### 関数

`generateScatterPlots(Y_actual, Y_pred, output_dir)`

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
  - `figure`, `scatter`, `plot`, `mean`, `sum`, `saveas`, `axis`, `close`, `mkdir`

## 5. 処理の流れ

1. `Y_actual` と `Y_pred` の列数が 4 であることを確認する。
2. 行数一致を確認する。
3. 保存先ディレクトリを作成する。
4. 各出力について
   - 散布図を描く
   - 実測値と予測値の最小値・最大値から `y=x` の基準線を描く
   - R² を計算する
   - 図中へ R² を書く
   - 正方表示にして保存する
5. 完了メッセージを表示する。

## 6. 関数・ローカル関数詳細

### `generateScatterPlots`

- R² は `1 - SS_res / SS_tot` で算出します。
- 点の透明度は `MarkerFaceAlpha = 0.3` です。
- 保存ファイル名は `scatter_Fx.png` などの形式です。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `SS_tot == 0` の場合の特別処理はなく、R² は `NaN` や `Inf` になりえます。
- 出力単位は `Fx/Fy` が `N`、`Mx/Mz` が `Nm` に固定されています。
- 列数・行数以外の入力検証はありません。
