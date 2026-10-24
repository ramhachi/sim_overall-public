# `matlab/dl_tire_model/generateTimeSeriesPlots.m`

## 1. 概要

`generateTimeSeriesPlots` は、実測系列と予測系列を時間軸上で重ね描きし、個別図 4 枚と統合図 1 枚を保存する関数です。

## 2. ファイルの役割

- 系列比較用の時系列プロットを出力ごとに生成する
- 必要に応じてサブサンプリングして描画負荷を下げる
- 全出力のまとめ図も保存する

## 3. インターフェース

### 関数

`generateTimeSeriesPlots(Y_actual, Y_pred, output_dir, varargin)`

### 引数

- `Y_actual`
  - `N×4` 行列。実測 `[Fx, Fy, Mx, Mz]`。
- `Y_pred`
  - `N×4` 行列。予測 `[Fx, Fy, Mx, Mz]`。
- `output_dir`
  - 保存先ディレクトリ。
- `varargin`
  - `inputParser` で解釈される任意引数。
  - `TimeVector` : 時間軸。既定では `1:N`。
  - `MaxSamples` : 描画最大点数。既定では `1000`。

### 戻り値

- なし

## 4. 依存関係

- MATLAB 標準関数
  - `inputParser`, `plot`, `sqrt`, `mean`, `linspace`, `round`, `saveas`, `close`, `mkdir`

## 5. 処理の流れ

1. `TimeVector` と `MaxSamples` を取得する。
2. `Y_actual` と `Y_pred` の列数・行数を検証する。
3. 出力ディレクトリを作成する。
4. `TimeVector` が空なら `1:n_samples` を使う。
5. サンプル数が `MaxSamples` を超える場合、`round(linspace(...))` で等間隔サブサンプリングする。
6. 各出力について
   - 実測と予測を重ね描きする
   - RMSE を計算して図中へ表示する
   - `timeseries_<output>.png` として保存する
7. `subplot(2,2,...)` で全出力の統合図を作り、`timeseries_combined.png` を保存する。

## 6. 関数・ローカル関数詳細

### `generateTimeSeriesPlots`

- 個別図では `Actual` を青実線、`Predicted` を赤破線で描きます。
- 統合図でも同じ系列を 2×2 サブプロットに配置します。
- RMSE は各出力ごとに `sqrt(mean((Y_pred(:,i)-Y_actual(:,i)).^2))` です。

### ローカル関数

- 定義されていません。

## 7. 注意点

- `TimeVector` の長さがサンプル数と一致するかは検証していません。
- 軸ラベルは常に `Sample Index` で、カスタム `TimeVector` を渡しても文言は変わりません。
- サブサンプリングはプロット用にのみ適用され、元データ自体の保存はしません。
