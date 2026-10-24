# tmeasy.hpp

## ファイルの目的
- 過渡変形状態付きの TMeasy タイヤモデルを提供します。

## 実行上の役割
- 低速特異点回避、荷重感度、緩和長、簡易セルフアライニングトルクを含むタイヤ力計算を行います。
- 単輪版と 4 輪バッチ版の両方をヘッダ内テンプレートで提供します。

## 依存関係
- 外部: `<cmath>`, `<algorithm>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `TMeasyParams`, `TMeasyState`, `Fz`, `vsx`, `vsy`, `vcx`
- 出力: `Fx`, `Fy`, `Mz`, `dxi_dt`, `deta_dt`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct TMeasyParams`
  - `mu_x_max`, `mu_y_max`, `Fz_nom` は基準摩擦・基準荷重です。
  - `Lx`, `Ly` は縦横緩和長です。
  - `Cx`, `Cy` は縦横剛性です。
  - `v_N` は低速特異点回避用仮想速度です。
  - `dmu_dFz` は荷重感度、`pneumatic_trail` は簡易 Mz 用です。
- `template<typename Scalar> struct TMeasyState`
  - 接地面変形状態 `xi`, `eta` を持ちます。
- `template<typename Scalar> struct TMeasyOutput`
  - `Fx`, `Fy`, `Mz`, `dxi_dt`, `deta_dt` を返します。
- `compute_tmeasy(...)`
  - `Fz` を 1N 以上にクランプし、`v_eff = sqrt(vcx^2 + v_N^2)` を使って低速特異点を抑えます。
  - 状態微分 `dxi_dt`, `deta_dt` を一次遅れで計算します。
  - 荷重依存摩擦 `mu_x`, `mu_y` を求め、`xi/eta` を `[-1,1] m` に飽和して正規化変形量へ変換します。
  - 合成スリップ `s` に対して放物線近似または飽和で `mu_eff` を作り、`Fx/Fy` を返します。
  - `Mz` は `-pneumatic_trail * Fy` の簡易近似です。
- `compute_tmeasy_batch(...)`
  - 4 輪分の SoA 配列を受け、各輪ごとに `compute_tmeasy()` を呼んで `Fx/Fy/dxi/deta` を埋めます。
