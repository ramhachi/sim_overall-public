# `matlab/dl_tire_model/tmeasy_parameter_fitting.m`

## 1. 概要

`tmeasy_parameter_fitting.m` は、低荷重コーナリング実測データを 1000N 基準へ正規化し、そこから TMeasy パラメータ候補を推定・比較するスクリプトです。複数パラメータセットを並べて誤差評価し、比較図を保存します。

## 2. ファイルの役割

このスクリプトは、実データ由来の推定値、保守的な提案値、フォーミュラタイヤ標準値の 3 つを比較し、どのパラメータセットが実データへ近いかを確認する役割を持ちます。特に `C_alpha` と `mu_y` を実データから推定し、`C_kappa` と `mu_x` は仮定値として補います。

## 3. インターフェース

### スクリプトとしての入出力

- 入力ファイル
  - `../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`
- 出力
  - 標準出力への推定値、比較表、誤差指標、最終推奨値
  - `../../output/tire_params/tmeasy_fitting_comparison.png`

### 主要変数

- `Fz_min = 150`, `Fz_max = 200`
  - 抽出対象の実荷重帯。
- `Fz_ref = 1000`
  - 正規化先の基準荷重。
- `param_sets`
  - 比較対象 3 モデルの構造体配列。

## 4. 依存関係

- MATLAB 組み込み
  - `detectImportOptions`, `readtable`, `polyfit`, `interp1`, `mean`, `sqrt`, `semilogy`, `errorbar`, `saveas`
- ローカル関数
  - `tmeasy_force_model`
- 参照データ列
  - `FZ`, `FY`, `SA`, `SR`

## 5. 処理の流れ

### 5.1 実データの抽出と正規化

1. コーナリング CSV を読み込みます。
2. `150 < -FZ < 200` かつ `|SR| < 0.02` の行を抽出します。
3. `Fy_normalized = corn_filtered.FY .* (Fz_ref ./ (-corn_filtered.FZ))` で 1000N 基準に換算します。
4. `SA_bins = -12:1:12` 単位で平均 `Fy_data` とサンプル数を計算します。

### 5.2 実データ特性の算出

1. サンプル数があるビンだけを `SA_valid`, `Fy_valid` として抜き出します。
2. `Fy` の範囲、`SA=±3°`, `SA=±10°` 付近の値を `interp1` で確認します。

### 5.3 パラメータの直接推定

1. `max(abs(Fy_valid)) / Fz_ref` で `mu_y_data` を算出します。
2. `|SA| <= 2` の点を使って、対称性を考慮した `SA_combined`, `Fy_combined` を作ります。
3. `polyfit` の傾きから `C_alpha_data` を `N/rad` へ変換します。

### 5.4 比較対象モデルの構築

1. 実データ由来モデルを `param_sets(1)` として保存します。
2. 既定の保守的提案値を `param_sets(2)` に設定します。
3. 標準的なフォーミュラタイヤ値を `param_sets(3)` に設定します。
4. `SA = [-10, -5, 0, 5, 10]` で実データと各モデルを比較表示します。

### 5.5 誤差分析

1. 各モデルについて `SA_valid` 全点で `Fy_model_all` を計算します。
2. `MAE`, `MAPE`, `RMSE` を出力します。
3. `|Fy_valid| < 100` の点は `MAPE` 計算から除外します。

### 5.6 図生成と最終推奨出力

1. 実データと 3 モデルの `Fy-SA` 比較を描画します。
2. `|Fy|/Fz` を対数軸で示す摩擦係数プロットを描画します。
3. `SA=5°` における荷重依存性を描画します。
4. 各モデルの摩擦楕円を描画します。
5. 図を `../../output/tire_params/tmeasy_fitting_comparison.png` に保存します。
6. 最後に 2 つの推奨案を文章として出力します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 低荷重コーナリングデータの集計からパラメータ推定、複数候補の比較、誤差集計までを一括実行します。
- 実データ由来候補では `C_kappa_fit = 80000`, `mu_x_fit = 1.5` を仮定値として与えています。

### `tmeasy_force_model(SR, SA, Fz, params)`

- 入力 `params` は `C_alpha`, `C_kappa`, `mu_x`, `mu_y` を持つ構造体です。
- `SA` を rad へ変換し、線形力 `Fx_lin`, `Fy_lin` を計算します。
- 合力と上限力の比 `ratio` に応じて、`k = 1.8` の `tanh` 飽和を適用します。
- 方向を保ったまま力を縮退させた後、各軸上限へクランプします。
- 線形力が十分小さい場合は `Fx = 0`, `Fy = 0` を返します。

## 7. 注意点

- `C_kappa_fit` と `mu_x_fit` は「縦方向データがないので仮定」とコメントされており、実測からの推定ではありません。
- `Fy_valid` はビン平均値なので、元データ点群そのものを直接フィットしているわけではありません。
- `Fy_sweep_data` は `corn_filtered` から `SA≈5°` かつ特定荷重近傍を探しますが、データが無い荷重点では `NaN` になります。
