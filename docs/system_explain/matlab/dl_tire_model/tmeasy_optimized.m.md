# `matlab/dl_tire_model/tmeasy_optimized.m`

## 1. 概要

`tmeasy_optimized.m` は、コーナリング実測データを参照しながら、TMeasy 風の簡略タイヤモデルのパラメータを確認・比較するスクリプトです。実データから `C_alpha` と `mu_y` の概算値を出力しつつ、固定した最適化後パラメータで表・図を生成します。

## 2. ファイルの役割

このスクリプトは、`Fz ≈ 1000 N` かつ `SR ≈ 0` のコーナリングデータを基準に、TMeasy モデルの飽和特性が実データにどの程度近いかを簡易検証する用途です。最終的に推奨パラメータを表示し、横力・縦力・摩擦楕円の可視化画像を保存します。

## 3. インターフェース

### スクリプトとしての入出力

- 入力ファイル
  - `../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`
- 生成物
  - 標準出力への集計表・推奨値表示
  - `../../output/tire_params/tmeasy_optimized.png`

### スクリプト内の主要変数

- `Fz_target = 1000`
  - 比較対象とする基準垂直荷重。
- `Fz_tol = 100`
  - `Fz ≈ 1000 N` 抽出の許容幅。
- `C_alpha_opt = 35000`, `C_kappa_opt = 60000`, `mu_x_opt = 1.5`, `mu_y_opt = 1.6`
  - モデル比較に使う固定パラメータ。

## 4. 依存関係

- MATLAB 組み込み
  - `clear`, `clc`, `fprintf`, `detectImportOptions`, `readtable`, `abs`, `polyfit`, `interp1`, `figure`, `subplot`, `plot`, `legend`, `grid`, `axis`, `saveas`
- ローカル関数
  - `tmeasy_force_opt`
- 入力データ列
  - `SA`, `SR`, `FY`, `FZ` を参照します。

## 5. 処理の流れ

### 5.1 実データ読み込み

1. コーナリング CSV の import options を作成します。
2. `DataLines = [3, Inf]`、固定 `VariableNames` を設定して `readtable` で読み込みます。
3. `abs(-corn_data.FZ - Fz_target) < Fz_tol` と `abs(corn_data.SR) < 0.02` で比較対象データを抽出します。

### 5.2 実データの SA-Fy 集計

1. `SA_bins = -12:1:12` を用意します。
2. 各ビンの ±0.5 deg 範囲で `FY` の平均値と標準偏差を計算します。
3. `Fy_mean(i) ~= 0` の点だけ標準出力へ表示します。

### 5.3 実データからの概算値表示

1. `0.5 < |SA| < 3` の点で線形近似し、`C_alpha_estimated` を出力します。
2. `|SA| > 8` の点から `mu_y_estimated` を算出して表示します。
3. ただし、後続計算で使うのは推定値ではなく、固定値 `C_alpha_opt` などです。

### 5.4 固定パラメータでの比較表作成

1. `SA_test = [-12, -10, -8, -5, -3, -1, 0, 1, 3, 5, 8, 10, 12]` を走査します。
2. 実データは `interp1(SA_bins, Fy_mean, SA, 'linear', 0)` で取得します。
3. モデル値は `tmeasy_force_opt(0, SA, Fz_target, ...)` で計算します。
4. 実データ絶対値が 10 より大きい点では相対誤差[%]を算出し、表形式で表示します。

### 5.5 荷重依存とスリップ率スイープ

1. `Fz_levels = [500, 750, 1000, 1250, 1500]`、`SA = 5, 10` で `Fy` と有効摩擦係数 `mu_eff` を表示します。
2. `SR_range = [-0.15, -0.10, ... , 0.15]` で `Fx` を表示します。

### 5.6 結論表示と可視化

1. 推奨パラメータをテキストで出力します。
2. 3 分割図として以下を描画します。
   - `Fy vs SA`
   - `Fx vs SR`
   - 摩擦楕円と複合スリップ点
3. `../../output/tire_params/tmeasy_optimized.png` に保存します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 実データを読み、簡易推定値を表示しつつ、最終的には固定パラメータでモデル比較を行います。
- 推定結果と採用値が必ずしも一致しない点が、このファイルの重要な特徴です。

### `tmeasy_force_opt(SR, SA, Fz, C_kappa, C_alpha, mu_x, mu_y)`

- 入力
  - `SR`: スリップ率
  - `SA`: スリップ角[deg]
  - `Fz`: 垂直荷重[N]
  - `C_kappa`, `C_alpha`, `mu_x`, `mu_y`: モデル係数
- 出力
  - `Fx`, `Fy`
- 処理
  1. `SA` を rad に変換します。
  2. 線形域の力 `Fx_lin`, `Fy_lin` を計算します。
  3. 線形力の合力 `F_lin_mag` と摩擦上限 `F_max` を計算します。
  4. `ratio = F_lin_mag / F_max` に対し、`k = 2.0` の `tanh` ベース飽和で縮尺 `scale` を求めます。
  5. 方向を保ったまま `Fx`, `Fy` を縮退させ、最後に各軸で `[-Fx_max, Fx_max]`, `[-Fy_max, Fy_max]` にクランプします。
  6. 線形力がほぼ 0 のときは `Fx = 0`, `Fy = 0` を返します。

## 7. 注意点

- コメント内で推定した `C_alpha_estimated` と `mu_y_estimated` は表示目的で、採用値は固定定数です。
- 実データ抽出は `Fz ≈ 1000 N` に限定されるため、他荷重域との整合は別途確認が必要です。
- 実データのない `SA` 点は `interp1(..., 0)` により 0 補間されるため、比較表の誤差解釈には注意が必要です。
