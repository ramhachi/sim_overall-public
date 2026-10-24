# `matlab/dl_tire_model/magic_formula_fitting.m`

## 1. 概要

`magic_formula_fitting.m` は、Pacejka Magic Formula 5.2 形式のタイヤモデルを段階的にフィッティングするスクリプトです。純横力、純縦力、複合スリップ、セルフアライニングトルクの 4 段階でパラメータを推定し、検証・可視化・保存まで行います。

## 2. ファイルの役割

- `Cr_*` と `Db_*` の CSV 群を読み込む
- データを純横力・純縦力・複合スリップ用に分割する
- `lsqnonlin` で MF パラメータを推定する
- 評価指標、検証表、可視化、保存ファイルを出力する

## 3. インターフェース

### スクリプトの主要変数・入出力

- 外部から与えうる変数
  - `run_from_launcher`
    - 未定義なら `clear; clc; close all;` を実行します。
  - `tire_model`
    - 未定義なら `'Hoosier_18p0X6p0-10_R20'`。
  - `data_dir`
    - 未定義なら `data/tiredata/csv` または `../../data/tiredata/csv` を選びます。
- 固定設定
  - `Fz0 = -890`
  - `R0 = 18.0 / 2 * 0.0254`
  - `db_exclude_runs = {'Run068','Run071'}`
  - `cr_sr_threshold = 0.05`
  - `db_sa_threshold_pure = 1.0`
  - `max_subsample = 20000`
- 主な出力
  - `output/magic_formula_<tire_model>.png`
  - `output/magic_formula_<tire_model>.mat`
  - ワークスペース変数 `mf_params`, `mf_metrics`, 各段階のパラメータ配列

## 4. 依存関係

- プロジェクト関数
  - `loadTireData`
- 最適化・数値計算
  - `lsqnonlin`, `optimoptions`, `polyfit`, `prctile`, `randperm`
- MATLAB 標準関数
  - `dir`, `fullfile`, `sort`, `deg2rad`, `mean`, `median`, `sqrt`, `saveas`, `save`

## 5. 処理の流れ

### セクション: ユーザ設定

- 実行起点に応じてワークスペース初期化を切り替えます。
- `tire_model`, `data_dir`, `Fz0`, `R0`, 除外 Run、閾値、サブサンプル数、乱数シードを設定します。

### セクション: Step 1 データ読み込み

- `Cr_<tire_model>*.csv` を列挙し、`loadTireData` で `cr_raw` を作ります。
- `Db_<tire_model>*.csv` を列挙し、`Run068` と `Run071` を除外した後に `db_raw` を作ります。

### セクション: Step 2 データ準備

- `cr_raw` から `|SR| < cr_sr_threshold` の `cr_pure` を作ります。
- `db_raw` から `|SA| < db_sa_threshold_pure` の `db_pure` を作ります。
- `|SA| >= db_sa_threshold_pure` を `db_comb` とします。
- 3 つの table を `subsample` で `max_subsample` 件まで間引き、`cr_fit`, `db_fit`, `db_comb_fit` を作ります。

### セクション: Stage 1 純横力 Fy0 フィッティング

- `cr_fit` から `alpha_fy`, `Fz_fy`, `gamma_fy`, `Fy_meas` を抽出します。
- 15 パラメータの `fy_names`、初期値、上下限を定義します。
- `estimateStiffness(...,'lateral')` で初期剛性の推定値を作ります。
- `obj_fy = calc_Fy0(...) - Fy_meas` を `lsqnonlin` で最小化します。
- 結果を `cr_pure` 全体で再評価し、RMSE と R² を算出します。

### セクション: Stage 2 純縦力 Fx0 フィッティング

- `db_fit` から `kappa_fx`, `Fz_fx`, `gamma_fx`, `Fx_meas` を抽出します。
- Phase 1 では `Fz ≈ Fz0` の参照データに対し 4 パラメータ簡易 MF を 10 回マルチスタートでフィットします。
- Phase 2 では Phase 1 の結果を元に 15 パラメータ版 `calc_Fx0` を 5 回マルチスタートでフィットします。
- 結果を `db_pure` 全体で再評価します。

### セクション: Stage 3 複合スリップ フィッティング

- `db_comb_fit` を用い、既に得た `Fx0` と `Fy0` を基礎モデルとして使います。
- `Gxα` は `Fx` の減少係数、`Gyκ` は `Fy` の減少係数として別々に `lsqnonlin` で推定します。
- 評価は `db_raw` 全体に対して行い、複合 `Fx/Fy` の RMSE と R² を出します。

### セクション: Stage 4 Mz フィッティング

- `cr_fit` から `alpha_mz`, `Fz_mz`, `gamma_mz`, `Mz_meas` を抽出します。
- `fy_params` から `Fy0_mz`, `Kya_mz`, `SHy_mz`, `SVy_mz` を計算し、`calc_Mz` の入力に使います。
- 12 パラメータの `mz_x0_base` を元に、10 回マルチスタートで `lsqnonlin` を実行します。
- 結果を `cr_pure` 全体で再評価します。

### セクション: Step 3 検証

- `SR=0, SA=0` のゼロスリップ点で `Fx/Fy/Mz` を荷重別に表示します。
- `SR=0, Fz=800N` の `SA sweep` を表示します。
- `SA=0, Fz=800N` の `SR sweep` を表示します。

### セクション: Step 4 可視化

- `cr_pure`, `db_pure`, `db_raw` を可視化用に最大 5000 点へ再サブサンプルします。
- 12 個のサブプロットで実測対予測、残差、低減係数曲線などを描きます。
- `output/magic_formula_<tire_model>.png` を保存します。

### セクション: Step 5 保存

- `mf_params` に `Fy0/Fx0/Gxa/Gyk/Mz` を名前付き構造体として格納します。
- `mf_metrics` に各段階の RMSE と R² を格納します。
- パラメータ配列と名前配列も含めて `.mat` 保存します。

### セクション: 最終サマリー

- モデルごとの RMSE/R² とパラメータ総数をコンソールへ表示します。

## 6. 関数・ローカル関数詳細

### `calc_Fy0(p, alpha, Fz, gamma, Fz0)`

- 純横力 MF を計算します。
- `p` は 15 要素で、形状係数、ピーク係数、剛性、曲率、水平/垂直シフトを持ちます。
- `dfz`, `SHy`, `SVy`, `Cy`, `Dy`, `Kya`, `By`, `Ey` を経由して `Fy0` を返します。

### `calc_Fx0(p, kappa, Fz, gamma, Fz0)`

- 純縦力 MF を計算します。
- 15 要素パラメータから `SHx`, `SVx`, `Cx`, `Dx`, `Kxk`, `Bx`, `Ex` を作り、`Fx0` を返します。

### `calc_Gxa(p, alpha, kappa, Fz, Fz0)`

- 複合スリップ時の `Fx` 低減係数 `Gxα` を返します。
- `α=0` で 1 になるよう分母で正規化し、最後に `max(Gxa, 0)` を適用します。

### `calc_Gyk(p, alpha, kappa, Fz, Fz0)`

- 複合スリップ時の `Fy` 低減係数 `Gyκ` を返します。
- `κ=0` で 1 になるよう正規化し、最後に `max(Gyk, 0)` を適用します。

### `calc_Mz(p, alpha, Fz, gamma, Fy0, Kya, SHy, SVy, Fz0, R0)`

- `Mz = -trail * Fy0 + Mzr` を計算します。
- ニューマチックトレイルと残留トルクを別々に計算し、合成します。

### `calcKya(fy_params, Fz, gamma, Fz0)`

- `fy_params` からコーナリング剛性 `Kya` だけを再計算する補助関数です。

### `estimateStiffness(slip, force, Fz, Fz0, type)`

- `Fz ≈ Fz0` 近傍の小スリップ領域を使い、線形回帰で初期剛性を推定します。
- `type='lateral'` では `SA` を rad に変換して使います。

### `subsample(data, max_n)`

- `height(data)` が `max_n` を超える場合に `randperm` でランダム抽出します。

### `printParams(name, names, vals)`

- パラメータ名と値を整形表示します。

## 7. 注意点

- Stage 2 の参照荷重抽出は `abs(db_fit.Fz - Fz0) < 0.3 * Fz0` と実装されており、`Fz0=-890` をそのまま使っています。
- 検証部の `Fz_test = [200, 500, 800, 1100, 1400]` や `Fz=800` の sweep は、設定済み `Fz0=-890` と符号系が異なります。
- `lsqnonlin` と `optimoptions` を使うため、Optimization Toolbox 前提です。
- ローカル関数はこのファイル専用で、別ファイルから直接再利用する設計ではありません。
