# `matlab/vehicle_8dof_bw/run_8dof_bw_verification.m`

## 役割

このスクリプトは、Bouc-Wen ダンパーと ARB を含む `vehicle_8dof_bw_model` の MATLAB 結果を、C++ 側比較 CSV と照合する検証スクリプトです。  
C++ 結果が見つからない場合は MATLAB 単体モードへフォールバックします。

## 実行入口

- スクリプトです。
- `clear; clc; close all;` で初期化します。

## 外部依存

- `vehicle_8dof_bw_model(params, t_span, dt, input_func)`
- `readtable`
- `saveas`
- 比較用 CSV の探索候補
  - `../output/bouc_wen_comparison.csv`
  - `../../output/bouc_wen_comparison.csv`
  - `bouc_wen_comparison.csv`

## パラメータ設定

このスクリプトは `params = struct();` から全項目を明示的に設定します。

### 質量・慣性

- `m_sprung = 200`
- `m_unsprung = [10,10,10,10]`
- `a = 0.813`, `b = 0.787`, `L = a + b`
- `track_f = 1.2`, `track_r = 1.18`
- `h_cg = 0.28`
- `Ixx = 32`, `Iyy = 95`, `Izz = 85`, `Iw = 0.15`

### サスペンション

- `k_spring = [25000, 25000, 30000, 30000]`
- `motion_ratio = [0.9, 0.9, 0.9, 0.9]`

### Bouc-Wen

- `bw_c0 = 2000`
- `bw_k0 = 0`
- `bw_alpha = 1500`
- `bw_A = 100`
- `bw_beta = 300`
- `bw_gamma = 0`
- `bw_n = 1`

### ARB

- `k_arb_f = 5000`
- `k_arb_r = 3000`

### タイヤ

- `radius = 0.228`
- `mu_x_max = 1.6`, `mu_y_max = 1.8`
- `Fz_nom = 1500`
- `Lx = 0.15`, `Ly = 0.25`
- `Cx = 80000`, `Cy = 60000`
- `v_N = 0.5`
- `k_tire = 200000`
- `c_tire = 500`

### 空力・駆動

- `rho = 1.2`
- `CL_A = 2.5`
- `CD_A = 1.0`
- `max_torque = 55`
- `gear_ratio = 3.0`
- `efficiency = 0.9`
- `v0 = 15`

## シミュレーション条件

- `t_span = [0, 3]`
- `dt = 0.001`
- `t_steer = 1.0`
- `delta = 0.15`
- 入力関数は `throttle = 0.1` 固定、操舵は 1 秒以降に `0.15` rad

## 処理の流れ

### 1. MATLAB 計算

`vehicle_8dof_bw_model` を実行し、経過時間を表示します。

### 2. C++ 結果探索

`search_paths` を順に `exist(..., 'file')` で調べ、最初に見つかったファイルを使います。

### 3. C++ 比較モード

CSV が見つかった場合のみ比較へ進みます。

#### ロール角比較

`phi_bw` 列が存在する場合だけ比較処理を行います。

- `phi_cpp = cpp_data.phi_bw`
- `phi_m = state_m(:,4) * 180 / pi`

MATLAB 側は `idx_m = round(t_cpp / dt) + 1` で最近傍サンプルを選び、線形補間は使っていません。  
`idx_m` は `min(idx_m, length(phi_m))` で末尾超過を防いでいます。

誤差評価は

- 最大誤差 `max_err`
- RMSE `rmse`

で、`max_err < 1.0` のときのみ合格です。

#### 条件付き比較項目

- `bw_force_fl` 列があれば、`data_m.F_damper_bw(:,1)` と比較
- `damper_z_fl` 列があれば、`state_m(:,33)` と比較

存在確認は `ismember(..., cpp_data.Properties.VariableNames)` で行います。

#### 図

2 行 2 列で次を表示します。

1. ロール角比較
2. ロール角誤差
3. フロント左 Bouc-Wen ダンパー力比較（列がある場合）
4. フロント左履歴変数比較（列がある場合）

図は `../../output/verify_8dof_bw.png` に保存します。

### 4. MATLAB 単体モード

CSV が見つからない場合は `warning('C++ result file not found. Running MATLAB-only mode.')` を出し、比較なしで結果だけを表示します。

表示内容は次の 4 図です。

1. ロール角
2. `vx`
3. 全輪 `F_damper_bw`
4. 全輪 `damper_z`

加えて、

- 最大ロール角
- 最終ヒステリシス変数 `damper_z(end,:)`

を表示します。

## 補足

- C++ ファイルが見つかっても `phi_bw` 列がなければ、比較図も合否表示も行われません。
- ローカル関数はありません。
