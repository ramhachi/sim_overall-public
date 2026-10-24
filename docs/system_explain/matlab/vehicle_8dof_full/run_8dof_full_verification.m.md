# `matlab/vehicle_8dof_full/run_8dof_full_verification.m`

## 役割

このスクリプトは、Bouc-Wen と LuGre を含む `vehicle_8dof_full_model` の結果を検証するための実行スクリプトです。  
比較相手の C++ 結果がある場合はロール角と一部内部力・内部状態を比較し、ない場合は MATLAB 単体モードで結果を可視化します。

## 実行入口

- スクリプトです。
- `clear; clc; close all;` を実行します。

## 外部依存

- `vehicle_8dof_full_model(params, t_span, dt, input_func)`
- `readtable`
- `saveas`
- 比較 CSV 探索候補
  - `../../output/lugre_comparison.csv`
  - `../output/lugre_comparison.csv`
  - `lugre_comparison.csv`

## パラメータ設定

### 質量・慣性

- `m_sprung = 200`
- `m_unsprung = [10,10,10,10]`
- `a = 0.813`
- `b = 0.787`
- `L = a + b`
- `track_f = 1.2`
- `track_r = 1.18`
- `h_cg = 0.28`
- `Ixx = 32`, `Iyy = 95`, `Izz = 85`, `Iw = 0.15`

### サスペンション基礎

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

### LuGre

- `lugre_sigma0 = 10000`
- `lugre_sigma1 = 10`
- `lugre_sigma2 = 0`
- `lugre_Fc = 20`
- `lugre_Fs = 30`
- `lugre_vs = 0.01`

### ARB

- `k_arb_f = 5000`
- `k_arb_r = 3000`

### タイヤ

- `radius = 0.228`
- `mu_x_max = 1.6`
- `mu_y_max = 1.8`
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
- 入力関数は `throttle = 0.1` 固定

## 処理の流れ

### 1. MATLAB シミュレーション

`vehicle_8dof_full_model` を実行し、経過時間を表示します。

### 2. C++ 結果探索

`search_paths` を先頭から調べ、最初に見つかった CSV を使います。

### 3. C++ 比較モード

CSV が見つかった場合の処理です。

#### 時間合わせ

`t_cpp` に対して

- `idx_m = round(t_cpp / dt) + 1`
- `idx_m = min(idx_m, length(phi_m))`

で MATLAB 側添字を作ります。ここでも線形補間ではなく最近傍サンプル選択です。

#### ロール角比較

`phi_full` 列が存在する場合だけ比較を実行します。

- `phi_cpp = cpp_data.phi_full`
- `phi_m = state_m(:,4) * 180 / pi`

誤差は `err = phi_cpp - phi_m_interp` で、指標は

- `max_err`
- `rmse`

です。判定条件は `max_err < 1.0` です。

#### 条件付き比較

列が存在する場合に限って次を描きます。

- `damper_force_fl` と `data_m.F_damper_bw(:,1)`
- `friction_force_fl` と `data_m.F_friction_lugre(:,1)`
- `friction_z_fl` と `state_m(:,37)`

`state_m(:,37)` はフロント左 `friction_z(1)` に対応しています。

#### 図

2 行 3 列で次を表示します。

1. ロール角比較
2. ロール角誤差
3. フロント左 Bouc-Wen 力
4. フロント左 LuGre 摩擦力
5. フロント左 LuGre 毛変形
6. MATLAB 側の合計サスペンション力
   - `F_damper_bw + F_friction_lugre`

図は `../../output/verify_8dof_full.png` に保存します。

### 4. MATLAB 単体モード

CSV が見つからない場合は `warning` を出して比較なしで実行します。

#### 可視化内容

1. ロール角
2. 前進速度 `vx`
3. 全輪 Bouc-Wen 力
4. 全輪 LuGre 摩擦力
5. 全輪 `damper_z`
6. 全輪 `friction_z`

#### 追加表示

- 最大ロール角
- 最終 `friction_z`
- `F_friction_lugre` の最大絶対値

を表示します。

## 補足

- C++ ファイルが存在しても `phi_full` 列がなければ、比較本体には入りません。
- ローカル関数はありません。
