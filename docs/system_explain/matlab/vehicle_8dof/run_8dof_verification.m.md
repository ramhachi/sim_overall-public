# `matlab/vehicle_8dof/run_8dof_verification.m`

## 役割

このスクリプトは、`vehicle_8dof_model` の MATLAB 結果と C++ 側 8DOF 結果を比較する検証スクリプトです。  
ばね上 12 状態に加えて、4 輪ばね下の `z_u` と `vz_u` も比較対象に含めています。

## 実行入口

- スクリプトです。
- 冒頭で `clear; clc; close all;` を実行します。

## 外部依存

- `vehicle_8dof_model(params, t_span, dt, input_func)`
- `readtable`
- `interp1`
- `rad2deg`
- `saveas`
- C++ 結果ファイル `output/8dof_step_steer_result.csv`

ファイルが存在しない場合は `Run ./cpp/fsae_8dof first.` を含む `error` で停止します。

## 検証条件

### パラメータ

主な設定値は次の通りです。

- 質量・慣性
  - `m_sprung = 240`
  - `m_unsprung = [12, 12, 14, 14]`
  - `L = 1.55`, `a = 0.73`, `b = 0.82`
  - `track_f = 1.2`, `track_r = 1.18`
  - `h_cg = 0.28`
  - `Ixx = 32`, `Iyy = 95`, `Izz = 85`, `Iw = 0.5`
- タイヤ
  - `radius = 0.228`
  - `mu_x_max = 1.6`, `mu_y_max = 1.8`
  - `Lx = 0.15`, `Ly = 0.25`
  - `Cx = 80000`, `Cy = 60000`
  - `v_N = 0.5`
  - `k_tire = 200000`
  - `c_tire = 500`
- サスペンション
  - `k_spring = [25000, 25000, 30000, 30000]`
  - `motion_ratio = [0.85, 0.85, 0.90, 0.90]`
  - `c0 = 1500`
- 空力
  - `rho = 1.225`
  - `CD_A = 0.95`
  - `CL_A = 2.8`
- パワートレイン
  - `max_torque = 55`
  - `gear_ratio = 3.0`
  - `efficiency = 0.9`
- 初速 `v0 = 15.0`

### 入力と積分条件

- `step_time = 1.0`
- `step_steer = 0.1`
- `throttle = 0.3`
- `t_span = [0, 5]`
- `dt = 0.001`

## 処理の流れ

### 1. MATLAB シミュレーション

`vehicle_8dof_model` を呼び、実行時間を表示します。  
`data_matlab` はここでは使いません。

### 2. C++ 結果の読込み

`readtable` で `8dof_step_steer_result.csv` を読み込みます。

### 3. 比較対象の定義

- ばね上: `x, y, z, phi, theta, psi, vx, vy, vz, p, q, r`
- ばね下: `z_u0, z_u1, z_u2, z_u3, vz_u0, vz_u1, vz_u2, vz_u3`

合計 20 変数を `all_fields` にまとめています。

### 4. MATLAB 結果の補間

MATLAB 側は `t_cpp` に合わせて補間します。

- `x`, `y`, `z`, `vx`, `vy`, `vz`, `z_u*`, `vz_u*` はそのまま
- `phi`, `theta`, `psi`, `p`, `q`, `r` は `rad2deg` してから補間

なお、冒頭の `for i = 1:3` で `matlab_interp.x0`, `x1`, `x2` というフィールドも作っていますが、後続の比較では使っていません。

### 5. 誤差評価

単位配列と許容誤差配列は 20 要素で固定定義されています。

- 位置系: `1`, `1`, `0.1`
- 姿勢系: `2`, `2`, `5`
- 速度系: `0.5`, `0.5`, `0.1`
- 角速度系: `5`, `5`, `8`
- ばね下 `z_u*`: すべて `0.01`
- ばね下 `vz_u*`: すべて `0.1`

各変数について

- 最大絶対誤差
- RMSE
- 平均誤差

を計算し、`max_abs < tolerance` なら `✓ PASS` と表示します。

### 6. プロット

`figure('Position', [50, 50, 1600, 1000])` で 3 行 4 列の図を作ります。

1. 軌跡
2. ばね上 `z` と `z_u0`
3. ロール
4. ピッチ
5. ヨー
6. `vx`
7. `vy`
8. `r`
9. 4 輪ばね下 `z_u`
10. 位置誤差
11. 姿勢誤差
12. ヨーレート誤差

`z` と `z_u0` の比較だけはミリメートル換算で描画しています。

### 7. 保存

`output` ディレクトリを必要なら作成し、`output/verify_8dof.png` に保存します。

## 補足

- ばね下比較は `z_u*` と `vz_u*` に限定され、`omega` やタイヤ内部状態は比較しません。
- ローカル関数はありません。
