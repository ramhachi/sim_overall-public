# `matlab/vehicle_6dof/run_6dof_verification.m`

## 役割

このスクリプトは、MATLAB の `vehicle_6dof_model` と C++ 側 6 自由度モデルの結果を比較する検証用スクリプトです。  
位置・姿勢・速度・角速度の 12 変数を比較し、誤差表の表示と 12 分割プロットの保存を行います。

## 実行入口

- スクリプトです。
- 冒頭で `clear; clc; close all;` を実行します。

## 外部依存

- `vehicle_6dof_model(params, t_span, dt, input_func)`
- `readtable`
- `interp1`
- `rad2deg`
- `saveas`
- C++ 結果ファイル `output/6dof_step_steer_result.csv`

結果 CSV が存在しない場合は `Run ./cpp/fsae_6dof first.` を含む `error` で停止します。

## 検証条件

### パラメータ

スクリプト内で `params` を直接構築します。主な値は次の通りです。

- 質量・慣性
  - `m_sprung = 240`
  - `m_unsprung = [10,10,10,10]`
  - `L = 1.55`, `a = 0.73`, `b = 0.82`
  - `track_f = 1.2`, `track_r = 1.15`
  - `h_cg = 0.28`
  - `Ixx = 30`, `Iyy = 100`, `Izz = 85`, `Iw = 0.15`
- タイヤ
  - `radius = 0.228`
  - `mu_x_max = 1.6`, `mu_y_max = 1.8`
  - `Lx = 0.15`, `Ly = 0.25`
  - `Cx = 80000`, `Cy = 60000`
  - `v_N = 0.5`
- サスペンション
  - `k_spring = [25000, 25000, 30000, 30000]`
  - `motion_ratio = [0.9, 0.9, 0.85, 0.85]`
  - `c0 = 2000`
- 空力
  - `rho = 1.225`
  - `CD_A = 0.8`
  - `CL_A = -2.5`
- パワートレイン
  - `max_torque = 55`
  - `gear_ratio = 3.0`
  - `efficiency = 0.9`
- 初速
  - `v0 = 15.0`

### 入力

- `step_time = 1.0`
- `step_steer = 0.1`
- `throttle = 0.3`

時刻 1 秒で操舵角をステップ入力し、スロットルは固定です。

### シミュレーション設定

- `t_span = [0, 5]`
- `dt = 0.001`

## 処理の流れ

### 1. MATLAB 計算

`vehicle_6dof_model` を実行し、`tic` / `toc` で経過時間を表示します。  
戻り値 `data_matlab` は受け取りますが、このスクリプトでは利用しません。

### 2. C++ 結果の読込み

`readtable` で `6dof_step_steer_result.csv` を読み込みます。

### 3. 時間軸の統一

C++ 側 `time` を基準に MATLAB 結果を `interp1(..., 'linear', 'extrap')` で補間します。

補間対象は 12 変数です。

- 位置: `x`, `y`, `z`
- オイラー角: `phi`, `theta`, `psi`
- 並進速度: `vx`, `vy`, `vz`
- 角速度: `p`, `q`, `r`

角度・角速度だけ `rad2deg` を通して比較しています。

### 4. 誤差評価

比較順は `all_fields = [fields_pos, fields_euler, fields_vel, fields_omega]` です。  
単位配列は

- `m`, `m`, `m`
- `deg`, `deg`, `deg`
- `m/s`, `m/s`, `m/s`
- `deg/s`, `deg/s`, `deg/s`

です。

許容誤差は

```matlab
[1, 1, 0.1, 2, 2, 5, 0.5, 0.5, 0.1, 5, 5, 8]
```

で、コードコメントでは「6DOF の累積誤差を考慮してヨーレートは 8deg/s」に設定したと説明しています。

各変数で計算する値は次の 3 つです。

- 最大絶対誤差
- RMSE
- 平均誤差

加えて `errors.(field).diff` に時系列誤差を保存します。

### 5. 可視化

`figure('Position', [50, 50, 1600, 1000])` で 3 行 4 列の図を作成し、次を表示します。

1. 軌跡
2. `z`（ヒーブ）
3. ロール角
4. ピッチ角
5. ヨー角
6. `vx`
7. `vy`
8. `r`
9. 位置誤差
10. 姿勢誤差
11. 速度誤差
12. ヨーレート誤差

最後に `sgtitle('6DOF Vehicle Model: C++ vs MATLAB Verification')` を付けます。

### 6. 保存

`output` ディレクトリが存在しなければ作成し、図を `output/verify_6dof.png` に保存します。

## 補足

- 角度系比較は MATLAB 側を度へ変換して実施します。
- 誤差判定は `max_abs < tolerance` の厳密な不等号です。等しい場合は不合格です。
- ローカル関数はありません。
