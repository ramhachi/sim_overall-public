# verify_params_match.m

## 目的
`verify_params_match.m` は、MATLAB 側で使う代表パラメータを表示し、エンジントルクマップ、LSD、4D エアロマップの簡易テストを行う確認スクリプトです。タイトル表示では MATLAB と C++ のパラメータ一致確認を目的にしています。

## 入口
スクリプトです。ローカル関数はありません。

## 実行フェーズ
### 1. 初期化
- `clear; clc;`
- 見出しを表示

### 2. パラメータ構築
`params` をトップレベルで直接組み立てます。

#### 質量・慣性
- `m_sprung = 200`
- `m_unsprung = [10, 10, 10, 10]`
- `a = 0.813`
- `b = 0.787`
- `L = params.mass.a + params.mass.b`
- `track_f = 1.2`
- `track_r = 1.18`
- `h_cg = 0.28`
- `h_roll = 0.28`
- `Ixx = 32`
- `Iyy = 95`
- `Izz = 85`
- `Iw = 0.15`

#### サスペンション
- `k_spring = [25000, 25000, 30000, 30000]`
- `motion_ratio = [0.9, 0.9, 0.9, 0.9]`
- Bouc-Wen、LuGre、ARB、バンプストップ各定数

#### タイヤ
- `radius = 0.228`
- `mu_x_max = 1.6`
- `mu_y_max = 2.0`
- `Fz_nom = 1500`
- `Lx = 0.15`
- `Ly = 0.25`
- `Cx = 80000`
- `Cy = 60000`
- `v_N = 0.5`
- `k_tire = 200000`
- `c_tire = 500`

#### 空力
- `rho = 1.225`
- `A_ref = 1.0`
- `L_ref = 1.6`
- `x_cg = 0.8`
- `CL_default = -2.5`
- `CD_default = 1.0`
- `CoP_default = 0.45`

#### パワートレイン
- `gear_ratio = 3.0`
- `efficiency = 0.95`

#### 初速
- `v0 = 20`

### 3. パラメータ表示
以下の見出しごとに `fprintf` で値を表示します。
- Mass Properties
- Suspension
- Tire
- Aero (4D Map)
- Powertrain
- Initial Conditions

### 4. エンジントルクカーブ試験
- `test_rpms = [3000, 6000, 9000, 12000, 13000, 15000]`
- 各回転数で `engine_torque_map(rpm)` を呼び、トルクを表示します。

### 5. LSD 試験
- `lsd_params` を固定設定
  - `ramp_accel = 35.0`
  - `ramp_decel = 50.0`
  - `preload = 40.0`
  - `efficiency = 0.95`
- 入力条件
  - `omega_L = 100`
  - `omega_R = 120`
  - `T_in = 100`
- `salisbury_lsd` を呼び、左右出力トルクと合計効率を表示します。

### 6. 4D エアロマップ試験
基準条件:
- `v = 20`
- `rh_f = 40`
- `rh_r = 55`
- `roll = 0`
- `yaw = 0`

`aero_map_4d(...)` を呼び、`Fz`、`Fx`、`Fy`、`My` を表示します。

#### ライドハイト効果確認
- `rh_test = [25, 35, 45, 55]`
- 各前ライドハイトで `aero_map_4d(20, rh_test, 55, 0, 0, params.aero)` を呼びます。
- `gain = (aero_test.Fz / aero.Fz - 1.0) * 100` を計算して、基準に対する変化率を表示します。

## 外部依存
- `engine_torque_map`
- `salisbury_lsd`
- `aero_map_4d`

## しきい値・仮定
- これは印字中心の確認スクリプトで、厳密な自動判定は行いません。
- 「一致確認」という名前ですが、C++ ファイルを直接読む処理はありません。MATLAB 側で値を定義して表示し、関連関数の出力例を確認する構成です。

## 保存物・描画
保存処理も描画処理もありません。
