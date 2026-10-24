# `matlab/vehicle_6dof/vehicle_6dof_model.m`

## 役割

この関数は、6 自由度のばね上車体運動に 4 輪の車輪回転とタイヤ内部状態を組み合わせた車両モデルです。  
状態には `z`, `phi`, `theta` を含み、ロール・ピッチ・ヒーブを扱いますが、ばね下の独立自由度は持ちません。

## 関数入口

```matlab
[t, state, data] = vehicle_6dof_model(params, t_span, dt, input_func)
```

## 入力

- `params`: 質量、慣性、サスペンション、タイヤ、空力、駆動系を持つ構造体
- `t_span`: 時間範囲
- `dt`: 固定刻み
- `input_func`: `steering` と `throttle` を返す関数

## 出力

- `t`: 時間ベクトル
- `state`: `n_steps x 24` 状態履歴
- `data`
  - `ax`
  - `ay`
  - `Fz` 各輪垂直荷重

## 状態変数

24 状態を次の順に持ちます。

1-6. `x, y, z, phi, theta, psi`
7-12. `vx, vy, vz, p, q, r`
13-16. `omega(4)`
17-20. `tire_xi(4)`
21-24. `tire_eta(4)`

## メイン処理

### 1. 静的平衡計算

`compute_equilibrium(params)` を最初に呼び、以下を得ます。

- `static_Fz`
- `z_eq`

### 2. 初期状態

`x0 = zeros(24,1)` とし、

- `x0(3) = z_eq`
- `x0(7) = params.v0`
- `x0(13:16) = params.v0 / params.tire.radius`

を設定します。

### 3. RK4 積分

時間積分は固定刻み RK4 です。  
各ステップで `derivatives` を 4 回呼び、`state(k+1,:)` を更新します。

### 4. データ記録

各ステップの `forces.ax`, `forces.ay`, `forces.Fz` を保存し、最終行は 1 ステップ前を複写します。

## ローカル関数

### `compute_equilibrium(params)`

静的輪荷重と車体ヒーブ平衡位置を決めます。

#### 輪荷重

- `W = m_sprung * g`
- `Wf = W * b / L`
- `Wr = W * a / L`
- `static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2]`

#### ヒーブ平衡位置

4 輪分について

- `k_spring(i) * motion_ratio(i)^2`

を平均した `k_avg` を作り、

- `z_eq = W / (4 * k_avg)`

としています。

### `derivatives(x, input, params, static_Fz, z_eq)`

運動方程式本体です。

#### 状態展開

- 姿勢: `phi`, `theta`, `psi`
- 並進速度: `vx`, `vy`, `vz`
- 角速度: `p`, `q`, `r`
- 車輪角速度: `omega`
- タイヤ内部状態: `tire_xi`, `tire_eta`

#### 車輪位置

- `wheel_x = [a, a, -b, -b]`
- `wheel_y = [-tf, tf, -tr, tr]`

#### 座標変換

- `R = euler_to_rotation_matrix(phi, theta, psi)`
- `E = angular_velocity_to_euler_rate(phi, theta)`

を使い、

- 慣性座標系速度 `v_inertial = R' * v_body`
- オイラー角速度 `euler_rate = E * [p; q; r]`

を計算します。

#### サスペンション変位と速度

各輪で

- `z_sprung_at_wheel = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi)`
- `susp_travel = z_sprung_at_wheel - z_eq`
- `susp_velocity = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi)`

を計算します。

コードコメントには「符号修正 - C++ と一致」「SAE 座標系: +z=下向き、+theta=ノーズアップ、+phi=右下がり」とあります。

#### ばね・ダンパー力

- `k_eff = k_spring(i) * mr^2`
- `F_spring_static = static_Fz(i)`
- `F_spring_delta = k_eff * susp_travel`
- `F_spring = F_spring_static + F_spring_delta`
- `c_eff = c0 * mr^2`
- `F_damper = c_eff * susp_velocity`

垂直タイヤ荷重は

- `Fz_tire = max(F_spring + F_damper, 0)`

です。負荷重は 0 に切り詰められます。

#### タイヤ力

接地点速度、ホイール座標変換、`omega_r = omega(i) * radius`、`vsx`, `vsy`, `vcx` を計算し、  
`tmeasy_force(..., max(vcx, 0.1), ...)` を呼びます。

#### 合力・モーメント

縦横力は各輪の和です。鉛直力とモーメントは各輪のサスペンション力から組み立てています。

- `Fz_total = Fz_total - F_susp`
- `Mx_total = Mx_total - wheel_y(i) * F_susp + Fy_wheel(i) * h_cg`
- `My_total = My_total + wheel_x(i) * F_susp`
- `Mz_total = Mz_total + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i)`

#### 空力

- `Fz_aero = 0.5 * rho * v2 * CL_A`
- `Fx_aero = -0.5 * rho * v2 * CD_A`

を計算し、`Fx_total` と `Fz_total` に加えます。`v2 = vx^2 + vy^2` です。

#### 重力

`gravity_in_body(phi, theta, g)` で車体座標系重力ベクトルを求め、

- `ax = Fx_total / m + g_body(1) + r * vy - q * vz`
- `ay = Fy_total / m + g_body(2) - r * vx + p * vz`
- `az = Fz_total / m + g_body(3) + q * vx - p * vy`

を計算します。

#### 回転運動

- `dp = Mx_total / Ixx`
- `dq = My_total / Iyy`
- `dr = Mz_total / Izz`

です。ジャイロ項や慣性連成項は明示的には入っていません。

#### 車輪回転

エンジントルクは 3DOF と同じで、後輪 `i >= 3` にのみ与えます。  
タイヤ反力トルクはホイール座標系縦力から `T_tire = -Fx_w * R_tire` として計算します。

#### 戻り値

状態微分は

```matlab
[v_inertial; euler_rate; ax; ay; az; dp; dq; dr; domega; dxi; deta]
```

です。

`forces` には

- `ax`
- `ay = ay + vx * r`
- `Fz = Fz_tire`

を格納します。

### `euler_to_rotation_matrix(phi, theta, psi)`

3-2-1 形のオイラー角から回転行列 `R` を作ります。  
コード上は `R' * v_body` で慣性座標系速度へ変換しているため、この `R` は慣性→車体系の向きで使われています。

### `angular_velocity_to_euler_rate(phi, theta)`

角速度 `[p;q;r]` をオイラー角時間微分へ変換する行列 `E` を返します。  
`tan(theta)` と `1/cos(theta)` を含むため、`theta = ±pi/2` 近傍では特異になりますが、回避処理はありません。

### `gravity_in_body(phi, theta, g)`

`psi = 0` とした回転行列で慣性座標系重力 `[0;0;g]` を車体系に写します。

### `tmeasy_force(...)`

3DOF 版と同型の簡略 TMeasy です。

- `dxi = vsx - (vcx / Lx) * xi`
- `deta = vsy - (vcx / Ly) * eta`
- 正規化変形量から合成スリップ `s`
- `s < 1` なら `mu_eff = s * (2 - s)`
- それ以外は `mu_eff = 1`
- `s <= 1e-6` では力を 0

## 重要な前提

- 前輪左右は同一操舵角です。
- 後輪駆動です。
- ばね下自由度は持たず、タイヤ垂直荷重はサスペンション力から直接決めています。
- 積分は固定刻みで、イベント処理や適応刻みはありません。
