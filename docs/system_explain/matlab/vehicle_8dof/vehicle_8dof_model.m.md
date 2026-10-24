# `matlab/vehicle_8dof/vehicle_8dof_model.m`

## 役割

この関数は、6DOF のばね上車体に 4 輪のばね下鉛直自由度を追加した 8DOF 車両モデルです。  
ファイル冒頭コメントの通り、状態はばね上 12、ばね下 8、車輪回転 4、タイヤ内部状態 8 の合計 32 です。

## 関数入口

```matlab
[t, state, data] = vehicle_8dof_model(params, t_span, dt, input_func)
```

## 入出力

### 入力

- `params`
- `t_span`
- `dt`
- `input_func`

### 出力

- `t`
- `state`: `n_steps x 32`
- `data`
  - `ax`
  - `ay`
  - `Fz` 各輪垂直荷重

## 状態変数

1-6. `x, y, z, phi, theta, psi`  
7-12. `vx, vy, vz, p, q, r`  
13-16. `z_u(4)`  
17-20. `vz_u(4)`  
21-24. `omega(4)`  
25-28. `tire_xi(4)`  
29-32. `tire_eta(4)`

## 本体処理

### 1. 静的平衡

`compute_equilibrium(params)` で

- `static_Fz`
- `z_eq`
- `z_u_eq`

を求めます。

### 2. 初期状態

- `x0(3) = z_eq`
- `x0(7) = params.v0`
- `x0(13:16) = z_u_eq`
- `x0(21:24) = params.v0 / params.tire.radius`

です。

### 3. RK4 積分

`derivatives` を 4 回呼ぶ固定刻み RK4 です。

### 4. ログ

各ステップで `forces.ax`, `forces.ay`, `forces.Fz` を保存し、最終行は直前値をコピーします。

## ローカル関数

### `compute_equilibrium(params)`

#### ばね上静的輪荷重

6DOF 版と同様に

- `W = m_sprung * g`
- `Wf = W * b / L`
- `Wr = W * a / L`
- `static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2]`

です。

#### ばね上ヒーブ平衡

平均有効ばね定数 `k_avg` を用いて

- `z_eq = W / (4 * k_avg)`

を求めます。

#### ばね下平衡位置

タイヤばねだけで

- `total_load = static_Fz(i) + m_unsprung(i) * g`
- `z_u_eq(i) = total_load / k_tire`

を計算します。

### `derivatives(x, input, params, static_Fz, z_eq, z_u_eq)`

#### 状態展開

`z_u`, `vz_u`, `omega`, `tire_xi`, `tire_eta` を含めて展開します。

#### ばね上位置・姿勢の微分

- `R = euler_to_rotation_matrix(phi, theta, psi)`
- `E = angular_velocity_to_euler_rate(phi, theta)`
- `v_inertial = R' * [vx; vy; vz]`
- `euler_rate = E * [p; q; r]`

です。

#### 各輪のサスペンション変位

各輪で

- `z_sprung_at_wheel = z - wheel_x(i) * sin(theta) + wheel_y(i) * sin(phi)`
- `susp_travel = z_sprung_at_wheel - z_u(i)`

を計算します。  
6DOF 版と異なり、基準は `z_eq` ではなく実際のばね下位置 `z_u(i)` です。

#### サスペンション速度

- `vz_sprung_at_wheel = vz - wheel_x(i) * q * cos(theta) + wheel_y(i) * p * cos(phi)`
- `susp_velocity = vz_sprung_at_wheel - vz_u(i)`

です。

#### ばね・ダンパー力

- `k_eff = k_spring(i) * mr^2`
- `c_eff = c0 * mr^2`
- `susp_travel_from_eq = susp_travel - (z_eq - z_u_eq(i))`
- `F_spring = static_Fz(i) + k_eff * susp_travel_from_eq`
- `F_damper = c_eff * susp_velocity`

としています。

#### タイヤ垂直力

タイヤ変位と速度を

- `tire_deflection = z_u(i) - z_u_eq(i)`
- `tire_velocity = vz_u(i)`

と定義し、

```matlab
Fz_tire = static_Fz(i) + m_unsprung(i) * g + k_tire * tire_deflection + c_tire * tire_velocity
```

で計算します。負値は `max(..., 0)` で切り詰めます。

#### タイヤ縦横力

6DOF 版と同様に接地点速度から `tmeasy_force(..., max(vcx, 0.1), ...)` を呼びます。

#### ばね下運動

各輪のばね下加速度は

- `F_tire_reaction = -k_tire * tire_deflection - c_tire * tire_velocity`
- `F_susp_on_unsprung = F_spring + F_damper`
- `az_u = (F_tire_reaction + F_susp_on_unsprung) / m_u`

です。

#### ばね上合力とモーメント

サスペンション力 `F_susp = F_spring + F_damper` を用いて

- `Fz_total = Fz_total - F_susp`
- `Mx_total = Mx_total - wheel_y(i) * F_susp + Fy_wheel(i) * h_cg`
- `My_total = My_total + wheel_x(i) * F_susp`
- `Mz_total = Mz_total + wheel_x(i) * Fy_wheel(i) - wheel_y(i) * Fx_wheel(i)`

を組み立てます。

#### 空力

- `Fz_total = Fz_total + 0.5 * rho * v2 * CL_A`
- `Fx_total = Fx_total - 0.5 * rho * v2 * CD_A`

です。

#### 重力と並進運動

`gravity_in_body` を使って車体系重力を得て、

- `ax = Fx_total / m + g_body(1) + r * vy - q * vz`
- `ay = Fy_total / m + g_body(2) - r * vx + p * vz`
- `az = Fz_total / m + g_body(3) + q * vx - p * vy`

を計算します。

#### 回転運動

- `dp = Mx_total / Ixx`
- `dq = My_total / Iyy`
- `dr = Mz_total / Izz`

です。

#### 車輪回転

後輪 `i >= 3` にのみ駆動トルクを与え、タイヤ力による反力トルクを加えて `domega` を求めます。

#### 戻り値

状態微分は

```matlab
[v_inertial; euler_rate; ax; ay; az; dp; dq; dr; vz_u; az_u; domega; dxi; deta]
```

です。  
`forces` は `ax`, `ay + vx*r`, `Fz_tire` を返します。

### `euler_to_rotation_matrix(phi, theta, psi)`

6DOF 版と同じ回転行列生成関数です。

### `angular_velocity_to_euler_rate(phi, theta)`

6DOF 版と同じく `tan(theta)` と `1/cos(theta)` を含みます。

### `gravity_in_body(phi, theta, g)`

6DOF 版と同型です。

### `tmeasy_force(...)`

6DOF 版と同型の簡略 TMeasy です。

## 前提

- 前輪左右は同一操舵角です。
- 後輪駆動です。
- ばね下は鉛直方向のみです。
- タイヤ垂直力とばね下運動は線形ばね・線形ダンパで表現しています。
