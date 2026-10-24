# `matlab/vehicle_8dof_bw/vehicle_8dof_bw_model.m`

## 役割

この関数は、8DOF 車両モデルに ARB と Bouc-Wen ダンパー履歴状態を追加した MATLAB 実装です。  
ファイルコメントにある通り、状態数は 36 で、Bouc-Wen 履歴変数 `damper_z(4)` を含みます。

## 関数入口

```matlab
[t, state, data] = vehicle_8dof_bw_model(params, t_span, dt, input_func)
```

## 出力

- `t`
- `state`: `n_steps x 36`
- `data`
  - `ax`
  - `ay`
  - `Fz`
  - `F_damper_bw`

## 状態変数

1-6. `x, y, z, phi, theta, psi`  
7-12. `vx, vy, vz, p, q, r`  
13-16. `z_u(4)`  
17-20. `vz_u(4)`  
21-24. `omega(4)`  
25-28. `tire_xi(4)`  
29-32. `tire_eta(4)`  
33-36. `damper_z(4)`

## 本体処理

### 1. 静的平衡

`compute_equilibrium` は 8DOF 版と同じ考え方で

- `static_Fz`
- `z_eq`
- `z_u_eq`

を返します。

### 2. 初期状態

- `x0(3) = z_eq`
- `x0(7) = params.v0`
- `x0(13:16) = z_u_eq`
- `x0(21:24) = params.v0 / params.tire.radius`
- `x0(33:36) = 0`

で、Bouc-Wen 履歴変数はゼロ初期化です。

### 3. RK4 積分とログ

積分は固定刻み RK4、記録する補助データは `ax`, `ay`, `Fz`, `F_damper_bw` です。

## ローカル関数

### `compute_equilibrium(params)`

内容は `vehicle_8dof_model` と同じです。

- `static_Fz`: ばね上質量から計算
- `z_eq`: 平均有効ばね定数から計算
- `z_u_eq`: `k_tire` によるばね下平衡

### `derivatives(x, input, params, static_Fz, z_eq, z_u_eq)`

#### 追加される状態

`damper_z = x(33:36)` が Bouc-Wen 履歴変数です。

#### Bouc-Wen パラメータ

サスペンション構造体から次を読み出します。

- `bw_c0`
- `bw_k0`
- `bw_alpha`
- `bw_A`
- `bw_beta`
- `bw_gamma`
- `bw_n`

#### サスペンション変位と速度

8DOF 版と同様に

- `susp_travel = z_sprung_at_wheel - z_u(i)`
- `susp_velocity = vz_sprung_at_wheel - vz_u(i)`

を計算します。

#### スプリング力

- `susp_travel_from_eq = susp_travel(i) - (z_eq - z_u_eq(i))`
- `F_spring = static_Fz(i) + k_eff * susp_travel_from_eq`

です。

#### Bouc-Wen ダンパー力

コードコメントに式が明示されています。

- 力: `F = c0 * v + k0 * x + alpha * z`
- 履歴微分:
  - `dz/dt = A * v - beta * |v| * z * |z|^(n-1) - gamma * v * |z|^n`

実装は

- `F_damper_bw(i) = bw_c0 * susp_velocity + bw_k0 * susp_travel_from_eq + bw_alpha * z_bw`
- `d_damper_z(i) = ...`

です。

`abs_z_n_minus_1 = abs(z_bw)^(bw_n - 1)` を経由して `|z|^(n-1)` と `|z|^n` を作っています。

#### タイヤ垂直力と縦横力

ここは 8DOF 版と同じです。

- タイヤ垂直力は `static_Fz + m_unsprung*g + k_tire*deflection + c_tire*velocity`
- 負荷重は 0 に切り詰め
- 縦横力は簡略 TMeasy

#### ばね下運動

ばね下へ作用するサスペンション力は

- `F_susp_on_unsprung = F_spring + F_damper_bw`

です。ARB 力はここには入っていません。

#### ARB 力

各輪ループの後に `compute_arb_forces(susp_travel, params)` を呼びます。

#### ばね上合力

ばね上へ作用するサスペンション力は

- `F_spring`
- `F_damper_bw`
- `F_arb`

の合計です。

```matlab
F_susp = F_spring(i) + F_damper_bw(i) + F_arb(i)
```

を使って `Fz_total`, `Mx_total`, `My_total`, `Mz_total` を組みます。

#### 空力・重力・並進回転運動・車輪回転

これらは 8DOF 版と同じ構造です。

#### 戻り値

状態微分は

```matlab
[v_inertial; euler_rate; ax; ay; az; dp; dq; dr; vz_u; az_u; domega; dxi; deta; d_damper_z]
```

です。

`forces` は

- `ax`
- `ay + vx*r`
- `Fz`
- `F_damper_bw`

を返します。

### `compute_arb_forces(susp_travel, params)`

ARB 力を 4 輪分返します。

#### フロント

- `roll_diff_f = susp_travel(1) - susp_travel(2)`
- `roll_angle_f = roll_diff_f / track_f`
- `torque_arb_f = k_arb_f * roll_angle_f`
- 左右輪力へ `± torque / (track_f/2)` で配分

#### リア

リアも同型で `k_arb_r`, `track_r` を使います。

### `euler_to_rotation_matrix`

8DOF 版と同じです。

### `angular_velocity_to_euler_rate`

8DOF 版と同じです。

### `gravity_in_body`

8DOF 版と同じです。

### `tmeasy_force`

8DOF 版と同じ簡略 TMeasy です。

## 補足

- Bouc-Wen 力は平衡からの偏差 `susp_travel_from_eq` を使う実装です。コードコメントにも「C++ と一致」とあります。
- ARB 力はばね上側合力には入りますが、ばね下加速度計算には入りません。
