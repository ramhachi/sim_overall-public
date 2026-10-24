# `matlab/vehicle_8dof_full/vehicle_8dof_full_model.m`

## 役割

この関数は、8DOF 車両モデルに ARB、Bouc-Wen ダンパー、LuGre 摩擦を統合したモデルです。  
ファイルコメントの通り状態数は 40 で、`damper_z(4)` と `friction_z(4)` の 2 種類の内部状態を持ちます。

## 関数入口

```matlab
[t, state, data] = vehicle_8dof_full_model(params, t_span, dt, input_func)
```

## 出力

- `t`
- `state`: `n_steps x 40`
- `data`
  - `ax`
  - `ay`
  - `Fz`
  - `F_damper_bw`
  - `F_friction_lugre`

## 状態変数

1-6. `x, y, z, phi, theta, psi`  
7-12. `vx, vy, vz, p, q, r`  
13-16. `z_u(4)`  
17-20. `vz_u(4)`  
21-24. `omega(4)`  
25-28. `tire_xi(4)`  
29-32. `tire_eta(4)`  
33-36. `damper_z(4)`  
37-40. `friction_z(4)`

## 本体処理

### 1. 静的平衡

`compute_equilibrium` で

- `static_Fz`
- `z_eq`
- `z_u_eq`

を求めます。内容は 8DOF/BW 版と同じです。

### 2. 初期状態

- `x0(3) = z_eq`
- `x0(7) = params.v0`
- `x0(13:16) = z_u_eq`
- `x0(21:24) = params.v0 / params.tire.radius`
- `x0(33:36) = 0`
- `x0(37:40) = 0`

です。

### 3. RK4 積分

固定刻み RK4 で、各ステップで `forces` を記録します。

## ローカル関数

### `compute_equilibrium(params)`

8DOF/BW 版と同じです。

### `derivatives(x, input, params, static_Fz, z_eq, z_u_eq)`

#### 追加状態

- `damper_z = x(33:36)`  
- `friction_z = x(37:40)`

#### Bouc-Wen パラメータ

- `bw_c0`, `bw_k0`, `bw_alpha`, `bw_A`, `bw_beta`, `bw_gamma`, `bw_n`

#### LuGre パラメータ

- `lugre_sigma0`
- `lugre_sigma1`
- `lugre_sigma2`
- `lugre_Fc`
- `lugre_Fs`
- `lugre_vs`

#### サスペンション変位と速度

各輪ごとに

- `susp_travel = z_sprung_at_wheel - z_u(i)`
- `susp_velocity = vz_sprung_at_wheel - vz_u(i)`
- `susp_travel_from_eq = susp_travel - (z_eq - z_u_eq(i))`

を求めます。

#### スプリング力

- `F_spring = static_Fz(i) + k_eff * susp_travel_from_eq`

です。

#### Bouc-Wen 力と履歴微分

- `F_damper_bw = bw_c0 * susp_velocity + bw_k0 * susp_travel_from_eq + bw_alpha * z_bw`
- `d_damper_z = bw_A * susp_velocity - bw_beta * |v| * z * |z|^(n-1) - bw_gamma * v * |z|^n`

です。

#### LuGre 摩擦

コードコメントに式が書かれています。

- 力: `F = σ0*z + σ1*dz/dt + σ2*v`
- 毛変形微分: `dz/dt = v - (|v|/g(v))*z`
- ストライベック関数: `g(v) = Fc + (Fs - Fc)*exp(-(v/vs)^2)`

実装上は微分式に `lugre_sigma0 / g_v` を掛けており、コメントでも「正しい式」として

```matlab
dz/dt = v - (σ0 * |v| / g(v)) * z
```

と記述しています。

##### 通常速度域

`abs(susp_velocity) > 1e-10` のとき

- `g_v = Fc + (Fs - Fc) * exp(-(v/vs)^2)`
- `d_friction_z = v - (sigma0/g_v) * |v| * z_fr`

です。

##### 極低速時の分岐

速度が十分小さい場合は「スティック状態」として

- `z_ss = Fs / sigma0`
- `abs(z_fr) > z_ss` なら `d_friction_z = -z_fr / (z_ss / sigma1)`
- それ以外は `d_friction_z = 0`

としています。

##### 摩擦力

```matlab
F_friction_lugre = sigma0 * z_fr + sigma1 * d_friction_z + sigma2 * susp_velocity
```

です。

#### タイヤ垂直力と縦横力

8DOF/BW 版と同じです。

- `Fz_tire = static_Fz + m_unsprung*g + k_tire*deflection + c_tire*velocity`
- 下限は 0
- 縦横力は簡略 TMeasy

#### ばね下運動

ばね下へ作用する合計ダンパー力は

- `F_damper_total = F_damper_bw + F_friction_lugre`

で、ばね下加速度は

- `az_u = (F_tire_reaction + F_spring + F_damper_total) / m_u`

です。

#### ARB 力

`compute_arb_forces(susp_travel, params)` で計算します。

#### ばね上合力

各輪サスペンション力は

```matlab
F_susp = F_spring + F_damper_bw + F_friction_lugre + F_arb
```

です。  
これを使って `Fz_total`, `Mx_total`, `My_total`, `Mz_total` を計算します。

#### 空力・重力・並進回転運動・車輪回転

構造は BW 版と同じです。

#### 戻り値

状態微分は

```matlab
[v_inertial; euler_rate; ax; ay; az; dp; dq; dr; vz_u; az_u; domega; dxi; deta; d_damper_z; d_friction_z]
```

です。

`forces` は

- `ax`
- `ay + vx*r`
- `Fz`
- `F_damper_bw`
- `F_friction_lugre`

を返します。

### `compute_arb_forces(susp_travel, params)`

BW 版と同じく、前後それぞれの左右サスペンション変位差をトーションバー角に見立てて ARB 力へ変換します。

### `euler_to_rotation_matrix`

BW 版と同じです。

### `angular_velocity_to_euler_rate`

BW 版と同じです。

### `gravity_in_body`

BW 版と同じです。

### `tmeasy_force`

BW 版と同じ簡略 TMeasy です。

## 補足

- LuGre には低速時専用の分岐があり、停止近傍で `d_friction_z` を直接 0 または減衰形に切り替えます。
- ARB 力はばね上側へは加えますが、ばね下加速度計算には直接入れていません。
