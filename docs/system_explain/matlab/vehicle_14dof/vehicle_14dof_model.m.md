# vehicle_14dof_model.m

## 目的
`vehicle_14dof_model` は、TMeasy タイヤ、Bouc-Wen ダンパー、LuGre 摩擦、ARB、バンプストップ、4D 空力、LSD 駆動を含む 14DOF 完全車両モデルです。ファイル冒頭コメントでは状態数を 40 と明示しています。

## 入口とシグネチャ
```matlab
function [t, state, data] = vehicle_14dof_model(params, t_span, dt, input_func)
```

### 入力
- `params`: 車両、サスペンション、タイヤ、空力、駆動系の構造体
- `t_span`: シミュレーション時間範囲 `[t0, tf]`
- `dt`: 固定刻み幅
- `input_func`: 時刻 `t` を受けて入力構造体を返す関数

### 出力
- `t`: 時刻列
- `state`: `n_steps x 40` の状態履歴
- `data`: 付帯データ構造体
  - `ax`, `ay`
  - `Fz`, `Fx`, `Fy`
  - `F_spring`, `F_bw`, `F_lugre`

## 状態定義
コメントどおり 40 状態です。
- `1:3` 位置 `x, y, z`
- `4:6` 姿勢 `phi, theta, psi`
- `7:9` 速度 `vx, vy, vz`
- `10:12` 角速度 `p, q, r`
- `13:16` ばね下位置 `z_u`
- `17:20` ばね下速度 `vz_u`
- `21:24` 車輪角速度 `omega`
- `25:28` タイヤ緩和状態 `tire_xi`
- `29:32` タイヤ緩和状態 `tire_eta`
- `33:36` Bouc-Wen 状態 `damper_z`
- `37:40` LuGre 状態 `friction_z`

## トップレベル処理
1. `compute_equilibrium(params)` で `static_Fz`、`z_eq`、`z_u_eq` を計算
2. 初期状態 `x0` を 40 要素ゼロで作成
   - `x0(3) = z_eq`
   - `x0(7) = params.v0`
   - 各輪について
     - `x0(13:16)` に `z_u_eq`
     - `x0(21:24)` に `params.v0 / params.tire.radius`
     - `damper_z`、`friction_z` は 0
3. `t = t_span(1):dt:t_span(2)` を生成
4. `state` と `data` の配列を確保
5. 各ステップで固定刻み RK4 を実行
   - `k1` から `k4` まで `derivatives(...)` を呼ぶ
   - `k4` 呼び出し時の `forces` をログ用に採用
6. 最終行の `data` はひとつ前の値をコピー

## ローカル関数詳細
### `compute_equilibrium`
```matlab
function [static_Fz, z_eq, z_u_eq] = compute_equilibrium(params)
```

#### 役割
静的平衡荷重と平衡位置を返します。

#### 詳細
- `W = m * g`
- 前後軸荷重: `Wf = W * b / L`、`Wr = W * a / L`
- `static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2]`
- 平均有効ばね定数 `k_avg` は各輪 `k_spring(i) * motion_ratio(i)^2` の平均
- `z_eq = W / (4 * k_avg)`
- 各輪ばね下平衡位置 `z_u_eq(i) = (static_Fz(i) + params.mass.m_unsprung(i) * g) / k_tire`

### `derivatives`
```matlab
function [dxdt, forces] = derivatives(x, input, params, static_Fz, z_eq, z_u_eq)
```

#### 役割
40 状態の微分と、記録用の各種力を返します。

#### 前半の準備
- 状態を各物理量へ展開
- 質量、慣性、タイヤ定数、重心高さを取り出す
- Bouc-Wen、LuGre、バンプストップ定数を取り出す
- 車輪位置を
  - `wheel_x = [a, a, -b, -b]`
  - `wheel_y = [-tf, tf, -tr, tr]`
  で定義
- `euler_to_rotation_matrix`、`angular_velocity_to_euler_rate` で回転行列と変換行列を作成
- `input.steering` を前輪 2 輪のみに与えます。

#### 各輪ループ
各輪 `i=1:4` で以下を行います。
1. サスペンション変位 `susp_travel(i)` を計算
2. サスペンション速度 `susp_velocity` を計算
3. 平衡からの偏差 `susp_travel_from_eq`
4. スプリング力
   ```matlab
   F_spring = static_Fz + k_eff * susp_travel_from_eq
   ```
5. Bouc-Wen 力
   ```matlab
   F_bw = bw_c0 * susp_velocity + bw_k0 * susp_travel_from_eq + bw_alpha * z_bw
   ```
6. Bouc-Wen 状態微分
   - `bw_beta` 項
   - `bw_gamma` 項
   を含む
7. LuGre 力
   - `g_v = Fc + (Fs - Fc) * exp(-(v/vs)^2)`
   - `abs(susp_velocity) > 1e-10` なら通常更新
   - それ以下では静止近傍の緩和処理を行い、`z_ss = Fs / sigma0` を超えるときだけ減衰的に戻す
8. バンプストップ力
   - `susp_travel_from_eq > bump_gap` なら正方向ばね
   - `< -bump_gap` なら負方向ばね
   - それ以外は 0
9. タイヤ鉛直力
   ```matlab
   Fz_tire = static_Fz + params.mass.m_unsprung(i) * g + k_tire * tire_deflection + c_tire * tire_velocity
   ```
   のあと `max(...,0)` で負荷重を禁止
10. 接地点速度を車輪座標へ変換し、縦滑り `vsx`、横滑り `vsy` を作成
11. `tmeasy_force(...)` で `Fx_tire`、`Fy_tire`、`dxi`、`deta` を計算
12. 車体座標へ戻して `Fx_wheel`、`Fy_wheel` を得る
13. ばね下運動として
    ```matlab
    az_u = (F_tire_reaction + F_susp_on_unsprung) / m_u
    ```
    を計算

#### ARB と合力
- `compute_arb_forces(susp_travel, params)` を呼ぶ
- 各輪サスペンション力 `F_spring + F_bw + F_lugre + F_bump + F_arb` を合成
- 合力 `Fx_total`、`Fy_total`、`Fz_total`
- モーメント `Mx_total`、`My_total`、`Mz_total` を計算

#### 空力
- `v_magnitude = sqrt(vx^2 + vy^2)`
- ライドハイトは左右平均で mm へ変換
  - `rh_front`
  - `rh_rear`
- ヨー角として、速度が `0.1` を超える場合のみ `beta = atan2(vy, vx)`、それ以外は `0`
- `aero_map_4d(...)` を呼び、`Fx`、`Fy`、`Fz` と空力モーメントを合算

#### 重力と剛体運動
- `gravity_in_body(phi, theta, g)` で車体座標系の重力を得る
- ばね上加速度:
  - `ax = Fx_total / m + g_body(1) + r * vy - q * vz`
  - `ay = Fy_total / m + g_body(2) - r * vx + p * vz`
  - `az = Fz_total / m + g_body(3) + q * vx - p * vy`
- 角加速度:
  - `dp = Mx_total / Ixx`
  - `dq = My_total / Iyy`
  - `dr = Mz_total / Izz`

#### パワートレイン
- 後輪平均角速度から `engine_rpm = omega_diff * params.powertrain.gear_ratio * 30 / pi`
- `engine_torque = input.throttle * engine_torque_map(engine_rpm)`
- LSD パラメータをこの関数内で固定設定
  - `ramp_accel = 35`
  - `ramp_decel = 50`
  - `preload = 40`
  - `efficiency = params.powertrain.efficiency`
- `T_in = engine_torque * params.powertrain.gear_ratio`
- `salisbury_lsd(omega(3), omega(4), T_in, lsd_params)` で左右後輪トルクを分配
- 前輪駆動は 0、後輪のみ駆動

#### 出力整理
- `dxdt` は 40 要素の列ベクトル
- `forces` は `ax`、`ay`、`Fz`、`Fx`、`Fy`、`F_spring`、`F_bw`、`F_lugre`
- `forces.ay` は `ay + vx * r` を記録します。状態微分で使う `ay` と同一ではありません。

### `compute_arb_forces`
前後それぞれ左右サスペンション変位差から ARB トルクを計算し、左右輪の鉛直方向力へ変換します。

### `euler_to_rotation_matrix`
`phi, theta, psi` から 3×3 回転行列を返します。

### `angular_velocity_to_euler_rate`
車体角速度からオイラー角速度への変換行列を返します。

### `gravity_in_body`
`psi=0` の回転行列を使って慣性系重力 `[0;0;g]` を車体系へ変換します。

### `tmeasy_force`
簡略 TMeasy 力モデルです。
- `dxi = vsx - (vcx / Lx) * xi`
- `deta = vsy - (vcx / Ly) * eta`
- 正規化滑り量 `xi_norm`、`eta_norm`
- 合成滑り `s`
- `s < 1` では `mu_eff = s * (2 - s)`
- それ以上では `mu_eff = 1`
- `s <= 1e-6` では `Fx = Fy = 0`

## 外部依存
- `aero_map_4d`
- `engine_torque_map`
- `salisbury_lsd`

## しきい値・仮定
- 負のタイヤ鉛直荷重は 0 に飽和
- 低速のスリップ角算出ではなく、空力用 `beta` だけ `v_magnitude > 0.1` を条件に計算
- 駆動は後輪 2 輪のみ
- `t` ベクトルは `:` 演算で作るため、最終時刻が刻みで割り切れない場合の調整はありません

## 保存物・描画
この関数単体では保存も描画も行いません。
