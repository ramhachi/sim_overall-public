# vehicle_14dof_nn.m

## 目的
`vehicle_14dof_nn` は、14DOF 車両モデルのタイヤ部分を学習済み NN モデルに置き換え、タイヤ表面温度状態も同時に積分する関数です。TMeasy 緩和状態 `xi`、`eta` は持たず、状態数は 36 です。

## 入口とシグネチャ
```matlab
function [t, state, data] = vehicle_14dof_nn(params, t_span, dt, input_func, nn_model, norm_params)
```

### 入力
- `params`: 車両パラメータ構造体
- `t_span`: 時間範囲
- `dt`: 固定刻み幅
- `input_func`: ドライバ入力関数
- `nn_model`: 学習済み NN
- `norm_params`: NN 入出力正規化パラメータ

### 出力
- `t`: 時刻列
- `state`: `n_steps x 36`
- `data`: ログ構造体
  - `ax`, `ay`
  - `Fz`, `Fx`, `Fy`
  - `F_spring`, `F_bw`, `F_lugre`
  - `T_surf`

## 状態定義
- `1:12`: ばね上位置姿勢と速度角速度
- `13:16`: ばね下位置
- `17:20`: ばね下速度
- `21:24`: 車輪角速度
- `25:28`: タイヤ表面温度 `T_surf`
- `29:32`: Bouc-Wen 状態 `damper_z`
- `33:36`: LuGre 状態 `friction_z`

## トップレベル処理
1. `compute_equilibrium` で静的平衡を算出
2. 初期タイヤ温度 `T_surf_init = 25.0`
3. 36 状態の初期ベクトル `x0` を構成
   - `x0(3) = z_eq`
   - `x0(7) = params.v0`
   - 各輪 `z_u`、`vz_u=0`、`omega=v0/radius`、`T_surf=25`、`damper_z=0`、`friction_z=0`
4. 固定刻み RK4 で積分
5. `data` に力と温度を保存
6. `data.T_surf(end, :) = state(end, 25:28)` として最終温度を反映

## ローカル関数詳細
### `compute_equilibrium`
`vehicle_14dof_model` と同じ式で `static_Fz`、`z_eq`、`z_u_eq` を求めます。

### `derivatives`
```matlab
function [dxdt, forces] = derivatives(x, input, params, static_Fz, z_eq, z_u_eq, nn_model, norm_params)
```

#### 役割
36 状態の微分と、記録用の力・温度を返します。

#### 共通部分
サスペンション変位、Bouc-Wen、LuGre、バンプストップ、タイヤ鉛直力、ARB、剛体運動の多くは `vehicle_14dof_model` と同系統です。

#### 環境条件
- `params.environment.T_ambient` があればそれを使います。
- 無ければ `T_amb = 25.0`
- `v_wind = sqrt(vx^2 + vy^2)` とし、車速を風速として代用します。

#### 各輪ループでの NN タイヤ処理
1. 接地点速度を車輪座標へ変換
2. `omega_r = omega(i) * params.tire.radius`
3. `vsx = vx_wheel - omega_r`、`vsy = vy_wheel`
4. スリップ率 `SR`
   - 既定値 `0.0`
   - `abs(omega_r) > 0.1` のときだけ `SR = vsx / omega_r`
5. スリップ角 `SA`
   - 既定値 `0.0`
   - `abs(vx_wheel) > 0.1` のときだけ `SA = -atan2(vy_wheel, abs(vx_wheel)) * 180.0 / pi`
6. キャンバー角 `IA = phi * 180.0 / pi`
   - ロール角をそのままキャンバー角として代用しています。
7. 温度入力 `TSTC = T_surf(i)`
8. `nn_tire_predict(SR, SA, Fz_tire(i), IA, TSTC, nn_model, norm_params)` を呼びます。
9. 返り値のうち `Fx_tire`、`Fy_tire` だけを使い、`Mx` と `Mz` は捨てています。
10. 車体座標へ戻して `Fx_wheel`、`Fy_wheel` を計算します。

#### 温度微分
このファイルでは別関数 `tire_thermal_update` を呼ばず、同等の係数をファイル内に直接書いています。
- `m_tread = 2.0`
- `c_p = 1000.0`
- `h_conv = 50.0`
- `A_tread = 0.05`
- `Q_gen = abs(Fx_tire * vsx) + abs(Fy_tire * vsy)`
- `Q_conv = h_conv * A_tread * (T_surf(i) - T_amb) * (1 + 0.1 * abs(v_wind))`
- `dT_surf(i) = (Q_gen - Q_conv) / (m_tread * c_p)`

ここでは `tire_thermal_update` にある 0～120°C のクリップは行いません。温度の上限制限はこの関数内にはありません。

#### 駆動系
- `engine_torque = input.throttle * params.powertrain.max_torque`
- `drive_torque = engine_torque * params.powertrain.gear_ratio * params.powertrain.efficiency / 2`
- 後輪 2 輪に等分する RWD です。
- `vehicle_14dof_model` と異なり、`engine_torque_map` や `salisbury_lsd` は使いません。

#### 空力
- この関数の空力は 4D マップではなく単純二次式です。
  - `Fz_total = Fz_total + 0.5 * params.aero.rho * v2 * params.aero.CL_A`
  - `Fx_total = Fx_total - 0.5 * params.aero.rho * v2 * params.aero.CD_A`
- サイドフォース空力や空力モーメントは計算しません。

#### 出力
- `dxdt` は 36 要素列ベクトル
- 温度微分 `dT_surf(:)` が状態に含まれます。
- `forces.T_surf = T_surf'`
- `forces.ay = ay + vx * r`

### `compute_arb_forces`
左右変位差から前後 ARB 力を返します。式は `vehicle_14dof_model` と同じです。

### `euler_to_rotation_matrix`
3×3 回転行列を返します。

### `angular_velocity_to_euler_rate`
角速度からオイラー角速度への変換行列を返します。

### `gravity_in_body`
`psi=0` とした回転行列で重力を車体系へ変換します。

## 外部依存
- `nn_tire_predict`
- `vehicle_14dof_model` と異なり `aero_map_4d`、`engine_torque_map`、`salisbury_lsd` は呼びません。

## しきい値・仮定
- `abs(omega_r) > 0.1` でないとき `SR = 0`
- `abs(vx_wheel) > 0.1` でないとき `SA = 0`
- キャンバー角はロール角の代用
- 空力は単純 `CL_A` / `CD_A` モデル
- タイヤ温度のクリップは無し

## 保存物・描画
この関数単体では保存も描画も行いません。
