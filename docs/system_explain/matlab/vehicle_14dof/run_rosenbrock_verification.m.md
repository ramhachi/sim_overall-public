# run_rosenbrock_verification.m

## 目的
`run_rosenbrock_verification.m` は、14DOF モデルに対して 3 種類の積分設定を比較する検証スクリプトです。
- RK4、刻み幅 1 ms
- RK4、刻み幅 0.1 ms
- `ode23s`、コメント上は Rosenbrock 法

高精度 RK4 を基準として、位置誤差と計算時間を比較し、図保存まで行います。

## 入口
スクリプトです。トップレベル処理のあとに 5 個のローカル関数を持ちます。
- `ode_14dof`
- `euler_to_rotation`
- `angular_to_euler_rate`
- `compute_arb`
- `tmeasy`

## 実行フェーズ
### 1. 初期化とパラメータ設定
`run_14dof_verification.m` と同系統の `params` を構築します。
- 質量、慣性、サスペンション、タイヤ、空力、パワートレイン、初速を設定
- タイヤは `mu_x_max=1.6`、`mu_y_max=2.0`、`k_tire=200000`、`c_tire=500`

### 2. シミュレーション条件
- `t_span = [0, 10]`
- ステア開始 `1.0 s`
- 舵角 `0.1 rad`
- `input_func` は `throttle=0.1` と段差ステアを返します。

### 3. RK4 1ms 実行
- `dt1 = 0.001`
- `vehicle_14dof_model(...)` を実行
- 計算時間とステップ数を表示

### 4. RK4 0.1ms 実行
- `dt2 = 0.0001`
- 同じ `vehicle_14dof_model(...)` を実行
- 高精度基準データとして使います。

### 5. `ode23s` 実行準備
`ode23s` 用に静的平衡と初期状態を別途組み立てます。
- `static_Fz` を重心位置から算出
- 平均ばね定数 `k_avg` から `z_eq` を算出
- 各輪の `z_u_eq` を `total_load / k_tire` で計算
- 40 状態の初期ベクトル `x0` を構築
  - `x0(3) = z_eq`
  - `x0(7) = params.v0`
  - `x0(13:16)` に `z_u_eq`
  - `x0(21:24)` に `params.v0 / params.tire.radius`
- `odefun = @(t, x) ode_14dof(...)`
- `odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3)` を設定

### 6. `ode23s` 実行
- `[t3, state3] = ode23s(odefun, t_span, x0, options)`
- 時間とステップ数を表示

### 7. 結果表示
最終位置、最終速度、最大ロール角、計算時間を 3 手法で横並び表示します。

### 8. 描画
2 行 3 列の図を作成します。
1. 軌跡
2. ロール角
3. 前後速度
4. 計算時間棒グラフ
5. ステップ数棒グラフ
6. RK4 0.1ms 基準の位置誤差

#### 実装上の注意
「Forward Velocity」サブプロットでは、2 本目が `state2(:,7)` ではなく `state2(:,4)` を描いています。タイトルと軸ラベルは速度比較ですが、コード上は高精度 RK4 のロール角列を使っています。

### 9. 画像保存
- `../../output/rosenbrock_comparison.png`

### 10. 検証メッセージ
- `pos_err_rk4 < 0.5` なら RK4 1ms を accurate と表示
- `pos_err_ode23s < 0.5` なら `ode23s` を accurate と表示
- `time3 < time2` なら `ode23s` が fine RK4 より高速と表示

## 外部依存
- `vehicle_14dof_model`
- `ode23s`
- `odeset`
- `interp1`
- `saveas`

## ローカル関数詳細
### `ode_14dof`
```matlab
function dxdt = ode_14dof(t, x, input, params, static_Fz, z_eq, z_u_eq)
```

#### 役割
`ode23s` 用に 40 状態の時間微分を返します。構造は `vehicle_14dof_model` 内の `derivatives` と近いですが、空力、タイヤ、駆動がより簡素です。

#### 状態内訳
- `x(1:12)`: ばね上 6 自由度位置姿勢と速度角速度
- `x(13:16)`: ばね下変位
- `x(17:20)`: ばね下速度
- `x(21:24)`: 車輪角速度
- `x(25:28)`: `tire_xi`
- `x(29:32)`: `tire_eta`
- `x(33:36)`: `damper_z`
- `x(37:40)`: `friction_z`

#### 主処理
- 各輪ごとにサスペンション変位・速度を計算
- スプリング、Bouc-Wen、LuGre、タイヤ鉛直力を求める
- ステア角を考慮して接地点速度を車輪座標へ変換
- `tmeasy` で `Fx_t`、`Fy_t` と緩和状態微分を計算
- ばね下加速度 `az_u_vec` を計算
- `compute_arb` の結果を加算
- 全輪合計の力とモーメントを求める
- 空力は 4D マップではなく
  - `Fz += 0.5 * rho * v2 * CL_A`
  - `Fx -= 0.5 * rho * v2 * CD_A`
- 重力を車体座標へ変換し、ばね上加速度と角加速度を計算
- 駆動は後輪 2 輪に等分で与えます。`input.throttle * max_torque * gear_ratio * efficiency / 2`
- 最後に 40 状態分の微分列ベクトルを返します。

### `euler_to_rotation`
オイラー角 `phi, theta, psi` から 3×3 回転行列を作ります。

### `angular_to_euler_rate`
角速度 `[p;q;r]` をオイラー角速度へ変換する 3×3 行列を返します。

### `compute_arb`
前後それぞれ左右変位差からロール角相当量を求め、ARB トルクを左右輪力へ変換します。
- 前輪: `F(1), F(2)`
- 後輪: `F(3), F(4)`

### `tmeasy`
`xi`、`eta` の緩和モデルと摩擦飽和を含む簡略 TMeasy です。
- `dxi = vsx - (vcx / tire.Lx) * xi`
- `deta = vsy - (vcx / tire.Ly) * eta`
- 正規化量 `xi_n`、`eta_n`
- `s < 1` なら `mu = s * (2 - s)`、それ以上は `mu = 1`
- `s > 1e-6` のときのみ力を返し、そうでなければ `Fx = Fy = 0`

## しきい値
- `RelTol = 1e-6`
- `AbsTol = 1e-8`
- `MaxStep = 1e-3`
- 正確性判定: 位置誤差 `0.5 m`
- 高速性判定: `time3 < time2`

## 保存物
- `../../output/rosenbrock_comparison.png`
