# `matlab/vehicle_3dof/vehicle_3dof_model.m`

## 役割

この関数は、平面 3 自由度の車両運動と 4 輪の車輪回転・TMeasy 系タイヤ内部状態をまとめて時間積分する MATLAB 実装です。  
ファイル冒頭コメントでは「C++ との検証用」と明記されており、`run_3dof_verification.m` から呼ばれる前提の構成になっています。

## 関数入口

```matlab
[t, state, data] = vehicle_3dof_model(params, t_span, dt, input_func)
```

## 入力

- `params`: 車両・タイヤ・空力・駆動系パラメータを持つ構造体
- `t_span`: `[開始時刻, 終了時刻]`
- `dt`: 固定時間刻み
- `input_func`: `t` を受けて `struct('steering', ..., 'throttle', ...)` を返す関数

## 出力

- `t`: `t_span(1):dt:t_span(2)` で作る時間ベクトル
- `state`: 各時刻の 18 状態
- `data`: 付加ログ
  - `Fx`: 各輪の車両座標系縦力
  - `Fy`: 各輪の車両座標系横力
  - `ax`: 縦加速度
  - `ay`: 横加速度

## 状態変数

状態数は 18 です。

1. `x`
2. `y`
3. `psi`
4. `vx`
5. `vy`
6. `r`
7-10. `omega(4)`
11-14. `tire_xi(4)`
15-18. `tire_eta(4)`

## 関数本体の処理

### 1. 初期状態生成

`x0 = zeros(18,1)` で初期化し、

- `x0(4) = params.v0`
- `x0(7:10)` は各輪とも `params.v0 / params.tire.radius`

としています。初期車体速度と車輪周速度を一致させる構成です。

### 2. 静的輪荷重

ばね上質量だけを使って

- `W = m_sprung * 9.81`
- `Wf = W * b / L`
- `Wr = W * a / L`
- `static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2]`

を計算します。ここではばね下質量は静的輪荷重に含めていません。

### 3. RK4 積分

時間積分は固定刻みの古典 4 次 Runge-Kutta です。

- `k1 = derivatives(x, ...)`
- `k2 = derivatives(x + 0.5*dt*k1, ...)`
- `k3 = derivatives(x + 0.5*dt*k2, ...)`
- `k4 = derivatives(x + dt*k3, ...)`

更新式は `x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)` です。

### 4. ログ記録

各ステップで `forces` から

- `data.Fx(k,:)`
- `data.Fy(k,:)`
- `data.ax(k)`
- `data.ay(k)`

を保存します。最終行は 1 つ前の値をそのままコピーしています。

## ローカル関数

### `derivatives(x, input, params, static_Fz)`

この関数が状態微分と力計算の中心です。

#### 状態展開

- `psi = x(3)`
- `vx = x(4)`
- `vy = x(5)`
- `r = x(6)`
- `omega = x(7:10)`
- `tire_xi = x(11:14)`
- `tire_eta = x(15:18)`

#### 車輪位置

- 前輪 `x = +a`
- 後輪 `x = -b`
- 左右位置は `track_f/2`, `track_r/2` を使って
  - `y_pos = [-tf, tf, -tr, tr]`

としています。

#### 操舵

前輪のみ同一操舵角です。

- `steer = [delta, delta, 0, 0]`

左右差やアッカーマンはありません。

#### 各輪の接地点速度とタイヤ力

各輪ごとに次を計算します。

1. 車両座標系接地点速度
   - `vx_contact = vx - r * y_pos(i)`
   - `vy_contact = vy + r * x_pos(i)`
2. ホイール座標系への回転変換
3. スリップ速度
   - `vsx = vx_wheel - omega_r`
   - `vsy = vy_wheel`
   - `vcx = abs(omega_r)`
4. `tmeasy_force(...)` 呼出し
5. タイヤ力をホイール座標系から車両座標系へ戻す

`tmeasy_force` 呼出し時の転がり速度は `max(vcx, 0.1)` で下限を設けています。  
`params.tire.v_N` は `tmeasy_force` 内で読み出されますが、力計算では実際には使われていません。

#### 合力と空力

- `Fx_total = sum(Fx_wheel)`
- `Fy_total = sum(Fy_wheel)`
- `Mz_total = sum(x_pos .* Fy_wheel - y_pos .* Fx_wheel)`

に加えて、抗力だけを

- `Fx_aero = -0.5 * rho * (vx^2 + vy^2) * CD_A`

として `Fx_total` に加算します。ダウンフォースや横力はありません。

#### 並進・回転運動

ばね上質量 `m_sprung` と `Izz` を用いて

- `ax = Fx_total / m + r * vy`
- `ay = Fy_total / m - r * vx`
- `dr = Mz_total / Izz`

を計算します。

#### 慣性座標系位置微分

- `dx = cos(psi) * vx - sin(psi) * vy`
- `dy = sin(psi) * vx + cos(psi) * vy`
- `dpsi = r`

です。

#### 車輪回転

エンジントルクは

- `engine_torque = throttle * max_torque`
- `drive_torque = engine_torque * gear_ratio * efficiency / 2`

で計算し、`i >= 3` の後輪にのみ与えます。  
各輪のタイヤ反力トルクは、車両座標系力を再度ホイール座標系縦力へ戻して `T_tire = -Fx_w * R` としています。

#### 戻り値

- 状態微分
  - `[dx; dy; dpsi; ax; ay; dr; domega; dxi; deta]`
- `forces`
  - `Fx`, `Fy`
  - `ax`
  - `ay = ay + vx * r`

`forces.ay` は運動方程式で使う `ay` に `vx*r` を加えた値で、コメントでは「遠心力補正」とされています。

### `tmeasy_force(xi, eta, Fz, vsx, vsy, vcx, tire)`

TMeasy 風の簡略タイヤ力モデルです。

#### 使うパラメータ

- `mu_x_max`, `mu_y_max`
- `Cx`, `Cy`
- `Lx`, `Ly`
- `v_N`

ただし `v_N` はローカル変数に読み出すだけで、この関数内では使っていません。

#### 計算内容

1. 変形状態の微分
   - `dxi = vsx - (vcx / Lx) * xi`
   - `deta = vsy - (vcx / Ly) * eta`
2. 正規化変形量
   - `xi_norm = Cx * xi / (mu_x * Fz + 1)`
   - `eta_norm = Cy * eta / (mu_y * Fz + 1)`
3. 合成スリップ量 `s`
4. 飽和特性
   - `s < 1` のとき `mu_eff = s * (2 - s)`
   - それ以外は `mu_eff = 1`
5. 力
   - `s > 1e-6` のときのみ計算
   - それ未満では `Fx = 0`, `Fy = 0`

`mu_x * Fz + 1` のように分母へ `+1` を足しているため、荷重ゼロ付近でもゼロ除算を避ける作りです。

## 前提と簡略化

- 前輪は左右同じ操舵角です。
- 駆動は後輪のみです。
- 荷重移動は明示的に扱わず、静的輪荷重 `static_Fz` を全時刻で使います。
- 空力は抗力のみです。
- 積分刻みは可変ではなく `dt` 固定です。
