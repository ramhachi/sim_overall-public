# `matlab/rl_driver/generateRandomVehicleParams.m`

## 1. 役割

`generateRandomVehicleParams` は、Domain Randomization 用の車両パラメータ構造体を生成する関数です。  
`getVehicleParamBounds()` が与える許容範囲の中から、`strength` に応じた広さで各値をサンプリングし、シミュレータが使う完全な `params` 構造体へ落とし込みます。

## 2. 関数一覧

1. `generateRandomVehicleParams(bounds, strength, base_params)`
2. `randInRange(range, strength)`

## 3. `generateRandomVehicleParams(bounds, strength, base_params)` 詳細

### 初期状態の決定

- `base_params` が無ければ `create_vehicle_baseline_template()` を採用
- ある場合はその構造体を出発点にする

つまり randomization の対象外の値は、ベース構造体由来のまま残せます。

### Chassis の生成

- `m_total` をサンプル
- `front_dist` と `wheelbase` から `a`, `b` を再計算
- `h_cg`, `track_f`, `track_r` をサンプル
- 慣性モーメント `Ixx`, `Iyy`, `Izz` は質量比 `scale_m` でスケール

ここでは慣性を独立 randomization せず、質量に従属させています。

### Suspension の生成

- 前後ばね定数を取り、4 輪配列 `k_spring = [FF, FR, RL, RR]` を構築
- 前後減衰から平均 `c0` を生成
- 前後 ARB をサンプル
- `motion_ratio` と `camber_gain` を前後別に 4 要素配列化
- キャンバー・トーは度数でサンプル後、ラジアンに変換して左右対称配置

### Aero の生成

- `CL_A` をサンプル
- `CD_A = compute_aero_cd_a(CL_A, ...)` で派生計算
- `aero_balance` をサンプル

### Powertrain の生成

- `lsd_preload`
- `lsd_ramp_accel`
- `lsd_ramp_decel`
- `final_ratio`

### Tire / Steering の生成

- `mu_x_max`, `mu_y_max`
- `steering.stiffness`

### 拡張 tunable の適用

`bounds.extra_tunable_specs` が存在すれば:

1. 各 spec に `index` を付与
2. `[lower, upper]` から乱数 `x(i)` を生成
3. `apply_vehicle_extra_tunables(params, x, specs)` でまとめて反映

これにより core 項目外の追加設計変数も同じ仕組みで randomization できます。

## 4. `randInRange(range, strength)` 詳細

`[min, max]` の中央 `mid` と半幅 `half` を計算し、

- `strength = 1.0` なら全域から一様サンプル
- `strength = 0.0` なら中央固定
- その中間なら中央近傍だけに縮めた一様サンプル

を返します。最後に `max/min` で範囲内へクリップします。

## 5. 実装上の要点

- `strength` は「乱れ幅スカラー」であり、確率分布の形自体は一様です。  
- すべてを独立に振るのではなく、`a/b` や `CD_A`、慣性など一部は派生計算で整合を保っています。  
- `FSAEEnvironment.reset()` が Domain Randomization 周期に応じてこの関数を呼びます。
