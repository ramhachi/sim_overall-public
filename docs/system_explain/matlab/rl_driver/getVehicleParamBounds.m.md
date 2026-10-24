# `matlab/rl_driver/getVehicleParamBounds.m`

## 1. 役割

`getVehicleParamBounds` は、Domain Randomization や RL 用車両条件観測で使う車両パラメータ範囲を定義する関数です。  
コメントにもある通り、`build_full_vehicle_design_schema.m` の canonical bounds と shared tunables に整合する境界をここへ集約しています。

## 2. `getVehicleParamBounds()` 詳細

関数は `bounds` 構造体を組み立てて返します。  
冒頭で `addpath(fileparts(this_dir))` を行い、親ディレクトリの依存関数を利用できるようにしています。

### Chassis 系

- `mass.m_total = [250, 350]`
- `mass.front_dist = [0.40, 0.60]`
- `mass.h_cg = [0.25, 0.40]`
- `mass.wheelbase = [1.53, 1.85]`
- `mass.track_f = [1.10, 1.45]`
- `mass.track_r = [1.10, 1.45]`

車体寸法と重心位置の主要レンジです。

### Suspension 系

- 前後ばね定数 `k_spring_f`, `k_spring_r`
- 減衰 `c0_f`, `c0_r`
- 前後 ARB `k_arb_f`, `k_arb_r`
- 前後キャンバー `camber_f`, `camber_r`
- 前後トー `toe_f`, `toe_r`
- 前後 motion ratio
- 前後 camber gain
- フレーム関連 `K_chassis`, `C_chassis`, `I_twist`

サスペンションとフレーム柔軟性をまとめて randomization 対象にしています。

### Aero 系

- `aero.CL_A = [0, 5.0]`
- `aero.aero_balance = [0.30, 0.70]`

抗力 `CD_A` はここでは直接持たず、後段で `CL_A` から派生計算されます。

### Powertrain 系

- `lsd_preload`
- `lsd_ramp_accel`
- `lsd_ramp_decel`
- `final_ratio`

駆動系の主要調整ノブです。

### Tire 系

- `mu_x_max = [1.1, 2.2]`
- `mu_y_max = [1.3, 3.0]`

### Steering 系

- `steering.stiffness = [1000, 30000]`

### 拡張チューナブル項目

- `extra_tunable_specs = build_extended_vehicle_tunable_specs()`

ここは core bounds に含まれない拡張調整項目群です。  
`generateRandomVehicleParams()` 側がこの specs を参照し、追加 tunable を一括適用します。

## 3. 実装上の要点

- RL 側 randomization 範囲の単一情報源です。  
- `get_rl_observation_bounds()` の車両観測レンジとも直結します。  
- 拡張 tunable が `extra_tunable_specs` としてまとめられているため、将来の設計変数追加に対応しやすい構造です。
