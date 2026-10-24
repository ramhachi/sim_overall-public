# `matlab/rl_driver/get_rl_observation_bounds.m`

## 1. 役割

`get_rl_observation_bounds` は、RL 観測の各次元に対する下限・上限を 45×2 行列で返す共有契約関数です。  
MATLAB 環境の正規化処理と、将来的なモデル export 側の観測仕様を一致させるための基準点になっています。

## 2. 関数の責務

- ベース車両または上書き済み車両パラメータの解決
- 車両パラメータ bounds の取得
- 操舵モードに応じた操舵入力レンジ決定
- 空力 `CD_A` の範囲推定
- 45 次元観測それぞれの上下限定義

## 3. `get_rl_observation_bounds(vehicleParams)` 詳細

### 初期化部分

- 自身の親ディレクトリを `addpath` して依存関数を利用可能にする
- 引数省略時は空構造体を使用
- `getVehicleParamBounds()` で車両パラメータの許容範囲を取得
- `resolve_rl_vehicle_params(vehicleParams)` で現在の車両設定を確定

### 操舵レンジ決定

`vehicle_params.steering.torque_mode` が真なら、観測 7 番目の操舵入力レンジは `[-T_driver_max, T_driver_max]` です。  
偽なら `[-max_angle, max_angle]` になります。

### `CD_A` レンジの算出

`vehicle_bounds.aero.CL_A` の最小・最大を `compute_aero_cd_a(...)` へ通し、その 2 点を `sort` して `cd_a_bounds` にしています。  
つまり `CD_A` は独立変数ではなく、`CL_A` に従属する派生量として bounds を推定しています。

## 4. 45 次元観測の定義

以下の表で、返り値行列の各行がどの観測に対応するかを整理します。

| 次元 | 内容 | 範囲 |
|---|---|---|
| 1 | `vx` | `[0, 40]` |
| 2 | `vy` | `[-10, 10]` |
| 3 | yaw rate | `[-3, 3]` |
| 4 | `ax` | `[-15, 15]` |
| 5 | `ay` | `[-15, 15]` |
| 6 | roll | `[-0.15, 0.15]` |
| 7 | steering command | torque/angle モード依存 |
| 8 | throttle | `[0, 1]` |
| 9 | brake | `[0, 1]` |
| 10-13 | 各輪 slip ratio | `[-0.3, 0.3]` |
| 14 | `alpha_f` | `[-0.5, 0.5]` |
| 15 | `alpha_r` | `[-0.5, 0.5]` |
| 16 | lateral deviation | `[-5, 5]` |
| 17 | heading error | `[-1, 1]` |
| 18-23 | 現在/先読み曲率 | `[-0.2, 0.2]` |
| 24 | 左端まで距離 | `[0, 5]` |
| 25 | 右端まで距離 | `[0, 5]` |
| 26 | lap progress | `[0, 1]` |
| 27-30 | lookahead width | `[1, 10]` |
| 31 | frame twist | `[-0.10, 0.10]` |
| 32 | frame twist rate | `[-3, 3]` |
| 33 | steering wheel angle | `[-3, 3]` |
| 34 | steering wheel rate | `[-20, 20]` |
| 35 | total mass | `vehicle_bounds.mass.m_total` |
| 36 | `CL_A` | `vehicle_bounds.aero.CL_A` |
| 37 | `CD_A` | `cd_a_bounds` |
| 38 | `mu_y_max` | `vehicle_bounds.tire.mu_y_max` |
| 39 | `mu_x_max` | `vehicle_bounds.tire.mu_x_max` |
| 40 | front spring | `vehicle_bounds.suspension.k_spring_f` |
| 41 | front ARB | `vehicle_bounds.suspension.k_arb_f` |
| 42 | steering stiffness | `vehicle_bounds.steering.stiffness` |
| 43 | frame torsional stiffness | `vehicle_bounds.suspension.K_chassis` |
| 44 | frame damping | `vehicle_bounds.suspension.C_chassis` |
| 45 | frame twist inertia | `vehicle_bounds.suspension.I_twist` |

1〜34 がサーバー由来の動的状態、35〜45 が車両条件です。

## 5. 依存関数との関係

- `getVehicleParamBounds()` : 車両条件 35〜45 のレンジ源泉
- `resolve_rl_vehicle_params()` : 現在車両の操舵モード解決
- `compute_aero_cd_a()` : `CL_A` から `CD_A` を導出

## 6. 実装上の要点

- 正規化レンジ変更は学習済みモデル互換性に直結するため、この関数は観測契約の中心です。  
- 35〜45 の車両観測は `get_rl_vehicle_observation_vector()` と順序整合している必要があります。  
- 7 次元目の操舵レンジだけは torque mode か angle mode かで変わります。
