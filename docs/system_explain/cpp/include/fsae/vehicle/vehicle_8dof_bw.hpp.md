# vehicle_8dof_bw.hpp

## 1. ファイルの目的
8DOF + ARB モデルに Bouc-Wen ダンパーヒステリシスを追加したヘッダです。ダンパー履歴変数 `damper_z[4]` を状態として持ちます。

## 2. 実行上の役割
サスペンション減衰を単純粘性ではなくヒステリシス付きで再現したい場合に使います。状態次元は実装上 36 です。

## 3. 依存関係
- `vehicle_8dof_arb.hpp`
- `physics/suspension/bouc_wen.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`
- 出力: 更新後 `State8DOF_BW`
- デバッグ出力: `get_arb_forces()`, `get_bouc_wen_forces()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct State8DOF_BW`
- ばね上位置姿勢、速度、ばね下状態、車輪回転、`tire_xi/eta`、`damper_z[4]`
- `SIZE = 36`

### `class Vehicle8DOF_BW`
- `using Params = VehicleParams`
- `Vehicle8DOF_BW(const Params&)`: タイヤ設定、Bouc-Wen パラメータ設定、平衡計算
- `state() const / state()`, `params()`, `time()`, `z_equilibrium()`
- `set_initial_velocity(double)`
- `step(double, const DriverInput&)`
- `reset()`: `damper_z` を含めて初期化
- `get_arb_forces()`: ARB トルク取得
- `get_bouc_wen_forces()`: 各輪ダンパー力取得
- `setup_tire_params()`
- `setup_bouc_wen_params()`: `SuspensionParams` から Bouc-Wen 係数を設定
- `compute_equilibrium()`
- `state_to_vector(...)`, `vector_to_state(...)`
- `compute_arb_forces(...)`
- `compute_derivatives(...)`: サスペンション変位から Bouc-Wen 力とタイヤ力を計算し、履歴変数 `damper_z` の微分も返す
