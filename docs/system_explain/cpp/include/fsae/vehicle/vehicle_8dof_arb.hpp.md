# vehicle_8dof_arb.hpp

## 1. ファイルの目的
8DOF モデルに前後アンチロールバーを追加した車両モデルです。左右サスペンション変位差から ARB トルクを発生させます。

## 2. 実行上の役割
8DOF 基本モデルに対し、ロール剛性配分を明示的に持たせる拡張版として使われます。ARB 力はばね上・ばね下双方へ反映されます。

## 3. 依存関係
- `vehicle_8dof.hpp`: `State8DOF` を再利用
- `vehicle_params.hpp`, `vehicle_state.hpp`, `physics/tire/tmeasy.hpp`, `math/rotation.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`
- 出力: 更新後 `State8DOF`
- デバッグ出力: 前後 ARB 力 `get_arb_forces()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `class Vehicle8DOF_ARB`
- `using Params = VehicleParams`
- `using State8DOF = Vehicle8DOF::State8DOF`
- `Vehicle8DOF_ARB(const Params&)`
- `state() const / state()`, `params()`, `time()`, `z_equilibrium()`
- `set_initial_velocity(double)`
- `step(double, const DriverInput&)`
- `reset()`
- `get_arb_forces()`: フロント/リア ARB トルクを返す
- `setup_tire_params()`, `compute_equilibrium()`, `get_wheel_positions()`
- `state_to_vector(...)`, `vector_to_state(...)`
- `compute_arb_forces(...)`: 左右サスペンション変位差から各輪への ARB 力増分を計算
- `compute_derivatives(...)`: 8DOF 基本式に ARB 力を加えた微分計算
