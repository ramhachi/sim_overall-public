# vehicle_8dof_full.hpp

## 1. ファイルの目的
8DOF + ARB + Bouc-Wen に LuGre 摩擦まで加えたフルサスペンション非線形モデルです。ダンパーヒステリシスと摺動摩擦の両方を状態として持ちます。

## 2. 実行上の役割
サスペンションの非線形性を最も細かく確認するための中間モデルで、ばね上・ばね下・タイヤ・ダンパー・摩擦の相互作用を 40 状態で積分します。

## 3. 依存関係
- `vehicle_8dof_bw.hpp`
- `physics/suspension/lugre.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`
- 出力: 更新後 `State8DOF_Full`
- デバッグ出力: `get_arb_forces()`, `get_bouc_wen_forces()`, `get_lugre_forces()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct State8DOF_Full`
- 8DOF_BW の状態に `friction_z[4]` を追加した構造体です
- `SIZE = 40`

### `class Vehicle8DOF_Full`
- `using Params = VehicleParams`
- `Vehicle8DOF_Full(const Params&)`: タイヤ、Bouc-Wen、LuGre パラメータ、平衡位置を初期化
- `state() const / state()`, `params()`, `time()`, `z_equilibrium()`
- `set_initial_velocity(double)`
- `step(double, const DriverInput&)`
- `reset()`: `damper_z` と `friction_z` をゼロへ戻す
- `get_arb_forces()`, `get_bouc_wen_forces()`, `get_lugre_forces()`
- `setup_tire_params()`
- `setup_bouc_wen_params()`
- `setup_lugre_params()`: `SuspensionParams` から LuGre 係数を設定
- `compute_equilibrium()`
- `state_to_vector(...)`, `vector_to_state(...)`
- `compute_arb_forces(...)`
- `compute_derivatives(...)`: Bouc-Wen 力・LuGre 力・タイヤ力・空力・車輪回転を含む微分計算
- `compute_rotation_matrices(...)`: 車体系/オイラー角速度変換行列を返す helper
