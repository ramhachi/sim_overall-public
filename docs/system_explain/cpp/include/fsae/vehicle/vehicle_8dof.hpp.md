# vehicle_8dof.hpp

## 1. ファイルの目的
6DOF ばね上運動に各輪ばね下垂直動を加えた 8DOF モデルを定義するヘッダです。4 輪の `z_u` / `vz_u` を状態に持ち、路面入力やタイヤ縦剛性の影響を表現します。

## 2. 実行上の役割
`vehicle_6dof.hpp` より詳細な上下系モデルとして、サスペンションとタイヤの相互作用を計算します。状態次元は 32 です。

## 3. 依存関係
- `vehicle_params.hpp`, `vehicle_state.hpp`
- `physics/tire/tmeasy.hpp`
- `math/rotation.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`
- 出力: 更新後 `State8DOF`
- 補助出力: `z_equilibrium()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `class Vehicle8DOF`
#### `using Params = VehicleParams`
共有パラメータ型エイリアスです。

#### `struct State8DOF`
- ばね上位置姿勢: `x,y,z,phi,theta,psi`
- ばね上速度: `vx,vy,vz,p,q,r`
- ばね下状態: `z_u[4]`, `vz_u[4]`
- 補助状態: `omega[4]`, `tire_xi[4]`, `tire_eta[4]`
- `speed()`
- `SIZE = 32`

#### メソッド
- `Vehicle8DOF(const Params&)`
- `state() const / state()`, `params()`, `time()`, `z_equilibrium()`
- `set_initial_velocity(double)`
- `step(double, const DriverInput&)`
- `reset()`: ばね上/ばね下を平衡位置へ戻す
- `setup_tire_params()`
- `compute_equilibrium()`: 静荷重、ばね上/ばね下平衡位置計算
- `get_wheel_positions()`
- `state_to_vector(...)`, `vector_to_state(...)`
- `compute_derivatives(...)`: ばね上・ばね下の力学、TMeasy タイヤ力、空力、車輪回転を統合計算
