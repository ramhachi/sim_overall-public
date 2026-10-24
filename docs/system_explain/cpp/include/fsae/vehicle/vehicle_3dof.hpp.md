# vehicle_3dof.hpp

## 1. ファイルの目的
平面運動だけを扱う 3DOF 簡略車両モデルを定義するヘッダです。

## 2. 実行上の役割
デバッグや簡易試験向けに、z・ロール・ピッチを省いた最小構成の車両ダイナミクスを RK4 で積分します。

## 3. 依存関係
- `vehicle_params.hpp`, `vehicle_state.hpp`
- `physics/tire/tmeasy.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`, 初期速度
- 出力: 更新後 `State3DOF`

## 5. 関数・メソッド・クラスの網羅的な説明
### `class Vehicle3DOF`
#### `using Params = VehicleParams`
共有車両パラメータ型です。

#### `struct State3DOF`
`x,y,psi,vx,vy,r,omega[4],tire_xi[4],tire_eta[4]` を持ち、`speed()` で平面速度を返します。

#### メソッド
- `Vehicle3DOF(const Params&)`
- `state() const / state()`, `params()`, `time()`
- `set_initial_velocity(double)`
- `step(double, const DriverInput&)`
- `reset()`
- `setup_tire_params()`
- `compute_static_loads()`
- `state_to_vector(...)`
- `vector_to_state(...)`
- `compute_derivatives(...)`
