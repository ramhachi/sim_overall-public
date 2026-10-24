# vehicle_6dof.hpp

## 1. ファイルの目的
平面 3DOF にヒーブ・ロール・ピッチを加えた 6DOF 車両モデルを定義するヘッダです。ばね下垂直動は持たず、ばね上の姿勢変化までを扱います。

## 2. 実行上の役割
3DOF より高い忠実度で車体姿勢の影響を見たい場面向けの中間モデルです。TMeasy タイヤと回転行列を使い、24 状態を RK4 積分します。

## 3. 依存関係
- `vehicle_params.hpp`, `vehicle_state.hpp`
- `physics/tire/tmeasy.hpp`
- `math/rotation.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`, 初期速度
- 出力: 更新後 `State6DOF`
- 補助出力: 平衡位置 `z_equilibrium()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `class Vehicle6DOF`
#### `using Params = VehicleParams`
共有車両パラメータ型エイリアスです。

#### `struct State6DOF`
- 位置姿勢: `x,y,z,phi,theta,psi`
- 速度: `vx,vy,vz,p,q,r`
- 補助状態: `omega[4]`, `tire_xi[4]`, `tire_eta[4]`
- `speed()`: 平面速度
- `SIZE`: 状態次元 24

#### メソッド
- `Vehicle6DOF(const Params&)`: タイヤ設定と平衡計算
- `state() const / state()`, `params()`, `time()`
- `set_initial_velocity(double)`: 初速と車輪回転初期化
- `step(double, const DriverInput&)`: RK4 1 ステップ
- `reset()`: 状態と時刻を初期化し、`z` を平衡位置へ戻す
- `z_equilibrium()`: 静的平衡位置取得
- `setup_tire_params()`: TMeasy 設定
- `compute_equilibrium()`: 静荷重と平衡 `z` を計算
- `get_wheel_positions()`: 4 輪位置を返す
- `state_to_vector(...)`, `vector_to_state(...)`: 24 次元変換
- `compute_derivatives(...)`: 回転行列で慣性系速度へ変換し、各輪サスペンション変位・タイヤ力・ばね上運動・車輪回転を計算
