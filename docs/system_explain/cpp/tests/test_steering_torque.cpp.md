# `cpp/tests/test_steering_torque.cpp`

## 1. ファイルの目的

Phase A2 のステアリング操舵トルクモデルについて、単体関数と 14DOF 車両統合の両面から検証するテストです。

## 2. 実行上の役割

- ステアリングホイール角加速度計算、操舵比変換、トルクモード挙動、飽和、後方互換性、数値安定性を確認します。
- トルク入力型ドライバーや車両モデルの前提を支える土台テストです。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `Vehicle14DOF_New`, `DriverInput`
- `fsae/vehicle/vehicle_params.hpp`
  - `VehicleParams::default_fsae`
- `fsae/vehicle/steering_torque.hpp`
  - `compute_steering_acceleration`
  - `steering_wheel_to_tire_angle`
  - `tire_angle_to_steering_wheel`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - ステアリング慣性、ドライバトルク、SAT、操舵比、摩擦、粘性減衰
  - 14DOF 車両へのトルク / 角度入力
- 出力
  - ステアリング角加速度
  - タイヤ角とハンドル角の変換値
  - 車両内部のステアリングホイール角・角速度

## 5. 関数・メソッド・クラスの網羅的な説明

### `class SteeringTorqueTest : public ::testing::Test`

- トルクモード車両を構築するフィクスチャです。

#### `void SetUp() override`

- デフォルト車両へトルクモード関連係数を設定します。

#### メンバ

- `VehicleParams params`

### `TEST(SteeringTorqueFunction, TorqueBalance)`

- `compute_steering_acceleration` 単体を呼び、トルク収支から求まる角加速度が理論計算と一致することを確認します。

### `TEST(SteeringTorqueFunction, SteeringRatioConversion)`

- ハンドル角とタイヤ角の相互変換が操舵比 `R_steer` で可逆になることを確認します。

### `TEST_F(SteeringTorqueTest, TorqueModeSimulation)`

- 正のドライバトルクでシミュレーションすると、ステアリングホイール角が実際に回転することを確認します。

### `TEST_F(SteeringTorqueTest, TorqueSaturation)`

- 過大トルク入力でも飽和により角速度が有限範囲に抑えられることを確認します。

### `TEST_F(SteeringTorqueTest, AngleModeBackwardCompatibility)`

- 角度モードでは内部のステアリングホイール状態を更新せず、従来挙動を維持することを確認します。

### `TEST_F(SteeringTorqueTest, NumericalStability)`

- 10 秒相当の長時間シミュレーションでステア角と角速度が NaN / Inf にならないことを確認します。
