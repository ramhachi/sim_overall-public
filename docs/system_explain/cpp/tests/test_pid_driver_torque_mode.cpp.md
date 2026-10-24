# `cpp/tests/test_pid_driver_torque_mode.cpp`

## 1. ファイルの目的

PID 経路追従ドライバーが、従来の操舵角モードだけでなくトルクモードでも適切に制御入力を出せることを検証するテストです。

## 2. 実行上の役割

- 外側の経路追従ループと内側の操舵トルクループのつながりを確認します。
- ダンピング項、飽和、角度モード後方互換性の保証も担います。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/vehicle/vehicle_params.hpp`
- `fsae/driver/pid_driver.hpp`
  - `PIDPathFollowingDriver`, `PIDGains`, `conservative_gains`
- `fsae/track/track_model.hpp`
  - `create_acceleration_track`, `CurvatureBasedTrack`
- `fsae/laptime/velocity_profile.hpp`
  - `VelocityPoint`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - PID ゲイン、トルクモード設定、横偏差を持つ `State14DOF`
  - 100m 加速トラック、簡易速度プロファイル
- 出力
  - `DriverInput.steering`（トルクまたは角度）
  - 制御入力の飽和結果

## 5. 関数・メソッド・クラスの網羅的な説明

### `class PIDDriverTorqueTest : public ::testing::Test`

- PID ドライバーのトルクモード試験用フィクスチャです。

#### `void SetUp() override`

- デフォルト FSAE 車両をトルクモードへ切り替え、一定 10m/s の速度プロファイルを準備します。

#### メンバ

- `VehicleParams params`
- `CurvatureBasedTrack track`
- `std::vector<VelocityPoint> velocity_profile`

### `TEST_F(PIDDriverTorqueTest, InnerLoopTorqueOutput)`

- 横偏差を持つ状態に対し、内側操舵ループが右向き修正トルクを正符号で出力することを確認します。

### `TEST_F(PIDDriverTorqueTest, InnerLoopDampingEffect)`

- 既に操舵角速度がある場合、微分ゲインによるダンピングでトルク指令が減少することを確認します。

### `TEST_F(PIDDriverTorqueTest, AngleModeCompatibility)`

- `torque_mode = false` では従来どおり角度指令が返り、値域も角度らしい範囲にあることを確認します。
