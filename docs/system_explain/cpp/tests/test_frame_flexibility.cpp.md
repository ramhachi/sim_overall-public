# `cpp/tests/test_frame_flexibility.cpp`

## 1. ファイルの目的

Phase A1 で導入されたフレーム柔軟性モデルについて、剛体極限、ねじり発生、ロール配分、数値安定性、後方互換性を確認するテストです。

## 2. 実行上の役割

- `Vehicle14DOF_New` 内のシャシーねじり自由度が過大でも過小でもなく動作することを検証します。
- 従来の剛体シャシー前提を壊さないことも同時に確認します。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `Vehicle14DOF_New`, `DriverInput`
- `fsae/vehicle/vehicle_params.hpp`
  - `VehicleParams::default_fsae`
  - `suspension.K_chassis`, `C_chassis`, `I_twist`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - フレーム剛性・減衰・慣性の設定
  - 定常旋回を生む操舵・スロットル入力
- 出力
  - `get_frame_twist()`
  - `get_frame_twist_rate()`
  - `get_split_roll_angles()`
  - テスト結果

## 5. 関数・メソッド・クラスの網羅的な説明

### `class FrameFlexibilityTest : public ::testing::Test`

- フレーム柔軟性テスト用フィクスチャです。

#### `void SetUp() override`

- 既定 FSAE 車両を読み込み、柔軟フレームが観測できる代表パラメータを設定します。

#### メンバ

- `VehicleParams params`

### `TEST_F(FrameFlexibilityTest, RigidFrameDegeneration)`

- `K_chassis` を非常に大きくし、剛体フレーム極限でねじれ角がほぼ 0 になることを確認します。

### `TEST_F(FrameFlexibilityTest, MeasurableTwist)`

- 柔軟フレーム設定で、現実的範囲内の有意なねじれが発生することを確認します。

### `TEST_F(FrameFlexibilityTest, RollStiffnessDistribution)`

- 剛体車両と柔軟車両を比較し、柔軟車両の方が前後ロール角差が大きくなることを検証します。

### `TEST_F(FrameFlexibilityTest, NumericalStability)`

- 10 秒相当の長時間シミュレーションで `phi_twist` とその時間微分が NaN / Inf にならないことを確認します。

### `TEST_F(FrameFlexibilityTest, BackwardCompatibility)`

- デフォルトパラメータでは従来どおり剛体近似が保たれ、ねじれがほぼ発生しないことを確認します。
