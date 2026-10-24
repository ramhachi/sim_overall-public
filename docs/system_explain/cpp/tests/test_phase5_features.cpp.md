# `cpp/tests/test_phase5_features.cpp`

## 1. ファイルの目的

Phase 5 で追加された車両機能群――アッカーマン幾何、コンプライアンスステア、ARB、LuGre 安定性、適応タイムステップ、および総合統合動作――をまとめて検証するテストです。

## 2. 実行上の役割

- `Vehicle14DOF_New` に追加された新機能が単体でも組合せでも破綻しないことを確認します。
- 幾何計算・力学補助計算・数値安定性のそれぞれを分解して保証します。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `Vehicle14DOF_New`, `DriverInput`
- `fsae/vehicle/vehicle_params.hpp`
  - `VehicleParams`
- `fsae/physics/suspension/lugre.hpp`
  - `LuGreParams`, `compute_lugre`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - 車両パラメータ、舵角、アッカーマン比、セルフアライニングトルク、サスペンショントラベル、LuGre パラメータ、ドライバ入力
- 出力
  - ステア角配分
  - コンプライアンスステア量
  - ARB 力
  - LuGre の力と毛変形微分
  - 14DOF シミュレーション状態

## 5. 関数・メソッド・クラスの網羅的な説明

### `class Phase5Test : public ::testing::Test`

- Phase 5 機能の共通フィクスチャです。

#### `void SetUp() override`

- デフォルト `VehicleParams` を読み込み、各テストがそこから個別調整できる状態にします。

#### メンバ

- `VehicleParams params`

### `TEST_F(Phase5Test, AckermannGeometry_ZeroSteer_ParallelWheels)`

- ゼロ舵角時に全輪舵角が 0 になることを確認します。

### `TEST_F(Phase5Test, AckermannGeometry_LeftTurn_InnerWheelGreater)`

- 左旋回では内輪である左前輪の舵角が右前輪より大きくなることを確認します。

### `TEST_F(Phase5Test, AckermannGeometry_RightTurn_InnerWheelGreater)`

- 右旋回時も内外輪差が生じ、実装符号規約に従って左前輪側の絶対値が大きいことを確認します。

### `TEST_F(Phase5Test, AckermannGeometry_FullAckermann_vs_Parallel)`

- 100% アッカーマンと 0% パラレルを比較し、設定比率が舵角差へ正しく反映されることを確認します。

### `TEST_F(Phase5Test, ComplianceSteer_ZeroTorque_ZeroCompliance)`

- 前輪へ与えるセルフアライニングトルクが 0 ならコンプライアンスステアも 0 になることを確認します。

### `TEST_F(Phase5Test, ComplianceSteer_PositiveTorque_PositiveAngle)`

- 正のトルクに対して正のコンプライアンス変位が生じ、後輪は変化しないことを確認します。

### `TEST_F(Phase5Test, ComplianceSteer_AsymmetricTorque)`

- 左右異なるトルク入力で左右異なる舵角補正が出ること、かつ `Mz / K_steer` の比例関係を満たすことを確認します。

### `TEST_F(Phase5Test, ARB_ZeroRollDifference_ZeroForce)`

- 左右サスペンション変位差が無いと ARB 力も 0 になることを確認します。

### `TEST_F(Phase5Test, ARB_FrontRoll_OppositeForces)`

- フロントロール時にフロント左右へ逆向き・等大の ARB 力が発生することを確認します。

### `TEST_F(Phase5Test, ARB_RearRoll_OppositeForces)`

- リアロール時にリア左右へ逆向き・等大の ARB 力が発生することを確認します。

### `TEST_F(Phase5Test, LuGre_StiffnessConstraint)`

- LuGre モデルの `sigma_0` が設計上の安定性上限内にあることを確認します。

### `TEST_F(Phase5Test, LuGre_ZeroVelocity_NoForce)`

- 速度 0 で LuGre 計算が有限値を返し、毛変形速度が 0 であることを確認します。

### `TEST_F(Phase5Test, LuGre_StickSlipTransition)`

- 低速・高速で LuGre 計算がともに有限値を返すことを確認し、スティック / スリップ遷移時の破綻を防ぎます。

### `TEST_F(Phase5Test, AdaptiveTimestep_NormalCondition_StandardDt)`

- 通常状態で `step()` が例外なく実行でき、代表状態量が有限値を保つことを確認します。

### `TEST_F(Phase5Test, AdaptiveTimestep_StiffEvent_NoExplosion)`

- 強い沈み込みとフルブレーキを与える厳しい条件でも、100 ステップ連続で状態が発散しないことを確認します。

### `TEST_F(Phase5Test, Integration_CorneringWithAllFeatures)`

- 旋回シミュレーションを 1 秒実行し、全状態が有限で、姿勢・速度が物理的上限を超えず、実際に横変位とヨーレートが発生することを確認します。
