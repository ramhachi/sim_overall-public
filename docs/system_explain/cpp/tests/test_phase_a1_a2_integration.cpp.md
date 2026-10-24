# `cpp/tests/test_phase_a1_a2_integration.cpp`

## 1. ファイルの目的

柔軟フレーム（Phase A1）とトルクステアリング（Phase A2）を同時に有効化した 14DOF 車両の統合挙動を確認するテストです。

## 2. 実行上の役割

- 独立機能として成立していた 2 つの拡張が、同時有効時にも矛盾なく動作するかを確認します。
- 定常応答とステップ応答の両面から連成を見ます。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `Vehicle14DOF_New`, `DriverInput`
- `fsae/vehicle/vehicle_params.hpp`
  - `VehicleParams::default_fsae`
  - `suspension` / `steering` サブパラメータ
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - 柔軟フレーム関連係数
  - トルクステアリング関連係数
  - ドライバトルク入力、スロットル、ギヤ、時間刻み
- 出力
  - フレームねじれ角
  - ステアリングホイール角
  - 応答時間経過による変化量

## 5. 関数・メソッド・クラスの網羅的な説明

### `class IntegrationTest : public ::testing::Test`

- Phase A1 / A2 統合テスト用フィクスチャです。

#### `void SetUp() override`

- デフォルト車両へ柔軟フレーム設定とトルクステアリング設定を同時に注入します。

#### メンバ

- `VehicleParams params`

### `TEST_F(IntegrationTest, FlexibleFrameWithTorqueSteering)`

- 定常的な操舵トルク入力とスロットル入力を与え、フレームねじれとステアリングホイール角が有意に発生することを確認します。
- 連成後の定常特性確認に相当します。

### `TEST_F(IntegrationTest, DynamicResponse)`

- 直進から操舵トルクのステップ入力を与え、時間経過とともにステア角とフレームねじれが増大することを確認します。
- 操舵遅れとフレームねじれの連成動特性を見るテストです。
