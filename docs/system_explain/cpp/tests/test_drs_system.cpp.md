# `cpp/tests/test_drs_system.cpp`

## 1. ファイルの目的

DRS（Drag Reduction System）の状態機械、安全起動条件、統計更新、および 14DOF 車両・空力モデルとの結合を検証するテスト群です。

## 2. 実行上の役割

- DRS の起動遅延、速度条件、横加速度条件、操舵角条件、即時停止を確認します。
- `Vehicle14DOF_New` と `AeroMap4D` を使った統合確認により、DRS がシミュレーション全体へ正しく反映されることを保証します。

## 3. 依存関係

- `fsae/vehicle/drs_system.hpp`
  - `DRSConfig`, `DRSSystem`
- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `Vehicle14DOF_New`, `DriverInput`
- `fsae/vehicle/vehicle_params.hpp`
  - `VehicleParams`
- `fsae/physics/aero/aero_map_4d.hpp`
  - `AeroMap4D`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - DRS ボタン状態、速度、前進速度 `vx`、ヨーレート、ステア角、時間刻み
  - 統合テストでは車両状態・ドライバ入力
- 出力
  - DRS の active / inactive 状態
  - 統計情報 (`activation_count`, `total_active_time`, `avg_speed_when_active`)
  - 空力荷重比、車両シミュレーションでの DRS 起動結果

## 5. 関数・メソッド・クラスの網羅的な説明

### `class DRSSystemTest : public ::testing::Test`

- `DRSSystem` 単体テスト用フィクスチャです。

#### `void SetUp() override`

- 起動最低速度、横加速度上限、最大操舵角、起動遅延を設定し、`DRSSystem` を生成します。

#### `bool update_safe(bool button, double dt = 0.01)`

- 安全条件を満たす代表値で `drs->update(...)` を呼ぶヘルパーです。
- テスト本文から条件ノイズを除き、状態遷移の確認に集中させます。

#### `bool update_slow(bool button, double dt = 0.01)`

- 速度条件だけを意図的に違反させた低速入力ヘルパーです。

#### メンバ

- `DRSConfig config`
- `std::unique_ptr<DRSSystem> drs`

### `TEST_F(DRSSystemTest, InitialState)`

- 初期状態が非アクティブで、統計値がすべて 0 であることを確認します。

### `TEST_F(DRSSystemTest, ActivationDelay)`

- ボタン押下後すぐには起動せず、設定遅延経過後に `ACTIVE` へ遷移することを検証します。

### `TEST_F(DRSSystemTest, SpeedThresholdCheck)`

- 低速では起動せず、十分な速度では遅延後に起動することを確認します。

### `TEST_F(DRSSystemTest, LateralAccelCheck)`

- 横加速度が高いコーナリング中は起動を禁止し、低横 G 条件なら起動可能であることを確認します。

### `TEST_F(DRSSystemTest, SteeringAngleCheck)`

- 操舵角が大きい間は起動不可、小さい間は起動可能という操舵安全条件を確認します。

### `TEST_F(DRSSystemTest, InstantDeactivation)`

- ボタン解除時、または安全条件違反発生時に DRS が即座に停止することを確認します。

### `TEST_F(DRSSystemTest, Statistics)`

- 起動回数、累積有効時間、平均速度が期待通りに更新されることを確認します。

### `class DRSIntegrationTest : public ::testing::Test`

- 14DOF 車両へ DRS が統合された状態を確認するフィクスチャです。

#### `void SetUp() override`

- 既定 `VehicleParams` で `Vehicle14DOF_New` を生成し、初速を DRS 起動可能域に設定します。

#### メンバ

- `VehicleParams params`
- `std::unique_ptr<Vehicle14DOF_New> vehicle`

### `TEST_F(DRSIntegrationTest, AeroForceReduction)`

- `AeroMap4D` において DRS ON 時にダウンフォースとドラッグが規定倍率へ減少し、サイドフォースは不変であることを確認します。

### `TEST_F(DRSIntegrationTest, NoActivationBelowSpeedThreshold)`

- 実シミュレーション上でも低速車両では DRS ボタンを押しても有効化されないことを確認します。

### `TEST_F(DRSSystemTest, ResetClearsState)`

- `reset()` で active 状態と統計が初期化されることを確認します。
