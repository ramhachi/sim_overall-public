# `cpp/tests/test_pou.cpp`

## 1. ファイルの目的

POU（Pushrod on Upright）機構によるジャッキアップ変位と操舵誘発キャンバー、およびそれらを組み込んだ 14DOF 車両挙動を検証するテストです。

## 2. 実行上の役割

- 低レベルの幾何計算と、車両シミュレーションに組み込んだ場合の統合動作の両方を確認します。
- POU 係数 0 のときに旧挙動へ戻る後方互換性も保証します。

## 3. 依存関係

- `fsae/vehicle/vehicle_14dof_new.hpp`
  - `compute_pou_displacement`
  - `compute_camber_angle`
  - `get_pou_displacement`
  - `get_pou_camber`
  - `get_tire_Fz`
- `fsae/vehicle/vehicle_params.hpp`
- `fsae/vehicle/vehicle_state.hpp`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - POU ジャッキアップ係数、POU キャンバー係数、舵角、サスペンショントラベル、ドライバ入力
- 出力
  - POU 変位
  - キャンバー角 / POU キャンバー成分
  - タイヤ垂直荷重
  - 車両状態

## 5. 関数・メソッド・クラスの網羅的な説明

### `class POUKinematicsTest : public ::testing::Test`

- POU の局所幾何計算を検証するフィクスチャです。

#### `void SetUp() override`

- デフォルト `VehicleParams` を用意します。

#### メンバ

- `VehicleParams params`

### `TEST_F(POUKinematicsTest, ZeroSteerNoPOUEffect)`

- ゼロ舵角で POU 変位が 0、キャンバーも静的値から変化しないことを確認します。

### `TEST_F(POUKinematicsTest, JackUpDisplacement)`

- 1 次・2 次係数から計算されるジャッキアップ変位が理論式通りになることを確認します。

### `TEST_F(POUKinematicsTest, SteeringInducedCamber)`

- 静的キャンバーに POU 誘発キャンバーが加算されることを確認します。

### `TEST_F(POUKinematicsTest, BackwardCompatibility)`

- POU 係数 0 のとき、変位は 0 でキャンバー計算も従来式に一致することを確認します。

### `class POUIntegrationTest : public ::testing::Test`

- POU を含む 14DOF 車両統合テスト用フィクスチャです。

#### `void SetUp() override`

- デフォルト `VehicleParams` を用意します。

#### メンバ

- `VehicleParams params`

### `TEST_F(POUIntegrationTest, StaticJackUpTest)`

- 停車据え切り時にフロント輪へ期待量の POU 変位が現れ、リアは 0 のままであることを確認します。

### `TEST_F(POUIntegrationTest, SkidpadComparison)`

- 定常旋回で POU 変位と POU キャンバーが実際に現れ、シミュレーションが NaN / Inf なく継続することを確認します。

### `TEST_F(POUIntegrationTest, NumericalStability)`

- 極端な POU 係数でも 2 秒シミュレーションが破綻せず、状態量と垂直荷重が有限・正値を保つことを確認します。
