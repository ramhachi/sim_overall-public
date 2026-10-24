# `cpp/tests/test_mpc.cpp`

## 1. ファイルの目的

MPC 系で使うトラック距離クエリ、QP 定式化、sampling preview MPC ドライバー統合を検証するテストです。

## 2. 実行上の役割

- 参照軌道生成の前段である `TrackDistanceQuery` の正しさを確認します。
- `MPCProblem` が正しいサイズ・拘束構造の QP を構築することを確認します。
- MPC ドライバーが実際に制御入力を返し、予測評価時間も妥当であることを見ます。

## 3. 依存関係

- `fsae/track/track_model.hpp`
- `fsae/track/track_distance.hpp`
  - `TrackDistanceQuery`
- `fsae/vehicle/bicycle_model.hpp`
  - `BicycleModel6DOF`, `BicycleState6DOF`
- `fsae/control/mpc_problem.hpp`
  - `MPCConfig`, `MPCProblem`
- `fsae/driver/mpc_driver.hpp`
- `laptime::VelocityPoint`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - 円形トラック、速度プロファイル、初期状態、ホライズン長、サンプリング周期
- 出力
  - 最近接点情報 `s`, `n`, `distance`
  - 参照軌道列
  - MPC QP 構造体 (`P`, `q`, `A`, `l`, `u`, `n_vars`, `n_constraints`)
  - MPC ドライバーの制御入力と統計

## 5. 関数・メソッド・クラスの網羅的な説明

### `class TrackDistanceTest : public ::testing::Test`

- 円形トラックと距離クエリを共有初期化するフィクスチャです。

#### `void SetUp() override`

- 半径 30m の円形トラックを生成し、`TrackDistanceQuery` を 0.5m 刻みで構築します。

#### メンバ

- `std::unique_ptr<track::CurvatureBasedTrack> circle_track_`
- `std::unique_ptr<track::TrackDistanceQuery> query_`

### `TEST_F(TrackDistanceTest, NearestPoint_OnTrack)`

- トラック上の点 `(30, 0)` に対し、最近接距離ほぼ 0・`s` ほぼ 0 を確認します。

### `TEST_F(TrackDistanceTest, NearestPoint_OffTrack)`

- トラック外 5m の点に対して、距離が約 5m になることを確認します。

### `TEST_F(TrackDistanceTest, NearestPoint_WithHint)`

- `hint_s` を使った局所探索でも、近傍点へ正しく収束することを確認します。

### `TEST_F(TrackDistanceTest, ReferenceTrajectory_Consistency)`

- 一定速度プロファイルから作る参照軌道が、連続的に前進し、速度値も保たれることを確認します。

### `class MPCProblemTest : public ::testing::Test`

- 自転車モデルを使う MPC 定式化テスト用フィクスチャです。

#### `void SetUp() override`

- 代表的な自転車モデルパラメータで `BicycleModel6DOF` を生成します。

#### メンバ

- `std::unique_ptr<vehicle::BicycleModel6DOF> model_`

### `TEST_F(MPCProblemTest, QP_Dimensions)`

- ホライズン `N=10` のときに、変数数・制約数・行列サイズが期待通りであることを確認します。

### `TEST_F(MPCProblemTest, StraightLine_SmallSteering)`

- 直線参照でコスト行列の対角が非負であること、初期状態拘束が正しく埋め込まれていることを確認します。

### `class MPCDriverTest : public ::testing::Test`

- sampling preview MPC 経路追従ドライバー統合テスト用フィクスチャです。

#### `void SetUp() override`

- 円形トラック、一定速度プロファイル、短いホライズンの `MPCPathFollowingDriver` を生成します。

#### メンバ

- `std::unique_ptr<track::CurvatureBasedTrack> circle_track_`
- `std::vector<laptime::VelocityPoint> velocity_profile_`
- `std::unique_ptr<driver::MPCPathFollowingDriver> driver_`

### `TEST_F(MPCDriverTest, CircleTrack_ProducesControl)`

- 円形トラック上の車両状態に対し、操舵・スロットル・ブレーキが有効範囲内で返ることを確認します。

### `TEST_F(MPCDriverTest, SolveTime_Reasonable)`

- 複数回の予測評価後、平均評価時間が 50ms 未満であることを確認します。
