# `cpp/tests/test_bicycle_model.cpp`

## 1. ファイルの目的

6-DOF 自転車モデル `BicycleModel6DOF` の運動予測、線形化、定常旋回解、低速安定性、14DOF パラメータからの簡約化を検証するユニットテスト群です。

## 2. 実行上の役割

- MPC や経路追従で使う簡約車両モデルの基礎妥当性を確認します。
- 解析解と数値積分の整合、符号規約、RK4 積分精度をテストします。
- `VehicleParams` から `BicycleParams` を抽出する変換の後方互換性も担保します。

## 3. 依存関係

- `fsae/vehicle/bicycle_model.hpp`
  - `BicycleParams`
  - `BicycleState6DOF`
  - `BicycleControl`
  - `BicycleModel6DOF`
- `VehicleParams`（`BicycleParams::from_vehicle_params` 内で利用）
- `Eigen` ベースの状態ベクトル変換
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - 各テスト内で構築する車両パラメータ、初期状態、舵角、加速度、時間刻み
- 出力
  - `predict_step`, `linearize`, `compute_derivative`, `steady_state_cornering`, `from_vehicle_params` の戻り値
  - GoogleTest の成功 / 失敗結果

## 5. 関数・メソッド・クラスの網羅的な説明

### `class BicycleModelTest : public ::testing::Test`

- `BicycleModel6DOF` の共通セットアップを提供するフィクスチャです。

#### `void SetUp() override`

- 標準的な FSAE 値として質量、ヨー慣性、前後輪距離、前後コーナリング剛性を設定します。
- `model_` に `BicycleModel6DOF` を生成し、各テストで再利用可能にします。

#### メンバ

- `BicycleParams params_`
  - テスト対象モデルへ与えるパラメータ集合です。
- `std::unique_ptr<BicycleModel6DOF> model_`
  - 各テストが使う自転車モデル実体です。

### `TEST_F(BicycleModelTest, StraightLineMotion)`

- ゼロ舵角・ゼロ加速度で直進させ、`x` だけが等速で進むことを確認します。
- `predict_step` の基本積分挙動を検証する最小テストです。

### `TEST_F(BicycleModelTest, SteadyStateCircle)`

- `steady_state_cornering` が返す解析解のヨーレートが `v / R` に近いことを確認します。
- さらに短時間の数値積分で、解析解と同じ向きにヨーレートが発達することを確かめます。

### `TEST_F(BicycleModelTest, LinearizationAccuracy)`

- 非線形予測点まわりで得た `A, B` 行列が、微小摂動に対して 5% 以内で実際の非線形応答を近似できるかを検証します。
- MPC 用の局所線形モデル品質を担保するテストです。

### `TEST_F(BicycleModelTest, Acceleration)`

- 正の縦加速度入力で `vx` が `v = v0 + a t` に沿って増加することを確認します。

### `TEST_F(BicycleModelTest, Braking)`

- 負の加速度入力で `vx` が減少することを確認します。
- 加減速入力の符号解釈が正しいかを見るテストです。

### `TEST_F(BicycleModelTest, RightTurn)`

- 正の操舵入力が Phase 7 規約では右旋回に対応し、SAE ヨー符号では負の `r` を生むことを確認します。

### `TEST_F(BicycleModelTest, LowSpeedStability)`

- `vx` が極小でも特異点やクラッシュ 없이積分できることを確認します。
- 低速安定化処理が有効であることを保証します。

### `TEST(BicycleParamsTest, FromVehicleParams)`

- `VehicleParams` から `BicycleParams` を抽出する変換を検証します。
- 質量・慣性・ホイールベース成分のコピーと、前後コーナリング剛性が妥当な範囲に入ることを確認します。

### `TEST_F(BicycleModelTest, YawMomentSign)`

- `compute_derivative` が返すヨー角加速度の符号が操舵入力と整合することを確認します。

### `TEST_F(BicycleModelTest, RK4Accuracy)`

- 大きい 1 ステップと小さい 10 ステップの積分結果を比較し、RK4 実装の精度が高いことを確認します。
