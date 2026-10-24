# `cpp/tests/test_nn_tire_integration.cpp`

## 1. ファイルの目的

テキスト / JSON 形式で読み込む NN タイヤモデルと、タイヤ熱積分器の統合挙動を検証するテスト群です。  
実ファイル読込、推論の決定性、後処理、エクスポート済みテストベクトル一致、熱モデルの加熱・冷却を扱います。

## 2. 実行上の役割

- NN タイヤ推論器 `NNTextTireModel` がリポジトリ内エクスポート資産と整合するか確認します。
- ランタイム JSON の後処理（ゼロスリップ補正、摩擦円キャップ、対称性）が有効かを検証します。
- 併せて `TireThermalIntegrator` の初期化・発熱・冷却も確認します。

## 3. 依存関係

- `fsae/physics/tire/nn_lookup_tire.hpp`
  - `NNTextTireModel`, `TireInput`, `TireForces`
- `fsae/physics/tire/tire_thermal_integrator.hpp`
  - `TireThermalIntegrator`, `TireThermalParams`, `TireSlipInfo`
- リポジトリ内の推論資産
  - `../../cpp_export`
  - `output/tire_models/.../manifest.json`
  - `output/tire_models/.../test_vectors.json`
- `filesystem`, `fstream`, `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - テキスト重みファイル、正規化パラメータ、JSON ランタイム記述、エクスポート済みテストベクトル
  - タイヤ入力 `SR`, `SA`, `Fz`, `IA`, `TSTC`
  - 熱モデルではタイヤ力、スリップ速度、車速、外気温
- 出力
  - NN 推論の `Fx`, `Fy`, `Mx`, `Mz`
  - モデル読み込み状態、層数
  - 熱モデル温度
  - GoogleTest の成否

## 5. 関数・メソッド・クラスの網羅的な説明

### `class NNTireModelTest : public ::testing::Test`

- テスト用ダミー重みファイルと正規化ファイルを生成・削除するフィクスチャです。

#### `void SetUp() override`

- `create_test_weights()` と `create_test_norm_params()` を呼び、ローカルテスト資産を作成します。

#### `void TearDown() override`

- `test_weights.txt` と `test_norm_params.txt` を削除します。

#### `void create_test_weights()`

- 5→10→4 の 2 層全結合ネットワーク用ダミー重みをテキスト形式で出力します。

#### `void create_test_norm_params()`

- 入出力の平均・標準偏差を持つ正規化パラメータファイルを出力します。

### `TEST_F(NNTireModelTest, LoadFromTextFiles)`

- 旧ダミーファイル経路用テスト名を残しつつ、現在は `GTEST_SKIP()` で実資産ベーステストへ役割を譲っています。

### `TEST_F(NNTireModelTest, PredictionBasic)`

- 同上。名称は基本推論確認ですが、実際にはスキップされます。

### `TEST_F(NNTireModelTest, PredictionDeterministic)`

- 同上。決定性確認名を残しつつスキップされます。

### `TEST(RealNNModel, LoadFromExport)`

- `../../cpp_export` からモデルを読み込み、ロード成功と全結合層数 9 を確認します。
- 資産が無い環境ではスキップします。

### `TEST(RealNNModel, InferenceBasic)`

- 典型入力に対し推論を実行し、有限値かつ物理的に大きすぎない力を返すことを確認します。

### `TEST(RealNNModel, InferenceDeterministic)`

- 同一入力を 2 回与えて完全一致することを確認し、推論の決定性を検証します。

### `TEST(RealNNModel, InferenceContinuity)`

- わずかな入力差に対する `Fx`, `Fy` の変化が急峻すぎないことを確認します。

### `TEST(RealNNModel, RuntimePostprocessAppliesZeroSlipCorrectionAndFrictionCap)`

- テスト用 `pinn_runtime.json` を書き出し、ゼロスリップ補正・摩擦円キャップ・左右対称性がランタイム後処理で実際に効くことを確認します。

### `TEST(RealNNModel, ExportedRuntimeMatchesTestVectors)`

- manifest からロードしたモデルが、同梱 `test_vectors.json` の期待出力と絶対 / 相対許容差内で一致することを確認します。

### `class TireThermalTest : public ::testing::Test`

- タイヤ熱積分器テスト用フィクスチャです。

#### `void SetUp() override`

- 既定 `TireThermalParams` を用意します。

#### メンバ

- `TireThermalParams params`

### `TEST_F(TireThermalTest, Initialization)`

- 4 輪すべてが初期温度 `T_initial` で開始することを確認します。

### `TEST_F(TireThermalTest, HeatingFromFriction)`

- 摩擦仕事を与えると温度が上昇することを確認します。

### `TEST_F(TireThermalTest, CoolingFromConvection)`

- 高温から無負荷状態で更新すると対流冷却により温度が低下することを確認します。

### `int main(int argc, char** argv)`

- このファイル専用の GoogleTest ランナーです。
- `InitGoogleTest` を行い、全テスト実行結果を返します。
