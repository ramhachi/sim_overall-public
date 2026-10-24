# `cpp/tests/test_tmeasy.cpp`

## 1. ファイルの目的

TMeasy タイヤモデルのゼロスリップ、横スリップ、ゼロ速度安定化、荷重依存性、バッチ計算を検証するテストです。

## 2. 実行上の役割

- PINN / Magic Formula 以外の代替タイヤモデルとして、TMeasy 実装の基礎妥当性を保証します。
- 特に低速特異点回避の実装確認として重要です。

## 3. 依存関係

- `fsae/physics/tire/tmeasy.hpp`
  - `TMeasyParams`
  - `TMeasyState`
  - `compute_tmeasy`
  - `compute_tmeasy_batch`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - TMeasy パラメータ、内部変形状態、垂直荷重、縦横スリップ速度、接地点速度
- 出力
  - `Fx`, `Fy`, `dxi_dt`, `deta_dt`
  - バッチ計算配列出力

## 5. 関数・メソッド・クラスの網羅的な説明

### `class TMeasyTest : public ::testing::Test`

- TMeasy の代表パラメータを設定するフィクスチャです。

#### `void SetUp() override`

- 縦横最大摩擦係数、公称荷重、緩和長、低速安定化用仮想速度 `v_N` を設定します。

#### メンバ

- `TMeasyParams<double> params`

### `TEST_F(TMeasyTest, ZeroSlipProducesZeroForce)`

- スリップも変形もない場合に `Fx`, `Fy` が 0 になることを確認します。

### `TEST_F(TMeasyTest, LateralSlipProducesLateralForce)`

- 横変形と横スリップで横力が発生し、縦力は小さいままであることを確認します。

### `TEST_F(TMeasyTest, ZeroSpeedDoesNotDiverge)`

- `vcx = 0` の極限でも計算が例外なく、全出力が有限値であることを確認します。

### `TEST_F(TMeasyTest, LoadSensitivity)`

- 高荷重ほど横力絶対値は増える一方、正規化摩擦係数は低荷重側が高いことを確認します。

### `TEST_F(TMeasyTest, BatchComputation)`

- 4 本分の配列バッチ計算がすべて有限値を返すことを確認します。
