# `cpp/tests/test_spline.cpp`

## 1. ファイルの目的

`CubicSpline` の補間、微分、範囲外入力処理を確認する数学ユーティリティテストです。

## 2. 実行上の役割

- ルックアップやトラック / タイヤ特性補間で使うスプライン基盤の健全性を保証します。
- ノット一致・中間補間・微分値・クランプ挙動を押さえています。

## 3. 依存関係

- `fsae/math/spline.hpp`
  - `CubicSpline`
- `vector`, `cmath`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - ノット列 `x = [0,1,2,3,4]`
  - 値列 `y = x^2`
- 出力
  - 補間値
  - 一階微分値
  - 範囲外入力時のクランプ済み値

## 5. 関数・メソッド・クラスの網羅的な説明

### `class SplineTest : public ::testing::Test`

- 2 次関数 `y = x^2` のサンプル点を共有するフィクスチャです。

#### メンバ

- `std::vector<double> x`
- `std::vector<double> y`

### `TEST_F(SplineTest, InterpolatesAtKnots)`

- ノット位置では補間値が元データと厳密一致することを確認します。

### `TEST_F(SplineTest, InterpolatesBetweenKnots)`

- ノット間で妥当な中間値を返し、代表点 1.5 付近で 2 次関数近傍になることを確認します。

### `TEST_F(SplineTest, DerivativeIsReasonable)`

- `x=2` での微分値が理論値 `2x=4` 近傍になることを確認します。

### `TEST_F(SplineTest, ClampsOutOfRange)`

- 定義域外の入力でも有限値を返し、補間器が安全にクランプ動作することを確認します。
