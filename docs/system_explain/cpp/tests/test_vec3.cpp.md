# `cpp/tests/test_vec3.cpp`

## 1. ファイルの目的

3 次元ベクトル型 `Vec3d` の基本演算を確認する数学基盤テストです。

## 2. 実行上の役割

- 物理・幾何演算で前提となるベクトル生成、加算、内積、外積、ノルム、正規化が正しく動くことを保証します。

## 3. 依存関係

- `fsae/math/vec3.hpp`
  - `Vec3d`
  - `Vec3d::unit_x`, `Vec3d::unit_y`
  - 演算子 `+`
  - `dot`, `cross`, `norm`, `normalized`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - 定数ベクトル成分
- 出力
  - 生成済みベクトルの各成分
  - 演算結果の成分およびスカラー値

## 5. 関数・メソッド・クラスの網羅的な説明

### `TEST(Vec3Test, DefaultConstructor)`

- デフォルト構築で `(0, 0, 0)` になることを確認します。

### `TEST(Vec3Test, ParameterizedConstructor)`

- 引数付き構築で与えた `x, y, z` がそのまま格納されることを確認します。

### `TEST(Vec3Test, Addition)`

- ベクトル加算演算子が成分ごとの和を返すことを確認します。

### `TEST(Vec3Test, DotProduct)`

- 直交ベクトルの内積が 0、自己内積が長さ二乗になることを確認します。

### `TEST(Vec3Test, CrossProduct)`

- 単位 X と単位 Y の外積が単位 Z になることを確認します。

### `TEST(Vec3Test, Norm)`

- `(3,4,0)` のノルムが 5 になることを確認します。

### `TEST(Vec3Test, Normalize)`

- 正規化後ベクトルのノルムが 1 で、成分比が 0.6 / 0.8 になることを確認します。
