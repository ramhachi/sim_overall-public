# `cpp/tests/test_bouc_wen.cpp`

## 1. ファイルの目的

サスペンション減衰で使う Bouc-Wen ヒステリシスモデルの力計算と内部履歴変数更新式を検証する単体テストです。

## 2. 実行上の役割

- Bouc-Wen モデルがゼロ入力、低速入力、履歴蓄積、速度反転で期待どおりの挙動を示すか確認します。
- ヒステリシスを持つ減衰モデルがサスペンション系に導入されても基本数値性質を満たすことを保証します。

## 3. 依存関係

- `fsae/physics/suspension/bouc_wen.hpp`
  - `BoucWenParams`
  - `compute_bouc_wen_force`
  - `compute_bouc_wen_dz`
  - `compute_bouc_wen`
- `gtest/gtest.h`
- `cmath`

## 4. 入力 / 出力

- 入力
  - Bouc-Wen パラメータ、変位 `x`、速度 `v`、履歴変数 `z`
- 出力
  - 減衰力、履歴変数時間微分、複合出力構造体
  - テスト成否

## 5. 関数・メソッド・クラスの網羅的な説明

### `class BoucWenTest : public ::testing::Test`

- Bouc-Wen モデルの標準パラメータをまとめて初期化するフィクスチャです。

#### `void SetUp() override`

- 粘性減衰係数 `c0`、履歴力係数 `alpha`、進化係数 `A`、`beta`、`gamma`、指数 `n` を設定します。
- 線形ばね項 `k0` は 0 とし、ヒステリシス寄与を見やすくしています。

#### メンバ

- `BoucWenParams<double> params`
  - 各テストで共有する Bouc-Wen パラメータです。

### `TEST_F(BoucWenTest, ZeroVelocityZeroForce)`

- 変位・速度・履歴がすべて 0 のとき、出力力が 0 になることを確認します。

### `TEST_F(BoucWenTest, LinearDampingAtLowVelocity)`

- 低速かつ履歴 0 の条件では、力がほぼ `c0 * v` の粘性減衰項で支配されることを確認します。

### `TEST_F(BoucWenTest, HysteresisVariableEvolution)`

- 正速度で `z` が増加すること、`z` が大きいほど増加率が抑えられることを検証します。
- `compute_bouc_wen_dz` 単体の性質確認です。

### `TEST_F(BoucWenTest, VelocityReversalHysteresis)`

- 履歴変数が蓄積された状態で速度を正負反転させ、力の絶対値が異なることを確認します。
- Bouc-Wen モデルが単なる線形減衰ではなく履歴依存性を持つことを示す代表テストです。
