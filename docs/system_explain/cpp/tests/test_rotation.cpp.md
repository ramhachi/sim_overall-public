# `cpp/tests/test_rotation.cpp`

## 1. ファイルの目的

回転行列生成と、Euler 角から見た車体座標系重力ベクトル変換の基本挙動を確認する数学ユーティリティテストです。

## 2. 実行上の役割

- SAE J670e 座標系と Z-Y-X Euler 角規約に基づく姿勢変換の土台を検証します。
- 車両力学で頻用される重力変換の符号ミスを防ぎます。

## 3. 依存関係

- `fsae/math/rotation.hpp`
  - `EulerAngles`
  - `euler_to_rotation_matrix`
  - `gravity_in_body`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - Euler 角（ロール・ピッチ・ヨー）
  - 重力加速度
- 出力
  - 回転行列
  - 車体系での重力ベクトル

## 5. 関数・メソッド・クラスの網羅的な説明

### `TEST(RotationTest, IdentityRotation)`

- 角度 0 のとき、回転行列が単位行列になることを確認します。

### `TEST(RotationTest, YawRotation90)`

- 90 度ヨーで X 軸が Y 方向へ回ることを確認します。

### `TEST(RotationTest, GravityInBody)`

- 姿勢 0 では重力が車体系 Z 正方向へそのまま入ることを確認します。

### `TEST(RotationTest, PitchAffectsGravity)`

- 正のピッチで重力に X 成分が現れ、Z 成分は正のまま減少することを確認します。
