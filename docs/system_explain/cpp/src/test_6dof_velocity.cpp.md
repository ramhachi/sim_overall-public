# `cpp/src/test_6dof_velocity.cpp`

## 1. ファイルの目的
`Vehicle6DOF` を速度付き状態で動かし、惰行と軽い加速の 2 条件で安定に積分できるかを確認するデバッグ用実行ファイルです。

## 2. 実行上の役割
- 6DOF モデルに初速を与えたときの基本健全性チェックです。
- ゼロスロットル時の慣性走行安定性と、`throttle=0.3` 入力時の前進応答を簡易確認します。
- 姿勢自由度が速度入力で発散しないかを見る用途です。

## 3. 依存関係
- `fsae/vehicle/vehicle_6dof.hpp`
- `fsae/math/rotation.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle6DOF`, `DriverInput`
- `vehicle.set_initial_velocity()`, `vehicle.step()`, `vehicle.reset()`, `vehicle.time()`
- `spdlog`
- `<cmath>` の `std::isnan`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 初速 `15.0 m/s`
  - `dt=0.001 s`
  - Test 1: `throttle=0`, `steering=0`, 1000 ステップ
  - Test 2: `throttle=0.3`, `steering=0`, 100 ステップ
- 出力:
  - 標準ログに初期状態、途中状態、各テスト結果
  - 発散時は `return 1`
  - 出力ファイルなし

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 共通初期化:
  1. `Vehicle6DOF` を生成
  2. `set_initial_velocity(15.0)` を実行
  3. 初期 `vx` と `omega[0]` を表示

#### `main()` 内 Test 1: Coasting
- ゼロ入力で 1 秒間積分します。
- 先頭 10 ステップと 100 ステップごとに `vx`, `z`, `theta` を表示します。
- 発散条件:
  - `std::isnan(s.vx)`
  - `|s.z| > 1`
  - `|s.theta| > 0.5`
- 確認内容:
  - 惰行中に車速と姿勢が不自然に暴れないこと
  - 速度付き初期状態でも数値計算が安定なこと

#### `main()` 内 Test 2: With throttle
- `vehicle.reset()` 後に再度 `set_initial_velocity(15.0)` を実行します。
- `throttle=0.3`, `steering=0` で 100 ステップ積分します。
- 先頭 10 ステップと 20 ステップごとに `vx`, `z`, `theta` を表示します。
- 発散条件は Test 1 と同じです。
- 確認内容:
  - 軽い加速入力で前進挙動が壊れないこと
  - 鉛直・ピッチ自由度が速度変化とともに発散しないこと

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - Coasting
  - With throttle
