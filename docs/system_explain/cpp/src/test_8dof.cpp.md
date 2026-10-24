# `cpp/src/test_8dof.cpp`

## 1. ファイルの目的
`Vehicle8DOF` の基本挙動を、静止平衡・縦方向走行・ステアリング旋回の 3 条件で確認する総合スモークテストです。

## 2. 実行上の役割
- 8DOF 車両モデル変更時の初期確認用実行ファイルです。
- 上下動、ピッチ、ロール、ヨーの基本安定性を短時間で見ます。
- ログ主体で NaN や過大姿勢角を検出する簡易回帰テストです。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle8DOF`, `DriverInput`
- `vehicle.z_equilibrium()`, `vehicle.set_initial_velocity()`, `vehicle.step()`, `vehicle.reset()`, `vehicle.time()`
- `spdlog`
- `<cmath>` の `std::isnan`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - `dt=0.001 s`
  - Test 1: 静止平衡、100 ステップ
  - Test 2: 初速 15 m/s、`throttle=0.3`、1000 ステップ
  - Test 3: 初速 15 m/s、`throttle=0.3`、1 秒後から `steering=0.1 rad`、3000 ステップ
- 出力:
  - 標準ログに平衡値、途中状態、テスト結果
  - いずれかで発散したら `return 1`
  - 出力ファイルなし

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- まず `z_equilibrium()` と前左輪の `z_u[0]` を表示し、上下系の初期値を確認します。

#### `main()` 内 Test 1: Static Equilibrium
- `vehicle.reset()` 後、ゼロ入力で 100 ステップ積分します。
- 20 ステップごとに `z`, `z_u[0]`, `vz` を表示します。
- 発散条件:
  - `std::isnan(s.z)`
  - `|s.z| > 1`
  - `|s.vz| > 10`
- 確認内容:
  - 静止状態の上下系が破綻しないこと

#### `main()` 内 Test 2: Velocity
- `reset()` 後に初速 15 m/s を与え、`throttle=0.3` で 1 秒積分します。
- 200 ステップごとに `vx`, `z`, `theta` を表示します。
- 発散条件:
  - `std::isnan(s.vx)`
  - `|s.z| > 1`
  - `|s.theta| > 0.5`
- 確認内容:
  - 加速時に縦方向とピッチ系が安定なこと

#### `main()` 内 Test 3: Steering
- 再初期化後、初速 15 m/s を与えます。
- `vehicle.time() >= 1.0` になったら `steering=0.1 rad` を印加します。
- 500 ステップごとに `vx`, `vy`, `r`, `phi` を表示します。
- 発散条件:
  - `std::isnan(s.vx)`
  - `|s.z| > 1`
  - `|s.phi| > 0.5`
- 確認内容:
  - 横運動とロール応答がステップ操舵で破綻しないこと

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - Static Equilibrium
  - Velocity
  - Steering
