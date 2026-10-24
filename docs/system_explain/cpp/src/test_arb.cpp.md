# `cpp/src/test_arb.cpp`

## 1. ファイルの目的
アンチロールバー（ARB）を持たない基準 8DOF モデルと、ARB を組み込んだ `Vehicle8DOF_ARB` を比較し、ステップ操舵時のロール角低減効果を確認するテストです。

## 2. 実行上の役割
- ARB モデル導入後の回帰確認に使う比較実行ファイルです。
- 「ロール角が十分減るか」を pass/fail 条件として持ちます。
- 横運動での ARB 効果を定量確認する入口です。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof.hpp`
- `fsae/vehicle/vehicle_8dof_arb.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle8DOF`, `Vehicle8DOF_ARB`, `DriverInput`
- `vehicle.step()`, `vehicle.state()`, `vehicle.set_initial_velocity()`
- `spdlog`
- `<cmath>` の `std::isnan`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - `k_arb_f=5000 Nm/rad`
  - `k_arb_r=3000 Nm/rad`
  - 初速 `15 m/s`
  - `throttle=0.3`
  - `t>=1.0 s` で `steering=0.15 rad`
  - `dt=0.001 s`
  - 総時間 `3.0 s`
- 出力:
  - 標準ログに途中ロール角、低減率、最終最大ロール角
  - 外部ファイルなし
  - NaN 発生またはロール低減不足で `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. 標準車両パラメータへ ARB 剛性を設定
  2. 基準モデル `Vehicle8DOF vehicle_no_arb(params)` を生成
  3. 比較用に `params_no_arb` を作るが、これは実際の生成には使っていない
  4. `Vehicle8DOF_ARB vehicle_with_arb(params)` を生成
  5. 両車両へ初速 `15 m/s` を与える
  6. 3 秒間、同一入力で両モデルを積分
  7. 各ステップで `max_roll_no_arb`, `max_roll_with_arb` を更新
  8. 500 ステップごとにロール角と低減率を表示
  9. 最終最大ロール角から合格判定

#### `main()` が確認するシナリオ
- `t<1.0 s` は直進
- `t>=1.0 s` で `0.15 rad` のステップ舵角
- ロール低減率の観測が目的です

#### `main()` の判定
- `max_roll_with_arb < max_roll_no_arb * 0.95`
- コメントでは「少なくとも10%低減」とありますが、実装上の閾値は 5% 低減です。

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の ARB 比較試験 1 本
