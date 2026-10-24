# `cpp/src/test_6dof_equilibrium.cpp`

## 1. ファイルの目的
`Vehicle6DOF` を静止・ゼロ入力の平衡付近で動かし、車高 `z`、鉛直速度 `vz`、ピッチ角 `theta` が短時間で破綻しないかを確認するデバッグ用実行ファイルです。

## 2. 実行上の役割
- 6DOF モデルの初期平衡設定確認に使います。
- ゼロ入力時に不自然な沈み込みや姿勢発散がないかを見るスモークテストです。
- 単体テストフレームワークではなく、ログを読んで健全性を判断する用途です。

## 3. 依存関係
- `fsae/vehicle/vehicle_6dof.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle6DOF`, `DriverInput`
- `vehicle.state()`, `vehicle.step()`, `vehicle.time()`, `vehicle.z_equilibrium()`
- `spdlog`
- `<cmath>` の `std::isnan`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 車両諸元は `VehicleParams::default_fsae()`
  - 初期 `vx=0`
  - 全輪 `omega[i]=0`
  - `dt=0.001 s`
  - 入力は `DriverInput{}` のゼロ入力
- 出力:
  - 標準ログに初期状態、途中状態、最終状態
  - 発散条件検知時は `error` ログ
  - 出力ファイルなし
- 備考:
  - 発散時は `break` でループを抜けるだけで、終了コードは `0` のままです。

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `Vehicle6DOF` を標準パラメータで生成
  2. `state().vx` と 4 輪 `state().omega` を 0 に設定
  3. `z` と `z_equilibrium()` の差を表示
  4. `dt=0.001`、ゼロ入力で 100 ステップ積分
  5. 先頭 10 ステップと 10 ステップごとに `z`, `vz`, `theta`, `q` を表示
  6. `std::isnan(s.z)`、`|s.z|>1`、`|s.theta|>0.5` を発散条件として監視
  7. 最終状態を表示して終了
- このシナリオが確認すること:
  - 静止初期化後に上下・ピッチ自由度が暴走しないこと
  - 平衡付近で数値積分が NaN を出さないこと
  - `z_equilibrium()` 周辺での落ち着き具合

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の「静止平衡観察」1 本
