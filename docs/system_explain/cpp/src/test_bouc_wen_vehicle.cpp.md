# `cpp/src/test_bouc_wen_vehicle.cpp`

## 1. ファイルの目的
Bouc-Wen ダンパを組み込んだ `Vehicle8DOF_BW` と、比較対象の線形ダンパ相当 `Vehicle8DOF_ARB` を同条件で走らせ、ヒステリシス状態変数とロール応答を確認する統合テストです。

## 2. 実行上の役割
- 非線形ダンパ導入後の動作確認に使います。
- ヒステリシスが本当に発生しているかを CSV とログで見られます。
- 線形比較モデルとの差分を可視化するデモでもあります。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof_bw.hpp`
- `fsae/vehicle/vehicle_8dof_arb.hpp`
- `Vehicle8DOF_BW`, `Vehicle8DOF_ARB`, `VehicleParams`, `DriverInput`
- `model_bw.get_bouc_wen_forces()`
- `spdlog`
- `<fstream>`
- `<cmath>` の `std::abs`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - Bouc-Wen 係数はファイル内で明示設定
  - ARB 剛性も設定
  - 初速 `15 m/s`
  - `throttle=0.1`
  - `t>=1.0 s` で `steering=0.15 rad`
  - `dt=0.001 s`
  - 総時間 `3.0 s`
- 出力:
  - `bouc_wen_comparison.csv`
  - `hysteresis_loop.csv`
  - 標準ログに最大ロール角、最終ヒステリシス変数、最終ダンパ力、判定結果
  - 失敗時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `VehicleParams` に Bouc-Wen と ARB の値を設定
  2. `Vehicle8DOF_BW model_bw` と `Vehicle8DOF_ARB model_linear` を生成
  3. 両モデルへ初速 `15 m/s` を設定
  4. 比較 CSV とヒステリシスループ CSV を作成
  5. 3 秒間積分して、`max_phi_bw`, `max_phi_linear` を更新
  6. 10 ステップごとに前左輪の Bouc-Wen 力と `damper_z[0]` を記録
  7. 最終ヒステリシス変数とダンパ力を表示
  8. 判定を実施

#### `main()` が確認するシナリオ
- 初速付き走行に 1 秒後のステップ操舵を重ねた試験です。
- 比較対象は線形ダンパモデルです。

#### `main()` の判定
- `0.5° < max_phi_bw < 10.0°`
- 4 輪 `|damper_z|` の総和 `z_sum > 1e-6`
- これにより、ヒステリシスが有効に働いているかを見ます。

#### CSV の役割
- `bouc_wen_comparison.csv`:
  - Bouc-Wen と線形のロール応答差を比較
- `hysteresis_loop.csv`:
  - 前左輪のサスペンション速度近似とダンパ力の関係を後から描画可能

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の Bouc-Wen vs 線形比較試験 1 本
