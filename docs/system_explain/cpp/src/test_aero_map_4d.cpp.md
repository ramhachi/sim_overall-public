# `cpp/src/test_aero_map_4d.cpp`

## 1. ファイルの目的
`AeroMap4D` の補間グリッド、基準係数、力換算、グラウンドエフェクト傾向をまとめて確認する空力モデル単体テストです。

## 2. 実行上の役割
- 空力マップ生成ロジックの回帰確認に使います。
- `CL/CD/CY` や `Fz/Fx` のレンジ、符号、車高依存性をまとめて確認できます。
- 小規模グリッドを CSV 化するサンプルとしても機能します。

## 3. 依存関係
- `fsae/physics/aero/aero_map_4d.hpp`
- `AeroMapParams<double>`, `AeroMap4D<double>`
- `aero_map.grid()`, `aero_map.compute()`, `aero_map.compute_forces()`
- `spdlog`
- `<fstream>`
- `<cmath>` の `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 空力基準値はファイル内で手動設定
    - `rho=1.225`
    - `A_ref=1.0`
    - `L_ref=1.6`
    - `x_cg=0.8`
    - `CL_default=-2.5`
    - `CD_default=1.0`
  - Baseline: `rh_f=40`, `rh_r=55`, `roll=0`, `yaw=0`, `V=20 m/s`
  - Ride height / roll / yaw / speed の各スイープ
- 出力:
  - `aero_map_data.csv`
  - 標準ログにグリッド情報、各シナリオ結果、検証結果
  - 失敗時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `AeroMapParams<double>` を設定
  2. `AeroMap4D<double> aero_map(params)` を生成
  3. `grid()` から RH / roll / yaw の格子点数と範囲を表示
  4. Baseline 条件で係数と力を計算
  5. Ride height, roll, yaw, speed の各スイープを実施
  6. 小さな 4D グリッドを `aero_map_data.csv` に保存
  7. 妥当性判定を行う

#### `main()` 内 Test 1: Baseline Condition
- `compute()` で `CL, CD, CY, CoP_x, CoP_dist`
- `compute_forces()` で `Fz, Fx, Fy, Fz_front, Fz_rear`
- 確認内容:
  - 基準点での空力係数と力の大きさ

#### `main()` 内 Test 2: Ride Height Effect
- フロントライドハイト `25, 35, 45, 55 mm`
- 確認内容:
  - 低車高で `CL` と `Fz` がどう変化するか

#### `main()` 内 Test 3: Roll Effect
- `0, 2.5, 5 deg` に相当するロールで `CL` の変化を表示
- 確認内容:
  - ロールに伴う空力性能低下

#### `main()` 内 Test 4: Yaw Effect
- `0, 5, 10, 15 deg` のヨー角で `CL/CD/CY` を比較
- 確認内容:
  - 偏流時の抗力・横力変化

#### `main()` 内 Test 5: Velocity Sweep
- `10, 15, 20, 25, 30 m/s`
- 確認内容:
  - 速度に対する空力荷重の増加傾向

#### `main()` 内 Verification
- `CL` が `-4.0 < CL < -1.0`
- `CD` が `0.5 < CD < 2.0`
- `Fz < 0`
- `Fx < 0`
- 低ライドハイトの方が `CL` がより負になる
- これらのいずれかで外れると失敗です。

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - Baseline Condition
  - Ride Height Effect
  - Roll Effect
  - Yaw Effect
  - Velocity Sweep
  - Verification
