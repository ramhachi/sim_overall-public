# `cpp/src/test_arb_debug.cpp`

## 1. ファイルの目的
ARB 付き 8DOF モデルの内部挙動を詳細ログで観察するデバッグ用実行ファイルです。操舵入力直後のロール角、ロールレート、ARB トルクを重点的に確認します。

## 2. 実行上の役割
- `test_arb.cpp` より詳しい観察用です。
- ARB 効果が弱い、符号がおかしい、といったときの切り分けに使います。
- 過渡時の ARB トルク立ち上がり確認に向いています。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof.hpp`
- `fsae/vehicle/vehicle_8dof_arb.hpp`
- `VehicleParams::default_fsae()`
- `Vehicle8DOF`, `Vehicle8DOF_ARB`, `DriverInput`
- `vehicle_with_arb.get_arb_forces()`
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
  - 総時間 `2.5 s`
- 出力:
  - 標準ログにパラメータ、操舵直後 11 サンプルの詳細、定期サマリ、最終低減率
  - 外部ファイルなし
  - NaN 発生時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. ログレベルを `debug` に設定
  2. ARB 剛性と車両諸元を表示
  3. `Vehicle8DOF` と `Vehicle8DOF_ARB` を生成
  4. 両者に初速 15 m/s を設定
  5. 2500 ステップ積分
  6. `step 1000`〜`1010` では毎ステップ詳細出力
  7. 500 ステップごとにサマリ表示
  8. 最終ロール角と低減率を計算

#### `main()` 内の重点観測区間
- `t=1.000`〜`1.010 s`
- 表示項目:
  - no-ARB / with-ARB の `phi`
  - no-ARB / with-ARB の `p`
  - `get_arb_forces()` による前後 ARB トルク

#### 何を検証するか
- 操舵開始直後に ARB トルクが立ち上がるか
- with-ARB のロール角・ロールレートが基準より抑えられるか
- 数値発散が起きないか

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の ARB 過渡デバッグ試験 1 本
