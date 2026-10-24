# `cpp/src/test_lugre_vehicle.cpp`

## 1. ファイルの目的
Bouc-Wen に LuGre 摩擦を加えた `Vehicle8DOF_Full` と、Bouc-Wen のみの `Vehicle8DOF_BW` を比較し、摩擦力と毛変形状態が有効に働いているかを確認する統合テストです。

## 2. 実行上の役割
- サスペンション摩擦モデル導入時の回帰確認に使います。
- ロール応答だけでなく、LuGre の内部状態 `friction_z` と摩擦力が非ゼロになるかを見ます。
- CSV により比較解析しやすい形で結果を残します。

## 3. 依存関係
- `fsae/vehicle/vehicle_8dof_full.hpp`
- `fsae/vehicle/vehicle_8dof_bw.hpp`
- `Vehicle8DOF_Full`, `Vehicle8DOF_BW`, `VehicleParams`, `DriverInput`
- `model_full.get_lugre_forces()`
- `model_full.get_bouc_wen_forces()`
- `spdlog`
- `<fstream>`
- `<cmath>` の `std::abs`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - Bouc-Wen / LuGre / ARB の係数をファイル内で明示設定
  - 初速 `15 m/s`
  - `throttle=0.1`
  - `t>=1.0 s` で `steering=0.15 rad`
  - `dt=0.001 s`
  - 総時間 `3.0 s`
- 出力:
  - `lugre_comparison.csv`
  - 標準ログに最大ロール角、最大摩擦力、最終 `friction_z`、最終 LuGre 力、判定結果
  - 失敗時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `VehicleParams` に Bouc-Wen, LuGre, ARB の値を設定
  2. `Vehicle8DOF_Full model_full` と `Vehicle8DOF_BW model_bw` を生成
  3. 両モデルへ初速 `15 m/s` を設定
  4. `lugre_comparison.csv` を作成
  5. 3 秒間積分して `max_phi_full`, `max_phi_bw`, `max_friction_force` を更新
  6. 10 ステップごとに前左輪 LuGre 力、`friction_z[0]`、Bouc-Wen 力を CSV 保存
  7. 最終 LuGre 状態と力を表示
  8. 妥当性判定を実施

#### `main()` の判定
- `0.5° < max_phi_full < 10.0°`
- `max_friction_force > 1.0 N`
- 4 輪 `|friction_z|` の総和 `z_sum > 1e-9`

#### `main()` が確認すること
- LuGre 摩擦要素が実際に動いているか
- 摩擦追加後も車両応答が極端でないか
- Bouc-Wen のみモデルとの差分が観測できるか

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の BW+LuGre vs BW 比較試験 1 本
