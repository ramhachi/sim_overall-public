# `cpp/src/test_aero_integration.cpp`

## 1. ファイルの目的
4D 空力マップ `AeroMap4D` を、速度・ライドハイト・ロール・ヨーの代表条件で評価し、ダウンフォース、ドラッグ、横力の変化傾向を確認する統合デモです。

## 2. 実行上の役割
- 空力モデル単体の出力傾向を短時間で確認するスタンドアロン実行ファイルです。
- 代表シナリオを順に流し、結果をログと CSV に残します。
- 14DOF 本体へ統合する前後の基礎確認に向いています。

## 3. 依存関係
- `fsae/physics/aero/aero_map_4d.hpp`
- `AeroMap4D<double>`
- `aero_map.compute_forces()`
- `spdlog`
- `<fstream>`
- `<cmath>` の `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - `AeroMap4D<double> aero_map` をデフォルト構成で生成
  - Test 1: 速度スイープ `5`〜`30 m/s`
  - Test 2: フロントライドハイトスイープ `25`〜`55 mm`
  - Test 3: ロールスイープ `0`〜`5 deg`
  - Test 4: ヨースイープ `0`〜`15 deg`
  - Test 5: 複合条件 `25 m/s, 30/50 mm, roll 3 deg, yaw 8 deg`
- 出力:
  - `aero_integration_test.csv`
  - 標準ログに各シナリオ結果
  - 明示的失敗判定はなく、正常終了のみ

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. デフォルト `AeroMap4D<double>` を生成
  2. `aero_integration_test.csv` を開き速度スイープ結果を書き込み
  3. 5 つの試験シナリオを順に実行
  4. 最後に CSV 保存完了と成功メッセージを表示

#### `main()` 内 Test 1: Speed Sweep
- 条件: `40/55 mm`, `roll=0`, `yaw=0`
- `v=5` から `30 m/s` を 5 m/s 刻みで `compute_forces()` 評価
- `Fz`, `Fx`, `Fy`, 前後空力荷重を CSV に保存
- 確認内容:
  - 速度増加で空力荷重がどう増えるか

#### `main()` 内 Test 2: Ride Height Effect
- 条件: `V=20 m/s`, `rear=55 mm`
- フロント車高のみ変え、ベースライン比のダウンフォース変化率を表示
- 確認内容:
  - グラウンドエフェクト感度

#### `main()` 内 Test 3: Roll Effect
- 条件: `V=20 m/s`, `40/55 mm`
- ロール角を `0`〜`5 deg` で変化
- 確認内容:
  - ロールによるダウンフォース損失率

#### `main()` 内 Test 4: Yaw Effect
- 条件: `V=20 m/s`, `40/55 mm`
- ヨー角を `0`〜`15 deg` で変化
- 確認内容:
  - ドラッグ増加
  - 横力発生

#### `main()` 内 Test 5: Combined Effect
- ベースライン条件と、低車高・ロール・ヨーを含むレース条件を比較
- `ΔFz`, `ΔFx` と比率を表示
- 確認内容:
  - 複合姿勢条件での空力差分

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - Speed Sweep
  - Ride Height Effect
  - Roll Effect
  - Yaw Effect
  - Combined Effect
