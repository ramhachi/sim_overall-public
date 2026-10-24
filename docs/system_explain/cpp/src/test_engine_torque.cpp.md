# `cpp/src/test_engine_torque.cpp`

## 1. ファイルの目的
CBR600RR ベースのエンジンモデルとギア比モデルが返すトルク、出力、速度換算値を表形式で確認するパワートレイン単体デモです。

## 2. 実行上の役割
- エンジントルクカーブやギア比定義の目視確認に使います。
- 数値アサーションではなく、仕様説明と手計算照合向けです。
- パワートレイン API の使い方例も兼ねています。

## 3. 依存関係
- `fsae/physics/powertrain/engine.hpp`
- `create_cbr600rr_engine()`
- `GearRatios::cbr600rr_default()`
- `engine.get_idle_rpm()`, `get_peak_torque()`, `get_peak_torque_rpm()`, `get_redline_rpm()`
- `engine.get_torque()`
- `gears.get_total_ratio()`, `gears.engine_rpm_to_wheel_omega()`
- 標準ライブラリ `iostream`, `iomanip`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - エンジンモデル: `create_cbr600rr_engine()`
  - トルクカーブ確認: `1000`〜`15000 rpm`
  - スロットル試験: `10000 rpm`, `0.0`〜`1.0`
  - 車速換算: 3 速 `10000 rpm`, タイヤ半径 `0.228 m`
- 出力:
  - 標準出力に仕様表、トルクカーブ、スロットル別トルク、ギア比、車速換算、各ギア最高速
  - 外部ファイルなし
  - 明示的失敗判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `create_cbr600rr_engine()` でエンジン生成
  2. アイドル回転数、最大トルク、レッドラインを表示
  3. `1000`〜`15000 rpm` で全開トルクと出力[kW]を表表示
  4. `10000 rpm` でスロットル 0〜100% のトルクを表示
  5. `GearRatios::cbr600rr_default()` を生成し、各ギア比と総減速比を表示
  6. 3 速 `10000 rpm` の車輪角速度と車速を計算
  7. レッドライン回転数時の各ギア理論最高速を表示

#### `main()` が確認すること
- エンジントルクカーブの大きさと回転数依存
- スロットルによるトルク変化
- ギア比換算と車速換算の整合性

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - Torque Curve
  - Throttle Test
  - Gear Ratios Test
  - Speed Calculation
  - Max Speed per Gear
