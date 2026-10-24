# `scripts/analysis/plot_autocross_result.py`

## 1. 役割

オートクロス結果 CSV から、**軌跡・横偏差・操作入力・速度時系列** を一括で可視化する解析スクリプトです。

## 2. 入出力

- 実行例: `python3 scripts/analysis/plot_autocross_result.py <csv_file> <output_prefix>`
- 入力列:
  - `x`, `y`, `speed`, `n`, `time`, `target_speed`, `steering`, `throttle`, `brake`
- 出力:
  - `<prefix>_trajectory_speed.png`
  - `<prefix>_trajectory_error.png`
  - `<prefix>_telemetry.png`

## 3. 関数

### `plot_results(csv_file, output_prefix)`
- 役割: 3 種類の図を連続生成します。
- 図の内容:
  1. 速度着色付き軌跡
  2. 横偏差 `|n|` 着色付き軌跡
  3. 速度、操舵/スロットル/ブレーキ、横偏差の時系列
- 副作用: 3 枚の PNG 保存、各保存先 print、例外時 traceback 表示

## 4. CLI 挙動

- 引数不足時は usage を出して終了コード 1 で終了します。
