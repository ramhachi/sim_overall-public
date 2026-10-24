# `scripts/analysis/plot_trajectory_quick.py`

## 1. 役割

結果 CSV の XY 軌跡を素早く 1 枚描き、開始点・終了点の状態を確認する **簡易クイックルック用スクリプト** です。

## 2. 入出力

- 実行例: `python3 scripts/analysis/plot_trajectory_quick.py <csv_file> <output_png>`
- 入力列: `x`, `y`, `psi`
- 出力:
  - 軌跡 PNG
  - 標準出力の開始/終了座標

## 3. 関数

### `plot_trajectory(csv_file, output_file)`
- 役割: CSV を読み、軌跡を 1 枚描きます。
- 副作用:
  - PNG 保存
  - 開始点の `x,y,psi`
  - 終点の `x,y`
  を print します。
- 失敗時: 例外メッセージのみ表示します。

## 4. CLI 挙動

- 引数不足時は usage を出して終了コード 1 で終了します。
