# `scripts/analysis/compare_track_shape.py`

## 1. 役割

生トラック CSV と再構成・シミュレーション結果の XY 軌跡を重ね、**形状一致を視覚比較** するスクリプトです。

## 2. 入出力

- 実行例: `python3 scripts/analysis/compare_track_shape.py <raw_csv> <sim_csv> <output_png>`
- 入力列: どちらも `x`, `y`
- 出力: 比較プロット PNG

## 3. 関数

### `compare_tracks(raw_csv, sim_csv, output_file)`
- 役割: 2 つの CSV を読み、基準トラックと再構成軌跡を重ねて描きます。
- 表示内容:
  - Raw を黒破線
  - Sim を赤実線
  - それぞれの開始点マーカー
- 副作用: PNG 保存、例外時 traceback 表示

## 4. CLI 挙動

- 引数不足時は usage を表示しつつ、既定ファイル名
  - `ASE_autocross_sim.csv`
  - `lat_error_test_drift_fix.csv`
  - `track_comparison.png`
 でそのまま実行します。

## 5. 注意点

- usage 文は `<raw_csv> <sim_csv> <output_png>` ですが、分岐条件は `len(sys.argv) < 3` なので 3 引数不足時に既定実行へ入ります。
