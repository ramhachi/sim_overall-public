# `matlab/compare_results.m`

## 1. ファイルの目的

デフォルト制御と最適化後制御のラップタイム CSV を読み比べ、軌跡・誤差・入力・統計をまとめて比較表示する関数です。

## 2. 主な入出力または依存関係

- 入力:
  - `default_csv`
  - `optimized_csv`
  - `track_type`
  - `track_radius`
- 出力: なし（図表示と標準出力）
- 依存関係:
  - `readtable`
  - ローカル関数 `generate_track_centerline`
  - CSV 内の `x`, `y`, `time`, `lateral_error`, `heading_error_deg`, `steering`, `speed`

## 3. 処理の流れ / アルゴリズム

1. 2 つの CSV を読み込みます。
2. `generate_track_centerline()` で対象トラックの中心線を再構成します。
3. 1 枚目の図で軌跡、横偏差、ヘディング誤差、ステア角、速度を並べて描画します。
4. RMS / 最大誤差や平均速度などの統計を計算し、改善・悪化を標準出力へ表形式で出します。
5. 2 枚目の図で横偏差とヘディング誤差のヒストグラムを重ね描きします。

## 4. ファイル内の全関数

### `compare_results(default_csv, optimized_csv, track_type, track_radius)`

- メイン関数です。
- 引数未指定時は `default_sim.csv` と `optimized_sim.csv`、`figure_eight`, `25m` を使います。
- 指標の比較では、速度系以外は小さい方が良いとみなし、変化率に応じて簡易マーカーを出します。

### `generate_track_centerline(track_type, radius)`

- 比較用背景としてトラック中心線を生成するローカル関数です。
- `circle`, `figure_eight`, `straight_corner` の 3 種をサポートします。
- 円や 8 の字は曲率積分、直線+コーナーは直線部と円弧部の連結で点列化します。

## 5. 注意点・前提条件

- 2 つの CSV は同じ列名を持つ前提です。
- 速度が高いほど良いかは文脈依存のため、表出力では速度系だけ改善/悪化マーカーを付けていません。
- ローカルの `generate_track_centerline()` 実装は可視化用であり、C++ 側の真のトラックモデル完全一致を保証するものではありません。
