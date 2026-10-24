# `matlab/quarter_car/run_and_export.m`

## 概要

`run_and_export.m` は quarter-car モデルを実行し、比較用 CSV と 4 分割プロット画像を出力する運用スクリプトである。  
既定の `quarter_car_ode()` を呼び出して ground truth を生成し、C++ と同じ並びの列構成で `output` ディレクトリへ保存する。

## 主要関数・クラス

- `quarter_car_ode()`  
  シミュレーション本体。既定では `t=0.2 s` に 0.02 m のステップ入力を与える。
- `writetable(output_data, output_file)`  
  時系列結果を CSV 化する。
- プロット生成部  
  4 パネルで以下を出力する。  
  - ばね上／ばね下変位と路面入力
  - ばね上／ばね下速度
  - スプリング・ダンパ・タイヤ力
  - ダンパ相対速度－ダンパ力ヒステリシス

## 理論・背景

このファイル自体は新しい力学モデルを持たないが、縦振動応答を可搬なデータへ変換する役目を持つ。  
変位・速度・力・ヒステリシスを同時に保存することで、単なる時刻歴比較だけでなく、エネルギ散逸や非線形性の形状まで確認できる。

特に F-v ヒステリシスは Bouc-Wen ダンパの特徴確認に有効で、C++ 側ダンパ実装差分の検出に向いている。  
そのため、検証とデータ受け渡しの両方を兼ねた実務的なスクリプトになっている。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
標準の `table`、`writetable`、描画関数、`saveas` を用いる。

## 入出力

- 入力
  - 明示引数なし
  - 内部では `quarter_car_ode()` の既定条件を利用
- 出力
  - CSV: `../../output/quarter_car_matlab.csv`
  - 画像: `../../output/quarter_car_matlab.png`
  - 列: `time, z_s, z_u, dz_s, dz_u, F_spring, F_damper, F_tire, z_road`

## システム上の役割

MATLAB 側の ground truth 生成とファイル出力パイプラインを担う。  
サスペンション単体系の結果を C++ 検証・可視化・回帰確認へ流すための橋渡しファイルである。
