# python/fsae_sim/viz/plot_results.py

## 役割
車両応答 CSV と MATLAB/C++ 比較 CSV を可視化する共通関数集です。

## 主な関数
- `setup_plot_style()`: 描画スタイル統一
- `plot_vehicle_response()`: 軌跡、速度、姿勢、ヨーレート、タイヤ力・温度を描画
- `plot_comparison()`: MATLAB/C++ の状態時系列を比較

## 背景
結果 CSV の列構成が部分的に異なっても、存在列だけを描ける柔軟な可視化コードです。

## 使用ライブラリ
- `pandas`, `numpy`, `matplotlib`

## 入出力
- 入力: 結果 CSV
- 出力: `vehicle_response.png` や `comparison.png`

## パイプライン上の位置
シミュレーション後解析の共通可視化層です。
