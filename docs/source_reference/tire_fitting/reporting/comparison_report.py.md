# tire_fitting/reporting/comparison_report.py

## 役割
TMeasy と Magic Formula の比較を Markdown レポートと PNG 図として出力します。

## 主な型・関数
- `ModelResult`
- `ComparisonReportData`
- `generate_comparison_report()`
- `_generate_visualizations()`
- `_plot_longitudinal_comparison()`
- `_plot_lateral_comparison()`
- `_build_report_content()`

## 背景
単に最良モデルを選ぶだけでなく、なぜその選択になったかを共有する必要があります。本ファイルは実験データ、予測曲線、指標表、性能差分析を 1 つの Markdown 成果物にまとめます。

## 使用ライブラリ
- `numpy`, `matplotlib`, `dataclasses`, `datetime`
- `tire_fitting.models`

## 入出力
- 入力: モデル結果、実験データ、選択理由
- 出力: Markdown レポート、比較プロット群

## パイプライン上の位置
モデル選択の説明責任を担うレポーティング層です。
