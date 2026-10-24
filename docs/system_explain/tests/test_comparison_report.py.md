# tests/test_comparison_report.py

## 役割
比較レポート生成器が、複数モデルの指標表と可視化を含む Markdown レポートを正しく出力することを確認します。

## 守っている振る舞い
- `test_comparison_report_generation()` は `ModelResult` を使った模擬 `tmeasy` / `magic_formula` 結果と模擬実験データから、レポート本体と `plots/` 配下 PNG が作られることを確認します。
- 同テストは `Executive Summary`、`Fitting Metrics Comparison`、`Force-Slip Curve Visualizations`、`Fitted Model Parameters` など主要セクションが本文に入ることを固定します。
- `test_report_with_missing_direction()` は `fy` 実験データ欠落時に `ValueError("Missing experimental data for fy")` を出すことを確認します。

## 主なフィクスチャ/ヘルパー
- 明示 fixture はなく、各テスト内で `ComparisonReportData` と 100 点 / 50 点の合成データを組み立てます。
- `tempfile.TemporaryDirectory()` を使って出力先を隔離します。

## 重要なアサーション・生成物
- 本文に `RMSE`、`TIC`、`R²`、`Peak Error`、モデル名、タイヤ名が残ることを見ています。
- 生成物はレポート Markdown と `plots/test_comparison_fx.png`, `plots/test_comparison_fy.png` です。
- エラー系まで含め、比較レポートが「生成できた」だけでなく入力整合性も検査することを保証します。
