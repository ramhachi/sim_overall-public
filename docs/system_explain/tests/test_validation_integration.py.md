# tests/test_validation_integration.py

## 役割
`validation` モジュールが、良否判定・閾値変更・欠損方向・異常値・ログ出力を正しく扱うことを確認する統合テストです。

## 守っている振る舞い
- `TestValidationIntegration` では良好結果で `PASS`、悪化結果で `FAIL`、厳格閾値で不合格、レポート本文生成、方向欠落時の部分判定、warning ログ出力、閾値境界・NaN・Inf の扱いを確認します。
- `test_validation_boundary_cases()` は閾値ちょうどの値が不合格になることを明示します。
- `TestValidationWithRealData.test_validation_with_actual_fitting()` は実データ経由の遅い経路を用意していますが、通常は手動専用として即 `pytest.skip()` します。

## 主なフィクスチャ/ヘルパー
- `data_paths` は実データパスを返します。
- `fitting_results_mock` は `fx=magic_formula`, `fy=tmeasy` の模擬結果辞書を返します。
- `caplog` を使うテストでは warning メッセージの内容まで確認します。

## 重要なアサーション・スキップ
- `report.passed`、`overall_status`、方向別 validation の有無が中心です。
- 実データ用テストは `skipif` と明示 `pytest.skip()` の二段構えで通常実行から外されています。
- 生成物はなく、検証レポートテキストとログが主な成果物です。
