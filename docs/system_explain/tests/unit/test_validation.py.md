# tests/unit/test_validation.py

## 役割
タイヤモデル適合品質の閾値判定・方向別判定・総合レポート生成が、しきい値仕様どおり動くかを検証します。

## 守っている振る舞い
- `ValidationThresholds` の既定値と custom 値が正しく保持され、不正範囲では `ValueError` になること。
- `validate_metric()` が `less_than` / `greater_than` の両比較で PASS/FAIL と説明文を正しく作ること。
- `validate_direction()` が RMSE/TIC/R² の個別失敗を方向別ステータスへ集約できること。
- `validate_fitting_quality()` が FX/FY の有無に応じて PASS/FAIL/WARNING を返し、部分結果や結果なしも扱えること。
- custom thresholds で総合判定が変わること。
- `get_failed_metrics()` が失敗項目一覧を正しく返すこと。
- `generate_validation_report_text()` が見出し、閾値、方向別節、失敗項目を含むレポート文字列を作ること。

## 主なフィクスチャ/ヘルパー
- `TestValidationThresholds`、`TestValidateMetric`、`TestValidateDirection`、`TestValidateFittingQuality`、`TestGenerateValidationReportText` に責務を分割しています。
- すべて辞書ベースの簡潔な `fitting_results` を使い、判定条件だけを明示的に操作しています。

## アサーションの意味
数値比較だけでなくメッセージ文面まで見ることで、機械判定と人間向けレポートの両方を固定しています。品質ゲートは CI やレポート配布の判断基準なので、しきい値と説明のズレを防ぐ意義が大きいです。
