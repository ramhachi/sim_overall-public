# tire_fitting/validation.py

## 役割
フィット品質が閾値を満たすかを判定し、方向別/全体のレポートを生成します。

## 主な型・関数
- `ValidationStatus`
- `ValidationThresholds`
- `MetricValidation`, `DirectionValidation`, `ValidationReport`
- `validate_metric()`
- `validate_direction()`
- `validate_fitting_quality()`
- `generate_validation_report_text()`

## 背景
RMSE/TIC/R² の閾値判定をロジック化し、CLI とテストから一貫利用できるようにした品質ゲートです。

## 入出力
- 入力: フィット結果辞書
- 出力: dataclass ベースの検証レポートと整形済みテキスト

## パイプライン上の位置
モデル選択後、書き出し前に品質を保証するチェックポイントです。
