# tire_fitting/__init__.py

## 役割
`tire_fitting` の公開 API 入口です。ローダ、前処理、TMeasy、最適化、評価、エクスポータを再公開します。

## 主な内容
- `TireDataLoader`
- `TireDataPreprocessor`
- `TMeasyModel`
- `TireParameterOptimizer`
- `FitEvaluator`
- `ParameterExporter`

## パイプライン上の位置
CLI やテストがモジュールを簡潔に import するための集約点です。
