# `tire_fitting/__init__.py`

## 役割

`tire_fitting` パッケージの公開窓口です。  
内部実装のうち、データ読込・前処理・TMeasy 計算・最適化・評価・エクスポートで使う主要クラスだけを再公開します。

## 実行と入出力

- 直接実行するファイルではありません。
- 入力:
  - `tire_fitting.data_loader.TireDataLoader`
  - `tire_fitting.preprocessor.TireDataPreprocessor`
  - `tire_fitting.tmeasy_model.TMeasyModel`
  - `tire_fitting.optimizer.TireParameterOptimizer`
  - `tire_fitting.evaluator.FitEvaluator`
  - `tire_fitting.exporter.ParameterExporter`
- 出力:
  - `from tire_fitting import ...` で使える `__all__`

## 主要要素

### 再公開シンボル
- `TireDataLoader`
- `TireDataPreprocessor`
- `TMeasyModel`
- `TireParameterOptimizer`
- `FitEvaluator`
- `ParameterExporter`

## データフロー

1. 各サブモジュールからクラスを import します。
2. `__all__` に載せ、パッケージ外からの import 面を固定します。

## システム内での位置づけ

実装本体ではなく **パッケージ境界の整備用 glue** です。  
`main.py` や `main_hybrid.py` からの import を短くし、利用者に公開したい API を絞る役目です。
