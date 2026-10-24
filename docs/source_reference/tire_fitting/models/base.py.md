# tire_fitting/models/base.py

## 役割
すべてのタイヤモデルが従う抽象インタフェースを定義します。

## 主な型
- `TireForces`
- `TireModel` (`calculate_fx`, `calculate_fy`, `get_model_type`, `get_parameters`, `set_parameters`)

## 背景
TMeasy・Magic Formula・Hybrid を共通 API で扱うため、同定器や比較レポートをモデル非依存にできます。

## 入出力
- 入力: スリップ比、スリップ角、荷重など
- 出力: 力または `TireForces`

## パイプライン上の位置
モデル差し替えを可能にする抽象層です。
