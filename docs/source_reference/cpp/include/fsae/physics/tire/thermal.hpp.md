# thermal.hpp

- **目的**: 温度依存摩擦係数を持つ簡易タイヤ熱モデルを提供します。
- **主要要素**: `TireThermalParams`、`TireThermalState`、`TireThermalOutput`、`compute_tire_thermal()`。
- **理論背景**: 摩擦仕事、対流、路面伝導の熱収支から表面温度変化率とグリップ係数倍率を返します。
- **依存**: `cmath`、`algorithm`。
- **システム上の位置づけ**: タイヤグリップの温度補正に使う局所モデルです。

