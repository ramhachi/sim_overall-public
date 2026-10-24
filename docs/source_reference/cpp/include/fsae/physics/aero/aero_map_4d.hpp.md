# aero_map_4d.hpp

- **目的**: 4 次元エアロマップを補間し、係数または直接荷重として空力出力を返します。
- **主要要素**: `AeroOutput`、`AeroForces`、`AeroMapParams`、`AeroMapGrid`、`AeroMap4D`。
- **理論背景**: 前後ライドハイト、ロール、ヨー、DRS 状態を入力に空力係数/力を線形補間します。
- **依存**: `nlohmann/json`、`vector`、`array`。
- **システム上の位置づけ**: 速度計画と 14DOF モデルの両方に空力効果を供給します。

