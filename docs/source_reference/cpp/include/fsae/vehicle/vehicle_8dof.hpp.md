# vehicle_8dof.hpp

- **目的**: 6DOF に 4 輪ばね下上下動を加えた 8DOF 車両モデルを実装します。
- **主要要素**: `Vehicle8DOF`、`State8DOF`。
- **理論背景**: sprung/unsprung の上下連成、TMeasy 緩和長、簡易空力、後輪駆動を RK4 で統合します。
- **依存**: `vehicle_params.hpp`、`vehicle_state.hpp`、`tmeasy.hpp`、`rotation.hpp`。
- **システム上の位置づけ**: サスペンション詳細追加前の 4 輪基本モデルです。

