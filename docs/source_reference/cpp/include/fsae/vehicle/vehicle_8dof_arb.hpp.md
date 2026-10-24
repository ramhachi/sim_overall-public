# vehicle_8dof_arb.hpp

- **目的**: 8DOF モデルへ前後アンチロールバーを追加します。
- **主要要素**: `Vehicle8DOF_ARB`、`compute_arb_forces()`。
- **理論背景**: 左右サスペンション変位差からロール角を近似し、ARB トルクを左右輪力へ変換します。
- **依存**: `vehicle_8dof.hpp`、`Eigen`。
- **システム上の位置づけ**: ロール剛性配分の影響を扱う拡張モデルです。

