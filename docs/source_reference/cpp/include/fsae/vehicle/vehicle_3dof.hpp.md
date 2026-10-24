# vehicle_3dof.hpp

- **目的**: 平面 3DOF 車両モデルを提供します。
- **主要要素**: `Vehicle3DOF`、`State3DOF`。
- **理論背景**: x-y-ψ、車体速度、車輪回転、TMeasy 変形のみを持つ最小モデルです。
- **依存**: `vehicle_params.hpp`、`vehicle_state.hpp`、`tmeasy.hpp`、`Eigen`。
- **システム上の位置づけ**: 高次モデルのデバッグ基準となる簡略モデルです。

