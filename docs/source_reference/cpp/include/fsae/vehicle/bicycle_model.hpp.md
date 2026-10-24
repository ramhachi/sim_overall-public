# bicycle_model.hpp

- **目的**: MPC 予測用の 6DOF 動的自転車モデルを提供します。
- **主要要素**: `BicycleState6DOF`、`BicycleControl`、`BicycleParams`、`BicycleModel6DOF`。
- **理論背景**: 線形タイヤ近似と RK4、有限差分線形化、定常円旋回解析を持ちます。
- **依存**: `Eigen`、`vehicle_params.hpp`。
- **システム上の位置づけ**: 6DOF MPC の予測モデルです。

