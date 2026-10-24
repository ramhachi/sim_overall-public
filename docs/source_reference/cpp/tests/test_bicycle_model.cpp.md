# test_bicycle_model.cpp

- **目的**: 6DOF 自転車モデルの直進、定常円、線形化、加減速、低速安定性を検証します。
- **主要要素**: `BicycleModelTest`、`BicycleParamsTest`。
- **理論背景**: MPC 予測モデルの基礎力学と線形化精度をテストします。
- **依存**: `gtest`、`bicycle_model.hpp`。
- **システム上の位置づけ**: 6DOF MPC の土台を守るテストです。

