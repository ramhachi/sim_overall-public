# test_steering_torque.cpp

- **目的**: ステアリングトルク平衡式と torque mode 統合動作を検証します。
- **主要要素**: `SteeringTorqueFunction`、`SteeringTorqueTest`。
- **理論背景**: SAT 反映、ギア比変換、トルク飽和、角度モード後方互換性を確認します。
- **依存**: `gtest`、`vehicle_14dof_new.hpp`、`steering_torque.hpp`。
- **システム上の位置づけ**: Phase A2 操舵系追加の回帰です。

