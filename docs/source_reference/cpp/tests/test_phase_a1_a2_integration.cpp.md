# test_phase_a1_a2_integration.cpp

- **目的**: フレーム柔軟性（A1）とステアリングトルク（A2）の統合挙動を検証します。
- **主要要素**: `IntegrationTest` fixture。
- **理論背景**: 前後ロール分離と torque-mode steering が同時に入っても数値的に破綻しないことを見ます。
- **依存**: `gtest`、`vehicle_14dof_new.hpp`。
- **システム上の位置づけ**: A1/A2 の横断的回帰です。

