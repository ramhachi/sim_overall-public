# test_frame_flexibility.cpp

- **目的**: フレーム柔軟性モデルの剛体退化、ねじり量、ロール剛性分配、数値安定性を検証します。
- **主要要素**: `FrameFlexibilityTest`。
- **理論背景**: `phi_twist` を導入した Phase A1 の前後ロール分離が妥当かを確認します。
- **依存**: `gtest`、`vehicle_14dof_new.hpp`。
- **システム上の位置づけ**: フレーム柔軟性機能の回帰保証です。

