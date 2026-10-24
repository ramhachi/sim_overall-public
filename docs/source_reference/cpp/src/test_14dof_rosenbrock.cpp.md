# test_14dof_rosenbrock.cpp

- **目的**: 14DOF モデルを Rosenbrock 法で積分した場合の挙動を評価します。
- **主要要素**: `main()`。
- **理論背景**: 剛性 ODE 向けの Rosenbrock23 を使い、既定 RK4 と比較するための検証です。
- **依存**: `vehicle_14dof_new.hpp`、`rosenbrock.hpp`、`spdlog`、`chrono`。
- **システム上の位置づけ**: ソルバ選定のための実験プログラムです。

