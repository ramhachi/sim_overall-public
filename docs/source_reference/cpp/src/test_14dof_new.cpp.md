# test_14dof_new.cpp

- **目的**: `Vehicle14DOF_New` の代表設定を与えてステップステア応答を評価します。
- **主要要素**: `main()`。
- **理論背景**: Bouc-Wen、LuGre、ARB を含む新 14DOF モデルの静荷重・姿勢・横加速度を確認します。
- **依存**: `vehicle_14dof_new.hpp`、`spdlog`、`fstream`。
- **システム上の位置づけ**: 新モデル移行時の手動回帰チェックです。

