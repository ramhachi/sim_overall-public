# test_lugre_vehicle.cpp

- **目的**: Bouc-Wen 版と LuGre 追加版 8DOF 車両の差を比較します。
- **主要要素**: `main()`。
- **理論背景**: サスペンション摺動摩擦が応答や数値安定性に与える影響を見る統合テストです。
- **依存**: `vehicle_8dof_full.hpp`、`vehicle_8dof_bw.hpp`、`spdlog`。
- **システム上の位置づけ**: 非線形サスペンション機能追加の確認用です。

