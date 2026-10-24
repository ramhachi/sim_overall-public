# test_bouc_wen_vehicle.cpp

- **目的**: Bouc-Wen ダンパーを統合した 8DOF 車両を比較評価します。
- **主要要素**: `main()`。
- **理論背景**: ヒステリシス履歴変数がサスペンション力へ与える影響を 8DOF 車両応答で確認します。
- **依存**: `vehicle_8dof_bw.hpp`、`vehicle_8dof_arb.hpp`、`spdlog`。
- **システム上の位置づけ**: ダンパーモデル統合後の回帰確認です。

