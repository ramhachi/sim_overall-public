# vehicle_8dof_bw.hpp

- **目的**: 8DOF+ARB モデルへ Bouc-Wen ダンパーヒステリシスを追加します。
- **主要要素**: `State8DOF_BW`、`Vehicle8DOF_BW`。
- **理論背景**: ダンパー履歴変数 `damper_z` を状態に含め、非線形減衰を 4 輪個別に扱います。
- **依存**: `vehicle_8dof_arb.hpp`、`bouc_wen.hpp`。
- **システム上の位置づけ**: ヒステリシス付きサスペンション統合モデルです。

