# vehicle_8dof_full.hpp

- **目的**: 8DOF+ARB+Bouc-Wen に LuGre 摩擦まで含めたフルサスペンションモデルです。
- **主要要素**: `State8DOF_Full`、`Vehicle8DOF_Full`。
- **理論背景**: 履歴減衰と stick-slip 摩擦を同時に積分し、サスペンションの非線形性を強化します。
- **依存**: `vehicle_8dof_bw.hpp`、`lugre.hpp`。
- **システム上の位置づけ**: Phase 5 系サスペンション詳細モデルの集約版です。

