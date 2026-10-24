# bouc_wen.hpp

- **目的**: Bouc-Wen ヒステリシスダンパーモデルを純関数で提供します。
- **主要要素**: `BoucWenParams`、`compute_bouc_wen_force()`、`compute_bouc_wen_dz()`、`compute_bouc_wen()`。
- **理論背景**: 履歴変数 `z` を使って非線形減衰ループを表現します。
- **依存**: `cmath`。
- **システム上の位置づけ**: 8DOF/14DOF 車両のダンパー力計算コアです。

