# vehicle_6dof.hpp

- **目的**: 3DOF にロール・ピッチ・ヒーブを加えた 6DOF 車両モデルを実装します。
- **主要要素**: `Vehicle6DOF`、`State6DOF`。
- **理論背景**: 姿勢角と車体速度を回転行列で結び、サスペンション反力とタイヤ力を統合します。
- **依存**: `vehicle_params.hpp`、`vehicle_state.hpp`、`tmeasy.hpp`、`rotation.hpp`。
- **システム上の位置づけ**: 8DOF/14DOF へ進む前の中間モデルです。

