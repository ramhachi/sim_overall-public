# rotation.hpp

- **目的**: SAE J670e に基づくオイラー角と回転行列の相互関係を提供します。
- **主要要素**: `EulerAngles`、`euler_to_rotation_matrix()`、`angular_velocity_to_euler_rate()`、`gravity_in_body()`。
- **理論背景**: Z-Y-X（yaw-pitch-roll）姿勢表現を使い、車体座標系での重力や角速度変換を行います。
- **依存**: `Eigen::Dense`、`cmath`。
- **システム上の位置づけ**: すべての 6DOF 以上の車両モデルが使う基礎運動学です。

