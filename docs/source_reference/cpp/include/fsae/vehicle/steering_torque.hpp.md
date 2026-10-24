# steering_torque.hpp

- **目的**: ステアリングホイールのトルク平衡モデルを純関数として提供します。
- **主要要素**: `compute_steering_acceleration()`、舵角/ハンドル角変換関数。
- **理論背景**: SAT、クーロン摩擦、粘性減衰、復元ばねを含む 1 自由度操舵系モデルです。
- **依存**: `cmath`、`algorithm`。
- **システム上の位置づけ**: Phase A2 の torque-mode steering コアです。

