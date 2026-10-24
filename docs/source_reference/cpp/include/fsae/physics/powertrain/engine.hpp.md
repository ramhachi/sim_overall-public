# engine.hpp

- **目的**: エンジントルクカーブとギア比換算を提供します。
- **主要要素**: `EngineTorqueMap`、`create_cbr600rr_engine()`、`GearRatios`。
- **理論背景**: RPM-トルク表の線形補間と、エンジン/車輪回転数の双方向変換を扱います。
- **依存**: `vector`、`algorithm`、`cmath`。
- **システム上の位置づけ**: 14DOF モデルや加速試験が使う駆動源です。

