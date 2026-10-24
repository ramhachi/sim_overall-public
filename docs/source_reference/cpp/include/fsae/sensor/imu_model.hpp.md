# imu_model.hpp

- **目的**: Allan 分散ベースの IMU ノイズモデルを提供します。
- **主要要素**: `IMUSensorParams`、`IMUSensorOutput`、`IMUSensorModel`。
- **理論背景**: ARW/VRW、バイアス不安定、ランダムウォーク、測定レンジ制限を扱います。
- **依存**: `sensor_interface.hpp`、`Eigen`。
- **システム上の位置づけ**: 姿勢・加速度観測を模擬する慣性センサ層です。

