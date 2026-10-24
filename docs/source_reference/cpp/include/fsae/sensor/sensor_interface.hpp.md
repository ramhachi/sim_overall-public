# sensor_interface.hpp

- **目的**: センサ計測統合構造とノイズ生成器、センサ共通インターフェースを定義します。
- **主要要素**: `SensorMeasurement`、`NoiseGenerator`、`ISensorModel`。
- **理論背景**: IMU/GPS/車輪速など異種計測を `optional` ベースで一つの構造へ集約します。
- **依存**: `Eigen`、`optional`、`random`。
- **システム上の位置づけ**: センサモデルと推定器の共通基盤です。

