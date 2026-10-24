# gps_model.hpp

- **目的**: Gauss-Markov 相関誤差を持つ GPS センサモデルを提供します。
- **主要要素**: `GPSSensorParams`、`GPSSensorOutput`、`GPSSensorModel`。
- **理論背景**: DOP 変動、ドロップアウト、マルチパス、遅延を含むレース向け GPS ノイズを表現します。
- **依存**: `sensor_interface.hpp`、`Eigen`、`deque`。
- **システム上の位置づけ**: 推定器検証やセンサフュージョン用の位置観測源です。

