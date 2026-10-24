# estimator_interface.hpp

- **目的**: EKF/UKF などを差し替え可能にする状態推定インターフェースを定義します。
- **主要要素**: `IStateEstimator`、`IProcessModel`、`IObservationModel`、`VehicleProcessModel`、`VehicleObservationModel`、`create_ekf/ukf()`。
- **理論背景**: 予測更新と観測更新を分離し、車両モデルとセンサモデルを推定器へ接続する設計です。
- **依存**: `vehicle_state.hpp`、`sensor_interface.hpp`、`Eigen`。
- **システム上の位置づけ**: 将来の状態推定拡張点を規定する契約層です。

