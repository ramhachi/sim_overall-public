# estimator_interface.hpp

## ファイルの目的
- 状態推定器・プロセスモデル・観測モデルの共通インターフェースを定義します。

## 実行上の役割
- EKF/UKF 等の推定器を差し替え可能にし、車両状態・入力・センサ測定の接続点を統一します。
- このヘッダ内では `VehicleProcessModel` / `VehicleObservationModel` の公開 API まで宣言します。

## 依存関係
- リポジトリ内: `../vehicle/vehicle_state.hpp`, `../sensor/sensor_interface.hpp`
- 外部: `Eigen/Dense`, `<memory>`

## 入力/出力
- 入力: `vehicle::DriverInput`, `sensor::SensorMeasurement`, `vehicle::VehicleState`, `Eigen::VectorXd`
- 出力: 推定状態、共分散、イノベーション、予測観測、生成済み推定器

## 関数・メソッド・クラスの網羅的な説明
- `class IStateEstimator`
  - `virtual ~IStateEstimator() = default`
  - `predict(double dt, const vehicle::DriverInput& input)`
    - 時間更新 API です。
  - `update(const sensor::SensorMeasurement& measurement)`
    - 観測更新 API です。
  - `get_estimated_state() const`
    - 車両状態形式で推定結果を返します。
  - `get_covariance() const`
    - 共分散行列を返します。
  - `get_innovation() const`
    - 観測残差ベクトルを返します。
  - `reset(const vehicle::VehicleState& initial_state, const Eigen::MatrixXd* initial_covariance = nullptr)`
    - 初期状態・任意初期共分散で再初期化します。
  - `name() const`
    - 推定器名を返します。
- `class IProcessModel`
  - `virtual ~IProcessModel() = default`
  - `predict_state(const Eigen::VectorXd& state, const vehicle::DriverInput& input, double dt) const`
    - 状態遷移関数です。
  - `get_state_jacobian(...) const`
    - 状態ヤコビアンです。
  - `get_process_noise(double dt) const`
    - プロセスノイズ共分散です。
  - `state_dim() const`
    - 状態次元を返します。
- `class IObservationModel`
  - `virtual ~IObservationModel() = default`
  - `predict_observation(const Eigen::VectorXd& state, const sensor::SensorMeasurement& available_sensors) const`
    - 利用可能センサに対応した予測観測ベクトルです。
  - `get_observation_jacobian(...) const`
    - 観測ヤコビアンです。
  - `get_observation_noise(...) const`
    - 観測ノイズ共分散です。
  - `measurement_to_vector(const sensor::SensorMeasurement& measurement) const`
    - 測定構造体を観測ベクトルへ変換します。
  - `observation_dim(const sensor::SensorMeasurement& available_sensors) const`
    - 現在使える観測次元を返します。
- `class VehicleProcessModel : public IProcessModel`
  - `VehicleProcessModel(const vehicle::VehicleParams& params);`
    - 実装は別ソースですが、車両パラメータを受けて構築されます。
  - `predict_state(...) const override`
    - 状態遷移の本体宣言です。
  - `get_state_jacobian(...) const override`
    - 線形化ヤコビアン宣言です。
  - `get_process_noise(double dt) const override`
    - ノイズ共分散生成宣言です。
  - `state_dim() const override`
    - 既定 `state_dim_` を返します。
  - `set_process_noise_std(double position_std, double velocity_std, double attitude_std)`
    - プロセスノイズ標準偏差を変更します。
  - private `params_`, `state_dim_`, `position_noise_std_`, `velocity_noise_std_`, `attitude_noise_std_`
    - 車両パラメータ、状態次元、ノイズ設定です。
- `class VehicleObservationModel : public IObservationModel`
  - `VehicleObservationModel();`
    - 実装は別ソースにあります。
  - `predict_observation(...) const override`
  - `get_observation_jacobian(...) const override`
  - `get_observation_noise(...) const override`
  - `measurement_to_vector(...) const override`
  - `observation_dim(...) const override`
    - いずれも利用センサに応じた観測モデル宣言です。
  - `set_imu_noise(double accel_std, double gyro_std)`
  - `set_gps_noise(double position_std, double velocity_std)`
  - `set_wheel_speed_noise(double std)`
    - センサごとのノイズ設定変更 API です。
  - private `accel_noise_std_`, `gyro_noise_std_`, `gps_position_noise_std_`, `gps_velocity_noise_std_`, `wheel_speed_noise_std_`
    - 観測ノイズ設定です。
- `create_ekf(std::unique_ptr<IProcessModel> process_model, std::unique_ptr<IObservationModel> observation_model)`
  - 与えられたモデル組から EKF 実装を生成するファクトリ宣言です。
- `create_ukf(std::unique_ptr<IProcessModel> process_model, std::unique_ptr<IObservationModel> observation_model)`
  - UKF 版ファクトリ宣言です。
