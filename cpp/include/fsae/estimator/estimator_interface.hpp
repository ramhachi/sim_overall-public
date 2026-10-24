#pragma once
/// @file estimator_interface.hpp
/// @brief 状態推定器インターフェース
///
/// EKF/UKF/粒子フィルタ等のプラガブルな状態推定器インターフェースを定義。

#include "../vehicle/vehicle_state.hpp"
#include "../sensor/sensor_interface.hpp"
#include <Eigen/Dense>
#include <memory>

namespace fsae::estimator {

/// 状態推定器の抽象インターフェース
class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;
    
    /// 予測ステップ（時間更新）
    /// @param dt タイムステップ [s]
    /// @param input ドライバー入力
    virtual void predict(double dt, const vehicle::DriverInput& input) = 0;
    
    /// 更新ステップ（観測更新）
    /// @param measurement センサ計測値
    virtual void update(const sensor::SensorMeasurement& measurement) = 0;
    
    /// 推定状態を取得
    virtual vehicle::VehicleState get_estimated_state() const = 0;
    
    /// 状態共分散行列を取得
    virtual Eigen::MatrixXd get_covariance() const = 0;
    
    /// イノベーション（観測残差）を取得
    virtual Eigen::VectorXd get_innovation() const = 0;
    
    /// 状態推定器をリセット
    /// @param initial_state 初期状態
    /// @param initial_covariance 初期共分散（オプション）
    virtual void reset(const vehicle::VehicleState& initial_state,
                       const Eigen::MatrixXd* initial_covariance = nullptr) = 0;
    
    /// 推定器名を取得
    virtual const char* name() const = 0;
};

/// プロセスモデルの抽象インターフェース
/// 状態の時間発展を記述
class IProcessModel {
public:
    virtual ~IProcessModel() = default;
    
    /// 状態遷移関数 f(x, u, dt)
    virtual Eigen::VectorXd predict_state(const Eigen::VectorXd& state,
                                          const vehicle::DriverInput& input,
                                          double dt) const = 0;
    
    /// 状態遷移ヤコビアン ∂f/∂x
    virtual Eigen::MatrixXd get_state_jacobian(const Eigen::VectorXd& state,
                                                const vehicle::DriverInput& input,
                                                double dt) const = 0;
    
    /// プロセスノイズ共分散 Q
    virtual Eigen::MatrixXd get_process_noise(double dt) const = 0;
    
    /// 状態次元
    virtual int state_dim() const = 0;
};

/// 観測モデルの抽象インターフェース
/// センサ計測値と状態の関係を記述
class IObservationModel {
public:
    virtual ~IObservationModel() = default;
    
    /// 観測関数 h(x)
    virtual Eigen::VectorXd predict_observation(
        const Eigen::VectorXd& state,
        const sensor::SensorMeasurement& available_sensors) const = 0;
    
    /// 観測ヤコビアン ∂h/∂x
    virtual Eigen::MatrixXd get_observation_jacobian(
        const Eigen::VectorXd& state,
        const sensor::SensorMeasurement& available_sensors) const = 0;
    
    /// 観測ノイズ共分散 R
    virtual Eigen::MatrixXd get_observation_noise(
        const sensor::SensorMeasurement& available_sensors) const = 0;
    
    /// 計測値をベクトルに変換
    virtual Eigen::VectorXd measurement_to_vector(
        const sensor::SensorMeasurement& measurement) const = 0;
    
    /// 計測値次元（可変）
    virtual int observation_dim(
        const sensor::SensorMeasurement& available_sensors) const = 0;
};

/// 車両ダイナミクス用のプロセスモデル
class VehicleProcessModel : public IProcessModel {
public:
    VehicleProcessModel(const vehicle::VehicleParams& params);
    
    Eigen::VectorXd predict_state(const Eigen::VectorXd& state,
                                  const vehicle::DriverInput& input,
                                  double dt) const override;
    
    Eigen::MatrixXd get_state_jacobian(const Eigen::VectorXd& state,
                                        const vehicle::DriverInput& input,
                                        double dt) const override;
    
    Eigen::MatrixXd get_process_noise(double dt) const override;
    
    int state_dim() const override { return state_dim_; }
    
    /// プロセスノイズパラメータを設定
    void set_process_noise_std(double position_std, 
                               double velocity_std,
                               double attitude_std);

private:
    vehicle::VehicleParams params_;
    int state_dim_ = 12;  // 簡略化：x, y, psi, vx, vy, r, ax, ay, p, q, phi, theta
    
    // プロセスノイズパラメータ
    double position_noise_std_ = 0.01;
    double velocity_noise_std_ = 0.1;
    double attitude_noise_std_ = 0.01;
};

/// 車両用の観測モデル
class VehicleObservationModel : public IObservationModel {
public:
    VehicleObservationModel();
    
    Eigen::VectorXd predict_observation(
        const Eigen::VectorXd& state,
        const sensor::SensorMeasurement& available_sensors) const override;
    
    Eigen::MatrixXd get_observation_jacobian(
        const Eigen::VectorXd& state,
        const sensor::SensorMeasurement& available_sensors) const override;
    
    Eigen::MatrixXd get_observation_noise(
        const sensor::SensorMeasurement& available_sensors) const override;
    
    Eigen::VectorXd measurement_to_vector(
        const sensor::SensorMeasurement& measurement) const override;
    
    int observation_dim(
        const sensor::SensorMeasurement& available_sensors) const override;
    
    /// IMUノイズパラメータを設定
    void set_imu_noise(double accel_std, double gyro_std);
    
    /// GPSノイズパラメータを設定
    void set_gps_noise(double position_std, double velocity_std);
    
    /// 車輪速ノイズパラメータを設定
    void set_wheel_speed_noise(double std);

private:
    // センサノイズパラメータ
    double accel_noise_std_ = 0.1;
    double gyro_noise_std_ = 0.01;
    double gps_position_noise_std_ = 2.0;
    double gps_velocity_noise_std_ = 0.1;
    double wheel_speed_noise_std_ = 0.5;
};

/// 推定器ファクトリー
std::unique_ptr<IStateEstimator> create_ekf(
    std::unique_ptr<IProcessModel> process_model,
    std::unique_ptr<IObservationModel> observation_model);

std::unique_ptr<IStateEstimator> create_ukf(
    std::unique_ptr<IProcessModel> process_model,
    std::unique_ptr<IObservationModel> observation_model);

} // namespace fsae::estimator
