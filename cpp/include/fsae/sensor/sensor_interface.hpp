#pragma once
/// @file sensor_interface.hpp
/// @brief センサモデル共通インターフェース

#include <Eigen/Dense>
#include <array>
#include <optional>
#include <random>
#include <cmath>

namespace fsae::sensor {

/// センサ計測値の統合構造体
struct SensorMeasurement {
    double timestamp = 0;
    
    // IMU (Inertial Measurement Unit)
    std::optional<Eigen::Vector3d> accel;       // 加速度 [m/s²]
    std::optional<Eigen::Vector3d> gyro;        // 角速度 [rad/s]
    
    // GPS (Global Positioning System)
    std::optional<Eigen::Vector2d> gps_position; // 位置 (x, y) [m]
    std::optional<Eigen::Vector2d> gps_velocity; // 速度 (vx, vy) [m/s]
    
    // 車輪速センサ
    std::optional<std::array<double, 4>> wheel_speeds; // [rad/s]
    
    // ステアリング角センサ
    std::optional<double> steering_angle;       // [rad]
    
    // サスペンションストロークセンサ
    std::optional<std::array<double, 4>> suspension_travel; // [m]
};

/// ランダム数生成器のラッパー
class NoiseGenerator {
public:
    explicit NoiseGenerator(uint64_t seed = 42) 
        : gen_(seed), normal_(0.0, 1.0), uniform_(0.0, 1.0) {}
    
    /// 正規分布ノイズ
    double gaussian(double mean = 0.0, double std = 1.0) {
        return mean + std * normal_(gen_);
    }
    
    /// 一様分布
    double uniform(double min = 0.0, double max = 1.0) {
        return min + (max - min) * uniform_(gen_);
    }
    
    /// 3次元ガウスノイズ
    Eigen::Vector3d gaussian_vector(double std) {
        return Eigen::Vector3d(
            gaussian(0.0, std),
            gaussian(0.0, std),
            gaussian(0.0, std)
        );
    }
    
    /// 共分散付き3次元ガウスノイズ
    Eigen::Vector3d gaussian_vector(const Eigen::Matrix3d& covariance) {
        // Cholesky分解
        Eigen::LLT<Eigen::Matrix3d> llt(covariance);
        Eigen::Matrix3d L = llt.matrixL();
        
        Eigen::Vector3d z(normal_(gen_), normal_(gen_), normal_(gen_));
        return L * z;
    }
    
    void set_seed(uint64_t seed) {
        gen_.seed(seed);
    }
    
private:
    std::mt19937_64 gen_;
    std::normal_distribution<double> normal_;
    std::uniform_real_distribution<double> uniform_;
};

/// センサモデルの抽象基底クラス
class ISensorModel {
public:
    virtual ~ISensorModel() = default;
    
    /// 真値を入力し、ノイズ付き計測値を出力
    virtual void update(double dt) = 0;
    
    /// センサ名を取得
    virtual const char* name() const = 0;
    
    /// サンプリングレートを取得 [Hz]
    virtual double sample_rate() const = 0;
};

} // namespace fsae::sensor
