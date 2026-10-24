#pragma once
/// @file imu_model.hpp
/// @brief IMUセンサモデル（Allan分散ベース）
///
/// Allan分散に基づく現実的なIMUノイズモデルを実装。
/// - Angle/Velocity Random Walk
/// - Bias Instability
/// - Bias Random Walk

#include "sensor_interface.hpp"
#include <Eigen/Dense>

namespace fsae::sensor {

/// IMUセンサパラメータ
struct IMUSensorParams {
    // === 加速度計 ===
    double accel_noise_density = 0.01;      // [m/s²/√Hz] Velocity Random Walk
    double accel_bias_instability = 0.001;  // [m/s²] Bias Instability
    double accel_random_walk = 0.0001;      // [m/s²·√Hz] Bias Random Walk
    
    // === ジャイロスコープ ===
    double gyro_noise_density = 0.001;      // [rad/s/√Hz] Angle Random Walk
    double gyro_bias_instability = 0.0001;  // [rad/s] Bias Instability
    double gyro_random_walk = 0.00001;      // [rad/s·√Hz] Rate Random Walk
    
    // === 軸間干渉 ===
    double cross_axis_sensitivity = 0.01;   // [%] 軸間感度
    
    // === サンプリング ===
    double sample_rate_hz = 200.0;          // [Hz]
    double latency_s = 0.002;               // [s] 遅延
    
    // === 測定範囲 ===
    double accel_range = 160.0;             // [m/s²] ±16G
    double gyro_range = 35.0;               // [rad/s] ±2000 deg/s
    
    // === デフォルトパラメータセット ===
    
    /// 高精度IMU (Tactical Grade)
    static IMUSensorParams tactical_grade() {
        IMUSensorParams p;
        p.accel_noise_density = 0.002;      // 0.2 mg/√Hz
        p.accel_bias_instability = 0.0001;  // 10 µg
        p.gyro_noise_density = 0.0001;      // 0.33 deg/hr/√Hz
        p.gyro_bias_instability = 0.00001;  // 0.1 deg/hr
        return p;
    }
    
    /// 民生グレードIMU (MEMS)
    static IMUSensorParams consumer_grade() {
        IMUSensorParams p;
        p.accel_noise_density = 0.05;       // 5 mg/√Hz
        p.accel_bias_instability = 0.01;    // 1 mg
        p.gyro_noise_density = 0.005;       // 0.3 deg/s/√Hz
        p.gyro_bias_instability = 0.001;    // 36 deg/hr
        return p;
    }
    
    /// FSAE典型値 (レーシング用MEMS)
    static IMUSensorParams fsae_typical() {
        IMUSensorParams p;
        p.accel_noise_density = 0.02;       // 2 mg/√Hz
        p.accel_bias_instability = 0.005;
        p.gyro_noise_density = 0.002;
        p.gyro_bias_instability = 0.0005;
        p.sample_rate_hz = 100.0;
        return p;
    }
};

/// IMUセンサ出力
struct IMUSensorOutput {
    Eigen::Vector3d accel;          // 加速度計測値 [m/s²]
    Eigen::Vector3d gyro;           // ジャイロ計測値 [rad/s]
    Eigen::Vector3d accel_bias;     // 現在のバイアス状態
    Eigen::Vector3d gyro_bias;      // 現在のバイアス状態
    double timestamp;
    bool valid = false;
};

/// IMUセンサモデル
class IMUSensorModel : public ISensorModel {
public:
    explicit IMUSensorModel(const IMUSensorParams& params = IMUSensorParams())
        : params_(params)
        , noise_(42)
        , accel_bias_(Eigen::Vector3d::Zero())
        , gyro_bias_(Eigen::Vector3d::Zero())
    {}
    
    /// 真値を設定
    void set_ground_truth(const Eigen::Vector3d& true_accel,
                          const Eigen::Vector3d& true_gyro) {
        true_accel_ = true_accel;
        true_gyro_ = true_gyro;
    }
    
    /// センサモデルを更新
    void update(double dt) override {
        time_since_last_sample_ += dt;
        
        double sample_period = 1.0 / params_.sample_rate_hz;
        if (time_since_last_sample_ >= sample_period) {
            time_since_last_sample_ -= sample_period;
            
            // バイアスのランダムウォークを更新
            update_biases(dt);
            
            // ノイズ付き計測値を生成
            output_.accel = compute_noisy_accel(true_accel_);
            output_.gyro = compute_noisy_gyro(true_gyro_);
            output_.accel_bias = accel_bias_;
            output_.gyro_bias = gyro_bias_;
            output_.timestamp = total_time_;
            output_.valid = true;
        }
        
        total_time_ += dt;
    }
    
    /// 計測値を取得
    const IMUSensorOutput& output() const { return output_; }
    
    /// パラメータを取得
    const IMUSensorParams& params() const { return params_; }
    
    /// センサ名
    const char* name() const override { return "IMU"; }
    
    /// サンプリングレート
    double sample_rate() const override { return params_.sample_rate_hz; }
    
    /// バイアスをリセット
    void reset_bias() {
        accel_bias_.setZero();
        gyro_bias_.setZero();
    }
    
    /// 乱数シードを設定
    void set_seed(uint64_t seed) {
        noise_.set_seed(seed);
    }

private:
    IMUSensorParams params_;
    NoiseGenerator noise_;
    
    Eigen::Vector3d true_accel_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d true_gyro_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias_;
    Eigen::Vector3d gyro_bias_;
    
    IMUSensorOutput output_;
    double time_since_last_sample_ = 0;
    double total_time_ = 0;
    
    /// バイアスのランダムウォーク更新
    void update_biases(double dt) {
        // バイアス不安定性（一次マルコフ過程）
        double tau_accel = 1.0 / (2.0 * M_PI * 0.1);  // 相関時間
        double tau_gyro = 1.0 / (2.0 * M_PI * 0.1);
        
        // 指数減衰 + ランダムウォーク
        for (int i = 0; i < 3; ++i) {
            accel_bias_(i) = accel_bias_(i) * std::exp(-dt / tau_accel)
                           + params_.accel_random_walk * std::sqrt(dt) * noise_.gaussian();
            gyro_bias_(i) = gyro_bias_(i) * std::exp(-dt / tau_gyro)
                          + params_.gyro_random_walk * std::sqrt(dt) * noise_.gaussian();
        }
        
        // バイアスをクランプ
        for (int i = 0; i < 3; ++i) {
            accel_bias_(i) = std::clamp(accel_bias_(i), 
                                        -3.0 * params_.accel_bias_instability,
                                        3.0 * params_.accel_bias_instability);
            gyro_bias_(i) = std::clamp(gyro_bias_(i),
                                       -3.0 * params_.gyro_bias_instability,
                                       3.0 * params_.gyro_bias_instability);
        }
    }
    
    /// ノイズ付き加速度を計算
    Eigen::Vector3d compute_noisy_accel(const Eigen::Vector3d& true_accel) {
        double dt = 1.0 / params_.sample_rate_hz;
        
        // ホワイトノイズ（Velocity Random Walk）
        double noise_std = params_.accel_noise_density * std::sqrt(params_.sample_rate_hz);
        Eigen::Vector3d noise = noise_.gaussian_vector(noise_std);
        
        // 計測値 = 真値 + バイアス + ノイズ
        Eigen::Vector3d measurement = true_accel + accel_bias_ + noise;
        
        // 測定範囲でクランプ
        for (int i = 0; i < 3; ++i) {
            measurement(i) = std::clamp(measurement(i), 
                                        -params_.accel_range, 
                                        params_.accel_range);
        }
        
        return measurement;
    }
    
    /// ノイズ付き角速度を計算
    Eigen::Vector3d compute_noisy_gyro(const Eigen::Vector3d& true_gyro) {
        // ホワイトノイズ（Angle Random Walk）
        double noise_std = params_.gyro_noise_density * std::sqrt(params_.sample_rate_hz);
        Eigen::Vector3d noise = noise_.gaussian_vector(noise_std);
        
        // 計測値 = 真値 + バイアス + ノイズ
        Eigen::Vector3d measurement = true_gyro + gyro_bias_ + noise;
        
        // 測定範囲でクランプ
        for (int i = 0; i < 3; ++i) {
            measurement(i) = std::clamp(measurement(i),
                                        -params_.gyro_range,
                                        params_.gyro_range);
        }
        
        return measurement;
    }
};

} // namespace fsae::sensor
