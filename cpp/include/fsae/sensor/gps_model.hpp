#pragma once
/// @file gps_model.hpp
/// @brief GPSセンサモデル（Gauss-Markovプロセス）
///
/// GPS特有のエラー特性をモデル化：
/// - 時間相関のある位置誤差（Gauss-Markov）
/// - DOP（精度劣化率）変動
/// - 遅延とドロップアウト

#include "sensor_interface.hpp"
#include <Eigen/Dense>
#include <deque>

namespace fsae::sensor {

/// GPSセンサパラメータ
struct GPSSensorParams {
    // === 精度 ===
    double position_noise_std = 2.0;    // [m] 水平位置標準偏差
    double altitude_noise_std = 4.0;    // [m] 高度標準偏差
    double velocity_noise_std = 0.1;    // [m/s] 速度標準偏差
    double heading_noise_std = 0.05;    // [rad] ヘディング標準偏差
    
    // === Gauss-Markov相関 ===
    double correlation_time = 60.0;     // [s] 時間相関 (GPS誤差の自己相関)
    
    // === DOP (Dilution of Precision) ===
    double hdop_nominal = 1.2;          // 標準HDOP
    double hdop_variation = 0.3;        // HDOP変動幅
    double vdop_nominal = 1.8;          // 標準VDOP
    
    // === サンプリング ===
    double update_rate_hz = 10.0;       // [Hz] 更新レート
    double latency_s = 0.05;            // [s] 遅延 (50ms typical)
    
    // === 信頼性 ===
    double dropout_probability = 0.01;  // ドロップアウト確率
    double multipath_std = 1.0;         // [m] マルチパス誤差
    
    // === プリセット ===
    
    /// RTK-GPS（高精度）
    static GPSSensorParams rtk_gps() {
        GPSSensorParams p;
        p.position_noise_std = 0.02;    // 2cm
        p.velocity_noise_std = 0.01;
        p.update_rate_hz = 20.0;
        p.latency_s = 0.02;
        p.dropout_probability = 0.02;
        return p;
    }
    
    /// 標準GPS
    static GPSSensorParams standard_gps() {
        GPSSensorParams p;
        p.position_noise_std = 3.0;
        p.velocity_noise_std = 0.2;
        p.update_rate_hz = 10.0;
        return p;
    }
    
    /// FSAE典型（RaceLogic等）
    static GPSSensorParams fsae_typical() {
        GPSSensorParams p;
        p.position_noise_std = 1.0;
        p.velocity_noise_std = 0.05;
        p.update_rate_hz = 20.0;
        p.latency_s = 0.03;
        return p;
    }
};

/// GPS出力
struct GPSSensorOutput {
    Eigen::Vector2d position;       // [m] x, y
    double altitude = 0;            // [m] z
    Eigen::Vector2d velocity;       // [m/s] vx, vy
    double heading = 0;             // [rad]
    double hdop = 1.0;
    double vdop = 1.0;
    int num_satellites = 12;
    double timestamp = 0;
    bool valid = false;
};

/// GPSセンサモデル
class GPSSensorModel : public ISensorModel {
public:
    explicit GPSSensorModel(const GPSSensorParams& params = GPSSensorParams())
        : params_(params)
        , noise_(123)
        , correlated_error_(Eigen::Vector2d::Zero())
    {}
    
    /// 真値を設定
    void set_ground_truth(const Eigen::Vector2d& true_position,
                          double true_altitude,
                          const Eigen::Vector2d& true_velocity,
                          double true_heading) {
        true_position_ = true_position;
        true_altitude_ = true_altitude;
        true_velocity_ = true_velocity;
        true_heading_ = true_heading;
    }
    
    /// センサモデルを更新
    void update(double dt) override {
        time_since_last_sample_ += dt;
        total_time_ += dt;
        
        double sample_period = 1.0 / params_.update_rate_hz;
        if (time_since_last_sample_ >= sample_period) {
            time_since_last_sample_ -= sample_period;
            
            // ドロップアウトチェック
            if (noise_.uniform() < params_.dropout_probability) {
                output_.valid = false;
                return;
            }
            
            // 相関誤差を更新（Gauss-Markovプロセス）
            update_correlated_error(dt);
            
            // DOP変動
            double hdop = params_.hdop_nominal + 
                          params_.hdop_variation * std::sin(total_time_ * 0.1);
            
            // ノイズ付き計測値を生成
            output_.position = compute_noisy_position(true_position_, hdop);
            output_.altitude = compute_noisy_altitude(true_altitude_);
            output_.velocity = compute_noisy_velocity(true_velocity_);
            output_.heading = compute_noisy_heading(true_heading_);
            output_.hdop = hdop;
            output_.num_satellites = 8 + static_cast<int>(noise_.uniform(0, 6));
            output_.timestamp = total_time_ - params_.latency_s;
            output_.valid = true;
        }
    }
    
    /// 計測値を取得
    const GPSSensorOutput& output() const { return output_; }
    
    /// パラメータを取得
    const GPSSensorParams& params() const { return params_; }
    
    /// センサ名
    const char* name() const override { return "GPS"; }
    
    /// サンプリングレート
    double sample_rate() const override { return params_.update_rate_hz; }
    
    /// 乱数シードを設定
    void set_seed(uint64_t seed) {
        noise_.set_seed(seed);
    }

private:
    GPSSensorParams params_;
    NoiseGenerator noise_;
    
    Eigen::Vector2d true_position_{Eigen::Vector2d::Zero()};
    double true_altitude_ = 0;
    Eigen::Vector2d true_velocity_{Eigen::Vector2d::Zero()};
    double true_heading_ = 0;
    
    Eigen::Vector2d correlated_error_;  // Gauss-Markov相関誤差
    
    GPSSensorOutput output_;
    double time_since_last_sample_ = 0;
    double total_time_ = 0;
    
    /// Gauss-Markov相関誤差を更新
    void update_correlated_error(double dt) {
        double alpha = dt / params_.correlation_time;
        double decay = std::exp(-alpha);
        double noise_scale = params_.position_noise_std * std::sqrt(1.0 - decay * decay);
        
        correlated_error_(0) = decay * correlated_error_(0) + 
                               noise_scale * noise_.gaussian();
        correlated_error_(1) = decay * correlated_error_(1) + 
                               noise_scale * noise_.gaussian();
    }
    
    /// ノイズ付き位置を計算
    Eigen::Vector2d compute_noisy_position(const Eigen::Vector2d& true_pos, double hdop) {
        double scale = hdop / params_.hdop_nominal;
        
        // 相関誤差 + 白色ノイズ + マルチパス
        Eigen::Vector2d noise;
        noise(0) = correlated_error_(0) + 
                   params_.multipath_std * noise_.gaussian() * scale;
        noise(1) = correlated_error_(1) + 
                   params_.multipath_std * noise_.gaussian() * scale;
        
        return true_pos + noise;
    }
    
    /// ノイズ付き高度を計算
    double compute_noisy_altitude(double true_alt) {
        return true_alt + params_.altitude_noise_std * noise_.gaussian();
    }
    
    /// ノイズ付き速度を計算
    Eigen::Vector2d compute_noisy_velocity(const Eigen::Vector2d& true_vel) {
        Eigen::Vector2d noise;
        noise(0) = params_.velocity_noise_std * noise_.gaussian();
        noise(1) = params_.velocity_noise_std * noise_.gaussian();
        return true_vel + noise;
    }
    
    /// ノイズ付きヘディングを計算
    double compute_noisy_heading(double true_hdg) {
        return true_hdg + params_.heading_noise_std * noise_.gaussian();
    }
};

} // namespace fsae::sensor
