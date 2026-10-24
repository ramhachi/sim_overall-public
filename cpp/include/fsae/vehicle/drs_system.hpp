#pragma once
/// @file drs_system.hpp
/// @brief DRS (Drag Reduction System) state machine and safety logic
///
/// DRS is a performance enhancement feature that reduces aerodynamic drag on straights
/// by adjusting the rear wing. When activated, downforce (CL) is reduced by ~46% and
/// drag (CD) is reduced by ~26%, allowing higher top speeds.
///
/// Safety constraints (3-layer activation checks):
///   1. Speed threshold: >= 15 m/s (54 km/h)
///   2. Lateral acceleration: <= 0.5g (~5 m/s²)
///   3. Steering angle: <= 0.05 rad (~3°)
///
/// State machine: INACTIVE → ACTIVATING (0.1s delay) → ACTIVE → INACTIVE (immediate)

#include <cmath>

namespace fsae {
namespace vehicle {

/// DRSシステム設定
struct DRSConfig {
    // 安全性制約
    double min_speed_for_activation = 15.0;   // [m/s] 最低速度 (54km/h)
    double max_lateral_accel = 0.5 * 9.81;    // [m/s²] 最大横G (約5 m/s²)
    double max_steering_angle = 0.2;         // [rad] 最大舵角 (約12°)

    // 遷移特性
    double activation_delay = 0.1;            // [s] 起動遅延
    double deactivation_delay = 0.0;          // [s] 停止遅延（即座）

    // 空力効果（係数比率）
    double cl_ratio_active = 0.4;            // DRS ON時のCL倍率 (1.5/2.8)
    double cd_ratio_active = 0.6;            // DRS ON時のCD倍率 (0.70/0.95)
};

/// DRS制御システム
class DRSSystem {
public:
    explicit DRSSystem(const DRSConfig& config = DRSConfig())
        : config_(config), state_(State::INACTIVE), timer_(0.0) {}

    /// ドライバー入力と車両状態からDRS状態を更新
    /// @param drs_button ドライバーのDRSボタン入力 (true=起動要求)
    /// @param speed 車速 [m/s] = sqrt(vx²+vy²)
    /// @param vx 縦速度 [m/s]（横加速度推定に使用）
    /// @param yaw_rate ヨーレート [rad/s]（横加速度推定に使用）
    /// @param steering_angle ステアリング角 [rad]
    /// @param dt タイムステップ [s]
    /// @return DRS起動中ならtrue
    bool update(bool drs_button, double speed, double vx, double yaw_rate,
                double steering_angle, double dt) {
        // 安全性チェック
        bool safety_ok = can_activate(speed, vx, yaw_rate, steering_angle);

        // 状態遷移
        switch (state_) {
            case State::INACTIVE:
                if (drs_button && safety_ok) {
                    state_ = State::ACTIVATING;
                    timer_ = 0.0;
                }
                break;

            case State::ACTIVATING:
                if (!drs_button || !safety_ok) {
                    // ボタン解除または安全性違反で即座にキャンセル
                    state_ = State::INACTIVE;
                    timer_ = 0.0;
                } else {
                    timer_ += dt;
                    if (timer_ >= config_.activation_delay) {
                        state_ = State::ACTIVE;
                        timer_ = 0.0;
                        stats_.activation_count++;
                    }
                }
                break;

            case State::ACTIVE:
                if (!drs_button || !safety_ok) {
                    // 即座に停止
                    state_ = State::INACTIVE;
                    timer_ = 0.0;
                } else {
                    // 統計更新
                    stats_.total_active_time += dt;
                    // 移動平均の更新（簡易版）
                    if (stats_.total_active_time > 0.0) {
                        stats_.avg_speed_when_active =
                            (stats_.avg_speed_when_active * (stats_.total_active_time - dt) + speed * dt)
                            / stats_.total_active_time;
                    }
                }
                break;

            case State::DEACTIVATING:
                // 現在未使用（即座停止なので）
                state_ = State::INACTIVE;
                break;
        }

        return is_active();
    }

    /// 現在の状態取得
    bool is_active() const {
        return state_ == State::ACTIVE;
    }

    /// 統計情報
    struct Stats {
        double total_active_time = 0.0;    // 累計起動時間 [s]
        int activation_count = 0;          // 起動回数
        double avg_speed_when_active = 0.0; // 起動時平均速度 [m/s]
    };

    /// 統計情報取得
    Stats get_stats() const {
        return stats_;
    }

    /// 設定取得
    const DRSConfig& config() const {
        return config_;
    }

    /// リセット（状態と統計をクリア）
    void reset() {
        state_ = State::INACTIVE;
        timer_ = 0.0;
        stats_ = Stats();
    }

private:
    enum class State {
        INACTIVE,       // 非起動
        ACTIVATING,     // 起動中（遅延待ち）
        ACTIVE,         // 起動済み
        DEACTIVATING    // 停止中（現在未使用）
    };

    DRSConfig config_;
    State state_;
    double timer_;
    Stats stats_;

    /// 安全性チェック（3層）
    bool can_activate(double speed, double vx, double yaw_rate,
                      double steering_angle) const {
        // 1. 速度チェック
        if (speed < config_.min_speed_for_activation) {
            return false;
        }

        // 2. 横加速度チェック（簡易推定: |vx * r|）
        double lateral_accel = std::abs(vx * yaw_rate);
        if (lateral_accel > config_.max_lateral_accel) {
            return false;
        }

        // 3. ステアリング角チェック
        if (std::abs(steering_angle) > config_.max_steering_angle) {
            return false;
        }

        return true;
    }
};

} // namespace vehicle
} // namespace fsae
