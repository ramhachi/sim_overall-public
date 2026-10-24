#pragma once
/// @file open_loop_driver.hpp
/// @brief オープンループドライバーモデル（Level 0）
///
/// 時系列で定義された入力パターンを補間して出力する。
/// サブシステム検証やステップ応答テストに使用。

#include "../vehicle/vehicle_state.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

namespace fsae::driver {

/// 時系列データポイント
struct TimeSeriesPoint {
    double time;        // [s]
    double steering;    // [rad]
    double throttle;    // [0-1]
    double brake;       // [0-1]
    int gear;           // ギア段
};

/// オープンループドライバー基底クラス
class OpenLoopDriver {
public:
    virtual ~OpenLoopDriver() = default;

    /// 時刻tでの入力を取得
    /// @param t 現在時刻 [s]
    /// @return ドライバー入力
    virtual vehicle::DriverInput get_input(double t) const = 0;

    /// マニューバの総時間
    virtual double duration() const = 0;
};

/// 線形補間ユーティリティ
inline double lerp(double t, double t0, double t1, double v0, double v1) {
    if (t <= t0) return v0;
    if (t >= t1) return v1;
    double alpha = (t - t0) / (t1 - t0);
    return v0 + alpha * (v1 - v0);
}

/// ステップステア入力
class StepSteerDriver : public OpenLoopDriver {
public:
    /// @param step_time ステップ開始時刻 [s]
    /// @param steer_angle ステップ後の舵角 [rad]
    /// @param duration_s 総時間 [s]
    /// @param initial_speed 初速 [m/s]（スロットル設定用）
    StepSteerDriver(double step_time = 1.0,
                    double steer_angle = 0.1,
                    double duration_s = 10.0,
                    double initial_speed = 15.0)
        : step_time_(step_time)
        , steer_angle_(steer_angle)
        , duration_(duration_s)
        , initial_speed_(initial_speed)
    {}

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        // ステップステア
        input.steering = (t >= step_time_) ? steer_angle_ : 0.0;

        // 一定スロットル（速度維持）
        input.throttle = 0.3;
        input.brake = 0.0;
        input.gear = 3;

        return input;
    }

    double duration() const override { return duration_; }

private:
    double step_time_;
    double steer_angle_;
    double duration_;
    double initial_speed_;
};

/// ランプステア入力
class RampSteerDriver : public OpenLoopDriver {
public:
    /// @param ramp_start ランプ開始時刻 [s]
    /// @param ramp_duration ランプ時間 [s]
    /// @param final_steer_angle 最終舵角 [rad]
    /// @param total_duration 総時間 [s]
    RampSteerDriver(double ramp_start = 1.0,
                    double ramp_duration = 2.0,
                    double final_steer_angle = 0.15,
                    double total_duration = 10.0)
        : ramp_start_(ramp_start)
        , ramp_end_(ramp_start + ramp_duration)
        , final_steer_(final_steer_angle)
        , duration_(total_duration)
    {}

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        // ランプステア
        if (t < ramp_start_) {
            input.steering = 0.0;
        } else if (t < ramp_end_) {
            double alpha = (t - ramp_start_) / (ramp_end_ - ramp_start_);
            input.steering = alpha * final_steer_;
        } else {
            input.steering = final_steer_;
        }

        input.throttle = 0.3;
        input.brake = 0.0;
        input.gear = 3;

        return input;
    }

    double duration() const override { return duration_; }

private:
    double ramp_start_;
    double ramp_end_;
    double final_steer_;
    double duration_;
};

/// サイン波ステア入力（ウィーブマニューバ）
class SineSteerDriver : public OpenLoopDriver {
public:
    /// @param frequency 周波数 [Hz]
    /// @param amplitude 振幅 [rad]
    /// @param duration_s 総時間 [s]
    SineSteerDriver(double frequency = 0.5,
                    double amplitude = 0.1,
                    double duration_s = 10.0)
        : frequency_(frequency)
        , amplitude_(amplitude)
        , duration_(duration_s)
    {}

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        // サイン波ステア
        constexpr double PI = 3.14159265358979323846;
        input.steering = amplitude_ * std::sin(2.0 * PI * frequency_ * t);

        input.throttle = 0.3;
        input.brake = 0.0;
        input.gear = 3;

        return input;
    }

    double duration() const override { return duration_; }

private:
    double frequency_;
    double amplitude_;
    double duration_;
};

/// ダブルレーンチェンジ（ISO 3888-1 簡略版）
class DoubleLaneChangeDriver : public OpenLoopDriver {
public:
    /// @param lane_change_time レーンチェンジ開始時刻 [s]
    /// @param steer_amplitude 舵角振幅 [rad]
    /// @param duration_s 総時間 [s]
    DoubleLaneChangeDriver(double lane_change_time = 2.0,
                           double steer_amplitude = 0.12,
                           double duration_s = 10.0)
        : t_start_(lane_change_time)
        , amplitude_(steer_amplitude)
        , duration_(duration_s)
    {}

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        // ダブルレーンチェンジパターン（台形波近似）
        double t_rel = t - t_start_;

        if (t_rel < 0.0) {
            // 直進
            input.steering = 0.0;
        } else if (t_rel < 1.0) {
            // 左へレーンチェンジ
            input.steering = amplitude_ * t_rel;
        } else if (t_rel < 2.0) {
            // 保持
            input.steering = amplitude_;
        } else if (t_rel < 3.0) {
            // 中央復帰
            double alpha = (t_rel - 2.0);
            input.steering = amplitude_ * (1.0 - 2.0 * alpha);
        } else if (t_rel < 4.0) {
            // 右へレーンチェンジ
            input.steering = -amplitude_;
        } else if (t_rel < 5.0) {
            // 元のレーンへ復帰
            double alpha = (t_rel - 4.0);
            input.steering = -amplitude_ * (1.0 - alpha);
        } else {
            // 直進
            input.steering = 0.0;
        }

        input.throttle = 0.4;
        input.brake = 0.0;
        input.gear = 3;

        return input;
    }

    double duration() const override { return duration_; }

private:
    double t_start_;
    double amplitude_;
    double duration_;
};

/// 加速・減速テスト
class AccelBrakeDriver : public OpenLoopDriver {
public:
    /// @param accel_duration 加速時間 [s]
    /// @param brake_start ブレーキ開始時刻 [s]
    /// @param total_duration 総時間 [s]
    AccelBrakeDriver(double accel_duration = 5.0,
                     double brake_start = 6.0,
                     double total_duration = 10.0)
        : accel_duration_(accel_duration)
        , brake_start_(brake_start)
        , duration_(total_duration)
    {}

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        input.steering = 0.0;

        if (t < accel_duration_) {
            // フル加速
            input.throttle = 1.0;
            input.brake = 0.0;
            input.gear = 3;
        } else if (t < brake_start_) {
            // 慣性走行
            input.throttle = 0.0;
            input.brake = 0.0;
            input.gear = 3;
        } else {
            // ブレーキ
            input.throttle = 0.0;
            input.brake = 0.8;
            input.gear = 3;
        }

        return input;
    }

    double duration() const override { return duration_; }

private:
    double accel_duration_;
    double brake_start_;
    double duration_;
};

/// カスタム時系列ドライバー
class TimeSeriesDriver : public OpenLoopDriver {
public:
    /// @param timeseries 時系列データポイント（時刻昇順にソート済みであること）
    explicit TimeSeriesDriver(std::vector<TimeSeriesPoint> timeseries)
        : timeseries_(std::move(timeseries))
    {
        // 時刻でソート
        std::sort(timeseries_.begin(), timeseries_.end(),
                  [](const TimeSeriesPoint& a, const TimeSeriesPoint& b) {
                      return a.time < b.time;
                  });
    }

    vehicle::DriverInput get_input(double t) const override {
        vehicle::DriverInput input;

        if (timeseries_.empty()) {
            return input;  // デフォルト（ゼロ）
        }

        // 範囲外の場合は最初/最後の値を使用
        if (t <= timeseries_.front().time) {
            const auto& pt = timeseries_.front();
            input.steering = pt.steering;
            input.throttle = pt.throttle;
            input.brake = pt.brake;
            input.gear = pt.gear;
            return input;
        }

        if (t >= timeseries_.back().time) {
            const auto& pt = timeseries_.back();
            input.steering = pt.steering;
            input.throttle = pt.throttle;
            input.brake = pt.brake;
            input.gear = pt.gear;
            return input;
        }

        // 線形補間
        for (size_t i = 0; i < timeseries_.size() - 1; ++i) {
            const auto& p0 = timeseries_[i];
            const auto& p1 = timeseries_[i + 1];

            if (t >= p0.time && t < p1.time) {
                input.steering = lerp(t, p0.time, p1.time, p0.steering, p1.steering);
                input.throttle = lerp(t, p0.time, p1.time, p0.throttle, p1.throttle);
                input.brake = lerp(t, p0.time, p1.time, p0.brake, p1.brake);
                input.gear = (t < (p0.time + p1.time) / 2.0) ? p0.gear : p1.gear;
                break;
            }
        }

        return input;
    }

    double duration() const override {
        return timeseries_.empty() ? 0.0 : timeseries_.back().time;
    }

private:
    std::vector<TimeSeriesPoint> timeseries_;
};

} // namespace fsae::driver
