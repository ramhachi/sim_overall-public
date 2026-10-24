#pragma once
/// @file analytic_safety_filter.hpp
/// @brief Constant-time safety projection for high-level driver commands.
///
/// This layer is deliberately independent of the control law.  It projects a
/// requested tire angle and speed onto steering-rate, track-margin and local
/// stability envelopes without a numerical optimizer.  It is an operational
/// safety governor, not a formal CBF or friction-circle proof.

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsae::driver {

struct AnalyticSafetyConfig {
    double max_tire_angle = 0.35;       // [rad], driver convention: right positive
    double max_tire_rate = 2.8;         // [rad/s]
    double soft_track_margin = 0.90;    // [m]
    double hard_track_margin = 0.30;    // [m]
    double margin_speed_cap = 7.0;      // [m/s]
    double max_sideslip = 0.16;         // [rad]
    double max_yaw_error = 0.55;        // [rad/s]
    double stability_speed_cap = 8.0;   // [m/s]
    double fallback_speed = 1.0;        // [m/s]
};

struct AnalyticSafetyRequest {
    double target_tire_angle = 0.0;     // [rad], right positive
    double target_speed = 0.0;          // [m/s]
    double base_target_speed = 0.0;     // [m/s]
};

struct AnalyticSafetyObservation {
    double lateral_error = 0.0;         // [m], left positive
    double heading_error = 0.0;         // [rad], CCW positive
    double track_margin = 0.0;          // [m], CG-to-boundary margin
    double speed = 0.0;                 // [m/s]
    double sideslip = 0.0;              // [rad]
    double yaw_rate_error = 0.0;        // [rad/s]
};

struct AnalyticSafetyOutput {
    double target_tire_angle = 0.0;
    double target_speed = 0.0;
    double track_margin_speed_cap = std::numeric_limits<double>::infinity();
    double stability_speed_cap = std::numeric_limits<double>::infinity();
    bool track_recovery_active = false;
    bool finite_fallback_active = false;
};

class AnalyticSafetyFilter {
public:
    explicit AnalyticSafetyFilter(const AnalyticSafetyConfig& config)
        : config_(config) {}

    AnalyticSafetyOutput apply(
        const AnalyticSafetyRequest& request,
        const AnalyticSafetyObservation& observation,
        double dt)
    {
        AnalyticSafetyOutput output;
        if (!finite(request) || !finite(observation) || !std::isfinite(dt) || dt <= 0.0) {
            output.target_tire_angle = rate_limit(0.0, std::max(dt, 0.001));
            output.target_speed = config_.fallback_speed;
            output.finite_fallback_active = true;
            previous_tire_angle_ = output.target_tire_angle;
            return output;
        }

        const double outward_rate = observation.lateral_error == 0.0
            ? 0.0
            : (observation.lateral_error > 0.0 ? 1.0 : -1.0)
                * observation.speed * std::sin(observation.heading_error);
        double target_angle = request.target_tire_angle;
        if (observation.track_margin < config_.soft_track_margin && outward_rate > 0.0) {
            const double risk = std::clamp(
                (config_.soft_track_margin - observation.track_margin)
                    / std::max(config_.soft_track_margin - config_.hard_track_margin, 1e-6),
                0.0,
                1.0);
            // Positive lateral error is left of center; positive driver angle
            // steers right and therefore points back toward the centerline.
            const double recovery = std::clamp(
                0.22 * observation.lateral_error,
                -config_.max_tire_angle,
                config_.max_tire_angle);
            target_angle = (1.0 - risk) * target_angle + risk * recovery;
            output.track_recovery_active = true;
        }
        if (observation.track_margin <= config_.hard_track_margin) {
            target_angle = std::clamp(
                0.35 * observation.lateral_error,
                -config_.max_tire_angle,
                config_.max_tire_angle);
            output.track_recovery_active = true;
        }
        output.target_tire_angle = rate_limit(target_angle, dt);

        if (observation.track_margin < config_.soft_track_margin && outward_rate > 0.0) {
            const double ratio = std::clamp(
                (observation.track_margin - config_.hard_track_margin)
                    / std::max(config_.soft_track_margin - config_.hard_track_margin, 1e-6),
                0.0,
                1.0);
            output.track_margin_speed_cap = config_.margin_speed_cap
                + ratio * ratio
                    * (std::max(request.base_target_speed, config_.margin_speed_cap)
                       - config_.margin_speed_cap);
        }
        if (std::abs(observation.sideslip) > config_.max_sideslip
                || std::abs(observation.yaw_rate_error) > config_.max_yaw_error) {
            output.stability_speed_cap = config_.stability_speed_cap;
        }
        output.target_speed = std::max(
            config_.fallback_speed,
            std::min({request.target_speed,
                      output.track_margin_speed_cap,
                      output.stability_speed_cap}));
        previous_tire_angle_ = output.target_tire_angle;
        return output;
    }

    void reset() { previous_tire_angle_ = 0.0; }

private:
    double rate_limit(double target, double dt) const {
        const double max_step = std::max(config_.max_tire_rate, 0.0) * dt;
        return std::clamp(
            std::clamp(target, previous_tire_angle_ - max_step, previous_tire_angle_ + max_step),
            -std::max(config_.max_tire_angle, 0.0),
            std::max(config_.max_tire_angle, 0.0));
    }

    static bool finite(const AnalyticSafetyRequest& value) {
        return std::isfinite(value.target_tire_angle)
            && std::isfinite(value.target_speed)
            && std::isfinite(value.base_target_speed);
    }

    static bool finite(const AnalyticSafetyObservation& value) {
        return std::isfinite(value.lateral_error)
            && std::isfinite(value.heading_error)
            && std::isfinite(value.track_margin)
            && std::isfinite(value.speed)
            && std::isfinite(value.sideslip)
            && std::isfinite(value.yaw_rate_error);
    }

    AnalyticSafetyConfig config_;
    double previous_tire_angle_ = 0.0;
};

} // namespace fsae::driver
