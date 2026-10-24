#pragma once
/// @file attack_speed_planner.hpp
/// @brief Fixed-course speed planning for the attack driver.

#include "../laptime/velocity_profile.hpp"
#include "../track/track_model.hpp"
#include "../vehicle/vehicle_params.hpp"
#include "track_phase_analyzer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace fsae::driver {

enum class AttackSpeedLimitReason {
    Profile,
    Phase,
    Curvature,
    Steering,
    Margin,
    ExternalCap,
    Braking,
    Acceleration,
};

inline const char* attack_speed_limit_reason_name(AttackSpeedLimitReason reason) {
    switch (reason) {
        case AttackSpeedLimitReason::Profile:
            return "profile";
        case AttackSpeedLimitReason::Phase:
            return "phase";
        case AttackSpeedLimitReason::Curvature:
            return "curvature";
        case AttackSpeedLimitReason::Steering:
            return "steering";
        case AttackSpeedLimitReason::Margin:
            return "margin";
        case AttackSpeedLimitReason::ExternalCap:
            return "external_cap";
        case AttackSpeedLimitReason::Braking:
            return "braking";
        case AttackSpeedLimitReason::Acceleration:
            return "acceleration";
    }
    return "unknown";
}

struct AttackSpeedPlannerConfig {
    double sample_ds = 1.0;                  // [m]
    double min_speed = 2.0;                  // [m/s]
    double max_speed = 30.0;                 // [m/s]
    double lateral_safety = 0.78;            // [-], static plan; local MPC still applies dynamic caps
    double braking_safety = 0.78;            // [-]
    double accel_limit = 5.5;                // [m/s^2], reproducible open-loop cap
    double understeer_gradient = 0.002;      // [rad/(m/s^2)]
    double narrow_margin_threshold = 1.05;   // [m]
    double narrow_margin_speed = 8.0;        // [m/s]
    // Plant δ_ss(v, κ) table (Issue #27); when set it replaces L·κ + K_us·v²·κ in the steering cap.
    std::shared_ptr<const laptime::GGVMap> steady_state;
    std::vector<std::array<double, 3>> external_speed_caps; // {s [m], speed [m/s], radius [m]}
    std::vector<std::array<double, 3>> external_speed_windows; // {s_start [m], s_end [m], speed [m/s]}
};

inline double attack_tire_lateral_capacity(
    const vehicle::VehicleParams& vehicle_params,
    double fz)
{
    const double load = std::max(fz, 1.0);
    const double mu = std::max(
        vehicle_params.tire.mu_y_max
            + vehicle_params.tire.dmu_dFz * (load - vehicle_params.tire.Fz_nom),
        0.1);
    return mu * load;
}

inline double attack_front_lateral_transfer_fraction(
    const vehicle::VehicleParams& vehicle_params)
{
    const double front_motion_ratio =
        0.5 * (vehicle_params.suspension.motion_ratio[0]
             + vehicle_params.suspension.motion_ratio[1]);
    const double rear_motion_ratio =
        0.5 * (vehicle_params.suspension.motion_ratio[2]
             + vehicle_params.suspension.motion_ratio[3]);
    const double front_roll_stiffness =
        0.5 * (vehicle_params.suspension.k_spring[0]
             + vehicle_params.suspension.k_spring[1])
        * front_motion_ratio * front_motion_ratio
        * 0.5 * vehicle_params.mass.track_f * vehicle_params.mass.track_f
        + vehicle_params.suspension.k_arb_f;
    const double rear_roll_stiffness =
        0.5 * (vehicle_params.suspension.k_spring[2]
             + vehicle_params.suspension.k_spring[3])
        * rear_motion_ratio * rear_motion_ratio
        * 0.5 * vehicle_params.mass.track_r * vehicle_params.mass.track_r
        + vehicle_params.suspension.k_arb_r;
    const double total = front_roll_stiffness + rear_roll_stiffness;
    return total > 1e-6
        ? std::clamp(front_roll_stiffness / total, 0.05, 0.95)
        : 0.50;
}

inline double attack_lateral_accel_limit(
    const vehicle::VehicleParams& vehicle_params,
    double speed)
{
    const double mass = std::max(vehicle_params.mass.m_total, 1e-6);
    const double rho = vehicle_params.aero.rho > 0.1 ? vehicle_params.aero.rho : 1.225;
    const double downforce =
        0.5 * rho * std::max(vehicle_params.aero.CL_A, 0.0) * speed * speed;
    const double static_weight = mass * 9.81;
    double ay = std::max(vehicle_params.tire.mu_y_max, 0.1)
        * (static_weight + downforce)
        / mass;

    if (vehicle_params.tire.model_type == "tmeasy") {
        const double wheelbase = std::max(vehicle_params.mass.wheelbase, 1e-6);
        const double front_static_fraction =
            std::clamp(vehicle_params.mass.b / wheelbase, 0.05, 0.95);
        const double front_aero_balance =
            std::clamp(vehicle_params.aero.aero_balance, 0.05, 0.95);
        const double front_total =
            static_weight * front_static_fraction + downforce * front_aero_balance;
        const double rear_total = static_weight + downforce - front_total;
        const double transfer_fraction =
            attack_front_lateral_transfer_fraction(vehicle_params);
        const double track_front = std::max(vehicle_params.mass.track_f, 0.1);
        const double track_rear = std::max(vehicle_params.mass.track_r, 0.1);
        const double h_cg = std::max(vehicle_params.mass.h_cg, 0.0);

        for (int iter = 0; iter < 6; ++iter) {
            const double roll_moment = mass * std::abs(ay) * h_cg;
            const double front_transfer = std::min(
                roll_moment * transfer_fraction / track_front,
                std::max(0.5 * front_total - 1.0, 0.0));
            const double rear_transfer = std::min(
                roll_moment * (1.0 - transfer_fraction) / track_rear,
                std::max(0.5 * rear_total - 1.0, 0.0));
            const double lateral_capacity =
                attack_tire_lateral_capacity(vehicle_params, 0.5 * front_total + front_transfer)
              + attack_tire_lateral_capacity(vehicle_params, 0.5 * front_total - front_transfer)
              + attack_tire_lateral_capacity(vehicle_params, 0.5 * rear_total + rear_transfer)
              + attack_tire_lateral_capacity(vehicle_params, 0.5 * rear_total - rear_transfer);
            ay = lateral_capacity / mass;
        }
    }

    return std::clamp(ay, 2.5, 35.0);
}

inline double attack_vehicle_lateral_safety_scale(
    const vehicle::VehicleParams& vehicle_params)
{
    constexpr double kReferenceMassKg = 282.0;
    const double mass = std::max(vehicle_params.mass.m_total, 1e-6);
    const double mass_scale = std::pow(kReferenceMassKg / mass, 0.675);
    return std::clamp(mass_scale, 0.82, 1.08);
}

struct AttackSpeedSample {
    double s = 0.0;
    double profile_speed = 0.0;
    double phase_speed_cap = std::numeric_limits<double>::infinity();
    double curvature_speed_cap = std::numeric_limits<double>::infinity();
    double steering_speed_cap = std::numeric_limits<double>::infinity();
    double margin_speed_cap = std::numeric_limits<double>::infinity();
    double external_speed_cap = std::numeric_limits<double>::infinity();
    double external_window_speed_cap = std::numeric_limits<double>::infinity();
    double planned_speed = 0.0;
    AttackSpeedLimitReason limiting_reason = AttackSpeedLimitReason::Profile;
};

class AttackSpeedProfile {
public:
    AttackSpeedProfile() = default;

    AttackSpeedProfile(std::vector<AttackSpeedSample> samples, double length, bool closed_loop)
        : samples_(std::move(samples))
        , length_(std::max(0.0, length))
        , closed_loop_(closed_loop)
    {}

    bool empty() const { return samples_.empty(); }
    double length() const { return length_; }
    const std::vector<AttackSpeedSample>& samples() const { return samples_; }

    const AttackSpeedSample& sample_at(double s) const {
        static const AttackSpeedSample kFallback{};
        if (samples_.empty()) {
            return kFallback;
        }
        const double query_s = normalize_s(s);
        auto it = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            query_s,
            [](const AttackSpeedSample& sample, double value) {
                return sample.s < value;
            });
        if (it == samples_.begin()) {
            return *it;
        }
        if (it == samples_.end()) {
            return samples_.back();
        }
        const auto prev = std::prev(it);
        return (query_s - prev->s) <= (it->s - query_s) ? *prev : *it;
    }

    double planned_speed_at(double s) const {
        if (samples_.empty()) {
            return 0.0;
        }
        const double query_s = normalize_s(s);
        auto it = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            query_s,
            [](const AttackSpeedSample& sample, double value) {
                return sample.s < value;
            });
        if (it == samples_.begin()) {
            return it->planned_speed;
        }
        if (it == samples_.end()) {
            return samples_.back().planned_speed;
        }
        const auto prev = std::prev(it);
        const double span = std::max(it->s - prev->s, 1e-9);
        const double alpha = std::clamp((query_s - prev->s) / span, 0.0, 1.0);
        return prev->planned_speed
            + alpha * (it->planned_speed - prev->planned_speed);
    }

    double preview_min_planned_speed(double s, double preview_distance, double step = 1.0) const {
        if (samples_.empty()) {
            return 0.0;
        }
        double speed = planned_speed_at(s);
        const double distance = std::max(preview_distance, 0.0);
        const double ds = std::clamp(step, 0.25, 5.0);
        for (double preview_s = ds; preview_s <= distance + 1e-12; preview_s += ds) {
            speed = std::min(speed, planned_speed_at(s + preview_s));
        }
        return speed;
    }

private:
    double normalize_s(double s) const {
        if (length_ <= 1e-9) {
            return std::max(0.0, s);
        }
        if (!closed_loop_) {
            return std::clamp(s, 0.0, length_);
        }
        double wrapped = std::fmod(s, length_);
        if (wrapped < 0.0) {
            wrapped += length_;
        }
        return wrapped;
    }

    std::vector<AttackSpeedSample> samples_;
    double length_ = 0.0;
    bool closed_loop_ = false;
};

class AttackSpeedPlanner {
public:
    static AttackSpeedProfile plan(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const vehicle::VehicleParams& vehicle_params,
        const TrackPhaseProfile& phase_profile,
        const AttackSpeedPlannerConfig& config = AttackSpeedPlannerConfig())
    {
        const double length = track.length();
        if (length <= 1e-9) {
            return AttackSpeedProfile({}, 0.0, false);
        }

        const bool closed_loop = track.is_closed_loop();
        AttackSpeedPlannerConfig effective_config = config;
        sanitize_external_speed_caps(effective_config, length, closed_loop);
        sanitize_external_speed_windows(effective_config, length, closed_loop);

        const double ds = std::clamp(effective_config.sample_ds, 0.25, 5.0);
        std::vector<AttackSpeedSample> samples;
        samples.reserve(static_cast<std::size_t>(std::ceil(length / ds)) + 1);
        for (double s = 0.0; s <= length + 1e-9; s += ds) {
            AttackSpeedSample sample;
            sample.s = std::min(s, length);
            sample.profile_speed = velocity_at(velocity_profile, sample.s);
            sample.phase_speed_cap = phase_cap_at(phase_profile, sample.s, sample.profile_speed);
            sample.curvature_speed_cap =
                curvature_speed_cap(track, sample.s, vehicle_params, effective_config);
            sample.steering_speed_cap =
                steering_angle_speed_cap(track, sample.s, vehicle_params, effective_config);
            sample.margin_speed_cap =
                margin_speed_cap(track, sample.s, sample.profile_speed, effective_config);
            sample.external_speed_cap =
                external_speed_cap_at(effective_config, length, closed_loop, sample.s);
            sample.external_window_speed_cap =
                external_speed_window_cap_at(effective_config, length, closed_loop, sample.s);

            sample.planned_speed = std::isfinite(sample.phase_speed_cap) && sample.phase_speed_cap > 0.0
                ? sample.phase_speed_cap
                : sample.profile_speed;
            sample.limiting_reason = sample.planned_speed == sample.profile_speed
                ? AttackSpeedLimitReason::Profile
                : AttackSpeedLimitReason::Phase;
            apply_point_cap(sample.curvature_speed_cap, AttackSpeedLimitReason::Curvature, sample);
            apply_point_cap(sample.steering_speed_cap, AttackSpeedLimitReason::Steering, sample);
            apply_point_cap(sample.margin_speed_cap, AttackSpeedLimitReason::Margin, sample);
            apply_point_cap(sample.external_speed_cap, AttackSpeedLimitReason::ExternalCap, sample);
            apply_point_cap(sample.external_window_speed_cap, AttackSpeedLimitReason::ExternalCap, sample);
            sample.planned_speed = std::clamp(
                sample.planned_speed,
                effective_config.min_speed,
                effective_config.max_speed);
            samples.push_back(sample);
        }

        apply_longitudinal_feasibility(samples, vehicle_params, effective_config);
        return AttackSpeedProfile(std::move(samples), length, closed_loop);
    }

private:
    static void sanitize_external_speed_caps(
        AttackSpeedPlannerConfig& config,
        double track_length,
        bool closed_loop)
    {
        std::vector<std::array<double, 3>> sanitized;
        sanitized.reserve(config.external_speed_caps.size());
        for (auto cap : config.external_speed_caps) {
            if (!std::isfinite(cap[0]) || !std::isfinite(cap[1]) || !std::isfinite(cap[2])) {
                continue;
            }
            if (cap[1] <= 0.0 || cap[2] <= 0.0) {
                continue;
            }
            if (track_length > 1e-9 && closed_loop) {
                cap[0] = normalize_track_position(cap[0], track_length, true);
                cap[2] = std::min(cap[2], 0.5 * track_length);
            } else if (track_length > 1e-9) {
                if (cap[0] + cap[2] < 0.0 || cap[0] - cap[2] > track_length) {
                    continue;
                }
                cap[0] = std::clamp(cap[0], 0.0, track_length);
                cap[2] = std::min(cap[2], track_length);
            } else {
                cap[0] = std::max(cap[0], 0.0);
            }
            cap[1] = std::clamp(cap[1], config.min_speed, config.max_speed);
            sanitized.push_back(cap);
        }
        std::sort(
            sanitized.begin(),
            sanitized.end(),
            [](const auto& a, const auto& b) { return a[0] < b[0]; });
        config.external_speed_caps = std::move(sanitized);
    }

    static void sanitize_external_speed_windows(
        AttackSpeedPlannerConfig& config,
        double track_length,
        bool closed_loop)
    {
        std::vector<std::array<double, 3>> sanitized;
        sanitized.reserve(config.external_speed_windows.size());
        for (auto window : config.external_speed_windows) {
            if (!std::isfinite(window[0])
                || !std::isfinite(window[1])
                || !std::isfinite(window[2])) {
                continue;
            }
            if (window[2] <= 0.0) {
                continue;
            }
            if (track_length > 1e-9 && closed_loop) {
                window[0] = normalize_track_position(window[0], track_length, true);
                window[1] = normalize_track_position(window[1], track_length, true);
            } else if (track_length > 1e-9) {
                const double start = std::min(window[0], window[1]);
                const double end = std::max(window[0], window[1]);
                if (end < 0.0 || start > track_length) {
                    continue;
                }
                window[0] = std::clamp(start, 0.0, track_length);
                window[1] = std::clamp(end, 0.0, track_length);
                if (window[1] <= window[0] + 1e-9) {
                    continue;
                }
            } else {
                window[0] = std::max(window[0], 0.0);
                window[1] = std::max(window[1], 0.0);
            }
            window[2] = std::clamp(window[2], config.min_speed, config.max_speed);
            sanitized.push_back(window);
        }
        std::sort(
            sanitized.begin(),
            sanitized.end(),
            [](const auto& a, const auto& b) { return a[0] < b[0]; });
        config.external_speed_windows = std::move(sanitized);
    }

    static double track_distance(double a, double b, double length, bool closed_loop) {
        const double direct = std::abs(a - b);
        if (!closed_loop || length <= 1e-9) {
            return direct;
        }
        return std::min(direct, std::max(0.0, length - direct));
    }

    static double normalize_track_position(double s, double length, bool closed_loop) {
        if (length <= 1e-9) {
            return std::max(0.0, s);
        }
        if (!closed_loop) {
            return std::clamp(s, 0.0, length);
        }
        double wrapped = std::fmod(s, length);
        if (wrapped < 0.0) {
            wrapped += length;
        }
        return wrapped;
    }

    static double external_speed_cap_at(
        const AttackSpeedPlannerConfig& config,
        double track_length,
        bool closed_loop,
        double s)
    {
        double cap_speed = std::numeric_limits<double>::infinity();
        for (const auto& cap : config.external_speed_caps) {
            const double distance = track_distance(s, cap[0], track_length, closed_loop);
            if (distance <= cap[2]) {
                cap_speed = std::min(cap_speed, cap[1]);
            }
        }
        return cap_speed;
    }

    static bool window_contains_s(
        const std::array<double, 3>& window,
        double s,
        double track_length)
    {
        const double start = window[0];
        const double end = window[1];
        if (track_length <= 1e-9 || start <= end) {
            return s >= start && s <= end;
        }
        return s >= start || s <= end;
    }

    static double external_speed_window_cap_at(
        const AttackSpeedPlannerConfig& config,
        double track_length,
        bool closed_loop,
        double s)
    {
        const double query_s = normalize_track_position(s, track_length, closed_loop);
        double cap_speed = std::numeric_limits<double>::infinity();
        for (const auto& window : config.external_speed_windows) {
            if (window_contains_s(window, query_s, track_length)) {
                cap_speed = std::min(cap_speed, window[2]);
            }
        }
        return cap_speed;
    }

    static double velocity_at(
        const std::vector<laptime::VelocityPoint>& profile,
        double s)
    {
        if (profile.empty()) {
            return 12.0;
        }
        auto it = std::lower_bound(
            profile.begin(),
            profile.end(),
            s,
            [](const laptime::VelocityPoint& point, double value) {
                return point.s < value;
            });
        if (it == profile.begin()) {
            return it->v;
        }
        if (it == profile.end()) {
            return profile.back().v;
        }
        const auto prev = std::prev(it);
        const double span = std::max(it->s - prev->s, 1e-9);
        const double alpha = std::clamp((s - prev->s) / span, 0.0, 1.0);
        return prev->v + alpha * (it->v - prev->v);
    }

    static double phase_cap_at(
        const TrackPhaseProfile& phase_profile,
        double s,
        double fallback_speed)
    {
        if (phase_profile.empty()) {
            return fallback_speed;
        }
        const double cap = phase_profile.sample_at(s).static_speed_cap;
        return std::isfinite(cap) && cap > 0.0 ? cap : fallback_speed;
    }

    static double curvature_speed_cap(
        const track::CurvatureBasedTrack& track,
        double s,
        const vehicle::VehicleParams& vehicle_params,
        const AttackSpeedPlannerConfig& config)
    {
        const double abs_kappa = std::abs(track.get_curvature(s));
        if (abs_kappa <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        double cap_speed = std::clamp(config.max_speed, config.min_speed, config.max_speed);
        for (int iter = 0; iter < 6; ++iter) {
            const double ay_limit =
                attack_lateral_accel_limit(vehicle_params, cap_speed)
              * std::clamp(config.lateral_safety, 0.2, 1.0);
            cap_speed = std::sqrt(std::max(ay_limit / abs_kappa, 0.0));
        }
        return std::clamp(
            cap_speed,
            config.min_speed,
            config.max_speed);
    }

    static double steering_angle_speed_cap(
        const track::CurvatureBasedTrack& track,
        double s,
        const vehicle::VehicleParams& vehicle_params,
        const AttackSpeedPlannerConfig& config)
    {
        const double abs_kappa = std::abs(track.get_curvature(s));
        if (abs_kappa <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        const double wheelbase = std::max(vehicle_params.mass.wheelbase, 1e-6);
        const double max_angle = std::max(vehicle_params.steering.max_angle, 1e-6);
        if (config.steady_state && config.steady_state->has_steady_state()) {
            const auto& table = *config.steady_state;
            auto reachable = [&](double v) { return table.steer_angle(abs_kappa, v) <= max_angle; };
            if (!reachable(config.min_speed)) return config.min_speed;
            if (reachable(config.max_speed)) return config.max_speed;
            double lo = config.min_speed, hi = config.max_speed;
            for (int it = 0; it < 40; ++it) {
                const double mid = 0.5 * (lo + hi);
                (reachable(mid) ? lo : hi) = mid;
            }
            return lo;
        }
        const double understeer = std::max(config.understeer_gradient, 1e-9);
        const double remaining = max_angle / abs_kappa - wheelbase;
        if (remaining <= 0.0) {
            return config.min_speed;
        }
        return std::clamp(
            std::sqrt(remaining / understeer),
            config.min_speed,
            config.max_speed);
    }

    static double margin_speed_cap(
        const track::CurvatureBasedTrack& track,
        double s,
        double profile_speed,
        const AttackSpeedPlannerConfig& config)
    {
        const auto point = track.get_point(s);
        const double margin = std::min(point.width_left, point.width_right);
        if (margin >= config.narrow_margin_threshold) {
            return std::numeric_limits<double>::infinity();
        }
        const double ratio = std::clamp(
            margin / std::max(config.narrow_margin_threshold, 1e-6),
            0.0,
            1.0);
        return std::clamp(
            config.narrow_margin_speed
                + (profile_speed - config.narrow_margin_speed) * ratio * ratio,
            config.min_speed,
            config.max_speed);
    }

    static void apply_point_cap(
        double cap,
        AttackSpeedLimitReason reason,
        AttackSpeedSample& sample)
    {
        if (std::isfinite(cap) && cap < sample.planned_speed) {
            sample.planned_speed = cap;
            sample.limiting_reason = reason;
        }
    }

    static void apply_longitudinal_feasibility(
        std::vector<AttackSpeedSample>& samples,
        const vehicle::VehicleParams& vehicle_params,
        const AttackSpeedPlannerConfig& config)
    {
        if (samples.size() < 2) {
            return;
        }
        const double brake_decel = std::max(vehicle_params.tire.mu_x_max, 0.1)
            * 9.81
            * std::clamp(config.braking_safety, 0.2, 1.0);
        const double accel = std::max(config.accel_limit, 0.1);

        for (int i = static_cast<int>(samples.size()) - 2; i >= 0; --i) {
            const double ds = std::max(samples[static_cast<std::size_t>(i + 1)].s
                    - samples[static_cast<std::size_t>(i)].s,
                1e-6);
            const double reachable = std::sqrt(
                std::max(
                    samples[static_cast<std::size_t>(i + 1)].planned_speed
                    * samples[static_cast<std::size_t>(i + 1)].planned_speed
                    + 2.0 * brake_decel * ds,
                    0.0));
            auto& sample = samples[static_cast<std::size_t>(i)];
            if (reachable < sample.planned_speed) {
                sample.planned_speed = std::max(reachable, config.min_speed);
                sample.limiting_reason = AttackSpeedLimitReason::Braking;
            }
        }

        for (std::size_t i = 1; i < samples.size(); ++i) {
            const double ds = std::max(samples[i].s - samples[i - 1].s, 1e-6);
            const double reachable = std::sqrt(
                std::max(
                    samples[i - 1].planned_speed * samples[i - 1].planned_speed
                    + 2.0 * accel * ds,
                    0.0));
            if (reachable < samples[i].planned_speed) {
                samples[i].planned_speed = std::max(reachable, config.min_speed);
                samples[i].limiting_reason = AttackSpeedLimitReason::Acceleration;
            }
        }
    }
};

} // namespace fsae::driver
