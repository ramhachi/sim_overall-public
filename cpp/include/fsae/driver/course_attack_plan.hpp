#pragma once
/// @file course_attack_plan.hpp
/// @brief Fixed-course attack reference plan for local MPC tracking.

#include "../laptime/velocity_profile.hpp"
#include "../track/track_model.hpp"
#include "../vehicle/vehicle_params.hpp"
#include "attack_speed_planner.hpp"
#include "line_planner.hpp"
#include "track_phase_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace fsae::driver {

struct CourseAttackSample {
    double s = 0.0;                       // [m]
    double profile_speed = 0.0;           // [m/s], baseline velocity profile
    double v_ref = 0.0;                   // [m/s], fixed-course attack speed reference
    double a_ref = 0.0;                   // [m/s^2], fixed-course longitudinal acceleration reference
    double n_ref = 0.0;                   // [m], Frenet left-positive lateral reference
    double kappa_ref = 0.0;               // [1/m], planned-line curvature reference
    double centerline_kappa = 0.0;        // [1/m]
    TrackPhase phase = TrackPhase::Straight;
    int segment_id = -1;
    bool attack_allowed = false;
    double static_speed_cap = 0.0;        // [m/s], phase-level cap
    AttackSpeedLimitReason speed_reason = AttackSpeedLimitReason::Profile;
};

class CourseAttackPlan {
public:
    CourseAttackPlan() = default;

    CourseAttackPlan(std::vector<CourseAttackSample> samples, double length, bool closed_loop)
        : samples_(std::move(samples))
        , length_(std::max(0.0, length))
        , closed_loop_(closed_loop)
    {}

    static CourseAttackPlan build(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const vehicle::VehicleParams& vehicle_params,
        const TrackPhaseProfile& phase_profile,
        const BoundaryLinePlannerConfig& line_config,
        const AttackSpeedPlannerConfig& speed_config = AttackSpeedPlannerConfig())
    {
        const double length = track.length();
        if (length <= 1e-9) {
            return CourseAttackPlan({}, 0.0, false);
        }

        const auto attack_speed_profile = AttackSpeedPlanner::plan(
            track,
            velocity_profile,
            vehicle_params,
            phase_profile,
            speed_config);

        if (attack_speed_profile.empty()) {
            return CourseAttackPlan({}, length, track.is_closed_loop());
        }

        std::vector<CourseAttackSample> samples;
        samples.reserve(attack_speed_profile.samples().size());
        for (const auto& speed_sample : attack_speed_profile.samples()) {
            const double s = std::clamp(speed_sample.s, 0.0, length);
            const auto phase_sample = phase_profile.empty()
                ? TrackPhaseSample{}
                : phase_profile.sample_at(s);
            const auto geometry = compute_planned_line_geometry(track, s, line_config);

            CourseAttackSample sample;
            sample.s = s;
            sample.profile_speed = speed_sample.profile_speed;
            sample.v_ref = speed_sample.planned_speed;
            sample.n_ref = geometry.target_n;
            sample.kappa_ref = geometry.curvature;
            sample.centerline_kappa = geometry.centerline_curvature;
            sample.phase = phase_profile.empty() ? TrackPhase::Straight : phase_sample.phase;
            sample.segment_id = phase_profile.empty() ? -1 : phase_sample.segment_id;
            sample.attack_allowed = !phase_profile.empty() && phase_sample.attack_allowed;
            sample.static_speed_cap = phase_profile.empty() ? speed_sample.profile_speed : phase_sample.static_speed_cap;
            sample.speed_reason = speed_sample.limiting_reason;
            samples.push_back(sample);
        }

        assign_reference_acceleration(samples, vehicle_params, speed_config);
        return CourseAttackPlan(std::move(samples), length, track.is_closed_loop());
    }

    bool empty() const { return samples_.empty(); }
    double length() const { return length_; }
    bool closed_loop() const { return closed_loop_; }
    const std::vector<CourseAttackSample>& samples() const { return samples_; }

    const CourseAttackSample& sample_at(double s) const {
        static const CourseAttackSample kFallback{};
        if (samples_.empty()) {
            return kFallback;
        }
        const double query_s = normalize_s(s);
        auto it = lower_bound(query_s);
        if (it == samples_.begin()) {
            return *it;
        }
        if (it == samples_.end()) {
            return samples_.back();
        }
        const auto prev = std::prev(it);
        return (query_s - prev->s) <= (it->s - query_s) ? *prev : *it;
    }

    double v_ref_at(double s) const { return interpolate(s, &CourseAttackSample::v_ref); }
    double a_ref_at(double s) const { return interpolate(s, &CourseAttackSample::a_ref); }
    double n_ref_at(double s) const { return interpolate(s, &CourseAttackSample::n_ref); }
    double kappa_ref_at(double s) const { return interpolate(s, &CourseAttackSample::kappa_ref); }

    double preview_min_v_ref(double s, double preview_distance, double step = 1.0) const {
        if (samples_.empty()) {
            return 0.0;
        }
        double speed = v_ref_at(s);
        const double distance = std::max(preview_distance, 0.0);
        const double ds = std::clamp(step, 0.25, 5.0);
        for (double preview_s = ds; preview_s <= distance + 1e-12; preview_s += ds) {
            speed = std::min(speed, v_ref_at(s + preview_s));
        }
        return speed;
    }

private:
    using ConstIterator = std::vector<CourseAttackSample>::const_iterator;

    static void assign_reference_acceleration(
        std::vector<CourseAttackSample>& samples,
        const vehicle::VehicleParams& vehicle_params,
        const AttackSpeedPlannerConfig& config)
    {
        if (samples.size() < 2U) {
            return;
        }

        const double accel_limit = std::max(config.accel_limit, 0.1);
        const double decel_limit = std::max(vehicle_params.tire.mu_x_max, 0.1)
            * 9.81
            * std::clamp(config.braking_safety, 0.2, 1.0);
        std::vector<double> accel(samples.size(), 0.0);
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const std::size_t lo = i == 0U ? i : i - 1U;
            const std::size_t hi = std::min(i + 1U, samples.size() - 1U);
            const double ds = std::max(samples[hi].s - samples[lo].s, 1e-6);
            const double dv_ds = (samples[hi].v_ref - samples[lo].v_ref) / ds;
            accel[i] = std::clamp(
                std::max(samples[i].v_ref, 0.5) * dv_ds,
                -decel_limit,
                accel_limit);
        }

        constexpr double kReferenceJerkLimit = 35.0; // [m/s^3], feed-forward smoothing only
        auto limit_accel_step = [](double previous, double target, double ds, double speed) {
            const double dt = ds / std::max(speed, 0.5);
            const double max_da = kReferenceJerkLimit * std::max(dt, 1e-4);
            return std::clamp(target, previous - max_da, previous + max_da);
        };

        for (std::size_t i = 1; i < samples.size(); ++i) {
            const double ds = std::max(samples[i].s - samples[i - 1U].s, 1e-6);
            const double speed = 0.5 * (samples[i].v_ref + samples[i - 1U].v_ref);
            accel[i] = limit_accel_step(accel[i - 1U], accel[i], ds, speed);
        }
        for (std::size_t i = samples.size() - 1U; i > 0U; --i) {
            const double ds = std::max(samples[i].s - samples[i - 1U].s, 1e-6);
            const double speed = 0.5 * (samples[i].v_ref + samples[i - 1U].v_ref);
            accel[i - 1U] = limit_accel_step(accel[i], accel[i - 1U], ds, speed);
        }

        for (std::size_t i = 0; i < samples.size(); ++i) {
            samples[i].a_ref = std::clamp(accel[i], -decel_limit, accel_limit);
        }
    }

    ConstIterator lower_bound(double s) const {
        return std::lower_bound(
            samples_.begin(),
            samples_.end(),
            s,
            [](const CourseAttackSample& sample, double value) {
                return sample.s < value;
            });
    }

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

    double interpolate(double s, double CourseAttackSample::* member) const {
        if (samples_.empty()) {
            return 0.0;
        }
        const double query_s = normalize_s(s);
        auto it = lower_bound(query_s);
        if (it == samples_.begin()) {
            return (*it).*member;
        }
        if (it == samples_.end()) {
            return samples_.back().*member;
        }
        const auto prev = std::prev(it);
        const double span = std::max(it->s - prev->s, 1e-9);
        const double alpha = std::clamp((query_s - prev->s) / span, 0.0, 1.0);
        return ((*prev).*member) + alpha * (((*it).*member) - ((*prev).*member));
    }

    std::vector<CourseAttackSample> samples_;
    double length_ = 0.0;
    bool closed_loop_ = false;
};

} // namespace fsae::driver
