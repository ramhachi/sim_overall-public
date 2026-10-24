#pragma once
/// @file track_phase_analyzer.hpp
/// @brief Static track-phase classification for attack-driver scheduling.

#include "../laptime/velocity_profile.hpp"
#include "../track/track_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace fsae::driver {

enum class TrackPhase {
    Straight,
    Brake,
    TurnIn,
    Apex,
    Exit,
    Complex,
};

inline const char* track_phase_name(TrackPhase phase) {
    switch (phase) {
        case TrackPhase::Straight:
            return "straight";
        case TrackPhase::Brake:
            return "brake";
        case TrackPhase::TurnIn:
            return "turn_in";
        case TrackPhase::Apex:
            return "apex";
        case TrackPhase::Exit:
            return "exit";
        case TrackPhase::Complex:
            return "complex";
    }
    return "unknown";
}

struct TrackPhaseConfig {
    double sample_ds = 1.0;              // [m]
    double straight_kappa = 0.015;       // [1/m]
    double corner_kappa = 0.025;         // [1/m]
    double preview_distance = 25.0;      // [m]
    double local_window = 8.0;           // [m]
    double brake_dv_threshold = -0.60;   // [m/s] over preview_distance
    double accel_dv_threshold = 0.35;    // [m/s] over preview_distance
    double apex_peak_ratio = 0.82;       // [-]
    double exit_release_ratio = 0.72;    // [-]
    double straight_speed_multiplier = 1.08; // [-], static attack cap vs. base profile
    double exit_speed_multiplier = 1.08;     // [-]
    double complex_speed_multiplier = 1.02;  // [-]
    double turn_in_speed_multiplier = 1.00;  // [-]
    double apex_speed_multiplier = 1.00;     // [-]
    double brake_speed_multiplier = 0.96;    // [-]
    double min_static_speed_cap = 3.0;       // [m/s]
    double max_static_speed_cap = 30.0;      // [m/s]
};

struct TrackPhaseSample {
    double s = 0.0;
    TrackPhase phase = TrackPhase::Straight;
    double kappa = 0.0;
    double abs_kappa = 0.0;
    double local_peak_abs_kappa = 0.0;
    double future_peak_abs_kappa = 0.0;
    double future_end_abs_kappa = 0.0;
    double preview_dv = 0.0;
    double profile_speed = 0.0;       // [m/s], base velocity profile at s
    double static_speed_cap = 0.0;    // [m/s], fixed-course attack speed cap
    int segment_id = 0;               // contiguous phase segment id
    bool attack_allowed = false;      // true where high-speed candidates are admissible
};

class TrackPhaseProfile {
public:
    TrackPhaseProfile() = default;

    explicit TrackPhaseProfile(std::vector<TrackPhaseSample> samples, double length)
        : samples_(std::move(samples))
        , length_(std::max(0.0, length))
    {}

    TrackPhase phase_at(double s) const {
        return sample_at(s).phase;
    }

    double static_speed_cap_at(double s) const {
        return sample_at(s).static_speed_cap;
    }

    int segment_id_at(double s) const {
        return sample_at(s).segment_id;
    }

    bool attack_allowed_at(double s) const {
        return sample_at(s).attack_allowed;
    }

    const TrackPhaseSample& sample_at(double s) const {
        static const TrackPhaseSample kFallback{};
        if (samples_.empty()) {
            return kFallback;
        }
        const double query_s = normalize_s(s);
        auto it = std::lower_bound(
            samples_.begin(),
            samples_.end(),
            query_s,
            [](const TrackPhaseSample& sample, double value) {
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

    const std::vector<TrackPhaseSample>& samples() const { return samples_; }
    double length() const { return length_; }
    bool empty() const { return samples_.empty(); }

private:
    double normalize_s(double s) const {
        if (length_ <= 1e-9) {
            return std::max(0.0, s);
        }
        double wrapped = std::fmod(s, length_);
        if (wrapped < 0.0) {
            wrapped += length_;
        }
        return wrapped;
    }

    std::vector<TrackPhaseSample> samples_;
    double length_ = 0.0;
};

class TrackPhaseAnalyzer {
public:
    static TrackPhaseProfile analyze(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const TrackPhaseConfig& config = TrackPhaseConfig())
    {
        const double length = track.length();
        if (length <= 1e-9) {
            return TrackPhaseProfile({}, 0.0);
        }

        const double ds = std::clamp(config.sample_ds, 0.25, 5.0);
        std::vector<TrackPhaseSample> samples;
        samples.reserve(static_cast<std::size_t>(std::ceil(length / ds)) + 1);

        for (double s = 0.0; s <= length + 1e-9; s += ds) {
            TrackPhaseSample sample;
            sample.s = std::min(s, length);
            sample.kappa = kappa_at(track, sample.s);
            sample.abs_kappa = std::abs(sample.kappa);
            sample.local_peak_abs_kappa =
                max_abs_kappa(track, sample.s - config.local_window, sample.s + config.local_window, ds);
            sample.future_peak_abs_kappa =
                max_abs_kappa(track, sample.s, sample.s + config.preview_distance, ds);
            sample.future_end_abs_kappa =
                std::abs(kappa_at(track, sample.s + config.preview_distance));
            sample.profile_speed = velocity_at(velocity_profile, sample.s);
            sample.preview_dv =
                velocity_at(velocity_profile, sample.s + config.preview_distance)
              - sample.profile_speed;
            sample.phase = classify(track, sample, config, ds);
            sample.static_speed_cap = static_speed_cap(sample, config);
            sample.attack_allowed = attack_allowed(sample.phase);
            samples.push_back(sample);
        }
        assign_segment_ids(samples);

        return TrackPhaseProfile(std::move(samples), length);
    }

private:
    static TrackPhase classify(
        const track::CurvatureBasedTrack& track,
        const TrackPhaseSample& sample,
        const TrackPhaseConfig& config,
        double ds)
    {
        const bool sign_change_near = has_kappa_sign_change(
            track,
            sample.s - config.local_window,
            sample.s + config.preview_distance,
            ds);
        if (sign_change_near
                && sample.abs_kappa >= config.straight_kappa
                && sample.future_peak_abs_kappa >= config.corner_kappa) {
            return TrackPhase::Complex;
        }

        if (sample.abs_kappa < config.straight_kappa) {
            const bool corner_ahead = sample.future_peak_abs_kappa >= config.corner_kappa;
            if (corner_ahead && sample.preview_dv <= config.brake_dv_threshold) {
                return TrackPhase::Brake;
            }
            return TrackPhase::Straight;
        }

        const bool near_peak =
            sample.local_peak_abs_kappa > 1e-9
            && sample.abs_kappa >= config.apex_peak_ratio * sample.local_peak_abs_kappa;
        const bool releasing =
            sample.future_end_abs_kappa <= config.exit_release_ratio * sample.abs_kappa
            || sample.preview_dv >= config.accel_dv_threshold;
        const bool building =
            sample.future_peak_abs_kappa > sample.abs_kappa * 1.12
            && !near_peak;

        if (near_peak) {
            return TrackPhase::Apex;
        }
        if (releasing && !building) {
            return TrackPhase::Exit;
        }
        return TrackPhase::TurnIn;
    }

    static double phase_speed_multiplier(TrackPhase phase, const TrackPhaseConfig& config) {
        switch (phase) {
            case TrackPhase::Straight:
                return config.straight_speed_multiplier;
            case TrackPhase::Exit:
                return config.exit_speed_multiplier;
            case TrackPhase::Complex:
                return config.complex_speed_multiplier;
            case TrackPhase::TurnIn:
                return config.turn_in_speed_multiplier;
            case TrackPhase::Apex:
                return config.apex_speed_multiplier;
            case TrackPhase::Brake:
                return config.brake_speed_multiplier;
        }
        return 1.0;
    }

    static double static_speed_cap(
        const TrackPhaseSample& sample,
        const TrackPhaseConfig& config)
    {
        const double cap = sample.profile_speed
            * phase_speed_multiplier(sample.phase, config);
        return std::clamp(
            cap,
            config.min_static_speed_cap,
            config.max_static_speed_cap);
    }

    static bool attack_allowed(TrackPhase phase) {
        return phase == TrackPhase::Straight
            || phase == TrackPhase::Exit
            || phase == TrackPhase::Complex;
    }

    static void assign_segment_ids(std::vector<TrackPhaseSample>& samples) {
        int segment_id = 0;
        TrackPhase previous_phase = samples.empty()
            ? TrackPhase::Straight
            : samples.front().phase;
        for (auto& sample : samples) {
            if (sample.phase != previous_phase) {
                ++segment_id;
                previous_phase = sample.phase;
            }
            sample.segment_id = segment_id;
        }
    }

    static double kappa_at(const track::CurvatureBasedTrack& track, double s) {
        const double length = track.length();
        if (track.is_closed_loop() && length > 1e-9) {
            double wrapped = std::fmod(s, length);
            if (wrapped < 0.0) {
                wrapped += length;
            }
            return track.get_curvature(wrapped);
        }
        return track.get_curvature(std::clamp(s, 0.0, length));
    }

    static double max_abs_kappa(
        const track::CurvatureBasedTrack& track,
        double s0,
        double s1,
        double ds)
    {
        if (s1 < s0) {
            std::swap(s0, s1);
        }
        double peak = 0.0;
        for (double s = s0; s <= s1 + 1e-9; s += std::max(ds, 0.25)) {
            peak = std::max(peak, std::abs(kappa_at(track, s)));
        }
        return peak;
    }

    static bool has_kappa_sign_change(
        const track::CurvatureBasedTrack& track,
        double s0,
        double s1,
        double ds)
    {
        int previous_sign = 0;
        for (double s = s0; s <= s1 + 1e-9; s += std::max(ds, 0.25)) {
            const double kappa = kappa_at(track, s);
            const int sign = (kappa > 0.010) - (kappa < -0.010);
            if (sign == 0) {
                continue;
            }
            if (previous_sign != 0 && sign != previous_sign) {
                return true;
            }
            previous_sign = sign;
        }
        return false;
    }

    static double velocity_at(
        const std::vector<laptime::VelocityPoint>& profile,
        double s)
    {
        if (profile.empty()) {
            return 10.0;
        }
        if (s <= profile.front().s) {
            return profile.front().v;
        }
        if (s >= profile.back().s) {
            return profile.back().v;
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
};

} // namespace fsae::driver
