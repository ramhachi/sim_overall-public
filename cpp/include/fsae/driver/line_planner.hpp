#pragma once
/// @file line_planner.hpp
/// @brief Boundary-aware geometric line offset planner for driver tests.

#include "../track/track_model.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace fsae::driver {

struct BoundaryLinePlannerConfig {
    bool enabled = false;
    double safety_margin = 0.55;      // [m] keep this much distance from either boundary
    double offset_fraction = 0.15;    // [-] fraction of usable half-width
    double curvature_threshold = 0.008; // [1/m] below this is treated as straight
    double preview_distance = 10.0;   // [m] entry/exit lookahead
    double smoothing_distance = 2.0;  // [m] curvature averaging half-window
    double scan_step = 1.0;           // [m] spacing for dominant preview curvature search
    double entry_offset_scale = 1.0;  // [-] outside offset scale before a corner
    double apex_offset_scale = 0.85;  // [-] inside offset scale near the corner center
    double exit_offset_scale = 0.75;  // [-] outside offset scale after a corner
    bool track_geometry = false;      // use planned-line tangent/curvature for driver tracking
};

struct BoundaryLinePlan {
    double target_n = 0.0;            // [m], Frenet left-positive lateral target
    double curvature_now = 0.0;       // [1/m]
    double curvature_ahead = 0.0;     // [1/m]
    double curvature_behind = 0.0;    // [1/m]
    double usable_half_width = 0.0;   // [m]
};

struct PlannedLineGeometry {
    double s = 0.0;                    // [m], wrapped centerline station
    double target_n = 0.0;             // [m], smoothed planned-line lateral target
    double heading = 0.0;              // [rad], planned-line tangent heading
    double heading_offset = 0.0;       // [rad], heading - centerline heading
    double curvature = 0.0;            // [1/m], planned-line curvature
    double dn_ds = 0.0;                // [-], lateral target slope
    double ddn_ds2 = 0.0;              // [1/m], lateral target second derivative
    double centerline_curvature = 0.0; // [1/m]
    BoundaryLinePlan plan;
    track::TrackPoint center_point{};
};

inline double planner_wrap_track_s(const track::CurvatureBasedTrack& track, double s) {
    const double length = track.length();
    if (!track.is_closed_loop() || length <= 1e-6) {
        return std::clamp(s, 0.0, std::max(length, 0.0));
    }
    while (s < 0.0) s += length;
    while (s > length) s -= length;
    return s;
}

inline double smooth_curvature(
    const track::CurvatureBasedTrack& track,
    double s,
    double smoothing_distance)
{
    const double window = std::max(0.0, smoothing_distance);
    if (window <= 1e-6) {
        return track.get_curvature(planner_wrap_track_s(track, s));
    }

    double sum = 0.0;
    int count = 0;
    constexpr double kStep = 1.0;
    for (double ds = -window; ds <= window + 1e-9; ds += kStep) {
        sum += track.get_curvature(planner_wrap_track_s(track, s + ds));
        count++;
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

inline double signed_available_offset(const track::TrackPoint& point, double raw, double safety_margin) {
    const double max_left = std::max(0.0, point.width_left - safety_margin);
    const double max_right = std::max(0.0, point.width_right - safety_margin);
    return std::clamp(raw, -max_right, max_left);
}

inline double curvature_sign(double kappa) {
    return (kappa > 0.0) ? 1.0 : -1.0;
}

inline double planner_smoothstep(double edge0, double edge1, double x) {
    if (edge1 <= edge0) {
        return x >= edge1 ? 1.0 : 0.0;
    }
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double dominant_curvature_in_window(
    const track::CurvatureBasedTrack& track,
    double s_begin,
    double s_end,
    double smoothing_distance,
    double scan_step)
{
    const double step = std::max(scan_step, 0.25);
    const double lo = std::min(s_begin, s_end);
    const double hi = std::max(s_begin, s_end);
    double dominant = 0.0;
    for (double s = lo; s <= hi + 1e-9; s += step) {
        const double kappa = smooth_curvature(track, s, smoothing_distance);
        if (std::abs(kappa) > std::abs(dominant)) {
            dominant = kappa;
        }
    }
    return dominant;
}

inline BoundaryLinePlan compute_boundary_line_plan(
    const track::CurvatureBasedTrack& track,
    double s,
    const BoundaryLinePlannerConfig& config)
{
    BoundaryLinePlan plan;
    const auto point = track.get_point(planner_wrap_track_s(track, s));
    plan.usable_half_width = std::max(
        0.0,
        std::min(point.width_left, point.width_right) - config.safety_margin);

    plan.curvature_now = smooth_curvature(track, s, config.smoothing_distance);
    const double preview = std::max(config.preview_distance, 0.0);
    plan.curvature_ahead = dominant_curvature_in_window(
        track,
        s + 0.25 * preview,
        s + preview,
        config.smoothing_distance,
        config.scan_step);
    plan.curvature_behind = dominant_curvature_in_window(
        track,
        s - preview,
        s - 0.25 * preview,
        config.smoothing_distance,
        config.scan_step);

    if (!config.enabled || plan.usable_half_width <= 1e-6) {
        return plan;
    }

    const double threshold = std::max(config.curvature_threshold, 1e-6);
    const double base_offset = plan.usable_half_width * std::clamp(config.offset_fraction, 0.0, 1.0);

    const double now_abs = std::abs(plan.curvature_now);
    const double ahead_abs = std::abs(plan.curvature_ahead);
    const double behind_abs = std::abs(plan.curvature_behind);

    const double corner_weight = planner_smoothstep(threshold, 4.0 * threshold, now_abs);
    const double entry_weight = planner_smoothstep(threshold, 4.0 * threshold, ahead_abs) * (1.0 - corner_weight);
    const double exit_weight = planner_smoothstep(threshold, 4.0 * threshold, behind_abs) * (1.0 - corner_weight);

    double weighted_offset = 0.0;
    double total_weight = 0.0;
    if (corner_weight > 1e-6) {
        weighted_offset += corner_weight
                         * curvature_sign(plan.curvature_now)
                         * base_offset
                         * std::clamp(config.apex_offset_scale, 0.0, 1.5);
        total_weight += corner_weight;
    }
    if (entry_weight > 1e-6 && ahead_abs > threshold) {
        weighted_offset += entry_weight
                         * -curvature_sign(plan.curvature_ahead)
                         * base_offset
                         * std::clamp(config.entry_offset_scale, 0.0, 1.5);
        total_weight += entry_weight;
    }
    if (exit_weight > 1e-6 && behind_abs > threshold) {
        weighted_offset += exit_weight
                         * -curvature_sign(plan.curvature_behind)
                         * base_offset
                         * std::clamp(config.exit_offset_scale, 0.0, 1.5);
        total_weight += exit_weight;
    }

    if (total_weight > 1e-6) {
        plan.target_n = signed_available_offset(point, weighted_offset / total_weight, config.safety_margin);
    }
    return plan;
}

inline double smoothed_planned_lateral_offset(
    const track::CurvatureBasedTrack& track,
    double s,
    const BoundaryLinePlannerConfig& config,
    double step)
{
    if (!config.enabled) {
        return 0.0;
    }

    const double smooth_radius = std::max(
        config.smoothing_distance,
        0.5 * std::max(config.preview_distance, 0.0));
    if (smooth_radius <= 1e-6) {
        return compute_boundary_line_plan(track, s, config).target_n;
    }

    const int radius_samples = std::max(1, static_cast<int>(std::round(smooth_radius / step)));
    double weighted_sum = 0.0;
    double weight_total = 0.0;
    for (int offset = -radius_samples; offset <= radius_samples; ++offset) {
        const double weight = 1.0 - std::abs(static_cast<double>(offset)) /
                                    (static_cast<double>(radius_samples) + 1.0);
        const auto sample_plan = compute_boundary_line_plan(
            track,
            s + static_cast<double>(offset) * step,
            config);
        weighted_sum += weight * sample_plan.target_n;
        weight_total += weight;
    }

    const double smoothed = weight_total > 1e-9
        ? weighted_sum / weight_total
        : compute_boundary_line_plan(track, s, config).target_n;
    return signed_available_offset(
        track.get_point(planner_wrap_track_s(track, s)),
        smoothed,
        config.safety_margin);
}

inline PlannedLineGeometry compute_planned_line_geometry(
    const track::CurvatureBasedTrack& track,
    double s,
    const BoundaryLinePlannerConfig& config)
{
    PlannedLineGeometry geometry;
    geometry.s = planner_wrap_track_s(track, s);
    geometry.center_point = track.get_point(geometry.s);
    geometry.centerline_curvature = geometry.center_point.kappa;
    geometry.plan = compute_boundary_line_plan(track, geometry.s, config);

    if (!config.enabled) {
        geometry.target_n = 0.0;
        geometry.heading = geometry.center_point.theta;
        geometry.curvature = geometry.center_point.kappa;
        return geometry;
    }

    const double step = std::max(config.scan_step, 0.5);
    const double n_prev = smoothed_planned_lateral_offset(track, s - step, config, step);
    const double n_now = smoothed_planned_lateral_offset(track, s, config, step);
    const double n_next = smoothed_planned_lateral_offset(track, s + step, config, step);
    const double k_prev = track.get_curvature(planner_wrap_track_s(track, s - step));
    const double k_now = track.get_curvature(geometry.s);
    const double k_next = track.get_curvature(planner_wrap_track_s(track, s + step));

    geometry.target_n = n_now;
    geometry.dn_ds = (n_next - n_prev) / (2.0 * step);
    geometry.ddn_ds2 = (n_next - 2.0 * n_now + n_prev) / (step * step);
    const double dk_ds = (k_next - k_prev) / (2.0 * step);

    const double A = 1.0 - k_now * n_now;
    const double B = geometry.dn_ds;
    const double A_prime = -dk_ds * n_now - k_now * geometry.dn_ds;
    const double speed_sq = A * A + B * B;
    const double denom = std::pow(speed_sq, 1.5);

    geometry.heading_offset = std::atan2(B, A);
    geometry.heading = geometry.center_point.theta + geometry.heading_offset;
    geometry.curvature = denom > 1e-9
        ? ((A * A + B * B) * k_now + A * geometry.ddn_ds2 - B * A_prime) / denom
        : geometry.center_point.kappa;
    return geometry;
}

inline std::vector<std::pair<double, double>> compute_planned_line_curvature_profile(
    const track::CurvatureBasedTrack& track,
    const BoundaryLinePlannerConfig& config,
    double ds = 1.0)
{
    std::vector<std::pair<double, double>> profile;
    const double length = track.length();
    const double step = std::max(ds, 0.25);
    if (length <= 1e-6) {
        return profile;
    }

    struct PlannedSample {
        double s = 0.0;
        double n = 0.0;
        double kappa = 0.0;
    };

    std::vector<PlannedSample> samples;
    for (double s = 0.0; s < length; s += step) {
        const auto plan = compute_boundary_line_plan(track, s, config);
        samples.push_back({
            s,
            plan.target_n,
            track.get_curvature(planner_wrap_track_s(track, s)),
        });
    }
    const auto end_plan = compute_boundary_line_plan(track, length, config);
    samples.push_back({
        length,
        end_plan.target_n,
        track.get_curvature(length),
    });

    if (samples.size() < 3) {
        for (const auto& sample : samples) {
            profile.emplace_back(sample.s, sample.kappa);
        }
        return profile;
    }

    const bool closed = track.is_closed_loop();
    const std::size_t unique_count = closed && samples.size() > 3 ? samples.size() - 1 : samples.size();
    std::vector<double> smoothed_n(unique_count, 0.0);
    const int smooth_radius_samples = std::max(
        1,
        static_cast<int>(std::round(std::max(config.smoothing_distance, 0.5 * config.preview_distance) / step)));
    for (std::size_t i = 0; i < unique_count; ++i) {
        double weighted_sum = 0.0;
        double weight_total = 0.0;
        for (int offset = -smooth_radius_samples; offset <= smooth_radius_samples; ++offset) {
            int raw_index = static_cast<int>(i) + offset;
            if (!closed && (raw_index < 0 || raw_index >= static_cast<int>(unique_count))) {
                continue;
            }
            std::size_t j = closed
                ? static_cast<std::size_t>((raw_index % static_cast<int>(unique_count) + static_cast<int>(unique_count)) %
                                           static_cast<int>(unique_count))
                : static_cast<std::size_t>(raw_index);
            const double weight = 1.0 - std::abs(static_cast<double>(offset)) /
                                        (static_cast<double>(smooth_radius_samples) + 1.0);
            weighted_sum += weight * samples[j].n;
            weight_total += weight;
        }
        smoothed_n[i] = weight_total > 1e-9 ? weighted_sum / weight_total : samples[i].n;
    }

    auto curvature_at_index = [&](std::size_t i) {
        const std::size_t i0 = closed
            ? (i + unique_count - 1) % unique_count
            : (i == 0 ? 0 : i - 1);
        const std::size_t i2 = closed
            ? (i + 1) % unique_count
            : std::min(i + 1, unique_count - 1);

        if (i0 == i || i == i2) {
            return samples[i].kappa;
        }

        const double n_prev = smoothed_n[i0];
        const double n_now = smoothed_n[i];
        const double n_next = smoothed_n[i2];
        const double k_prev = samples[i0].kappa;
        const double k_now = samples[i].kappa;
        const double k_next = samples[i2].kappa;

        const double dn = (n_next - n_prev) / (2.0 * step);
        const double ddn = (n_next - 2.0 * n_now + n_prev) / (step * step);
        const double dk = (k_next - k_prev) / (2.0 * step);

        const double A = 1.0 - k_now * n_now;
        const double B = dn;
        const double A_prime = -dk * n_now - k_now * dn;
        const double speed_sq = A * A + B * B;
        const double denom = std::pow(speed_sq, 1.5);
        if (denom <= 1e-9) {
            return samples[i].kappa;
        }
        return ((A * A + B * B) * k_now + A * ddn - B * A_prime) / denom;
    };

    for (std::size_t i = 0; i < unique_count; ++i) {
        profile.emplace_back(samples[i].s, curvature_at_index(i));
    }
    if (closed) {
        profile.emplace_back(length, profile.front().second);
    } else if (unique_count < samples.size()) {
        profile.emplace_back(samples.back().s, samples.back().kappa);
    }
    return profile;
}

} // namespace fsae::driver
