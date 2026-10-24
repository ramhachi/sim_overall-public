#pragma once
/// @file frenet.hpp
/// @brief Frenet座標系（路面追従座標系）
///
/// 総合ガイド §16.2 に基づく実装
/// 車両位置をトラック中心線からの相対位置 (s, n) で表現

#include "track_model.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <tuple>
#include <algorithm>
namespace fsae::track {

/// Frenet座標
struct FrenetCoordinate {
    double s;          // 弧長パラメータ（トラック上の進行距離） [m]
    double n;          // 中心線からの横偏差（左が正） [m]
    double d_heading;  // トラックヘディングからの角度偏差 [rad]
};

/// Frenet座標系ユーティリティ
class FrenetConverter {
public:
    explicit FrenetConverter(const CurvatureBasedTrack& track)
        : track_(track) {}

    /// グローバル座標(x, y)をFrenet座標(s, n)に変換
    /// 連続性保証: 前回のs値周辺を優先的に探索（自己交差トラック対応）
    /// @param x グローバルX座標 [m]
    /// @param y グローバルY座標 [m]
    /// @param psi 車両ヨー角 [rad]
    /// @return Frenet座標
    FrenetCoordinate global_to_frenet(double x, double y, double psi) const {
        FrenetCoordinate frenet;

        const auto& points = track_.points();
        if (points.empty()) {
            frenet.s = 0.0;
            frenet.n = 0.0;
            frenet.d_heading = 0.0;
            return frenet;
        }

        struct Candidate {
            bool found = false;
            double cost = std::numeric_limits<double>::max();
            double distance = std::numeric_limits<double>::max();
            double wrapped_s = 0.0;
            double unwrapped_s = 0.0;
            double theta = 0.0;
            double point_x = 0.0;
            double point_y = 0.0;
        };

        const double track_length = track_.length();
        const bool closed_loop = track_.is_closed_loop() && track_length > 1e-9;
        const double reference_s = initialized_ ? prev_s_unwrapped_ : 0.0;

        auto unwrap_s = [&](double s_wrapped) -> double {
            if (!closed_loop) return s_wrapped;
            double best = s_wrapped;
            double min_delta = std::abs(best - reference_s);
            for (double shift : {-track_length, 0.0, track_length}) {
                double candidate = s_wrapped + shift;
                double delta = std::abs(candidate - reference_s);
                if (delta < min_delta) {
                    min_delta = delta;
                    best = candidate;
                }
            }
            return best;
        };

        auto wrap_s = [&](double s_unwrapped) -> double {
            if (!closed_loop) return s_unwrapped;
            double wrapped = std::fmod(s_unwrapped, track_length);
            if (wrapped < 0.0) wrapped += track_length;
            if (std::abs(wrapped - track_length) < 1e-6) return 0.0;
            return wrapped;
        };

        auto consider_candidate = [&](Candidate& best, const auto& pt) {
            double dx = x - pt.x;
            double dy = y - pt.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double heading_error = std::abs(normalize_angle(psi - pt.theta));
            double unwrapped_s = unwrap_s(pt.s);
            double backward_jump = initialized_ ? std::max(0.0, reference_s - unwrapped_s - backward_slack_) : 0.0;
            double cost = distance
                        + heading_weight_ * heading_error
                        + backward_penalty_weight_ * backward_jump;

            if (!best.found || cost < best.cost) {
                best.found = true;
                best.cost = cost;
                best.distance = distance;
                best.wrapped_s = pt.s;
                best.unwrapped_s = unwrapped_s;
                best.theta = pt.theta;
                best.point_x = pt.x;
                best.point_y = pt.y;
            }
        };

        auto search = [&](double max_progress_jump, bool require_heading_gate) {
            Candidate best;
            const bool use_local_search = initialized_ && (max_progress_jump < track_length * 0.45);
            if (use_local_search) {
                double ref_s_wrapped = wrap_s(reference_s);
                auto it = std::lower_bound(points.begin(), points.end(), ref_s_wrapped,
                    [](const TrackPoint& pt, double val) { return pt.s < val; });
                size_t idx = std::distance(points.begin(), it);
                size_t n_points = points.size();
                if (idx >= n_points) idx = 0;

                // Scan right
                size_t r_idx = idx;
                size_t count = 0;
                while (count < n_points) {
                    const auto& pt = points[r_idx];
                    double unwrapped_s = unwrap_s(pt.s);
                    double ds = std::abs(unwrapped_s - reference_s);
                    if (ds > max_progress_jump) {
                        break;
                    }
                    if (require_heading_gate) {
                        double heading_error = std::abs(normalize_angle(psi - pt.theta));
                        if (heading_error > heading_gate_) {
                            r_idx = (r_idx + 1) % n_points;
                            count++;
                            continue;
                        }
                    }
                    consider_candidate(best, pt);
                    r_idx = (r_idx + 1) % n_points;
                    count++;
                }

                // Scan left
                size_t l_idx = (idx + n_points - 1) % n_points;
                count = 0;
                while (count < n_points) {
                    const auto& pt = points[l_idx];
                    double unwrapped_s = unwrap_s(pt.s);
                    double ds = std::abs(unwrapped_s - reference_s);
                    if (ds > max_progress_jump) {
                        break;
                    }
                    if (require_heading_gate) {
                        double heading_error = std::abs(normalize_angle(psi - pt.theta));
                        if (heading_error > heading_gate_) {
                            l_idx = (l_idx + n_points - 1) % n_points;
                            count++;
                            continue;
                        }
                    }
                    consider_candidate(best, pt);
                    l_idx = (l_idx + n_points - 1) % n_points;
                    count++;
                }
            } else {
                for (const auto& pt : points) {
                    double unwrapped_s = unwrap_s(pt.s);
                    double ds = initialized_ ? std::abs(unwrapped_s - reference_s) : 0.0;
                    if (initialized_ && ds > max_progress_jump) {
                        continue;
                    }
                    if (require_heading_gate) {
                        double heading_error = std::abs(normalize_angle(psi - pt.theta));
                        if (heading_error > heading_gate_) {
                            continue;
                        }
                    }
                    consider_candidate(best, pt);
                }
            }
            return best;
        };

        Candidate best;
        if (initialized_) {
            best = search(search_window_, false);
            if (!best.found || best.distance > fallback_threshold_) {
                best = search(max_progress_jump_, true);
            }
        }
        if (!best.found) {
            best = search(closed_loop ? track_length : std::numeric_limits<double>::max(), false);
        }

        double best_unwrapped_s = best.found ? best.unwrapped_s : 0.0;
        double best_s = best.found ? wrap_s(best_unwrapped_s) : 0.0;
        // Closed-loop tracks often duplicate the start point at s=track_length.
        // If the very first projection lands on that duplicate, the wrapped
        // value is correctly reported as s=0, but keeping the internal
        // unwrapped reference near track_length makes the next local search
        // jump away from the start. Canonicalize the initial seam projection.
        if (!initialized_ && closed_loop && best.found && best_s < 1e-6) {
            best_unwrapped_s = 0.0;
        }
        double best_theta = best.theta;
        double best_x = best.point_x;
        double best_y = best.point_y;
        if (!best.found) {
            const auto& start_pt = points.front();
            best_theta = start_pt.theta;
            best_x = start_pt.x;
            best_y = start_pt.y;
        }

        initialized_ = true;
        prev_s_unwrapped_ = best.found ? best_unwrapped_s : best_s;
        prev_s_ = best_s;

        // 横偏差nを計算（符号付き距離）
        double dx = x - best_x;
        double dy = y - best_y;
        double normal_x = -std::sin(best_theta);
        double normal_y = std::cos(best_theta);
        double n = dx * normal_x + dy * normal_y;

        frenet.s = best_s;
        frenet.n = n;
        frenet.d_heading = normalize_angle(psi - best_theta);

        return frenet;
    }

    /// Frenet座標(s, n)をグローバル座標(x, y)に変換
    std::tuple<double, double, double> frenet_to_global(const FrenetCoordinate& frenet) const {
        auto pt = track_.get_point(frenet.s);
        double normal_x = -std::sin(pt.theta);
        double normal_y = std::cos(pt.theta);
        double x = pt.x + frenet.n * normal_x;
        double y = pt.y + frenet.n * normal_y;
        double theta = pt.theta + frenet.d_heading;
        return {x, y, theta};
    }

    bool is_within_track(const FrenetCoordinate& frenet) const {
        auto pt = track_.get_point(frenet.s);
        return std::abs(frenet.n) <= std::min(pt.width_left, pt.width_right);
    }

    const CurvatureBasedTrack& track() const { return track_; }

    void reset() {
        initialized_ = false;
        prev_s_ = 0.0;
        prev_s_unwrapped_ = 0.0;
    }

private:
    static double normalize_angle(double angle) {
        if (std::isnan(angle) || std::isinf(angle)) return 0.0;
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    const CurvatureBasedTrack& track_;
    mutable bool initialized_ = false;
    mutable double prev_s_ = 0.0;
    mutable double prev_s_unwrapped_ = 0.0;
    static constexpr double search_window_ = 15.0;
    static constexpr double fallback_threshold_ = 10.0;
    static constexpr double max_progress_jump_ = 45.0;
    static constexpr double heading_weight_ = 2.0;
    static constexpr double heading_gate_ = 1.75;
    static constexpr double backward_slack_ = 2.0;
    static constexpr double backward_penalty_weight_ = 4.0;
};

/// レーシングライン（最小曲率経路）を計算
inline std::vector<FrenetCoordinate> compute_racing_line(
    const CurvatureBasedTrack& track,
    double safety_margin = 0.5)
{
    std::vector<FrenetCoordinate> racing_line;
    const auto& points = track.points();

    for (const auto& pt : points) {
        FrenetCoordinate frenet;
        frenet.s = pt.s;
        frenet.d_heading = 0.0;
        double max_offset = std::min(pt.width_left, pt.width_right) - safety_margin;

        if (std::abs(pt.kappa) > 1e-6) {
            double offset_ratio = std::min(std::abs(pt.kappa) * 10.0, 1.0);
            frenet.n = -std::copysign(max_offset * offset_ratio, pt.kappa);
        } else {
            frenet.n = 0.0;
        }
        racing_line.push_back(frenet);
    }
    return racing_line;
}

} // namespace fsae::track
