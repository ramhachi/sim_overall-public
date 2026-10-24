#pragma once
/// @file attack_residual_profile.hpp
/// @brief Small fixed-course residual corrections for the attack driver.

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fsae::driver {

struct AttackResidualCorrection {
    double delta_v_ref_mps = 0.0;       ///< additive speed reference correction [m/s]
    double delta_steer_ff_rad = 0.0;    ///< additive tire-angle feed-forward correction [rad]
    double delta_ax_bias_mps2 = 0.0;    ///< additive longitudinal acceleration correction [m/s^2]
    double delta_heading_relax_rad = 0.0; ///< safe reduction of heading-safety severity [rad]
};

struct AttackResidualSample {
    double s = 0.0;                     ///< station [m]
    AttackResidualCorrection correction;
};

class AttackResidualProfile {
public:
    AttackResidualProfile() = default;

    AttackResidualProfile(
        std::vector<AttackResidualSample> samples,
        double length,
        bool closed_loop,
        double max_delta_v_ref_mps = 2.0,
        double max_delta_steer_ff_rad = 0.035,
        double max_delta_ax_bias_mps2 = 1.0,
        double max_delta_heading_relax_rad = 0.08)
        : samples_(std::move(samples))
        , length_(std::max(0.0, length))
        , closed_loop_(closed_loop)
        , max_delta_v_ref_mps_(std::max(0.0, max_delta_v_ref_mps))
        , max_delta_steer_ff_rad_(std::max(0.0, max_delta_steer_ff_rad))
        , max_delta_ax_bias_mps2_(std::max(0.0, max_delta_ax_bias_mps2))
        , max_delta_heading_relax_rad_(std::max(0.0, max_delta_heading_relax_rad))
    {
        normalize_samples();
    }

    static AttackResidualProfile from_json(
        const nlohmann::json& root,
        double length,
        bool closed_loop)
    {
        if (!root.is_object()) {
            throw std::runtime_error("Attack residual profile must be a JSON object.");
        }

        double max_delta_v_ref_mps = 2.0;
        double max_delta_steer_ff_rad = 0.035;
        double max_delta_ax_bias_mps2 = 1.0;
        double max_delta_heading_relax_rad = 0.08;
        if (root.contains("limits")) {
            const auto& limits = root.at("limits");
            if (limits.contains("max_delta_v_ref_mps")) {
                max_delta_v_ref_mps = limits.at("max_delta_v_ref_mps").get<double>();
            } else if (limits.contains("max_delta_v_mps")) {
                max_delta_v_ref_mps = limits.at("max_delta_v_mps").get<double>();
            }
            if (limits.contains("max_delta_steer_ff_rad")) {
                max_delta_steer_ff_rad = limits.at("max_delta_steer_ff_rad").get<double>();
            } else if (limits.contains("max_delta_steer_rad")) {
                max_delta_steer_ff_rad = limits.at("max_delta_steer_rad").get<double>();
            }
            if (limits.contains("max_delta_ax_bias_mps2")) {
                max_delta_ax_bias_mps2 = limits.at("max_delta_ax_bias_mps2").get<double>();
            } else if (limits.contains("max_delta_ax_mps2")) {
                max_delta_ax_bias_mps2 = limits.at("max_delta_ax_mps2").get<double>();
            }
            if (limits.contains("max_delta_heading_relax_rad")) {
                max_delta_heading_relax_rad =
                    limits.at("max_delta_heading_relax_rad").get<double>();
            } else if (limits.contains("max_heading_relax_rad")) {
                max_delta_heading_relax_rad =
                    limits.at("max_heading_relax_rad").get<double>();
            }
        }

        if (!root.contains("samples") || !root.at("samples").is_array()) {
            throw std::runtime_error("Attack residual profile requires a samples array.");
        }

        std::vector<AttackResidualSample> samples;
        samples.reserve(root.at("samples").size());
        for (const auto& item : root.at("samples")) {
            samples.push_back(parse_sample(item));
        }

        return AttackResidualProfile(
            std::move(samples),
            length,
            closed_loop,
            max_delta_v_ref_mps,
            max_delta_steer_ff_rad,
            max_delta_ax_bias_mps2,
            max_delta_heading_relax_rad);
    }

    static AttackResidualProfile load_json(
        const std::string& filepath,
        double length,
        bool closed_loop)
    {
        std::ifstream input(filepath);
        if (!input) {
            throw std::runtime_error("Failed to open attack residual profile: " + filepath);
        }
        nlohmann::json root;
        input >> root;
        return from_json(root, length, closed_loop);
    }

    bool empty() const { return samples_.empty(); }
    double length() const { return length_; }
    bool closed_loop() const { return closed_loop_; }
    double max_delta_v_ref_mps() const { return max_delta_v_ref_mps_; }
    double max_delta_steer_ff_rad() const { return max_delta_steer_ff_rad_; }
    double max_delta_ax_bias_mps2() const { return max_delta_ax_bias_mps2_; }
    double max_delta_heading_relax_rad() const { return max_delta_heading_relax_rad_; }

    AttackResidualCorrection correction_at(double s) const {
        if (samples_.empty()) {
            return {};
        }
        if (samples_.size() == 1U) {
            return clamp_correction(samples_.front().correction);
        }

        const double query_s = normalize_s(s);
        auto upper = lower_bound(query_s);
        if (upper == samples_.begin()) {
            if (!closed_loop_) {
                return clamp_correction(upper->correction);
            }
            const auto& lo = samples_.back();
            const auto& hi = samples_.front();
            const double lo_s = lo.s - length_;
            return interpolate(lo, hi, lo_s, hi.s, query_s);
        }
        if (upper == samples_.end()) {
            if (!closed_loop_) {
                return clamp_correction(samples_.back().correction);
            }
            const auto& lo = samples_.back();
            const auto& hi = samples_.front();
            return interpolate(lo, hi, lo.s, hi.s + length_, query_s);
        }

        const auto& lo = *(upper - 1);
        const auto& hi = *upper;
        return interpolate(lo, hi, lo.s, hi.s, query_s);
    }

private:
    static double read_optional_number(
        const nlohmann::json& item,
        std::initializer_list<const char*> names,
        double default_value = 0.0)
    {
        for (const char* name : names) {
            if (item.contains(name)) {
                return item.at(name).get<double>();
            }
        }
        return default_value;
    }

    static AttackResidualSample parse_sample(const nlohmann::json& item) {
        AttackResidualSample sample;
        if (item.is_array()) {
            if (item.size() < 2U) {
                throw std::runtime_error(
                    "Attack residual array samples require at least [s, delta_v].");
            }
            sample.s = item.at(0).get<double>();
            sample.correction.delta_v_ref_mps = item.at(1).get<double>();
            if (item.size() >= 3U) {
                sample.correction.delta_steer_ff_rad = item.at(2).get<double>();
            }
            if (item.size() >= 4U) {
                sample.correction.delta_ax_bias_mps2 = item.at(3).get<double>();
            }
            if (item.size() >= 5U) {
                sample.correction.delta_heading_relax_rad = item.at(4).get<double>();
            }
        } else if (item.is_object()) {
            sample.s = read_optional_number(item, {"s", "s_m"});
            sample.correction.delta_v_ref_mps = read_optional_number(
                item,
                {"delta_v_ref_mps", "delta_v_mps", "dv_ref_mps", "speed_delta_mps"});
            sample.correction.delta_steer_ff_rad = read_optional_number(
                item,
                {"delta_steer_ff_rad", "delta_steer_rad", "dsteer_ff_rad"});
            sample.correction.delta_ax_bias_mps2 = read_optional_number(
                item,
                {"delta_ax_bias_mps2", "delta_ax_mps2", "ax_bias_mps2"});
            sample.correction.delta_heading_relax_rad = read_optional_number(
                item,
                {"delta_heading_relax_rad", "heading_relax_rad"});
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            if (item.contains("delta_steer_ff_deg")) {
                sample.correction.delta_steer_ff_rad =
                    item.at("delta_steer_ff_deg").get<double>() * kDegToRad;
            } else if (item.contains("delta_steer_deg")) {
                sample.correction.delta_steer_ff_rad =
                    item.at("delta_steer_deg").get<double>() * kDegToRad;
            }
            if (item.contains("delta_heading_relax_deg")) {
                sample.correction.delta_heading_relax_rad =
                    item.at("delta_heading_relax_deg").get<double>() * kDegToRad;
            } else if (item.contains("heading_relax_deg")) {
                sample.correction.delta_heading_relax_rad =
                    item.at("heading_relax_deg").get<double>() * kDegToRad;
            }
        } else {
            throw std::runtime_error("Attack residual sample must be object or array.");
        }

        if (!std::isfinite(sample.s)
                || !std::isfinite(sample.correction.delta_v_ref_mps)
                || !std::isfinite(sample.correction.delta_steer_ff_rad)
                || !std::isfinite(sample.correction.delta_ax_bias_mps2)
                || !std::isfinite(sample.correction.delta_heading_relax_rad)) {
            throw std::runtime_error("Attack residual sample contains non-finite values.");
        }
        return sample;
    }

    void normalize_samples() {
        samples_.erase(
            std::remove_if(
                samples_.begin(),
                samples_.end(),
                [](const AttackResidualSample& sample) {
                    return !std::isfinite(sample.s)
                        || !std::isfinite(sample.correction.delta_v_ref_mps)
                        || !std::isfinite(sample.correction.delta_steer_ff_rad)
                        || !std::isfinite(sample.correction.delta_ax_bias_mps2)
                        || !std::isfinite(sample.correction.delta_heading_relax_rad);
                }),
            samples_.end());

        for (auto& sample : samples_) {
            sample.s = normalize_s(sample.s);
            sample.correction = clamp_correction(sample.correction);
        }
        std::sort(
            samples_.begin(),
            samples_.end(),
            [](const AttackResidualSample& lhs, const AttackResidualSample& rhs) {
                return lhs.s < rhs.s;
            });

        std::vector<AttackResidualSample> unique_samples;
        unique_samples.reserve(samples_.size());
        for (const auto& sample : samples_) {
            if (!unique_samples.empty()
                    && std::abs(sample.s - unique_samples.back().s) < 1e-9) {
                unique_samples.back() = sample;
            } else {
                unique_samples.push_back(sample);
            }
        }
        samples_ = std::move(unique_samples);
    }

    AttackResidualCorrection clamp_correction(AttackResidualCorrection correction) const {
        correction.delta_v_ref_mps = std::clamp(
            correction.delta_v_ref_mps,
            -max_delta_v_ref_mps_,
            max_delta_v_ref_mps_);
        correction.delta_steer_ff_rad = std::clamp(
            correction.delta_steer_ff_rad,
            -max_delta_steer_ff_rad_,
            max_delta_steer_ff_rad_);
        correction.delta_ax_bias_mps2 = std::clamp(
            correction.delta_ax_bias_mps2,
            -max_delta_ax_bias_mps2_,
            max_delta_ax_bias_mps2_);
        correction.delta_heading_relax_rad = std::clamp(
            correction.delta_heading_relax_rad,
            0.0,
            max_delta_heading_relax_rad_);
        return correction;
    }

    AttackResidualCorrection interpolate(
        const AttackResidualSample& lo,
        const AttackResidualSample& hi,
        double lo_s,
        double hi_s,
        double query_s) const
    {
        const double span = std::max(hi_s - lo_s, 1e-9);
        const double alpha = std::clamp((query_s - lo_s) / span, 0.0, 1.0);
        AttackResidualCorrection correction;
        correction.delta_v_ref_mps =
            lo.correction.delta_v_ref_mps
            + alpha * (hi.correction.delta_v_ref_mps - lo.correction.delta_v_ref_mps);
        correction.delta_steer_ff_rad =
            lo.correction.delta_steer_ff_rad
            + alpha * (hi.correction.delta_steer_ff_rad - lo.correction.delta_steer_ff_rad);
        correction.delta_ax_bias_mps2 =
            lo.correction.delta_ax_bias_mps2
            + alpha * (hi.correction.delta_ax_bias_mps2 - lo.correction.delta_ax_bias_mps2);
        correction.delta_heading_relax_rad =
            lo.correction.delta_heading_relax_rad
            + alpha * (
                hi.correction.delta_heading_relax_rad
                - lo.correction.delta_heading_relax_rad);
        return clamp_correction(correction);
    }

    using ConstIterator = std::vector<AttackResidualSample>::const_iterator;

    ConstIterator lower_bound(double s) const {
        return std::lower_bound(
            samples_.begin(),
            samples_.end(),
            s,
            [](const AttackResidualSample& sample, double value) {
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

    std::vector<AttackResidualSample> samples_;
    double length_ = 0.0;
    bool closed_loop_ = false;
    double max_delta_v_ref_mps_ = 2.0;
    double max_delta_steer_ff_rad_ = 0.035;
    double max_delta_ax_bias_mps2_ = 1.0;
    double max_delta_heading_relax_rad_ = 0.08;
};

inline AttackResidualProfile load_attack_residual_profile(
    const std::string& filepath,
    double length,
    bool closed_loop)
{
    return AttackResidualProfile::load_json(filepath, length, closed_loop);
}

} // namespace fsae::driver
