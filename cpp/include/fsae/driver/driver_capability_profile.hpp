#pragma once
/// @file driver_capability_profile.hpp
/// @brief Vehicle-level capability profile used to initialize attack-driver settings.

#include "../control/mpc_problem.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace fsae::driver {

struct AttackDriverOverrides {
    double attack_speed_lateral_safety = std::numeric_limits<double>::quiet_NaN();
    double attack_speed_braking_safety = std::numeric_limits<double>::quiet_NaN();
    double attack_speed_accel_limit = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_straight = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_exit = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_complex = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_turn_in = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_apex = std::numeric_limits<double>::quiet_NaN();
    double attack_floor_brake = std::numeric_limits<double>::quiet_NaN();
    double attack_w_psi = std::numeric_limits<double>::quiet_NaN();
    double attack_w_ax_rate = std::numeric_limits<double>::quiet_NaN();
};

struct DriverCapabilityProfile {
    std::string schema = "driver_capability_profile_v1";
    std::string source = "driver_benchmark";

    double effective_mu_y = 0.0;
    double effective_mu_x_accel = 0.0;
    double effective_mu_x_brake = 0.0;
    double constant_radius_m = 15.0;
    double stable_corner_speed_mps = 0.0;
    double spin_or_plow_speed_mps = 0.0;
    double understeer_gradient_rad_per_g = 0.0;
    double yaw_response_time_s = 0.0;
    double yaw_overshoot_ratio = 0.0;
    double steering_response_time_s = 0.0;
    double slalom_spacing_m = 15.0;
    double slalom_stable_speed_mps = 0.0;
    double steering_reversal_time_s = 0.0;
    double corner_exit_throttle_limit = 0.0;
    double spin_risk_sideslip_rad = 0.0;
    double max_chassis_twist_rad = 0.0;
    double max_steering_lag_ratio = 0.0;

    AttackDriverOverrides attack_overrides;
};

inline bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

inline void set_if_finite(double value, double& target) {
    if (std::isfinite(value)) {
        target = value;
    }
}

inline nlohmann::json attack_overrides_to_json(const AttackDriverOverrides& overrides) {
    nlohmann::json j = nlohmann::json::object();
    auto put = [&j](const char* key, double value) {
        if (std::isfinite(value)) {
            j[key] = value;
        }
    };
    put("attack_speed_lateral_safety", overrides.attack_speed_lateral_safety);
    put("attack_speed_braking_safety", overrides.attack_speed_braking_safety);
    put("attack_speed_accel_limit", overrides.attack_speed_accel_limit);
    put("attack_floor_straight", overrides.attack_floor_straight);
    put("attack_floor_exit", overrides.attack_floor_exit);
    put("attack_floor_complex", overrides.attack_floor_complex);
    put("attack_floor_turn_in", overrides.attack_floor_turn_in);
    put("attack_floor_apex", overrides.attack_floor_apex);
    put("attack_floor_brake", overrides.attack_floor_brake);
    put("attack_w_psi", overrides.attack_w_psi);
    put("attack_w_ax_rate", overrides.attack_w_ax_rate);
    return j;
}

inline AttackDriverOverrides attack_overrides_from_json(const nlohmann::json& j) {
    AttackDriverOverrides overrides;
    if (!j.is_object()) {
        return overrides;
    }
    auto get = [&j](const char* key, double& value) {
        if (j.contains(key) && j.at(key).is_number()) {
            value = j.at(key).get<double>();
        }
    };
    get("attack_speed_lateral_safety", overrides.attack_speed_lateral_safety);
    get("attack_speed_braking_safety", overrides.attack_speed_braking_safety);
    get("attack_speed_accel_limit", overrides.attack_speed_accel_limit);
    get("attack_floor_straight", overrides.attack_floor_straight);
    get("attack_floor_exit", overrides.attack_floor_exit);
    get("attack_floor_complex", overrides.attack_floor_complex);
    get("attack_floor_turn_in", overrides.attack_floor_turn_in);
    get("attack_floor_apex", overrides.attack_floor_apex);
    get("attack_floor_brake", overrides.attack_floor_brake);
    get("attack_w_psi", overrides.attack_w_psi);
    get("attack_w_ax_rate", overrides.attack_w_ax_rate);
    return overrides;
}

inline nlohmann::json driver_capability_profile_to_json(const DriverCapabilityProfile& profile) {
    return {
        {"schema", profile.schema},
        {"source", profile.source},
        {"metrics", {
            {"effective_mu_y", profile.effective_mu_y},
            {"effective_mu_x_accel", profile.effective_mu_x_accel},
            {"effective_mu_x_brake", profile.effective_mu_x_brake},
            {"constant_radius_m", profile.constant_radius_m},
            {"stable_corner_speed_mps", profile.stable_corner_speed_mps},
            {"spin_or_plow_speed_mps", profile.spin_or_plow_speed_mps},
            {"understeer_gradient_rad_per_g", profile.understeer_gradient_rad_per_g},
            {"yaw_response_time_s", profile.yaw_response_time_s},
            {"yaw_overshoot_ratio", profile.yaw_overshoot_ratio},
            {"steering_response_time_s", profile.steering_response_time_s},
            {"slalom_spacing_m", profile.slalom_spacing_m},
            {"slalom_stable_speed_mps", profile.slalom_stable_speed_mps},
            {"steering_reversal_time_s", profile.steering_reversal_time_s},
            {"corner_exit_throttle_limit", profile.corner_exit_throttle_limit},
            {"spin_risk_sideslip_rad", profile.spin_risk_sideslip_rad},
            {"max_chassis_twist_rad", profile.max_chassis_twist_rad},
            {"max_steering_lag_ratio", profile.max_steering_lag_ratio},
        }},
        {"attack_overrides", attack_overrides_to_json(profile.attack_overrides)},
    };
}

inline DriverCapabilityProfile driver_capability_profile_from_json(const nlohmann::json& j) {
    DriverCapabilityProfile profile;
    if (!j.is_object()) {
        throw std::runtime_error("Driver capability profile must be a JSON object");
    }
    if (j.contains("schema") && j.at("schema").is_string()) {
        profile.schema = j.at("schema").get<std::string>();
    }
    if (j.contains("source") && j.at("source").is_string()) {
        profile.source = j.at("source").get<std::string>();
    }
    const auto& metrics = j.contains("metrics") && j.at("metrics").is_object()
        ? j.at("metrics")
        : j;
    auto get = [&metrics](const char* key, double& value) {
        if (metrics.contains(key) && metrics.at(key).is_number()) {
            value = metrics.at(key).get<double>();
        }
    };
    get("effective_mu_y", profile.effective_mu_y);
    get("effective_mu_x_accel", profile.effective_mu_x_accel);
    get("effective_mu_x_brake", profile.effective_mu_x_brake);
    get("constant_radius_m", profile.constant_radius_m);
    get("stable_corner_speed_mps", profile.stable_corner_speed_mps);
    get("spin_or_plow_speed_mps", profile.spin_or_plow_speed_mps);
    get("understeer_gradient_rad_per_g", profile.understeer_gradient_rad_per_g);
    get("yaw_response_time_s", profile.yaw_response_time_s);
    get("yaw_overshoot_ratio", profile.yaw_overshoot_ratio);
    get("steering_response_time_s", profile.steering_response_time_s);
    get("slalom_spacing_m", profile.slalom_spacing_m);
    get("slalom_stable_speed_mps", profile.slalom_stable_speed_mps);
    get("steering_reversal_time_s", profile.steering_reversal_time_s);
    get("corner_exit_throttle_limit", profile.corner_exit_throttle_limit);
    get("spin_risk_sideslip_rad", profile.spin_risk_sideslip_rad);
    get("max_chassis_twist_rad", profile.max_chassis_twist_rad);
    get("max_steering_lag_ratio", profile.max_steering_lag_ratio);
    if (j.contains("attack_overrides")) {
        profile.attack_overrides = attack_overrides_from_json(j.at("attack_overrides"));
    }
    return profile;
}

inline DriverCapabilityProfile load_driver_capability_profile(const std::string& filepath) {
    std::ifstream input(filepath);
    if (!input) {
        throw std::runtime_error("Failed to open driver capability profile: " + filepath);
    }
    nlohmann::json j;
    input >> j;
    return driver_capability_profile_from_json(j);
}

inline void save_driver_capability_profile(
    const DriverCapabilityProfile& profile,
    const std::string& filepath)
{
    std::ofstream output(filepath);
    if (!output) {
        throw std::runtime_error("Failed to write driver capability profile: " + filepath);
    }
    output << driver_capability_profile_to_json(profile).dump(2) << "\n";
}

inline void apply_attack_overrides(
    const AttackDriverOverrides& overrides,
    control::MPCConfig& config)
{
    set_if_finite(overrides.attack_speed_lateral_safety, config.attack_speed_lateral_safety);
    set_if_finite(overrides.attack_speed_braking_safety, config.attack_speed_braking_safety);
    set_if_finite(overrides.attack_speed_accel_limit, config.attack_speed_accel_limit);
    set_if_finite(overrides.attack_floor_straight, config.attack_floor_straight);
    set_if_finite(overrides.attack_floor_exit, config.attack_floor_exit);
    set_if_finite(overrides.attack_floor_complex, config.attack_floor_complex);
    set_if_finite(overrides.attack_floor_turn_in, config.attack_floor_turn_in);
    set_if_finite(overrides.attack_floor_apex, config.attack_floor_apex);
    set_if_finite(overrides.attack_floor_brake, config.attack_floor_brake);
    set_if_finite(overrides.attack_w_psi, config.w_psi);
    set_if_finite(overrides.attack_w_ax_rate, config.w_ax_rate);
}

inline AttackDriverOverrides make_attack_overrides_from_capability(
    const DriverCapabilityProfile& profile)
{
    AttackDriverOverrides overrides;
    const double mu_y = std::clamp(profile.effective_mu_y, 0.6, 2.2);
    const double mu_x_brake = std::clamp(profile.effective_mu_x_brake, 0.6, 2.2);
    const double accel_mu = std::clamp(profile.effective_mu_x_accel, 0.2, 1.2);
    const double slalom_ratio = finite_positive(profile.slalom_stable_speed_mps)
        ? std::clamp(profile.slalom_stable_speed_mps / 10.0, 0.72, 1.08)
        : 0.85;
    const bool weak_slalom = !finite_positive(profile.slalom_stable_speed_mps)
        || profile.slalom_stable_speed_mps < 9.0;
    const bool slow_yaw = finite_positive(profile.yaw_response_time_s)
        && profile.yaw_response_time_s > 0.35;
    const bool high_steering_lag = profile.max_steering_lag_ratio > 0.25;
    const double yaw_penalty = (slow_yaw || high_steering_lag) ? 0.94 : 1.0;

    if (mu_y < 1.05 || weak_slalom || slow_yaw || high_steering_lag) {
        overrides.attack_speed_lateral_safety =
            std::clamp(0.61 + 0.06 * mu_y, 0.58, 0.66);
    }
    if (mu_x_brake < 1.10 || weak_slalom) {
        overrides.attack_speed_braking_safety =
            std::clamp(0.70 + 0.06 * mu_x_brake, 0.68, 0.78);
    }
    if (accel_mu < 0.74) {
        overrides.attack_speed_accel_limit =
            std::clamp(9.81 * accel_mu * 0.75, 3.0, 5.5);
    }
    if (weak_slalom || slow_yaw || high_steering_lag) {
        overrides.attack_floor_straight = 0.94;
        overrides.attack_floor_exit = std::clamp(0.90 + 0.05 * slalom_ratio, 0.88, 0.95);
        overrides.attack_floor_complex =
            std::clamp(0.86 * slalom_ratio * yaw_penalty, 0.68, 0.86);
        overrides.attack_floor_turn_in =
            std::clamp(0.84 * slalom_ratio * yaw_penalty, 0.62, 0.84);
        overrides.attack_floor_apex = std::clamp(0.84 * slalom_ratio, 0.64, 0.84);
        overrides.attack_floor_brake = std::clamp(0.78 * slalom_ratio, 0.60, 0.78);
    }
    if (slow_yaw || high_steering_lag) {
        overrides.attack_w_psi = 2.7;
        overrides.attack_w_ax_rate = 0.0005;
    }
    return overrides;
}

} // namespace fsae::driver
