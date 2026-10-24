#pragma once

#include "../laptime/ggv_map.hpp"
#include "../track/frenet.hpp"
#include "../track/track_model.hpp"
#include "../vehicle/vehicle_14dof_new.hpp"
#include "../vehicle/vehicle_params.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

namespace fsae::driver {

inline bool rl_driver_speed_limits_disabled() {
    const char* raw = std::getenv("FOPT_DISABLE_RL_DRIVER_SPEED_LIMITS");
    if (raw == nullptr) {
        return false;
    }

    std::string value(raw);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return !(value == "0" || value == "false" || value == "off" || value == "no");
}

inline double rl_steering_command_limit(const vehicle::VehicleParams& params) {
    return params.steering.torque_mode ? params.steering.T_driver_max
                                       : params.steering.max_angle;
}

inline double smooth_rl_steering_command(double requested,
                                         double prev_applied,
                                         double steering_limit,
                                         bool torque_mode = false,
                                         double control_dt = 0.020,
                                         double torque_rate_limit = 0.0) {
    const double limit = std::max(std::abs(steering_limit), 1e-6);
    const double deadband = 0.015 * limit;
    double target = std::clamp(requested, -limit, limit);
    if (std::abs(target) < deadband) {
        target = 0.0;
    }

    if (torque_mode) {
        if (!(std::isfinite(control_dt) && control_dt > 0.0 &&
              std::isfinite(torque_rate_limit) && torque_rate_limit > 0.0)) {
            return target;
        }

        const double max_delta = torque_rate_limit * control_dt;
        const double delta = std::clamp(target - prev_applied, -max_delta, max_delta);
        return std::clamp(prev_applied + delta, -limit, limit);
    }

    double max_delta = 0.14 * limit;
    const bool hard_reversal = (target * prev_applied < 0.0) &&
                               (std::abs(target) > 0.12 * limit) &&
                               (std::abs(prev_applied) > 0.12 * limit);
    if (hard_reversal) {
        max_delta *= 0.45;
    }

    const double delta = std::clamp(target - prev_applied, -max_delta, max_delta);
    double filtered = prev_applied + delta;

    if (std::abs(filtered) < deadband && std::abs(target) < 2.0 * deadband) {
        filtered = 0.0;
    }

    return std::clamp(filtered, -limit, limit);
}

inline void apply_rl_curvature_speed_limit(const vehicle::State14DOF& state,
                                           const track::CurvatureBasedTrack& track,
                                           track::FrenetConverter& converter,
                                           vehicle::DriverInput& input,
                                           const vehicle::VehicleParams& params = vehicle::VehicleParams::default_fsae(),
                                           const laptime::GGVMap* ggv = nullptr) {
    const double mu = std::max({params.tire.mu_x_max, params.tire.mu_y_max, 0.1});
    constexpr double g = 9.81;
    const double safety = laptime::kDefaultFrictionMargin;  // single margin (Issue #26), was 0.82
    constexpr double min_speed = 5.0;

    auto frenet = converter.global_to_frenet(
        state.y,
        state.x,
        M_PI / 2.0 - state.psi);
    const double current_speed = std::max(state.vx, 0.1);

    double min_safe_speed = 1e9;
    const double lookahead_distances[] = {5, 10, 15, 20, 30, 40, 60, 80};
    for (double d : lookahead_distances) {
        double s_ahead = frenet.s + d;
        if (s_ahead > track.length()) {
            s_ahead -= track.length();
        }
        const double kappa = std::min(std::abs(track.get_curvature(s_ahead)), 0.25);
        if (kappa > 1e-4 && ggv != nullptr) {
            // Plant-derived GGV (Issues #23/#27): same table and margin as the planner.
            const double v_max = ggv->corner_speed(kappa, safety, 60.0);
            const double braking_decel = std::max(0.1, safety * ggv->ax_brake(v_max, 0.0));
            min_safe_speed = std::min(min_safe_speed, std::sqrt(v_max * v_max + 2.0 * braking_decel * d));
        } else if (kappa > 1e-4) {
            const double radius = 1.0 / kappa;
            const double rho = params.aero.rho > 0.1 ? params.aero.rho : 1.225;
            const double denom = 1.0 / radius - 0.5 * rho * params.aero.CL_A * mu / std::max(50.0, params.mass.m_total);
            double v_max = 0.0;
            if (denom > 1e-4) {
                v_max = std::sqrt((mu * g * safety) / denom);
            } else {
                v_max = std::sqrt(mu * g * radius * safety * 2.5);
            }
            const double braking_decel = std::max(0.1, params.tire.mu_x_max) * g * safety;
            const double v_at_entry = std::sqrt(v_max * v_max + 2.0 * braking_decel * d);
            min_safe_speed = std::min(min_safe_speed, v_at_entry);
        }
    }
    min_safe_speed = std::max(min_safe_speed, min_speed);

    if (current_speed > min_safe_speed * 1.1) {
        const double overspeed_ratio = (current_speed - min_safe_speed) / min_safe_speed;
        const double brake_intensity = std::clamp(overspeed_ratio * 1.5, 0.0, 1.0);
        input.throttle = 0.0;
        input.brake = std::max(input.brake, brake_intensity);
    } else if (current_speed > min_safe_speed) {
        input.throttle *= std::clamp(
            1.0 - (current_speed - min_safe_speed) / (min_safe_speed * 0.1), 0.0, 1.0);
    }

    const auto tp = track.get_point(frenet.s);
    const double half_width = std::max(0.1, 0.5 * (tp.width_left + tp.width_right));
    const double lateral_ratio = std::abs(frenet.n) / half_width;
    const double heading_abs = std::abs(frenet.d_heading);
    const double steering_rate_abs = std::abs(state.ddelta_sw);
    const double twist_abs = std::abs(state.phi_twist);
    const double twist_rate_abs = std::abs(state.dphi_twist);

    double risk = 0.0;
    risk = std::max(risk, std::clamp((lateral_ratio - 0.78) / 0.17, 0.0, 1.0));
    risk = std::max(risk, std::clamp((heading_abs - 0.45) / 0.25, 0.0, 1.0));
    if (params.steering.torque_mode) {
        // Relax steering rate threshold to 45 rad/s in torque mode to avoid false-positives
        risk = std::max(risk, std::clamp((steering_rate_abs - 45.0) / 25.0, 0.0, 1.0));
    }
    if (params.suspension.K_chassis >= 5000.0) {
        risk = std::max(risk, std::clamp((twist_abs - 0.05) / 0.05, 0.0, 1.0));
        risk = std::max(risk, std::clamp((twist_rate_abs - 1.5) / 1.5, 0.0, 1.0));
    } else {
        // Relax thresholds significantly for flexible chassis to prevent false-positives
        risk = std::max(risk, std::clamp((twist_abs - 0.25) / 0.25, 0.0, 1.0));
        risk = std::max(risk, std::clamp((twist_rate_abs - 8.0) / 8.0, 0.0, 1.0));
    }

    if (risk > 0.0) {
        input.throttle *= (1.0 - risk);
        input.brake = std::max(input.brake, 0.35 * risk);
    }

    if (params.steering.torque_mode) {
        const bool recovery_steering_needed = current_speed < 4.0 ||
                                              lateral_ratio > 0.35 ||
                                              heading_abs > 0.25;
        if (recovery_steering_needed) {
            const double wheelbase = std::max(0.5, params.mass.wheelbase);
            const double preview_s = std::fmod(
                frenet.s + std::clamp(current_speed * 0.8, 3.0, 10.0),
                std::max(track.length(), 1e-6));
            const double kappa = track.get_curvature(preview_s);
            const double delta_target = std::clamp(
                0.80 * frenet.n + 0.70 * frenet.d_heading - 1.30 * wheelbase * kappa,
                -0.45,
                0.45);
            const double target_sw = delta_target * std::max(params.steering.R_steer, 1e-6);
            const double recovery_torque = std::clamp(
                50.0 * (target_sw - state.delta_sw) - 1.5 * state.ddelta_sw,
                -params.steering.T_driver_max,
                params.steering.T_driver_max);

            const bool outside_recovery_band = lateral_ratio > 0.82 || heading_abs > 0.50;
            if (outside_recovery_band ||
                std::abs(recovery_torque) > std::abs(input.steering) ||
                current_speed < 2.0) {
                input.steering = recovery_torque;
            } else {
                input.steering = std::clamp(
                    0.55 * input.steering + 0.45 * recovery_torque,
                    -params.steering.T_driver_max,
                    params.steering.T_driver_max);
            }
        }

        const bool recovery_speed_needed = current_speed < 4.0 ||
                                           lateral_ratio > 0.75 ||
                                           heading_abs > 0.40;
        if (recovery_speed_needed) {
            double recovery_speed = 6.5;
            bool apply_braking_recovery = false;
            if (lateral_ratio > 0.85 || heading_abs > 0.55) {
                recovery_speed = 4.5;
                apply_braking_recovery = true;
            }
            const double speed_error = recovery_speed - current_speed;
            if (speed_error > 0.2) {
                input.brake = 0.0;
                input.throttle = std::max(input.throttle, std::clamp(0.12 * speed_error, 0.08, 0.35));
            } else if (apply_braking_recovery && speed_error < -1.0) {
                input.throttle = 0.0;
                input.brake = std::max(input.brake, std::clamp(-0.20 * speed_error, 0.0, 0.35));
            }
        }

        const double cruise_speed = std::clamp(0.80 * min_safe_speed, 6.0, 10.5);
        const bool stable_low_speed = current_speed < cruise_speed &&
                                      lateral_ratio < 0.20 &&
                                      heading_abs < 0.12;
        if (stable_low_speed) {
            input.brake = 0.0;
            const double throttle_floor = std::clamp(0.06 + 0.04 * (cruise_speed - current_speed), 0.06, 0.30);
            input.throttle = std::max(input.throttle, throttle_floor);
        }

        // Virtual hard stop for steering wheel angle to prevent out-of-bounds spin/explosion
        const double max_sw = params.steering.max_angle * params.steering.R_steer;
        const double sw_margin = 0.02;
        if (std::abs(state.delta_sw) > max_sw - sw_margin) {
            const double excess = std::abs(state.delta_sw) - (max_sw - sw_margin);
            const double sign = state.delta_sw > 0 ? 1.0 : -1.0;
            // Strong spring-damping torque opposing the excess angle
            const double restoring_torque = -180.0 * excess * sign - 4.5 * state.ddelta_sw;
            
            if (input.steering * sign > 0.0) {
                // If driver is steering further out, override with restoring torque
                input.steering = restoring_torque;
            } else {
                // If driver is steering back, mix restoring torque to add damping and assist return
                input.steering += restoring_torque;
            }
            input.steering = std::clamp(input.steering, -params.steering.T_driver_max, params.steering.T_driver_max);
        }
    }
}

}  // namespace fsae::driver
