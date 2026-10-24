#pragma once
/// @file driver_benchmark.hpp
/// @brief Lightweight vehicle capability maneuvers for driver initialization.

#include "driver_capability_profile.hpp"
#include "steering_actuator_controller.hpp"

#include "../vehicle/vehicle_14dof_new.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsae::driver {

struct DriverBenchmarkConfig {
    double dt = 0.002;
    bool quick = false;
    double constant_radius_m = 15.0;
    double slalom_spacing_m = 15.0;
};

struct DriverBenchmarkSample {
    double time = 0.0;
    double speed = 0.0;
    double ay = 0.0;
    double yaw_rate = 0.0;
    double sideslip = 0.0;
    double phi = 0.0;
    double phi_twist = 0.0;
    double tire_angle = 0.0;
    double target_tire_angle = 0.0;
    double steering_lag_ratio = 0.0;
    double steering_saturation = 0.0;
};

struct ManeuverResult {
    bool stable = true;
    double max_abs_ay = 0.0;
    double max_abs_sideslip = 0.0;
    double max_abs_phi = 0.0;
    double max_abs_phi_twist = 0.0;
    double max_steering_lag_ratio = 0.0;
    double steering_saturation_ratio = 0.0;
    double mean_speed = 0.0;
    double mean_abs_yaw_rate = 0.0;
    double mean_abs_tire_angle = 0.0;
    double mean_target_tire_angle = 0.0;
    std::vector<DriverBenchmarkSample> samples;
};

inline SteeringActuatorConfig make_benchmark_steering_actuator_config(
    const vehicle::VehicleParams& params)
{
    SteeringActuatorConfig config;
    config.R_steer = std::max(params.steering.R_steer, 1e-6);
    config.max_tire_angle = std::max(params.steering.max_angle, 1e-6);
    config.max_driver_torque = std::max(params.steering.T_driver_max, 1e-6);
    config.max_tire_steer_rate = 3.0;
    config.inner_loop_omega_n = 30.0;
    config.inner_loop_zeta = 0.70;
    const double I = std::max(params.steering.I_steer, 1e-6);
    const double C = std::max(params.steering.C_steer_torque, 0.0);
    config.Kp_inner = I * config.inner_loop_omega_n * config.inner_loop_omega_n;
    config.Kd_inner = std::max(
        2.0 * config.inner_loop_zeta * config.inner_loop_omega_n * I - C,
        0.0);
    config.Ki_inner = 0.10 * config.Kp_inner;
    config.sat_feedforward_gain = 1.0;
    return config;
}

inline int benchmark_gear_for_speed(double speed) {
    if (speed < 8.0) return 2;
    if (speed < 16.0) return 3;
    if (speed < 24.0) return 4;
    return 5;
}

inline double actual_tire_angle(
    const vehicle::Vehicle14DOF_New& vehicle,
    const vehicle::VehicleParams& params,
    double angle_mode_command)
{
    if (params.steering.torque_mode) {
        return std::clamp(
            vehicle.state().delta_sw / std::max(params.steering.R_steer, 1e-6),
            -params.steering.max_angle,
            params.steering.max_angle);
    }
    return std::clamp(angle_mode_command, -params.steering.max_angle, params.steering.max_angle);
}

inline vehicle::DriverInput make_benchmark_input(
    vehicle::Vehicle14DOF_New& vehicle,
    const vehicle::VehicleParams& params,
    SteeringActuatorConfig& actuator_config,
    SteeringActuatorState& actuator_state,
    double target_tire_angle,
    double throttle,
    double brake,
    int gear,
    double dt)
{
    vehicle::DriverInput input;
    input.throttle = std::clamp(throttle, 0.0, 1.0);
    input.brake = std::clamp(brake, 0.0, 1.0);
    input.gear = gear;
    const double target = std::clamp(
        target_tire_angle,
        -std::max(params.steering.max_angle, 1e-6),
        std::max(params.steering.max_angle, 1e-6));
    if (params.steering.torque_mode) {
        const auto& state = vehicle.state();
        const auto& mz = vehicle.tire_Mz();
        const auto output = compute_steering_actuator_control(
            actuator_config,
            actuator_state,
            SteeringActuatorCommand{
                target,
                state.delta_sw,
                state.ddelta_sw,
                dt,
                mz[vehicle::Vehicle14DOF_New::FL] + mz[vehicle::Vehicle14DOF_New::FR],
            });
        input.steering = output.steering_torque;
    } else {
        input.steering = target;
    }
    return input;
}

inline void update_maneuver_result(
    ManeuverResult& result,
    const vehicle::Vehicle14DOF_New& vehicle,
    const vehicle::VehicleParams& params,
    double target_tire_angle,
    double steering_lag_ratio,
    double steering_saturation,
    double dt,
    bool store_samples)
{
    const auto& state = vehicle.state();
    const double ay = state.vx * state.r;
    const double tire_angle = actual_tire_angle(vehicle, params, target_tire_angle);
    result.max_abs_ay = std::max(result.max_abs_ay, std::abs(ay));
    result.max_abs_sideslip = std::max(result.max_abs_sideslip, std::abs(state.sideslip()));
    result.max_abs_phi = std::max(result.max_abs_phi, std::abs(state.phi));
    result.max_abs_phi_twist = std::max(result.max_abs_phi_twist, std::abs(state.phi_twist));
    result.max_steering_lag_ratio =
        std::max(result.max_steering_lag_ratio, steering_lag_ratio);
    result.steering_saturation_ratio += steering_saturation * dt;
    result.mean_speed += state.speed() * dt;
    result.mean_abs_yaw_rate += std::abs(state.r) * dt;
    result.mean_abs_tire_angle += std::abs(tire_angle) * dt;
    result.mean_target_tire_angle += std::abs(target_tire_angle) * dt;
    if (!std::isfinite(state.vx) || !std::isfinite(state.vy) || !std::isfinite(state.r)
            || std::abs(state.sideslip()) > 0.70
            || std::abs(state.phi) > 0.55
            || state.speed() < 0.4) {
        result.stable = false;
    }
    if (store_samples) {
        result.samples.push_back(DriverBenchmarkSample{
            vehicle.time(),
            state.speed(),
            ay,
            state.r,
            state.sideslip(),
            state.phi,
            state.phi_twist,
            tire_angle,
            target_tire_angle,
            steering_lag_ratio,
            steering_saturation,
        });
    }
}

inline void finalize_maneuver_result(ManeuverResult& result, double duration) {
    const double denom = std::max(duration, 1e-6);
    result.mean_speed /= denom;
    result.mean_abs_yaw_rate /= denom;
    result.mean_abs_tire_angle /= denom;
    result.mean_target_tire_angle /= denom;
    result.steering_saturation_ratio =
        std::clamp(result.steering_saturation_ratio / denom, 0.0, 1.0);
}

inline std::pair<double, double> benchmark_speed_control(double target_speed, double current_speed) {
    const double error = target_speed - current_speed;
    if (error >= 0.0) {
        return {std::clamp(0.10 + 0.10 * error, 0.0, 0.75), 0.0};
    }
    return {0.0, std::clamp(-0.10 * error, 0.0, 0.85)};
}

inline ManeuverResult run_constant_radius_case(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config,
    double target_speed,
    bool store_samples = false)
{
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.set_initial_velocity(std::max(1.5, target_speed * 0.95));
    SteeringActuatorConfig actuator_config = make_benchmark_steering_actuator_config(params);
    SteeringActuatorState actuator_state;
    const double duration = config.quick ? 2.4 : 4.0;
    const double dt = std::clamp(config.dt, 0.001, 0.010);
    const double radius = std::max(config.constant_radius_m, 3.0);
    const double kinematic_delta =
        std::atan(params.mass.wheelbase / radius);
    double target_delta = kinematic_delta;
    ManeuverResult result;

    while (vehicle.time() < duration && result.stable) {
        const auto& state = vehicle.state();
        const double desired_yaw_rate = target_speed / radius;
        const double yaw_error = desired_yaw_rate - std::abs(state.r);
        target_delta = std::clamp(
            target_delta + 0.70 * yaw_error * dt,
            -params.steering.max_angle,
            params.steering.max_angle);
        const auto [throttle, brake] = benchmark_speed_control(target_speed, state.vx);
        auto input = make_benchmark_input(
            vehicle,
            params,
            actuator_config,
            actuator_state,
            target_delta,
            throttle,
            brake,
            benchmark_gear_for_speed(target_speed),
            dt);
        vehicle.step(dt, input);
        const double tire_angle = actual_tire_angle(vehicle, params, target_delta);
        const double lag_ratio = std::clamp(
            std::abs(target_delta - tire_angle) / std::max(params.steering.max_angle, 1e-6),
            0.0,
            1.0);
        const double saturation = params.steering.torque_mode
            ? (std::abs(input.steering) >= 0.999 * std::max(params.steering.T_driver_max, 1e-6)
                ? 1.0 : 0.0)
            : 0.0;
        update_maneuver_result(
            result,
            vehicle,
            params,
            target_delta,
            lag_ratio,
            saturation,
            dt,
            store_samples);
        if (target_delta > 0.98 * params.steering.max_angle
                && std::abs(state.r) < 0.75 * desired_yaw_rate) {
            result.stable = false;
        }
    }
    finalize_maneuver_result(result, std::min(vehicle.time(), duration));
    return result;
}

inline ManeuverResult run_step_steer_case(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config,
    double target_speed,
    double step_tire_angle,
    bool store_samples = false)
{
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.set_initial_velocity(target_speed);
    SteeringActuatorConfig actuator_config = make_benchmark_steering_actuator_config(params);
    SteeringActuatorState actuator_state;
    const double duration = config.quick ? 2.2 : 3.2;
    const double step_time = 0.45;
    const double dt = std::clamp(config.dt, 0.001, 0.010);
    ManeuverResult result;

    while (vehicle.time() < duration && result.stable) {
        const auto& state = vehicle.state();
        const double target_delta = vehicle.time() >= step_time ? step_tire_angle : 0.0;
        const auto [throttle, brake] = benchmark_speed_control(target_speed, state.vx);
        auto input = make_benchmark_input(
            vehicle,
            params,
            actuator_config,
            actuator_state,
            target_delta,
            throttle,
            brake,
            benchmark_gear_for_speed(target_speed),
            dt);
        vehicle.step(dt, input);
        const double tire_angle = actual_tire_angle(vehicle, params, target_delta);
        const double lag_ratio = std::clamp(
            std::abs(target_delta - tire_angle) / std::max(params.steering.max_angle, 1e-6),
            0.0,
            1.0);
        const double saturation = params.steering.torque_mode
            ? (std::abs(input.steering) >= 0.999 * std::max(params.steering.T_driver_max, 1e-6)
                ? 1.0 : 0.0)
            : 0.0;
        update_maneuver_result(
            result,
            vehicle,
            params,
            target_delta,
            lag_ratio,
            saturation,
            dt,
            store_samples);
    }
    finalize_maneuver_result(result, std::min(vehicle.time(), duration));
    return result;
}

inline double estimate_yaw_response_time(const ManeuverResult& result, double step_time) {
    double peak = 0.0;
    for (const auto& sample : result.samples) {
        if (sample.time >= step_time) {
            peak = std::max(peak, std::abs(sample.yaw_rate));
        }
    }
    if (peak <= 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    const double threshold = 0.63 * peak;
    for (const auto& sample : result.samples) {
        if (sample.time >= step_time && std::abs(sample.yaw_rate) >= threshold) {
            return sample.time - step_time;
        }
    }
    return std::numeric_limits<double>::infinity();
}

inline double estimate_steering_response_time(const ManeuverResult& result, double step_time) {
    double target = 0.0;
    for (const auto& sample : result.samples) {
        target = std::max(target, std::abs(sample.target_tire_angle));
    }
    if (target <= 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    const double threshold = 0.90 * target;
    for (const auto& sample : result.samples) {
        if (sample.time >= step_time && std::abs(sample.tire_angle) >= threshold) {
            return sample.time - step_time;
        }
    }
    return std::numeric_limits<double>::infinity();
}

inline double estimate_yaw_overshoot_ratio(const ManeuverResult& result, double step_time) {
    if (result.samples.empty()) {
        return 0.0;
    }
    double peak = 0.0;
    double steady_sum = 0.0;
    int steady_count = 0;
    for (const auto& sample : result.samples) {
        if (sample.time >= step_time) {
            peak = std::max(peak, std::abs(sample.yaw_rate));
        }
        if (sample.time >= result.samples.back().time - 0.45) {
            steady_sum += std::abs(sample.yaw_rate);
            ++steady_count;
        }
    }
    const double steady = steady_count > 0 ? steady_sum / steady_count : peak;
    return steady > 1e-9 ? std::max(0.0, peak / steady - 1.0) : 0.0;
}

inline ManeuverResult run_slalom_case(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config,
    double target_speed,
    bool store_samples = false)
{
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.set_initial_velocity(target_speed);
    SteeringActuatorConfig actuator_config = make_benchmark_steering_actuator_config(params);
    SteeringActuatorState actuator_state;
    const double dt = std::clamp(config.dt, 0.001, 0.010);
    const double spacing = std::max(config.slalom_spacing_m, 6.0);
    const double frequency = std::clamp(target_speed / (2.0 * spacing), 0.15, 1.4);
    const double requested_reversal_window = (config.quick ? 2.2 : 2.8) / frequency;
    const double duration = std::clamp(
        requested_reversal_window,
        config.quick ? 3.0 : 5.0,
        config.quick ? 8.0 : 12.0);
    const double amplitude = std::min(0.16, 0.45 * params.steering.max_angle);
    ManeuverResult result;
    int sign_changes = 0;
    double previous_sign = 0.0;
    double saturation_time = 0.0;

    while (vehicle.time() < duration && result.stable) {
        const auto& state = vehicle.state();
        const double target_delta = amplitude * std::sin(2.0 * M_PI * frequency * vehicle.time());
        const auto [throttle, brake] = benchmark_speed_control(target_speed, state.vx);
        auto input = make_benchmark_input(
            vehicle,
            params,
            actuator_config,
            actuator_state,
            target_delta,
            throttle,
            brake,
            benchmark_gear_for_speed(target_speed),
            dt);
        vehicle.step(dt, input);
        const double tire_angle = actual_tire_angle(vehicle, params, target_delta);
        const double lag_ratio = std::clamp(
            std::abs(target_delta - tire_angle) / std::max(params.steering.max_angle, 1e-6),
            0.0,
            1.0);
        const double saturation = params.steering.torque_mode
            ? (std::abs(input.steering) >= 0.999 * std::max(params.steering.T_driver_max, 1e-6)
                ? 1.0 : 0.0)
            : 0.0;
        saturation_time += saturation * dt;
        const double current_sign = tire_angle > 0.01 ? 1.0 : (tire_angle < -0.01 ? -1.0 : 0.0);
        if (current_sign != 0.0 && previous_sign != 0.0 && current_sign != previous_sign) {
            ++sign_changes;
        }
        if (current_sign != 0.0) {
            previous_sign = current_sign;
        }
        update_maneuver_result(
            result,
            vehicle,
            params,
            target_delta,
            lag_ratio,
            saturation,
            dt,
            store_samples);
        const double saturation_ratio =
            saturation_time / std::max(vehicle.time(), dt);
        if (result.max_steering_lag_ratio > 0.85 || saturation_ratio > 0.30) {
            result.stable = false;
        }
    }
    if (sign_changes < 2) {
        result.stable = false;
    }
    finalize_maneuver_result(result, std::min(vehicle.time(), duration));
    return result;
}

inline double run_corner_exit_throttle_limit(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config,
    double entry_speed)
{
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.set_initial_velocity(std::max(3.0, entry_speed));
    SteeringActuatorConfig actuator_config = make_benchmark_steering_actuator_config(params);
    SteeringActuatorState actuator_state;
    const double dt = std::clamp(config.dt, 0.001, 0.010);
    const double duration = config.quick ? 2.8 : 4.0;
    const double radius = std::max(config.constant_radius_m, 3.0);
    double target_delta = std::atan(params.mass.wheelbase / radius);
    double throttle_limit = 1.0;

    while (vehicle.time() < duration) {
        const auto& state = vehicle.state();
        const double throttle = std::clamp(0.12 + vehicle.time() / duration, 0.12, 1.0);
        const double desired_yaw_rate = std::max(state.vx, 1.0) / radius;
        target_delta = std::clamp(
            target_delta + 0.60 * (desired_yaw_rate - std::abs(state.r)) * dt,
            -params.steering.max_angle,
            params.steering.max_angle);
        auto input = make_benchmark_input(
            vehicle,
            params,
            actuator_config,
            actuator_state,
            target_delta,
            throttle,
            0.0,
            benchmark_gear_for_speed(state.vx),
            dt);
        vehicle.step(dt, input);
        const double rear_omega = 0.5 * (vehicle.state().omega[vehicle::Vehicle14DOF_New::RL]
            + vehicle.state().omega[vehicle::Vehicle14DOF_New::RR]);
        const double driven_speed = rear_omega * params.tire.radius;
        const double slip_ratio =
            (driven_speed - std::max(vehicle.state().vx, 0.0))
            / std::max(std::abs(vehicle.state().vx), 0.5);
        if (!std::isfinite(slip_ratio)
                || std::abs(slip_ratio) > 0.18
                || std::abs(vehicle.state().sideslip()) > 0.40) {
            throttle_limit = std::clamp(throttle - 0.04, 0.0, 1.0);
            break;
        }
    }
    return throttle_limit;
}

inline std::pair<double, double> run_accel_brake_mu(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config)
{
    const double dt = std::clamp(config.dt, 0.001, 0.010);
    double max_accel = 0.0;
    {
        vehicle::Vehicle14DOF_New vehicle(params);
        vehicle.set_initial_velocity(2.0);
        double previous_vx = vehicle.state().vx;
        const double duration = config.quick ? 1.8 : 2.8;
        while (vehicle.time() < duration) {
            vehicle::DriverInput input;
            input.throttle = 1.0;
            input.gear = benchmark_gear_for_speed(vehicle.state().vx);
            vehicle.step(dt, input);
            const double ax = (vehicle.state().vx - previous_vx) / dt;
            if (vehicle.time() > 0.15 && std::isfinite(ax)) {
                max_accel = std::max(max_accel, ax);
            }
            previous_vx = vehicle.state().vx;
        }
    }

    double max_decel = 0.0;
    {
        vehicle::Vehicle14DOF_New vehicle(params);
        vehicle.set_initial_velocity(12.0);
        double previous_vx = vehicle.state().vx;
        const double duration = config.quick ? 1.5 : 2.2;
        while (vehicle.time() < duration && vehicle.state().vx > 1.0) {
            vehicle::DriverInput input;
            input.brake = 1.0;
            input.gear = 3;
            vehicle.step(dt, input);
            const double ax = (vehicle.state().vx - previous_vx) / dt;
            if (vehicle.time() > 0.10 && std::isfinite(ax)) {
                max_decel = std::max(max_decel, -ax);
            }
            previous_vx = vehicle.state().vx;
        }
    }
    return {max_accel / 9.81, max_decel / 9.81};
}

inline DriverCapabilityProfile run_driver_capability_benchmarks(
    const vehicle::VehicleParams& params,
    const DriverBenchmarkConfig& config = DriverBenchmarkConfig())
{
    DriverCapabilityProfile profile;
    profile.constant_radius_m = config.constant_radius_m;
    profile.slalom_spacing_m = config.slalom_spacing_m;

    const std::vector<double> corner_speeds = config.quick
        ? std::vector<double>{4.0, 6.0, 8.0, 10.0, 12.0}
        : std::vector<double>{4.0, 5.5, 7.0, 8.5, 10.0, 11.5, 13.0, 14.5, 16.0};
    std::vector<std::pair<double, double>> steer_vs_g;
    for (double speed : corner_speeds) {
        const auto result = run_constant_radius_case(params, config, speed);
        profile.max_chassis_twist_rad =
            std::max(profile.max_chassis_twist_rad, result.max_abs_phi_twist);
        profile.max_steering_lag_ratio =
            std::max(profile.max_steering_lag_ratio, result.max_steering_lag_ratio);
        if (!result.stable || result.steering_saturation_ratio > 0.20) {
            profile.spin_or_plow_speed_mps = speed;
            break;
        }
        profile.stable_corner_speed_mps = speed;
        profile.effective_mu_y = std::max(profile.effective_mu_y, result.max_abs_ay / 9.81);
        if (result.max_abs_ay > 0.5) {
            const double kinematic_delta = std::atan(params.mass.wheelbase / config.constant_radius_m);
            const double delta_excess = std::max(0.0, result.mean_target_tire_angle - kinematic_delta);
            steer_vs_g.emplace_back(result.max_abs_ay / 9.81, delta_excess);
        }
    }
    if (profile.spin_or_plow_speed_mps <= 0.0) {
        profile.spin_or_plow_speed_mps = corner_speeds.back() + 1.5;
    }
    if (steer_vs_g.size() >= 2) {
        const auto& first = steer_vs_g.front();
        const auto& last = steer_vs_g.back();
        profile.understeer_gradient_rad_per_g =
            (last.second - first.second) / std::max(last.first - first.first, 1e-6);
    }

    const double step_speed = std::clamp(
        profile.stable_corner_speed_mps > 0.0 ? 0.85 * profile.stable_corner_speed_mps : 8.0,
        6.0,
        12.0);
    const double step_delta = std::min(0.10, 0.35 * params.steering.max_angle);
    const auto step = run_step_steer_case(params, config, step_speed, step_delta, true);
    profile.yaw_response_time_s = estimate_yaw_response_time(step, 0.45);
    profile.yaw_overshoot_ratio = estimate_yaw_overshoot_ratio(step, 0.45);
    profile.steering_response_time_s = estimate_steering_response_time(step, 0.45);
    profile.max_chassis_twist_rad =
        std::max(profile.max_chassis_twist_rad, step.max_abs_phi_twist);
    profile.max_steering_lag_ratio =
        std::max(profile.max_steering_lag_ratio, step.max_steering_lag_ratio);
    profile.spin_risk_sideslip_rad =
        std::max(profile.spin_risk_sideslip_rad, step.max_abs_sideslip);

    const std::vector<double> slalom_speeds = config.quick
        ? std::vector<double>{5.0, 7.0, 9.0, 11.0}
        : std::vector<double>{5.0, 6.5, 8.0, 9.5, 11.0, 12.5, 14.0};
    double previous_stable = 0.0;
    for (double speed : slalom_speeds) {
        const auto slalom = run_slalom_case(params, config, speed);
        profile.max_chassis_twist_rad =
            std::max(profile.max_chassis_twist_rad, slalom.max_abs_phi_twist);
        profile.max_steering_lag_ratio =
            std::max(profile.max_steering_lag_ratio, slalom.max_steering_lag_ratio);
        profile.spin_risk_sideslip_rad =
            std::max(profile.spin_risk_sideslip_rad, slalom.max_abs_sideslip);
        if (!slalom.stable) {
            break;
        }
        previous_stable = speed;
    }
    profile.slalom_stable_speed_mps = previous_stable;
    profile.steering_reversal_time_s =
        finite_positive(profile.slalom_stable_speed_mps)
            ? config.slalom_spacing_m / profile.slalom_stable_speed_mps
            : 0.0;

    const double exit_speed = std::clamp(
        profile.stable_corner_speed_mps > 0.0 ? 0.80 * profile.stable_corner_speed_mps : 7.0,
        4.0,
        10.0);
    profile.corner_exit_throttle_limit =
        run_corner_exit_throttle_limit(params, config, exit_speed);
    const auto [mu_x_accel, mu_x_brake] = run_accel_brake_mu(params, config);
    profile.effective_mu_x_accel = mu_x_accel;
    profile.effective_mu_x_brake = mu_x_brake;
    profile.attack_overrides = make_attack_overrides_from_capability(profile);
    return profile;
}

} // namespace fsae::driver
