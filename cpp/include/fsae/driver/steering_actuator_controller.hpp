#pragma once
/// @file steering_actuator_controller.hpp
/// @brief Shared steering actuator layer for tire-angle commands in torque mode.

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsae::driver {

struct SteeringActuatorConfig {
    double Kp_inner = 50.0;              // [Nm/rad]
    double Ki_inner = 5.0;               // [Nm/(rad*s)]
    double Kd_inner = 1.5;               // [Nm*s/rad]
    double R_steer = 4.5;                // steering wheel / tire angle
    double max_tire_angle = 0.35;        // [rad]
    double max_driver_torque = 25.0;     // [Nm]
    double max_tire_steer_rate = 3.0;    // [rad/s]
    double inner_loop_omega_n = 30.0;    // [rad/s]
    double inner_loop_zeta = 0.70;       // [-]
    double lag_lead_gain = 0.0;          // [-]
    double lag_lead_time = 0.0;          // [s], <=0 derives from omega_n
    double lag_lead_max_tire_angle = 0.06; // [rad]
    double integral_limit = 0.5;         // [rad*s]
    double sat_feedforward_gain = 1.0;   // [-], cancel measured reflected SAT disturbance
};

struct SteeringActuatorState {
    double prev_target_sw = 0.0;
    double inner_integral = 0.0;

    void reset() {
        prev_target_sw = 0.0;
        inner_integral = 0.0;
    }
};

struct SteeringActuatorCommand {
    double target_tire_angle = 0.0; // [rad]
    double delta_sw = 0.0;          // [rad]
    double ddelta_sw = 0.0;         // [rad/s]
    double dt = 0.001;              // [s]
    double T_SAT_total = 0.0;       // [Nm], summed tire self-aligning torque
};

struct SteeringActuatorOutput {
    double steering_torque = 0.0;   // [Nm], saturated command
    double target_sw = 0.0;         // [rad]
    double actual_delta_tire = 0.0; // [rad]
    double lag_lead_sw = 0.0;       // [rad]
    double sw_error = 0.0;          // [rad]
    double T_SAT_feedforward = 0.0; // [Nm], reflected SAT compensation
    double T_driver_raw = 0.0;      // [Nm]
    double T_driver_clamped = 0.0;  // [Nm]
    double torque_headroom = 1.0;   // [-]
    double steering_saturation = 0.0; // 0 or 1
    double steering_lag_ratio = 0.0;  // [-]
};

struct SteeringActuatorPredictionState {
    double tire_angle = 0.0; // [rad]
    double tire_rate = 0.0;  // [rad/s]
};

/// Predict the normalized closed-loop steering response. This model is for
/// controller rollouts: the 14DOF plant still integrates steering inertia,
/// damping, friction, SAT and driver torque directly.
inline SteeringActuatorPredictionState predict_steering_actuator_response(
    const SteeringActuatorConfig& config,
    SteeringActuatorPredictionState state,
    double target_tire_angle,
    double horizon)
{
    const double max_tire_angle = std::max(config.max_tire_angle, 0.0);
    const double max_tire_rate = std::max(config.max_tire_steer_rate, 0.0);
    const double omega_n = std::max(config.inner_loop_omega_n, 1e-6);
    const double zeta = std::max(config.inner_loop_zeta, 0.0);
    const double target = std::clamp(target_tire_angle, -max_tire_angle, max_tire_angle);
    const double duration = std::max(horizon, 0.0);
    if (duration <= 1e-12) {
        state.tire_angle = std::clamp(state.tire_angle, -max_tire_angle, max_tire_angle);
        state.tire_rate = std::clamp(state.tire_rate, -max_tire_rate, max_tire_rate);
        return state;
    }

    constexpr double kMaxIntegrationStep = 0.0025; // [s]
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / kMaxIntegrationStep)));
    const double dt = duration / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
        const double tire_accel =
            omega_n * omega_n * (target - state.tire_angle)
          - 2.0 * zeta * omega_n * state.tire_rate;
        state.tire_rate = std::clamp(
            state.tire_rate + tire_accel * dt,
            -max_tire_rate,
            max_tire_rate);
        state.tire_angle += state.tire_rate * dt;
        if (state.tire_angle > max_tire_angle) {
            state.tire_angle = max_tire_angle;
            state.tire_rate = std::min(state.tire_rate, 0.0);
        } else if (state.tire_angle < -max_tire_angle) {
            state.tire_angle = -max_tire_angle;
            state.tire_rate = std::max(state.tire_rate, 0.0);
        }
    }
    return state;
}

inline double estimate_steering_actuator_reach_time(
    const SteeringActuatorConfig& config,
    SteeringActuatorPredictionState state,
    double target_tire_angle,
    double deadband = 0.02,
    double max_horizon = 1.0)
{
    const double target = std::clamp(
        target_tire_angle,
        -std::max(config.max_tire_angle, 0.0),
        std::max(config.max_tire_angle, 0.0));
    const double tolerance = std::max(deadband, 0.0);
    double previous_error = target - state.tire_angle;
    if (std::abs(previous_error) <= tolerance) {
        return 0.0;
    }

    constexpr double kPredictionStep = 0.0025; // [s]
    const double horizon = std::max(max_horizon, 0.0);
    for (double t = kPredictionStep; t <= horizon + 1e-12; t += kPredictionStep) {
        state = predict_steering_actuator_response(config, state, target, kPredictionStep);
        const double error = target - state.tire_angle;
        if (std::abs(error) <= tolerance || error * previous_error <= 0.0) {
            return std::min(t, horizon);
        }
        previous_error = error;
    }
    return std::numeric_limits<double>::infinity();
}

inline double steering_actuator_lag_lead_time(const SteeringActuatorConfig& config) {
    if (config.lag_lead_time > 1e-6) {
        return std::clamp(config.lag_lead_time, 0.0, 0.150);
    }
    return std::clamp(
        1.0 / std::max(config.inner_loop_omega_n, 1e-6),
        0.010,
        0.080);
}

inline SteeringActuatorOutput compute_steering_actuator_control(
    const SteeringActuatorConfig& config,
    SteeringActuatorState& state,
    const SteeringActuatorCommand& command)
{
    SteeringActuatorOutput output;

    const double R = std::max(config.R_steer, 1e-6);
    const double max_tire_angle = std::max(config.max_tire_angle, 0.0);
    const double max_sw = max_tire_angle * R;
    double target_sw = std::clamp(command.target_tire_angle, -max_tire_angle, max_tire_angle) * R;

    const double lead_gain = std::max(config.lag_lead_gain, 0.0);
    if (lead_gain > 1e-9) {
        const double prediction_time = steering_actuator_lag_lead_time(config);
        const double predicted_sw = command.delta_sw + command.ddelta_sw * prediction_time;
        const double max_lead_sw = std::max(config.lag_lead_max_tire_angle, 0.0) * R;
        const double lead_sw = std::clamp(
            lead_gain * (target_sw - predicted_sw),
            -max_lead_sw,
            max_lead_sw);
        const double compensated_sw = std::clamp(target_sw + lead_sw, -max_sw, max_sw);
        output.lag_lead_sw = compensated_sw - target_sw;
        target_sw = compensated_sw;
    }

    const double dt = std::max(command.dt, 0.0001);
    const double max_change = std::max(config.max_tire_steer_rate, 0.0) * R * dt;
    const double target_change =
        std::clamp(target_sw - state.prev_target_sw, -max_change, max_change);
    target_sw = state.prev_target_sw + target_change;
    state.prev_target_sw = target_sw;

    const double sw_error = target_sw - command.delta_sw;
    state.inner_integral += sw_error * dt;
    state.inner_integral =
        std::clamp(state.inner_integral, -config.integral_limit, config.integral_limit);

    // command.T_SAT_total is the signed tire Mz sum. The vehicle adds it as a
    // restoring torque, so the actuator must feed forward the opposite sign.
    const double T_SAT_feedforward =
        -std::max(config.sat_feedforward_gain, 0.0) * command.T_SAT_total / R;
    const double raw_torque =
        config.Kp_inner * sw_error
      + config.Ki_inner * state.inner_integral
      - config.Kd_inner * command.ddelta_sw
      + T_SAT_feedforward;

    const double torque_limit = std::max(config.max_driver_torque, 1e-6);
    const double clamped_torque = std::clamp(raw_torque, -torque_limit, torque_limit);

    output.steering_torque = clamped_torque;
    output.target_sw = target_sw;
    output.actual_delta_tire = command.delta_sw / R;
    output.sw_error = sw_error;
    output.T_SAT_feedforward = T_SAT_feedforward;
    output.T_driver_raw = raw_torque;
    output.T_driver_clamped = clamped_torque;
    output.torque_headroom =
        std::clamp((torque_limit - std::abs(clamped_torque)) / torque_limit, 0.0, 1.0);
    output.steering_saturation = std::abs(raw_torque - clamped_torque) > 1e-9 ? 1.0 : 0.0;

    const double sw_range = std::max(max_sw, 1e-6);
    output.steering_lag_ratio = std::clamp(std::abs(sw_error) / sw_range, 0.0, 1.0);
    return output;
}

} // namespace fsae::driver
