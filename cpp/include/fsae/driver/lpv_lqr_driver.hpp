#pragma once
/// @file lpv_lqr_driver.hpp
/// @brief Vehicle-parameter-conditioned preview LPV-LQR driver.
///
/// The 14DOF model remains the truth plant.  This controller uses only a
/// linear bicycle error model and a precomputed speed-scheduled LQR gain table
/// at runtime. Torque mode augments the four vehicle states with measured tire
/// angle and tire-angle rate. Driver steering follows the repository convention
/// (positive = right); the LQR model follows the standard map convention
/// (positive lateral/yaw/steering = left).

#include "gear_selection.hpp"
#include "analytic_safety_filter.hpp"
#include "mpc_driver.hpp"
#include "steering_actuator_controller.hpp"
#include "../track/track_distance.hpp"
#include "../vehicle/bicycle_model.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace fsae::driver {

struct LPVLQRConfig {
    double control_dt = 0.025;              // [s], 40 Hz outer loop
    double min_model_speed = 3.0;           // [m/s]
    double max_model_speed = 40.0;          // [m/s]
    double gain_grid_step = 1.0;            // [m/s]
    // 0.32 s let the reference run ahead of the car through autocross
    // slaloms (peak |n| at curvature sign changes); 0.20 s cut that peak.
    double preview_time = 0.20;             // [s]
    double min_preview_distance = 2.5;      // [m]
    double max_preview_distance = 14.0;     // [m]
    // Weight of the preview-point curvature in the LQR reference curvature
    // (rest: current point). 0.70 made the car cut ahead of the centerline
    // and set the autocross max |n|; 0.20 keeps the lead without the cut.
    double preview_curvature_weight = 0.20; // [-]
    double speed_preview_time = 1.25;       // [s]
    double max_speed_preview_distance = 30.0; // [m]
    // Speed preview is braking-aware: a slower point ds ahead only caps the
    // target at sqrt(v^2 + 2*a*ds), instead of forcing its speed immediately.
    double speed_preview_decel = 4.0;       // [m/s^2]
    double speed_preview_step = 1.0;        // [m]

    // State cost: [lateral error, heading error, lateral velocity, yaw-rate error].
    std::array<double, 4> q = {9.0, 18.0, 0.35, 1.8};
    double r_steer = 2.2;

    // Torque-mode cost for [e_y, e_psi, dv_y, dr, ddelta, delta_rate]. The path
    // weights were tuned after the paired four-/six-state experiment; actuator
    // weights and R remain conservative to avoid trading lag for saturation.
    std::array<double, 6> actuator_q = {22.5, 24.0, 0.35, 1.8, 0.5, 0.05};
    double actuator_r_steer = 2.2;
    bool use_actuator_augmented_lqr = true;
    double steering_inner_loop_omega_n = 30.0; // [rad/s]
    double steering_inner_loop_zeta = 0.70;    // [-]
    // The outer safety layer owns the target-angle slew envelope. The 1 kHz
    // layer may follow that already-safe target faster to avoid a duplicate
    // 25 ms ramp and its added phase lag.
    double inner_target_steer_rate = 12.0;      // [rad/s]

    double max_steer_rate = 2.8;            // [rad/s], tire angle
    double max_lon_accel = 9.81;            // [m/s^2]
    double max_lon_decel = 1.8 * 9.81;      // [m/s^2]
    double speed_kp = 3.0;                  // [1/s], 0.85 left ~1 m/s mean underspeed
    double speed_ki = 0.12;                 // [1/s^2]
    double speed_integral_limit = 3.0;      // [m/s]

    // Lightweight safety filter.  These are conservative operational guards,
    // not a formal CBF guarantee.
    double lateral_accel_fraction = 0.86;   // fraction of mu_y * g (centerline curvature)
    // Curvature cap is braking-aware: a corner ds ahead allows
    // sqrt(v_corner^2 + 2*a*ds). Above ~4 m/s^2 the speed P-loop overshoots.
    double curvature_cap_decel = 3.0;       // [m/s^2]
    double soft_track_margin = 0.90;        // [m]
    double hard_track_margin = 0.30;        // [m]
    double margin_speed_cap = 7.0;          // [m/s]
    double max_sideslip = 0.16;             // [rad]
    double max_yaw_error = 0.55;             // [rad/s]
    double stability_speed_cap = 8.0;       // [m/s]
};

struct LPVEquilibrium {
    double lateral_velocity = 0.0; // [m/s], map left-positive
    double heading_error = 0.0;    // [rad], equilibrium body sideslip compensation
    double yaw_rate = 0.0;         // [rad/s], CCW positive
    double steering = 0.0;         // [rad], SAE left-positive
};

class PreviewLPVLQRDriver {
public:
    using StateMatrix = Eigen::Matrix<double, 4, 4>;
    using InputMatrix = Eigen::Matrix<double, 4, 1>;
    using Gain = Eigen::Matrix<double, 1, 4>;
    using ActuatorStateMatrix = Eigen::Matrix<double, 6, 6>;
    using ActuatorInputMatrix = Eigen::Matrix<double, 6, 1>;
    using ActuatorGain = Eigen::Matrix<double, 1, 6>;

    PreviewLPVLQRDriver(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const LPVLQRConfig& config = LPVLQRConfig(),
        const vehicle::VehicleParams& vehicle_params = vehicle::VehicleParams::default_fsae())
        : track_(track)
        , track_query_(track, 0.5)
        , config_(config)
        , vehicle_params_(vehicle_params)
        , bicycle_params_(vehicle::BicycleParams::from_vehicle_params(vehicle_params))
        , safety_filter_(make_safety_filter_config())
    {
        velocity_sv_.reserve(velocity_profile.size());
        for (const auto& point : velocity_profile) {
            velocity_sv_.emplace_back(point.s, point.v);
        }
        rebuild_gain_table();
        steering_actuator_config_ = make_steering_actuator_config();
    }

    vehicle::DriverInput compute_outer_control(
        const vehicle::VehicleState& state,
        double current_time)
    {
        const auto start = std::chrono::steady_clock::now();
        const double steering_ratio = std::max(vehicle_params_.steering.R_steer, 1e-6);
        const double actual_driver_tire_angle = last_vehicle_delta_sw_ / steering_ratio;
        const double actual_driver_tire_rate = last_vehicle_ddelta_sw_ / steering_ratio;
        double aligned_prediction_error = 0.0;
        bool aligned_prediction_error_valid = false;
        if (pending_prediction_valid_
                && std::abs(current_time - pending_prediction_target_time_)
                    <= std::max(0.5 * config_.control_dt, 1e-6)) {
            aligned_prediction_error =
                pending_predicted_tire_angle_ - actual_driver_tire_angle;
            if (std::isfinite(aligned_prediction_error)) {
                ++stats_.actuator_prediction_count;
                stats_.actuator_prediction_error_sum += aligned_prediction_error;
                stats_.actuator_prediction_abs_error_sum +=
                    std::abs(aligned_prediction_error);
                stats_.actuator_prediction_sq_error_sum +=
                    aligned_prediction_error * aligned_prediction_error;
                stats_.actuator_prediction_max_abs_error = std::max(
                    stats_.actuator_prediction_max_abs_error,
                    std::abs(aligned_prediction_error));
                aligned_prediction_error_valid = true;
            }
        }
        pending_prediction_valid_ = false;
        const auto nearest = track_query_.nearest(state.x, state.y, hint_s_);
        hint_s_ = nearest.s;
        const auto point = track_.get_point(nearest.s);
        const double heading_error = normalize_angle(state.psi - nearest.theta);
        const double speed = std::max(state.vx, 0.0);
        const double preview_distance = std::clamp(
            speed * config_.preview_time,
            config_.min_preview_distance,
            config_.max_preview_distance);
        const double preview_s = wrap_s(nearest.s + preview_distance);
        const auto preview_point = track_.get_point(preview_s);
        const double reference_curvature =
            (1.0 - config_.preview_curvature_weight) * nearest.kappa
            + config_.preview_curvature_weight * preview_point.kappa;

        double sae_steering = 0.0;
        double required_sae_steering = 0.0;
        double model_yaw_rate = speed * reference_curvature;
        if (speed < config_.min_model_speed) {
            sae_steering = low_speed_steering(state, preview_point, preview_distance);
            required_sae_steering = sae_steering;
        } else {
            const LPVEquilibrium equilibrium = steady_state_reference(speed, reference_curvature);
            required_sae_steering = equilibrium.steering;
            model_yaw_rate = equilibrium.yaw_rate;
            if (vehicle_params_.steering.torque_mode
                    && config_.use_actuator_augmented_lqr) {
                // Measured steering-wheel states use the driver right-positive
                // convention. Convert them to the LQR's SAE left-positive frame.
                const double actual_sae_tire_angle = -last_vehicle_delta_sw_ / steering_ratio;
                const double actual_sae_tire_rate = -last_vehicle_ddelta_sw_ / steering_ratio;
                Eigen::Matrix<double, 6, 1> error;
                error << nearest.n,
                         heading_error - equilibrium.heading_error,
                         state.vy - equilibrium.lateral_velocity,
                         state.r - equilibrium.yaw_rate,
                         actual_sae_tire_angle - equilibrium.steering,
                         actual_sae_tire_rate;
                sae_steering = equilibrium.steering
                             - interpolated_actuator_gain(speed) * error;
            } else {
                Eigen::Matrix<double, 4, 1> error;
                error << nearest.n,
                         heading_error - equilibrium.heading_error,
                         state.vy - equilibrium.lateral_velocity,
                         state.r - equilibrium.yaw_rate;
                sae_steering = equilibrium.steering - interpolated_gain(speed) * error;
            }
        }

        // Convert standard SAE/map left-positive steering to the driver's
        // right-positive convention before applying the independent guards.
        // LQR is unconstrained, so large transient errors can otherwise
        // produce physically meaningless multi-radian requests. Enforce the
        // rack-angle contract here; the independent safety filter below still
        // applies its rate, stability, and track-margin projections.
        const double requested_driver_steering = std::clamp(
            -sae_steering,
            -std::max(vehicle_params_.steering.max_angle, 1e-6),
            std::max(vehicle_params_.steering.max_angle, 1e-6));
        const double lateral_margin = track_margin(point, nearest.n);

        const double base_target_speed = lookup_target_speed(nearest.s);
        const double preview_target_speed = lookup_preview_min_speed(nearest.s, speed);
        const double curvature_cap = curvature_speed_cap(nearest.s, speed);
        const double beta = std::atan2(state.vy, std::max(speed, 0.5));
        const double yaw_error = state.r - speed * nearest.kappa;
        const auto safe = safety_filter_.apply(
            AnalyticSafetyRequest{
                requested_driver_steering,
                std::min({base_target_speed, preview_target_speed, curvature_cap}),
                base_target_speed,
            },
            AnalyticSafetyObservation{
                nearest.n,
                heading_error,
                lateral_margin,
                speed,
                beta,
                yaw_error,
            },
            config_.control_dt);
        double target_driver_steering = safe.target_tire_angle;
        const double target_speed = safe.target_speed;
        const auto predicted_steering = predict_steering_state(
            actual_driver_tire_angle,
            actual_driver_tire_rate,
            target_driver_steering,
            config_.control_dt);
        const bool steering_prediction_valid = vehicle_params_.steering.torque_mode
            && std::isfinite(predicted_steering.first)
            && std::isfinite(predicted_steering.second);
        if (steering_prediction_valid) {
            pending_predicted_tire_angle_ = predicted_steering.first;
            pending_prediction_target_time_ = current_time + config_.control_dt;
            pending_prediction_valid_ = true;
        }

        vehicle::DriverInput input = longitudinal_control(target_speed, speed);
        input.steering = vehicle_params_.steering.torque_mode ? 0.0 : target_driver_steering;
        input.gear = select_gear_from_vehicle_params(speed, vehicle_params_);

        const bool finite = std::isfinite(target_driver_steering)
                         && std::isfinite(target_speed)
                         && std::isfinite(input.throttle)
                         && std::isfinite(input.brake);
        if (!finite) {
            ++stats_.fail_count;
            target_driver_steering = 0.0;
            input.steering = 0.0;
            input.throttle = 0.0;
            input.brake = 0.5;
        }
        prev_delta_ = target_driver_steering;

        last_debug_ = MPCDebugInfo{};
        last_debug_.target_tire_angle = target_driver_steering;
        last_debug_.raw_target_tire_angle = requested_driver_steering;
        last_debug_.required_tire_angle = -required_sae_steering;
        last_debug_.required_reference_s = preview_s;
        last_debug_.required_reference_time = current_time
            + preview_distance / std::max(speed, 0.5);
        last_debug_.actual_tire_angle_pre = actual_driver_tire_angle;
        last_debug_.actual_tire_rate_pre = actual_driver_tire_rate;
        last_debug_.predicted_tire_angle = predicted_steering.first;
        last_debug_.predicted_tire_rate = predicted_steering.second;
        last_debug_.steering_prediction_error = aligned_prediction_error;
        last_debug_.steering_prediction_horizon = config_.control_dt;
        last_debug_.steering_prediction_target_time =
            current_time + config_.control_dt;
        last_debug_.steering_prediction_valid = steering_prediction_valid;
        last_debug_.steering_prediction_error_valid =
            aligned_prediction_error_valid;
        last_debug_.outer_sequence = ++outer_sequence_;
        last_debug_.model_yaw_rate = model_yaw_rate;
        last_debug_.actual_yaw_rate = state.r;
        last_debug_.yaw_rate_error = state.r - model_yaw_rate;
        last_debug_.selected_ax = last_ax_command_;
        last_debug_.target_speed_base = base_target_speed;
        last_debug_.target_speed = target_speed;
        last_debug_.steering_speed_cap = curvature_cap;
        last_debug_.track_margin_speed_cap = safe.track_margin_speed_cap;
        last_debug_.effective_tire_steer_rate = config_.max_steer_rate;
        last_debug_.current_lateral_margin = lateral_margin;
        last_debug_.current_s = nearest.s;
        last_debug_.predicted_terminal_n = nearest.n;
        last_debug_.predicted_terminal_heading = heading_error;
        last_debug_.predicted_min_margin = lateral_margin;
        last_debug_.predicted_terminal_speed = speed;
        last_debug_.torque_mode = vehicle_params_.steering.torque_mode;

        ++stats_.solve_count;
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        stats_.total_solve_time_ms += elapsed_ms;
        stats_.max_solve_time_ms = std::max(stats_.max_solve_time_ms, elapsed_ms);
        return input;
    }

    vehicle::DriverInput update_torque_actuator_control(
        vehicle::DriverInput input,
        double delta_sw,
        double ddelta_sw,
        double dt,
        double T_SAT_total = 0.0)
    {
        if (!vehicle_params_.steering.torque_mode) {
            return input;
        }
        if (!std::isfinite(delta_sw) || !std::isfinite(ddelta_sw)
                || !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(T_SAT_total)) {
            ++stats_.fail_count;
            steering_actuator_state_.reset();
            last_vehicle_delta_sw_ = 0.0;
            last_vehicle_ddelta_sw_ = 0.0;
            input.steering = 0.0;
            input.throttle = 0.0;
            input.brake = std::max(input.brake, 0.35);
            return input;
        }
        set_steering_state(delta_sw, ddelta_sw);
        const auto output = compute_steering_actuator_control(
            steering_actuator_config_,
            steering_actuator_state_,
            SteeringActuatorCommand{
                prev_delta_, delta_sw, ddelta_sw, dt, T_SAT_total,
            });
        last_debug_.steering_actuator = output;
        input.steering = output.steering_torque;
        return input;
    }

    const MPCStats& stats() const { return stats_; }
    const MPCDebugInfo& last_debug() const { return last_debug_; }
    const vehicle::BicycleParams& bicycle_params() const { return bicycle_params_; }
    std::size_t gain_table_size() const { return gain_table_.size(); }
    std::size_t actuator_gain_table_size() const { return actuator_gain_table_.size(); }

    void set_steering_state(double delta_sw, double ddelta_sw) {
        if (std::isfinite(delta_sw) && std::isfinite(ddelta_sw)) {
            last_vehicle_delta_sw_ = delta_sw;
            last_vehicle_ddelta_sw_ = ddelta_sw;
        }
    }

    Gain gain_at_speed(double speed) const { return interpolated_gain(speed); }
    ActuatorGain actuator_gain_at_speed(double speed) const {
        return interpolated_actuator_gain(speed);
    }

    std::pair<double, double> predict_steering_state(
        double tire_angle,
        double tire_rate,
        double target_tire_angle,
        double horizon) const
    {
        const double dt = std::max(horizon, 1e-6);
        const double omega_n = std::max(config_.steering_inner_loop_omega_n, 1e-6);
        const double zeta = std::max(config_.steering_inner_loop_zeta, 0.0);
        Eigen::Matrix2d A;
        A << 0.0, 1.0,
            -omega_n * omega_n, -2.0 * zeta * omega_n;
        Eigen::Vector2d B;
        B << 0.0, omega_n * omega_n;
        const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
        const Eigen::Matrix2d lhs = identity - 0.5 * dt * A;
        const Eigen::Matrix2d Ad = lhs.fullPivLu().solve(identity + 0.5 * dt * A);
        const Eigen::Vector2d Bd = lhs.fullPivLu().solve(dt * B);
        const Eigen::Vector2d predicted =
            Ad * Eigen::Vector2d(tire_angle, tire_rate) + Bd * target_tire_angle;
        return {predicted(0), predicted(1)};
    }

    double closed_loop_spectral_radius(double speed) const {
        StateMatrix Ac;
        InputMatrix Bc;
        continuous_model(
            std::clamp(speed, config_.min_model_speed, config_.max_model_speed), Ac, Bc);
        const StateMatrix identity = StateMatrix::Identity();
        const StateMatrix lhs = identity - 0.5 * config_.control_dt * Ac;
        const StateMatrix Ad = lhs.fullPivLu().solve(
            identity + 0.5 * config_.control_dt * Ac);
        const InputMatrix Bd = lhs.fullPivLu().solve(config_.control_dt * Bc);
        const Eigen::EigenSolver<StateMatrix> solver(
            Ad - Bd * interpolated_gain(speed), false);
        return solver.eigenvalues().cwiseAbs().maxCoeff();
    }

    double actuator_closed_loop_spectral_radius(double speed) const {
        ActuatorStateMatrix Ac;
        ActuatorInputMatrix Bc;
        continuous_actuator_model(
            std::clamp(speed, config_.min_model_speed, config_.max_model_speed), Ac, Bc);
        const ActuatorStateMatrix identity = ActuatorStateMatrix::Identity();
        const ActuatorStateMatrix lhs = identity - 0.5 * config_.control_dt * Ac;
        const ActuatorStateMatrix Ad = lhs.fullPivLu().solve(
            identity + 0.5 * config_.control_dt * Ac);
        const ActuatorInputMatrix Bd = lhs.fullPivLu().solve(config_.control_dt * Bc);
        const Eigen::EigenSolver<ActuatorStateMatrix> solver(
            Ad - Bd * interpolated_actuator_gain(speed), false);
        return solver.eigenvalues().cwiseAbs().maxCoeff();
    }

    LPVEquilibrium steady_state_reference(double speed, double curvature) const {
        const double vx = std::clamp(
            speed, config_.min_model_speed, config_.max_model_speed);
        StateMatrix A;
        InputMatrix B;
        continuous_model(vx, A, B);
        const double yaw_rate = vx * curvature;
        Eigen::Matrix2d matrix;
        matrix << A(2, 2), B(2),
                  A(3, 2), B(3);
        Eigen::Vector2d rhs;
        rhs << -A(2, 3) * yaw_rate,
               -A(3, 3) * yaw_rate;
        const Eigen::Vector2d solution = matrix.fullPivLu().solve(rhs);
        if (!solution.allFinite()) {
            return {
                0.0,
                0.0,
                yaw_rate,
                std::atan((bicycle_params_.lf + bicycle_params_.lr) * curvature),
            };
        }
        return {solution(0), -solution(0) / vx, yaw_rate, solution(1)};
    }

    static Gain solve_discrete_lqr(
        const StateMatrix& A,
        const InputMatrix& B,
        const StateMatrix& Q,
        double R)
    {
        StateMatrix P = Q;
        for (int iteration = 0; iteration < 1000; ++iteration) {
            const double denominator = std::max(R + (B.transpose() * P * B)(0, 0), 1e-12);
            const Gain K = (B.transpose() * P * A) / denominator;
            const StateMatrix next = A.transpose() * P * A
                                   - A.transpose() * P * B * K + Q;
            if ((next - P).cwiseAbs().maxCoeff() < 1e-11) {
                P = next;
                break;
            }
            P = next;
        }
        const double denominator = std::max(R + (B.transpose() * P * B)(0, 0), 1e-12);
        return (B.transpose() * P * A) / denominator;
    }

    static ActuatorGain solve_discrete_actuator_lqr(
        const ActuatorStateMatrix& A,
        const ActuatorInputMatrix& B,
        const ActuatorStateMatrix& Q,
        double R)
    {
        ActuatorStateMatrix P = Q;
        for (int iteration = 0; iteration < 1000; ++iteration) {
            const double denominator = std::max(R + (B.transpose() * P * B)(0, 0), 1e-12);
            const ActuatorGain K = (B.transpose() * P * A) / denominator;
            const ActuatorStateMatrix next = A.transpose() * P * A
                                           - A.transpose() * P * B * K + Q;
            if ((next - P).cwiseAbs().maxCoeff() < 1e-11) {
                P = next;
                break;
            }
            P = next;
        }
        const double denominator = std::max(R + (B.transpose() * P * B)(0, 0), 1e-12);
        return (B.transpose() * P * A) / denominator;
    }

private:
    struct GainPoint {
        double speed = 0.0;
        Gain gain = Gain::Zero();
    };

    struct ActuatorGainPoint {
        double speed = 0.0;
        ActuatorGain gain = ActuatorGain::Zero();
    };

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void continuous_model(double speed, StateMatrix& A, InputMatrix& B) const {
        const double vx = std::max(speed, config_.min_model_speed);
        const double m = std::max(bicycle_params_.m, 1e-6);
        const double iz = std::max(bicycle_params_.Izz, 1e-6);
        const double a = bicycle_params_.lf;
        const double b = bicycle_params_.lr;
        const double cf = std::max(bicycle_params_.Cf, 1e-3);
        const double cr = std::max(bicycle_params_.Cr, 1e-3);

        A.setZero();
        B.setZero();
        A(0, 1) = vx;
        A(0, 2) = 1.0;
        A(1, 3) = 1.0;
        A(2, 2) = -(cf + cr) / (m * vx);
        A(2, 3) = -vx + (-a * cf + b * cr) / (m * vx);
        A(3, 2) = (-a * cf + b * cr) / (iz * vx);
        A(3, 3) = -(a * a * cf + b * b * cr) / (iz * vx);
        B(2) = cf / m;
        B(3) = a * cf / iz;
    }

    void continuous_actuator_model(
        double speed,
        ActuatorStateMatrix& A,
        ActuatorInputMatrix& B) const
    {
        const double vx = std::max(speed, config_.min_model_speed);
        const double m = std::max(bicycle_params_.m, 1e-6);
        const double iz = std::max(bicycle_params_.Izz, 1e-6);
        const double a = bicycle_params_.lf;
        const double b = bicycle_params_.lr;
        const double cf = std::max(bicycle_params_.Cf, 1e-3);
        const double cr = std::max(bicycle_params_.Cr, 1e-3);
        const double omega_n = std::max(config_.steering_inner_loop_omega_n, 1e-6);
        const double zeta = std::max(config_.steering_inner_loop_zeta, 0.0);

        A.setZero();
        B.setZero();
        A(0, 1) = vx;
        A(0, 2) = 1.0;
        A(1, 3) = 1.0;
        A(2, 2) = -(cf + cr) / (m * vx);
        A(2, 3) = -vx + (-a * cf + b * cr) / (m * vx);
        A(2, 4) = cf / m;
        A(3, 2) = (-a * cf + b * cr) / (iz * vx);
        A(3, 3) = -(a * a * cf + b * b * cr) / (iz * vx);
        A(3, 4) = a * cf / iz;
        A(4, 5) = 1.0;
        A(5, 4) = -omega_n * omega_n;
        A(5, 5) = -2.0 * zeta * omega_n;
        B(5) = omega_n * omega_n;
    }

    void rebuild_gain_table() {
        gain_table_.clear();
        actuator_gain_table_.clear();
        StateMatrix Q = StateMatrix::Zero();
        for (int i = 0; i < 4; ++i) Q(i, i) = std::max(config_.q[i], 0.0);
        ActuatorStateMatrix actuator_Q = ActuatorStateMatrix::Zero();
        for (int i = 0; i < 6; ++i) {
            actuator_Q(i, i) = std::max(config_.actuator_q[i], 0.0);
        }
        const double step = std::max(config_.gain_grid_step, 0.25);
        for (double speed = config_.min_model_speed;
             speed <= config_.max_model_speed + 1e-9;
             speed += step) {
            StateMatrix Ac;
            InputMatrix Bc;
            continuous_model(speed, Ac, Bc);
            const StateMatrix identity = StateMatrix::Identity();
            const StateMatrix lhs = identity - 0.5 * config_.control_dt * Ac;
            const StateMatrix Ad = lhs.fullPivLu().solve(
                identity + 0.5 * config_.control_dt * Ac);
            const InputMatrix Bd = lhs.fullPivLu().solve(config_.control_dt * Bc);
            gain_table_.push_back({
                speed,
                solve_discrete_lqr(Ad, Bd, Q, std::max(config_.r_steer, 1e-9)),
            });

            ActuatorStateMatrix actuator_Ac;
            ActuatorInputMatrix actuator_Bc;
            continuous_actuator_model(speed, actuator_Ac, actuator_Bc);
            const ActuatorStateMatrix actuator_identity = ActuatorStateMatrix::Identity();
            const ActuatorStateMatrix actuator_lhs =
                actuator_identity - 0.5 * config_.control_dt * actuator_Ac;
            const ActuatorStateMatrix actuator_Ad = actuator_lhs.fullPivLu().solve(
                actuator_identity + 0.5 * config_.control_dt * actuator_Ac);
            const ActuatorInputMatrix actuator_Bd = actuator_lhs.fullPivLu().solve(
                config_.control_dt * actuator_Bc);
            actuator_gain_table_.push_back({
                speed,
                solve_discrete_actuator_lqr(
                    actuator_Ad,
                    actuator_Bd,
                    actuator_Q,
                    std::max(config_.actuator_r_steer, 1e-9)),
            });
        }
    }

    Gain interpolated_gain(double speed) const {
        if (gain_table_.empty()) return Gain::Zero();
        if (speed <= gain_table_.front().speed) return gain_table_.front().gain;
        if (speed >= gain_table_.back().speed) return gain_table_.back().gain;
        auto upper = std::lower_bound(
            gain_table_.begin(), gain_table_.end(), speed,
            [](const GainPoint& point, double value) { return point.speed < value; });
        const auto lower = std::prev(upper);
        const double alpha = (speed - lower->speed) /
            std::max(upper->speed - lower->speed, 1e-9);
        return (1.0 - alpha) * lower->gain + alpha * upper->gain;
    }

    ActuatorGain interpolated_actuator_gain(double speed) const {
        if (actuator_gain_table_.empty()) return ActuatorGain::Zero();
        if (speed <= actuator_gain_table_.front().speed) {
            return actuator_gain_table_.front().gain;
        }
        if (speed >= actuator_gain_table_.back().speed) {
            return actuator_gain_table_.back().gain;
        }
        auto upper = std::lower_bound(
            actuator_gain_table_.begin(), actuator_gain_table_.end(), speed,
            [](const ActuatorGainPoint& point, double value) { return point.speed < value; });
        const auto lower = std::prev(upper);
        const double alpha = (speed - lower->speed) /
            std::max(upper->speed - lower->speed, 1e-9);
        return (1.0 - alpha) * lower->gain + alpha * upper->gain;
    }

    double low_speed_steering(
        const vehicle::VehicleState& state,
        const track::TrackPoint& target,
        double lookahead) const
    {
        const double target_heading = std::atan2(target.y - state.y, target.x - state.x);
        const double alpha = normalize_angle(target_heading - state.psi);
        const double wheelbase = bicycle_params_.lf + bicycle_params_.lr;
        return std::atan2(2.0 * wheelbase * std::sin(alpha), std::max(lookahead, 0.5));
    }

    vehicle::DriverInput longitudinal_control(double target_speed, double speed) {
        const double error = target_speed - speed;
        speed_integral_ = std::clamp(
            speed_integral_ + error * config_.control_dt,
            -config_.speed_integral_limit,
            config_.speed_integral_limit);
        last_ax_command_ = std::clamp(
            config_.speed_kp * error + config_.speed_ki * speed_integral_,
            -config_.max_lon_decel,
            config_.max_lon_accel);
        vehicle::DriverInput input;
        if (last_ax_command_ >= 0.0) {
            input.throttle = std::clamp(
                last_ax_command_ / std::max(config_.max_lon_accel, 1e-6), 0.0, 1.0);
            input.brake = 0.0;
        } else {
            input.throttle = 0.0;
            input.brake = std::clamp(
                -last_ax_command_ / std::max(config_.max_lon_decel, 1e-6), 0.0, 1.0);
        }
        return input;
    }

    double lookup_target_speed(double s) const {
        if (velocity_sv_.empty()) return 10.0;
        auto upper = std::lower_bound(
            velocity_sv_.begin(), velocity_sv_.end(), s,
            [](const auto& point, double value) { return point.first < value; });
        if (upper == velocity_sv_.begin()) return upper->second;
        if (upper == velocity_sv_.end()) return velocity_sv_.back().second;
        const auto lower = std::prev(upper);
        const double alpha = (s - lower->first) / std::max(upper->first - lower->first, 1e-9);
        return lower->second + alpha * (upper->second - lower->second);
    }

    double lookup_preview_min_speed(double s, double speed) const {
        double result = lookup_target_speed(s);
        const double distance = std::clamp(
            std::max(speed, 3.0) * config_.speed_preview_time,
            4.0,
            config_.max_speed_preview_distance);
        for (double ds = config_.speed_preview_step;
             ds <= distance + 1e-9;
             ds += std::max(config_.speed_preview_step, 0.25)) {
            const double v = lookup_target_speed(wrap_s(s + ds));
            result = std::min(
                result, std::sqrt(v * v + 2.0 * config_.speed_preview_decel * ds));
        }
        return result;
    }

    double curvature_speed_cap(double s, double speed) const {
        const double lateral_accel = std::max(
            1.0,
            config_.lateral_accel_fraction * vehicle_params_.tire.mu_y_max * 9.81);
        double cap = std::numeric_limits<double>::infinity();
        const double distance = std::clamp(speed * 0.9, 4.0, 22.0);
        for (double ds = 0.0; ds <= distance + 1e-9; ds += 1.0) {
            const double curvature = std::abs(track_.get_curvature(wrap_s(s + ds)));
            if (curvature < 1e-5) continue;
            cap = std::min(cap, std::sqrt(
                lateral_accel / curvature + 2.0 * config_.curvature_cap_decel * ds));
        }
        return cap;
    }

    double wrap_s(double s) const {
        const double length = track_.length();
        if (!track_.is_closed_loop() || length <= 1e-9) return std::clamp(s, 0.0, length);
        s = std::fmod(s, length);
        return s < 0.0 ? s + length : s;
    }

    static double track_margin(const track::TrackPoint& point, double n) {
        return std::min(point.width_left - n, point.width_right + n);
    }

    AnalyticSafetyConfig make_safety_filter_config() const {
        AnalyticSafetyConfig safety;
        safety.max_tire_angle = std::max(vehicle_params_.steering.max_angle, 1e-6);
        safety.max_tire_rate = config_.max_steer_rate;
        safety.soft_track_margin = config_.soft_track_margin;
        safety.hard_track_margin = config_.hard_track_margin;
        safety.margin_speed_cap = config_.margin_speed_cap;
        safety.max_sideslip = config_.max_sideslip;
        safety.max_yaw_error = config_.max_yaw_error;
        safety.stability_speed_cap = config_.stability_speed_cap;
        return safety;
    }

    SteeringActuatorConfig make_steering_actuator_config() const {
        SteeringActuatorConfig actuator;
        actuator.R_steer = std::max(vehicle_params_.steering.R_steer, 1e-6);
        actuator.max_tire_angle = std::max(vehicle_params_.steering.max_angle, 1e-6);
        actuator.max_driver_torque = std::max(vehicle_params_.steering.T_driver_max, 1e-6);
        actuator.max_tire_steer_rate = std::max(config_.inner_target_steer_rate, 1e-6);
        actuator.inner_loop_omega_n = std::max(config_.steering_inner_loop_omega_n, 1e-6);
        actuator.inner_loop_zeta = std::max(config_.steering_inner_loop_zeta, 0.0);
        const double inertia = std::max(vehicle_params_.steering.I_steer, 1e-6);
        actuator.Kp_inner = inertia * actuator.inner_loop_omega_n * actuator.inner_loop_omega_n;
        actuator.Kd_inner = std::max(
            2.0 * actuator.inner_loop_zeta * actuator.inner_loop_omega_n * inertia
                - vehicle_params_.steering.C_steer_torque,
            0.0);
        actuator.Ki_inner = 0.10 * actuator.Kp_inner;
        return actuator;
    }

    const track::CurvatureBasedTrack& track_;
    track::TrackDistanceQuery track_query_;
    LPVLQRConfig config_;
    vehicle::VehicleParams vehicle_params_;
    vehicle::BicycleParams bicycle_params_;
    AnalyticSafetyFilter safety_filter_;
    std::vector<std::pair<double, double>> velocity_sv_;
    std::vector<GainPoint> gain_table_;
    std::vector<ActuatorGainPoint> actuator_gain_table_;
    double hint_s_ = -1.0;
    double prev_delta_ = 0.0;
    double last_vehicle_delta_sw_ = 0.0;
    double last_vehicle_ddelta_sw_ = 0.0;
    double pending_predicted_tire_angle_ = 0.0;
    double pending_prediction_target_time_ = 0.0;
    bool pending_prediction_valid_ = false;
    std::uint64_t outer_sequence_ = 0;
    double speed_integral_ = 0.0;
    double last_ax_command_ = 0.0;
    SteeringActuatorConfig steering_actuator_config_;
    SteeringActuatorState steering_actuator_state_;
    MPCDebugInfo last_debug_;
    MPCStats stats_;
};

} // namespace fsae::driver
