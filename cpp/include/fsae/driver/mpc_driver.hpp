#pragma once
/// @file mpc_driver.hpp
/// @brief Sampling-based preview MPC path-following driver.
///
/// This replaces the old OSQP/QP-only driver with a deterministic finite-horizon
/// controller that is always available in local builds.  It is intentionally
/// small: the driver evaluates a grid of constant steering/longitudinal
/// commands with a kinematic bicycle prediction, then applies the first command.

#include "../control/mpc_problem.hpp"
#include "../laptime/velocity_profile.hpp"
#include "../track/track_distance.hpp"
#include "../track/track_model.hpp"
#include "../vehicle/vehicle_params.hpp"
#include "../vehicle/vehicle_state.hpp"
#include "steering_actuator_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace fsae::driver {

struct MPCStats {
    double total_solve_time_ms = 0.0;
    double max_solve_time_ms = 0.0;
    int solve_count = 0;
    int fail_count = 0;
    int evaluated_rollouts = 0;
    int actuator_prediction_count = 0;
    double actuator_prediction_error_sum = 0.0;
    double actuator_prediction_abs_error_sum = 0.0;
    double actuator_prediction_sq_error_sum = 0.0;
    double actuator_prediction_max_abs_error = 0.0;

    double avg_solve_time_ms() const {
        return solve_count > 0 ? total_solve_time_ms / solve_count : 0.0;
    }

    double actuator_prediction_bias() const {
        return actuator_prediction_count > 0
            ? actuator_prediction_error_sum / actuator_prediction_count
            : 0.0;
    }

    double actuator_prediction_mae() const {
        return actuator_prediction_count > 0
            ? actuator_prediction_abs_error_sum / actuator_prediction_count
            : 0.0;
    }

    double actuator_prediction_rmse() const {
        return actuator_prediction_count > 0
            ? std::sqrt(actuator_prediction_sq_error_sum / actuator_prediction_count)
            : 0.0;
    }
};

struct MPCDebugInfo {
    double target_tire_angle = 0.0;     // [rad], driver convention: positive = right turn
    double raw_target_tire_angle = 0.0; // [rad], pre-safety, driver right-positive
    double required_tire_angle = 0.0;   // [rad], preview equilibrium, driver right-positive
    double required_reference_s = 0.0;  // [m], track station used by the preview requirement
    double required_reference_time = 0.0; // [s], estimated arrival time at reference station
    double actual_tire_angle_pre = 0.0; // [rad], rack-equivalent, driver right-positive
    double actual_tire_rate_pre = 0.0;  // [rad/s], rack-equivalent, driver right-positive
    double predicted_tire_angle = 0.0;  // [rad], prediction at t+h, driver right-positive
    double predicted_tire_rate = 0.0;   // [rad/s], prediction at t+h, driver right-positive
    double steering_prediction_error = 0.0; // [rad], previous prediction - aligned actual
    double steering_prediction_horizon = 0.0; // [s]
    double steering_prediction_target_time = 0.0; // [s]
    bool steering_prediction_valid = false;
    bool steering_prediction_error_valid = false;
    std::uint64_t outer_sequence = 0;
    double model_yaw_rate = 0.0;        // [rad/s], map/SAE CCW positive
    double actual_yaw_rate = 0.0;       // [rad/s], map/SAE CCW positive
    double yaw_rate_error = 0.0;        // [rad/s], actual - model equilibrium
    double selected_ax = 0.0;           // [m/s^2]
    double target_speed_base = 0.0;      // [m/s]
    double target_speed = 0.0;          // [m/s]
    double steering_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double track_margin_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double effective_tire_steer_rate = 0.0; // [rad/s]
    double current_lateral_margin = 0.0; // [m]
    double current_s = 0.0;             // [m]
    double rollout_progress = 0.0;       // [m], predicted forward track progress
    double rollout_progress_target = 0.0; // [m], feasible target progress over horizon
    double rollout_progress_deficit = 0.0; // [m]
    double rollout_cost = 0.0;
    int selected_plan_index = -1;
    double predicted_terminal_n = 0.0; // [m]
    double predicted_terminal_heading = 0.0; // [rad]
    double predicted_min_margin = std::numeric_limits<double>::infinity(); // [m]
    double predicted_terminal_speed = 0.0; // [m/s]
    double predicted_signed_progress = 0.0; // [m]
    int attack_selected_primitive = -1;
    std::string attack_selected_primitive_kind = "unknown";
    double attack_selected_speed_scale = 0.0;
    double attack_selected_target_speed = 0.0; // [m/s], first-step rollout target
    double attack_selected_ax = 0.0; // [m/s^2], first-step rollout acceleration
    double course_v_ref = 0.0; // [m/s], fixed-course reference speed at current s
    double course_n_ref = 0.0; // [m], fixed-course lateral reference at current s
    double course_kappa_ref = 0.0; // [1/m], fixed-course curvature reference at current s
    double course_speed_error = 0.0; // [m/s], actual speed - course_v_ref
    double course_lateral_error = 0.0; // [m], actual n - course_n_ref
    double course_a_ref = 0.0; // [m/s^2], fixed-course longitudinal acceleration reference
    double course_target_gap = 0.0; // [m/s], course_v_ref - actual speed
    double dynamic_cap_gap = 0.0; // [m/s], course_v_ref - safety-limited target speed
    bool braking_corridor_active = false;
    double braking_corridor_v_lower = 0.0; // [m/s], lower speed band in brake/turn-in corridor
    double braking_corridor_v_upper = 0.0; // [m/s], upper speed band in brake/turn-in corridor
    double attack_residual_delta_v_ref = 0.0; // [m/s], additive residual speed correction
    double attack_residual_delta_steer_ff = 0.0; // [rad], additive residual steering FF
    double attack_residual_delta_ax_bias = 0.0; // [m/s^2], additive safe acceleration bias
    double attack_residual_delta_heading_relax = 0.0; // [rad], safe heading-safety relaxation
    bool attack_residual_safety_gate_active = false;
    double attack_raw_heading_error = 0.0; // [rad], absolute heading error before residual relaxation
    double attack_effective_heading_error = 0.0; // [rad], heading error used by safety logic
    double attack_ax_before_residual = 0.0; // [m/s^2]
    double attack_ax_after_residual = 0.0; // [m/s^2]
    std::string longitudinal_mode = "unknown";
    bool healthy_underspeed = false;
    double accel_headroom = 0.0; // [m/s^2], remaining positive acceleration authority
    std::string track_phase = "unknown";
    double phase_speed_bias = 1.0;
    double phase_margin_bias = 0.0;
    double phase_progress_bias = 1.0;
    int attack_segment_id = -1;
    bool attack_allowed = false;
    double attack_static_speed_cap = 0.0;  // [m/s], track-phase planner cap
    double attack_dynamic_speed_cap = 0.0; // [m/s], first-step safety-limited target
    double attack_dynamic_base_speed = 0.0; // [m/s], first-step scaled base speed before caps
    double attack_curvature_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double attack_steering_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double attack_margin_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double attack_heading_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    double attack_outward_speed_cap = std::numeric_limits<double>::infinity(); // [m/s]
    std::string attack_limiting_cap_reason = "none";
    int attack_rollout_candidate_count = 0;
    int attack_rollout_reject_count = 0;
    int attack_rollout_boundary_reject_count = 0;
    int attack_rollout_nonfinite_reject_count = 0;
    bool attack_all_rejected = false;
    double attack_cost_tracking = 0.0;
    double attack_cost_speed = 0.0;
    double attack_cost_boundary = 0.0;
    double attack_cost_control_smoothness = 0.0;
    double attack_cost_progress = 0.0;
    double attack_cost_primitive_bias = 0.0;
    double attack_cost_overspeed = 0.0;
    double attack_cost_underspeed = 0.0;
    double attack_cost_time_progress = 0.0;
    double attack_best_tracking_cost = std::numeric_limits<double>::infinity();
    double attack_best_boundary_recovery_cost = std::numeric_limits<double>::infinity();
    double attack_best_low_speed_pure_pursuit_cost = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_065_cost = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_080_cost = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_095_cost = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_105_cost = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_095_cost_delta = std::numeric_limits<double>::infinity();
    double attack_best_speed_scale_095_tracking_delta = 0.0;
    double attack_best_speed_scale_095_speed_delta = 0.0;
    double attack_best_speed_scale_095_boundary_delta = 0.0;
    double attack_best_speed_scale_095_control_smoothness_delta = 0.0;
    double attack_best_speed_scale_095_progress_delta = 0.0;
    double attack_best_speed_scale_095_primitive_bias_delta = 0.0;
    bool torque_mode = false;
    SteeringActuatorOutput steering_actuator;
};

class MPCPathFollowingDriver {
public:
    MPCPathFollowingDriver(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const control::MPCConfig& config = control::MPCConfig(),
        const vehicle::VehicleParams& vehicle_params = vehicle::VehicleParams::default_fsae())
        : track_(track)
        , track_query_(track, 0.5)
        , config_(config)
    {
        velocity_sv_.reserve(velocity_profile.size());
        for (const auto& vp : velocity_profile) {
            velocity_sv_.emplace_back(vp.s, vp.v);
        }
        set_vehicle_params(vehicle_params);
    }

    vehicle::DriverInput compute_control(
        const vehicle::VehicleState& vehicle_state,
        double /*current_time*/,
        bool apply_torque_actuator = true)
    {
        const auto start_time = std::chrono::steady_clock::now();

        const auto nearest = track_query_.nearest(vehicle_state.x, vehicle_state.y, hint_s_);
        hint_s_ = nearest.s;

        const double speed = std::max(vehicle_state.vx, config_.vx_min);
        const double steer_limit = steering_limit();
        // Driver/14DOF convention: positive steering is a right turn. Track
        // curvature is positive for a left turn, so the feed-forward is negative.
        const double delta_ff = std::clamp(
            -std::atan(wheelbase_ * nearest.kappa),
            -steer_limit,
            steer_limit);

        const auto steer_candidates = build_steering_candidates(delta_ff);
        const double current_tire_angle = current_tire_angle_for_prediction();
        const double current_lateral_margin =
            lateral_margin(track_.get_point(nearest.s), nearest.n);
        const auto ax_candidates = build_accel_candidates(
            speed, nearest.s, current_tire_angle, current_lateral_margin);

        double best_cost = std::numeric_limits<double>::infinity();
        RolloutEvaluation best_eval;
        double best_delta = prev_delta_;
        double best_ax = prev_ax_;
        int evaluated = 0;
        int plan_index = 0;

        for (double delta : steer_candidates) {
            for (double ax : ax_candidates) {
                const RolloutEvaluation eval = evaluate_rollout(vehicle_state, delta, ax, plan_index);
                ++plan_index;
                ++evaluated;
                if (eval.cost < best_cost) {
                    best_cost = eval.cost;
                    best_eval = eval;
                    best_delta = delta;
                    best_ax = ax;
                }
            }
        }

        if (!std::isfinite(best_cost)) {
            ++stats_.fail_count;
            best_delta = prev_delta_;
            best_ax = prev_ax_;
        }

        prev_delta_ = best_delta;
        prev_ax_ = best_ax;
        stats_.evaluated_rollouts += evaluated;
        ++stats_.solve_count;

        const auto end_time = std::chrono::steady_clock::now();
        const double solve_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count();
        stats_.total_solve_time_ms += solve_ms;
        stats_.max_solve_time_ms = std::max(stats_.max_solve_time_ms, solve_ms);

        last_debug_ = MPCDebugInfo{};
        last_debug_.target_tire_angle = best_delta;
        last_debug_.selected_ax = best_ax;
        last_debug_.target_speed_base = lookup_base_target_speed(nearest.s);
        last_debug_.steering_speed_cap =
            compute_steering_speed_cap(nearest.s, current_tire_angle, speed);
        last_debug_.track_margin_speed_cap =
            compute_track_margin_speed_cap(current_lateral_margin, last_debug_.target_speed_base);
        last_debug_.target_speed =
            compute_actuator_aware_target_speed(
                nearest.s, current_tire_angle, speed, current_lateral_margin);
        last_debug_.effective_tire_steer_rate = effective_tire_steer_rate();
        last_debug_.current_lateral_margin = current_lateral_margin;
        last_debug_.current_s = nearest.s;
        last_debug_.rollout_progress = best_eval.progress;
        last_debug_.rollout_progress_target = best_eval.progress_target;
        last_debug_.rollout_progress_deficit = best_eval.progress_deficit;
        last_debug_.rollout_cost = best_eval.cost;
        last_debug_.selected_plan_index = best_eval.selected_plan_index;
        last_debug_.predicted_terminal_n = best_eval.terminal_n;
        last_debug_.predicted_terminal_heading = best_eval.terminal_heading;
        last_debug_.predicted_min_margin = best_eval.min_margin;
        last_debug_.predicted_terminal_speed = best_eval.terminal_speed;
        last_debug_.predicted_signed_progress = best_eval.signed_progress;
        last_debug_.torque_mode = vehicle_params_.steering.torque_mode;

        return convert_to_driver_input(
            best_delta,
            best_ax,
            vehicle_state.vx,
            apply_torque_actuator);
    }

    vehicle::DriverInput compute_outer_control(
        const vehicle::VehicleState& vehicle_state,
        double current_time)
    {
        return compute_control(vehicle_state, current_time, false);
    }

    vehicle::DriverInput update_torque_actuator_control(
        vehicle::DriverInput input,
        double delta_sw,
        double ddelta_sw,
        double dt,
        double T_SAT_total = 0.0)
    {
        set_steering_state(delta_sw, ddelta_sw);
        if (!vehicle_params_.steering.torque_mode) {
            return input;
        }

        const auto actuator_output = compute_torque_actuator_output(prev_delta_, dt, T_SAT_total);
        input.steering = actuator_output.steering_torque;
        return input;
    }

    const MPCStats& stats() const { return stats_; }
    const MPCDebugInfo& last_debug() const { return last_debug_; }
    double current_s() const { return hint_s_; }

    void set_vehicle_params(const vehicle::VehicleParams& vehicle_params) {
        vehicle_params_ = vehicle_params;
        wheelbase_ = std::max(vehicle_params_.mass.wheelbase, 1e-6);
        steering_actuator_config_ = make_steering_actuator_config();
    }

    void set_steering_state(double delta_sw, double ddelta_sw) {
        last_vehicle_delta_sw_ = delta_sw;
        last_vehicle_ddelta_sw_ = ddelta_sw;
    }

    void reset() {
        hint_s_ = -1.0;
        prev_delta_ = 0.0;
        prev_ax_ = 0.0;
        steering_actuator_state_.reset();
        last_debug_ = MPCDebugInfo{};
        stats_ = MPCStats{};
    }

private:
    struct RolloutEvaluation {
        double cost = std::numeric_limits<double>::infinity();
        double progress = 0.0;
        double progress_target = 0.0;
        double progress_deficit = 0.0;
        int selected_plan_index = -1;
        double terminal_n = 0.0;
        double terminal_heading = 0.0;
        double min_margin = std::numeric_limits<double>::infinity();
        double terminal_speed = 0.0;
        double signed_progress = 0.0;
    };

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double steering_limit() const {
        return std::max(1e-6, std::min(config_.delta_max, vehicle_params_.steering.max_angle));
    }

    std::array<double, 7> build_steering_candidates(double delta_ff) const {
        const double limit = steering_limit();
        const double max_step = std::max(effective_tire_steer_rate() * control_dt(), 1e-6);
        const double lo = std::max(-limit, prev_delta_ - max_step);
        const double hi = std::min(limit, prev_delta_ + max_step);
        const double center = std::clamp(delta_ff, lo, hi);
        const double step = std::max((hi - lo) / 6.0, 1e-6);
        return {
            std::clamp(center - 3.0 * step, lo, hi),
            std::clamp(center - 2.0 * step, lo, hi),
            std::clamp(center - 1.0 * step, lo, hi),
            std::clamp(center,              lo, hi),
            std::clamp(center + 1.0 * step, lo, hi),
            std::clamp(center + 2.0 * step, lo, hi),
            std::clamp(center + 3.0 * step, lo, hi),
        };
    }

    std::array<double, 5> build_accel_candidates(
        double current_speed,
        double s,
        double current_tire_angle,
        double current_lateral_margin) const
    {
        const double target_speed =
            compute_actuator_aware_target_speed(
                s, current_tire_angle, current_speed, current_lateral_margin);
        const double speed_error = target_speed - current_speed;
        const double cruise_ax = std::clamp(0.8 * speed_error, config_.ax_min, config_.ax_max);
        // Prevent the preview sampler from parking the car at low speed to avoid
        // near-term path cost when the speed target clearly asks for launch.
        if (current_speed < 4.0 && speed_error > 3.0) {
            return {
                0.0,
                std::min(config_.ax_max, 1.0),
                std::clamp(cruise_ax, 0.0, config_.ax_max),
                std::min(config_.ax_max, 2.0),
                std::min(config_.ax_max, 5.0),
            };
        }
        return {
            std::max(config_.ax_min, -6.0),
            std::max(config_.ax_min, -2.0),
            std::clamp(cruise_ax, config_.ax_min, config_.ax_max),
            std::min(config_.ax_max, 2.0),
            std::min(config_.ax_max, 5.0),
        };
    }

    double lookup_base_target_speed(double s) const {
        if (velocity_sv_.empty()) {
            return std::clamp(12.0, config_.vx_min, config_.vx_max);
        }
        auto it = std::lower_bound(
            velocity_sv_.begin(), velocity_sv_.end(), s,
            [](const auto& p, double value) { return p.first < value; });
        if (it == velocity_sv_.begin()) {
            return std::clamp(it->second, config_.vx_min, config_.vx_max);
        }
        if (it == velocity_sv_.end()) {
            return std::clamp(velocity_sv_.back().second, config_.vx_min, config_.vx_max);
        }
        const auto prev = std::prev(it);
        const double span = std::max(it->first - prev->first, 1e-9);
        const double t = std::clamp((s - prev->first) / span, 0.0, 1.0);
        return std::clamp(prev->second + t * (it->second - prev->second),
                          config_.vx_min,
                          config_.vx_max);
    }

    double compute_actuator_aware_target_speed(
        double s,
        double current_tire_angle,
        double current_speed,
        double current_lateral_margin) const
    {
        const double base_speed = lookup_base_target_speed(s);
        const double steering_cap = compute_steering_speed_cap(s, current_tire_angle, current_speed);
        const double margin_cap = compute_track_margin_speed_cap(current_lateral_margin, base_speed);
        double capped_speed = base_speed;
        if (std::isfinite(steering_cap)) {
            capped_speed = std::min(capped_speed, steering_cap);
        }
        if (std::isfinite(margin_cap)) {
            capped_speed = std::min(capped_speed, margin_cap);
        }
        return std::clamp(capped_speed, config_.vx_min, config_.vx_max);
    }

    RolloutEvaluation evaluate_rollout(
        const vehicle::VehicleState& initial,
        double delta,
        double ax,
        int plan_index) const
    {
        RolloutEvaluation eval;
        eval.selected_plan_index = plan_index;
        double x = initial.x;
        double y = initial.y;
        double psi = initial.psi;
        double vx = std::clamp(std::max(initial.vx, config_.vx_min), config_.vx_min, config_.vx_max);
        double hint_s = hint_s_;
        double prev_s = hint_s_ >= 0.0 ? hint_s_ : 0.0;
        SteeringActuatorPredictionState predicted_steering{
            current_tire_angle_for_prediction(),
            vehicle_params_.steering.torque_mode
                ? last_vehicle_ddelta_sw_ / std::max(vehicle_params_.steering.R_steer, 1e-6)
                : 0.0,
        };
        double cost = 0.0;

        const int steps = std::max(config_.N, 1);
        const double dt = control_dt();
        const double steer_rate = std::abs(delta - prev_delta_) / dt;
        cost += config_.w_delta_rate * steer_rate * steer_rate;

        for (int k = 0; k < steps; ++k) {
            const double max_delta_step = effective_tire_steer_rate() * dt;
            // Position mode keeps the direct rate constraint. Torque mode uses
            // the shared normalized second-order inner-loop response.
            if (vehicle_params_.steering.torque_mode
                    && config_.use_actuator_dynamics_in_prediction) {
                auto prediction_config = steering_actuator_config_;
                prediction_config.max_tire_steer_rate = effective_tire_steer_rate();
                predicted_steering = predict_steering_actuator_response(
                    prediction_config,
                    predicted_steering,
                    delta,
                    dt);
            } else {
                const double rate_limited_step = std::clamp(
                    delta - predicted_steering.tire_angle,
                    -max_delta_step,
                    max_delta_step);
                predicted_steering.tire_angle += rate_limited_step;
                predicted_steering.tire_rate = rate_limited_step / std::max(dt, 1e-6);
            }
            const double predicted_delta = predicted_steering.tire_angle;

            const double yaw_rate = -vx * std::tan(predicted_delta) / std::max(wheelbase_, 1e-6);
            psi = normalize_angle(psi + yaw_rate * dt);
            x += vx * std::cos(psi) * dt;
            y += vx * std::sin(psi) * dt;
            vx = std::clamp(vx + ax * dt, config_.vx_min, config_.vx_max);

            const auto nearest = track_query_.nearest(x, y, hint_s);
            hint_s = nearest.s;
            const double signed_progress = signed_track_progress(prev_s, nearest.s);
            eval.signed_progress += signed_progress;
            eval.progress += std::max(0.0, signed_progress);
            prev_s = nearest.s;
            const auto track_point = track_.get_point(nearest.s);
            const double heading_error = normalize_angle(psi - nearest.theta);
            const double lateral_margin = this->lateral_margin(track_point, nearest.n);
            eval.terminal_n = nearest.n;
            eval.terminal_heading = heading_error;
            eval.terminal_speed = vx;
            eval.min_margin = std::min(eval.min_margin, lateral_margin);
            const double target_speed =
                compute_actuator_aware_target_speed(nearest.s, predicted_delta, vx, lateral_margin);
            // Keep the progress objective exogenous. A rollout must not make its
            // own target easier by approaching the boundary and triggering a
            // safety speed cap.
            eval.progress_target += std::max(config_.vx_min, lookup_base_target_speed(nearest.s))
                * dt
                * std::clamp(config_.progress_target_fraction, 0.0, 1.0);
            const double speed_error = vx - target_speed;
            const double boundary_violation =
                std::max(0.0, config_.hard_boundary_margin - lateral_margin);
            const double soft_boundary_risk =
                std::max(0.0, config_.soft_boundary_margin - lateral_margin);
            const double outward_rate =
                outward_lateral_rate(nearest.n, heading_error, vx);
            const double terminal_scale = (k + 1 == steps) ? config_.terminal_weight : 1.0;
            cost += terminal_scale * (
                config_.w_y * nearest.n * nearest.n
              + config_.w_psi * heading_error * heading_error
              + config_.w_vx * speed_error * speed_error
              + config_.hard_boundary_weight * boundary_violation * boundary_violation
              + config_.soft_boundary_weight * soft_boundary_risk * soft_boundary_risk
              + config_.outward_rate_weight
                    * std::max(0.0, outward_rate) * std::max(0.0, outward_rate));
        }

        eval.progress_deficit = std::max(0.0, eval.progress_target - eval.progress);
        if (config_.terminal_progress_weight > 0.0) {
            cost += config_.terminal_progress_weight
                * eval.progress_deficit
                * eval.progress_deficit;
        }
        cost += config_.w_delta * delta * delta;
        cost += config_.w_ax * ax * ax;
        eval.cost = cost;
        return eval;
    }

    vehicle::DriverInput convert_to_driver_input(
        double delta,
        double ax,
        double current_vx,
        bool apply_torque_actuator)
    {
        vehicle::DriverInput input;
        const double target_delta = std::clamp(delta, -steering_limit(), steering_limit());
        if (vehicle_params_.steering.torque_mode) {
            input.steering = apply_torque_actuator
                ? compute_torque_actuator_output(target_delta, config_.dt).steering_torque
                : 0.0;
        } else {
            input.steering = target_delta;
        }

        if (ax >= 0.0) {
            input.throttle = std::clamp(ax / std::max(config_.ax_max, 1e-6), 0.0, 1.0);
            input.brake = 0.0;
        } else {
            input.throttle = 0.0;
            input.brake = std::clamp(-ax / std::max(-config_.ax_min, 1e-6), 0.0, 1.0);
        }

        if (current_vx < 8.0) input.gear = 2;
        else if (current_vx < 16.0) input.gear = 3;
        else if (current_vx < 24.0) input.gear = 4;
        else input.gear = 5;
        return input;
    }

    SteeringActuatorOutput compute_torque_actuator_output(
        double target_delta,
        double dt,
        double T_SAT_total = 0.0)
    {
        auto actuator_output = compute_steering_actuator_control(
            steering_actuator_config_,
            steering_actuator_state_,
            SteeringActuatorCommand{
                target_delta,
                last_vehicle_delta_sw_,
                last_vehicle_ddelta_sw_,
                dt,
                T_SAT_total,
            });
        last_debug_.steering_actuator = actuator_output;
        return actuator_output;
    }

    SteeringActuatorConfig make_steering_actuator_config() const {
        SteeringActuatorConfig actuator_config;
        actuator_config.R_steer = std::max(vehicle_params_.steering.R_steer, 1e-6);
        actuator_config.max_tire_angle = steering_limit();
        actuator_config.max_driver_torque = std::max(vehicle_params_.steering.T_driver_max, 1e-6);
        actuator_config.max_tire_steer_rate = std::max(config_.delta_rate_max / std::max(config_.dt, 1e-6), 1e-6);
        actuator_config.inner_loop_omega_n =
            std::max(config_.steering_inner_loop_omega_n, 1e-6);
        actuator_config.inner_loop_zeta =
            std::max(config_.steering_inner_loop_zeta, 0.0);

        const double I = std::max(vehicle_params_.steering.I_steer, 1e-6);
        const double C = std::max(vehicle_params_.steering.C_steer_torque, 0.0);
        actuator_config.Kp_inner = I * actuator_config.inner_loop_omega_n * actuator_config.inner_loop_omega_n;
        actuator_config.Kd_inner = std::max(
            2.0 * actuator_config.inner_loop_zeta * actuator_config.inner_loop_omega_n * I - C,
            0.0);
        actuator_config.Ki_inner = 0.10 * actuator_config.Kp_inner;
        return actuator_config;
    }

    double control_dt() const {
        return std::clamp(config_.dt, 0.005, 0.100);
    }

    double compute_steering_authority_scale() const {
        if (!vehicle_params_.steering.torque_mode) {
            return 1.0;
        }

        const double available_torque = std::max(
            vehicle_params_.steering.T_driver_max - std::max(vehicle_params_.steering.T_fric, 0.0),
            1e-3);
        const double authority = available_torque / std::max(vehicle_params_.steering.I_steer, 1e-6);
        constexpr double kNominalAuthority = (25.0 - 0.3) / 0.02;
        return std::clamp(authority / kNominalAuthority, 0.50, 1.25);
    }

    double effective_tire_steer_rate() const {
        const double base_rate = std::max(config_.delta_rate_max / control_dt(), 1e-6);
        return base_rate * compute_steering_authority_scale();
    }

    double current_tire_angle_for_prediction() const {
        if (vehicle_params_.steering.torque_mode) {
            return std::clamp(
                last_vehicle_delta_sw_ / std::max(vehicle_params_.steering.R_steer, 1e-6),
                -steering_limit(),
                steering_limit());
        }
        return std::clamp(prev_delta_, -steering_limit(), steering_limit());
    }

    double required_tire_angle(double kappa, double speed) const {
        constexpr double kUndersteerGradient = 0.002; // [rad/(m/s^2)]
        const double ay = speed * speed * kappa;
        const double required =
            -(wheelbase_ * kappa + kUndersteerGradient * ay);
        return std::clamp(required, -steering_limit(), steering_limit());
    }

    double compute_steering_speed_cap(
        double s,
        double current_tire_angle,
        double current_speed) const
    {
        const double rate = effective_tire_steer_rate();
        if (rate <= 1e-6 || wheelbase_ <= 1e-6) {
            return config_.vx_min;
        }

        const double preview_time = std::clamp(
            static_cast<double>(std::max(config_.N, 1)) * control_dt(),
            0.25,
            1.25);
        const double preview_distance = std::clamp(
            std::max(current_speed, config_.vx_min) * preview_time,
            3.0,
            35.0);

        double cap = std::numeric_limits<double>::infinity();
        constexpr double kStep = 1.0;
        constexpr double kDeadband = 0.02;
        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            const double sample_s = wrap_track_s(s + ds);
            const double kappa = track_.get_curvature(sample_s);
            const double required = required_tire_angle(kappa, std::max(current_speed, config_.vx_min));
            const double gap = std::max(0.0, std::abs(required - current_tire_angle) - kDeadband);
            if (gap <= 1e-9) {
                continue;
            }
            cap = std::min(cap, rate * ds / gap);
        }

        if (!std::isfinite(cap)) {
            return cap;
        }
        return std::clamp(cap, config_.vx_min, config_.vx_max);
    }

    double compute_track_margin_speed_cap(double lateral_margin, double base_speed) const {
        if (!config_.use_track_margin_speed_cap) {
            return std::numeric_limits<double>::infinity();
        }

        const double release_margin = std::max(config_.track_margin_speed_release, 1e-3);
        if (lateral_margin >= release_margin) {
            return std::numeric_limits<double>::infinity();
        }

        const double ratio = std::clamp(lateral_margin / release_margin, 0.0, 1.0);
        const double low_speed = std::clamp(
            config_.track_margin_speed_cap,
            config_.vx_min,
            std::max(config_.vx_min, base_speed));
        return std::clamp(
            low_speed + (base_speed - low_speed) * ratio * ratio,
            config_.vx_min,
            config_.vx_max);
    }

    double wrap_track_s(double s) const {
        const double length = track_.length();
        if (!track_.is_closed_loop() || length <= 1e-6) {
            return std::clamp(s, 0.0, length);
        }
        while (s < 0.0) s += length;
        while (s > length) s -= length;
        return s;
    }

    double forward_track_progress(double from_s, double to_s) const {
        const double length = track_.length();
        if (!std::isfinite(from_s) || !std::isfinite(to_s) || length <= 1e-6) {
            return 0.0;
        }

        double ds = to_s - from_s;
        if (track_.is_closed_loop()) {
            if (ds < -0.5 * length) {
                ds += length;
            } else if (ds > 0.5 * length) {
                ds -= length;
            }
        }
        return std::max(0.0, ds);
    }

    double signed_track_progress(double from_s, double to_s) const {
        const double length = track_.length();
        if (!std::isfinite(from_s) || !std::isfinite(to_s) || length <= 1e-6) {
            return 0.0;
        }

        double ds = to_s - from_s;
        if (track_.is_closed_loop()) {
            if (ds < -0.5 * length) {
                ds += length;
            } else if (ds > 0.5 * length) {
                ds -= length;
            }
        }
        return ds;
    }

    static double lateral_margin(const track::TrackPoint& point, double n) {
        return std::min(point.width_left - n, point.width_right + n);
    }

    static double outward_lateral_rate(double n, double heading_error, double speed) {
        if (std::abs(n) <= 1e-6) {
            return 0.0;
        }
        const double direction = n > 0.0 ? 1.0 : -1.0;
        return direction * speed * std::sin(heading_error);
    }

    const track::CurvatureBasedTrack& track_;
    track::TrackDistanceQuery track_query_;
    control::MPCConfig config_;
    vehicle::VehicleParams vehicle_params_;
    std::vector<std::pair<double, double>> velocity_sv_;

    double wheelbase_ = 1.55;
    double hint_s_ = -1.0;
    double prev_delta_ = 0.0;
    double prev_ax_ = 0.0;
    double last_vehicle_delta_sw_ = 0.0;
    double last_vehicle_ddelta_sw_ = 0.0;
    SteeringActuatorConfig steering_actuator_config_;
    SteeringActuatorState steering_actuator_state_;
    MPCDebugInfo last_debug_;
    MPCStats stats_;
};

} // namespace fsae::driver
