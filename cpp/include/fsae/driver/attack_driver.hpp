#pragma once
/// @file attack_driver.hpp
/// @brief Sequence-preview attack driver for torque-steered laptime simulation.

#include "../control/mpc_problem.hpp"
#include "../laptime/velocity_profile.hpp"
#include "../track/track_distance.hpp"
#include "../track/track_model.hpp"
#include "../vehicle/bicycle_model.hpp"
#include "../vehicle/vehicle_params.hpp"
#include "../vehicle/vehicle_state.hpp"
#include "attack_speed_planner.hpp"
#include "attack_residual_profile.hpp"
#include "course_attack_plan.hpp"
#include "line_planner.hpp"
#include "mpc_driver.hpp"
#include "steering_actuator_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace fsae::driver {

inline double braking_corridor_reachable_speed(
    double future_speed,
    double distance,
    double decel,
    double vx_min,
    double vx_max)
{
    const double safe_decel = std::max(decel, 1e-6);
    const double safe_distance = std::max(distance, 0.0);
    const double reachable = std::sqrt(std::max(
        future_speed * future_speed + 2.0 * safe_decel * safe_distance,
        0.0));
    return std::clamp(reachable, vx_min, vx_max);
}

struct BrakingCorridorSpeedBand {
    bool active = false;
    double v_lower = 0.0;
    double v_upper = 0.0;
};

inline BrakingCorridorSpeedBand make_braking_corridor_speed_band(
    double current_reference_speed,
    double reachable_upper_speed,
    double band_scale,
    double vx_min,
    double vx_max)
{
    BrakingCorridorSpeedBand band;
    band.active = true;
    band.v_upper = std::clamp(reachable_upper_speed, vx_min, vx_max);
    const double nominal_band_width = std::clamp(
        0.08 * std::max(current_reference_speed, band.v_upper),
        0.75,
        1.75);
    const double band_width = std::clamp(
        nominal_band_width * std::max(band_scale, 0.0),
        0.35,
        3.0);
    const double desired_lower = band.v_upper - band_width;
    band.v_lower = std::clamp(desired_lower, vx_min, band.v_upper);
    return band;
}

/// The attack driver keeps the old PID/MPC/RL implementations available while
/// using a stronger controller shape for laptime work: it evaluates short input
/// sequences rather than constant steering/acceleration pairs.
class AttackPathFollowingDriver {
public:
    AttackPathFollowingDriver(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const control::MPCConfig& config = control::MPCConfig(),
        const vehicle::VehicleParams& vehicle_params = vehicle::VehicleParams::default_fsae(),
        TrackPhaseProfile phase_profile = TrackPhaseProfile(),
        BoundaryLinePlannerConfig line_planner_config = BoundaryLinePlannerConfig(),
        AttackResidualProfile residual_profile = AttackResidualProfile())
        : track_(track)
        , track_query_(track, 0.5)
        , config_(config)
        , phase_profile_(phase_profile.empty()
            ? TrackPhaseAnalyzer::analyze(track, velocity_profile)
            : std::move(phase_profile))
        , line_planner_config_(line_planner_config)
        , residual_profile_(std::move(residual_profile))
    {
        velocity_profile_ = velocity_profile;
        velocity_sv_.reserve(velocity_profile.size());
        for (const auto& vp : velocity_profile) {
            velocity_sv_.emplace_back(vp.s, vp.v);
        }
        rebuild_planned_line_geometry_cache();
        set_vehicle_params(vehicle_params);
    }

    vehicle::DriverInput compute_control(
        const vehicle::VehicleState& map_frame_vehicle_state,
        double /*current_time*/,
        bool apply_torque_actuator = true)
    {
        const auto start_time = std::chrono::steady_clock::now();

        const auto nearest = track_query_.nearest(
            map_frame_vehicle_state.x,
            map_frame_vehicle_state.y,
            hint_s_);
        hint_s_ = nearest.s;

        const double raw_speed = std::clamp(
            std::max(map_frame_vehicle_state.vx, 0.0),
            0.0,
            config_.vx_max);
        const double current_tire_angle = current_tire_angle_for_prediction();
        const double current_lateral_margin =
            lateral_margin(track_.get_point(nearest.s), nearest.n);

        const auto primitives = build_tracking_primitives();
        const auto speed_scales = build_speed_scales();

        double best_cost = std::numeric_limits<double>::infinity();
        AttackEvaluation best_eval;
        std::array<double, 3> best_kind_cost = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
        };
        std::array<double, 4> best_speed_scale_cost = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
        };
        AttackEvaluation best_speed_scale_095_eval;
        int evaluated = 0;
        int rejected = 0;
        int boundary_rejected = 0;
        int nonfinite_rejected = 0;
        int plan_index = 0;

        for (std::size_t primitive_index = 0; primitive_index < primitives.size(); ++primitive_index) {
            const auto& primitive = primitives[primitive_index];
            for (std::size_t speed_index = 0; speed_index < speed_scales.size(); ++speed_index) {
                const double speed_scale = speed_scales[speed_index];
                const AttackEvaluation eval =
                    evaluate_rollout(
                        map_frame_vehicle_state,
                        primitive,
                        static_cast<int>(primitive_index),
                        speed_scale,
                        plan_index);
                ++plan_index;
                ++evaluated;
                if (eval.reject_reason != RejectReason::None) {
                    ++rejected;
                    if (eval.reject_reason == RejectReason::BoundaryMargin
                            || eval.reject_reason == RejectReason::PostBoundaryMargin) {
                        ++boundary_rejected;
                    } else if (eval.reject_reason == RejectReason::Nonfinite) {
                        ++nonfinite_rejected;
                    }
                }
                if (std::isfinite(eval.cost)
                        && eval.primitive_kind_index >= 0
                        && eval.primitive_kind_index < static_cast<int>(best_kind_cost.size())) {
                    best_kind_cost[static_cast<std::size_t>(eval.primitive_kind_index)] =
                        std::min(
                            best_kind_cost[static_cast<std::size_t>(eval.primitive_kind_index)],
                            eval.cost);
                    best_speed_scale_cost[speed_index] =
                        std::min(best_speed_scale_cost[speed_index], eval.cost);
                    if (speed_index == 2 && eval.cost < best_speed_scale_095_eval.cost) {
                        best_speed_scale_095_eval = eval;
                    }
                }
                if (eval.cost < best_cost) {
                    best_cost = eval.cost;
                    best_eval = eval;
                }
            }
        }

        if (!std::isfinite(best_cost)) {
            ++stats_.fail_count;
            const double heading_error =
                normalize_angle(map_frame_vehicle_state.psi - nearest.theta);
            const bool hard_boundary_violation =
                current_lateral_margin < -0.05
                || (raw_speed < kLowSpeedRecoverySpeed
                    && current_lateral_margin < config_.hard_boundary_margin
                    && std::abs(heading_error) > 0.45);
            const PhaseSchedule fallback_phase = phase_schedule_at(nearest.s);
            best_eval.phase = fallback_phase.phase;
            best_eval.phase_speed_bias = fallback_phase.speed_bias;
            best_eval.phase_margin_bias = fallback_phase.margin_bias;
            best_eval.phase_progress_bias = fallback_phase.progress_bias;
            best_eval.segment_id = fallback_phase.segment_id;
            best_eval.attack_allowed = fallback_phase.attack_allowed;
            best_eval.static_speed_cap = fallback_phase.static_speed_cap;
            best_eval.dynamic_speed_cap = raw_speed;
            if (hard_boundary_violation) {
                best_eval.first_delta = prev_delta_;
                best_eval.first_ax = std::clamp(
                    0.55 * config_.ax_min,
                    config_.ax_min,
                    config_.ax_max);
            } else {
                const double lookahead =
                    std::max(1.5, std::max(raw_speed, kPredictionSpeedFloor) * 0.75);
                best_eval.first_delta = compute_pure_pursuit_delta(
                    map_frame_vehicle_state.x,
                    map_frame_vehicle_state.y,
                    map_frame_vehicle_state.psi,
                    wrap_track_s(nearest.s + lookahead),
                    lookahead,
                    1.0);
                const bool allow_low_speed_fallback_accel =
                    raw_speed < kLowSpeedRecoverySpeed
                    && (current_lateral_margin > config_.hard_boundary_margin
                        || std::abs(heading_error) < 0.35);
                best_eval.first_ax = allow_low_speed_fallback_accel
                    ? std::clamp(1.0, config_.ax_min, config_.ax_max)
                    : std::min(0.0, prev_ax_);
                if (std::abs(heading_error) > 0.55 && raw_speed > kLowSpeedRecoverySpeed) {
                    best_eval.first_ax = std::min(best_eval.first_ax, 0.40 * config_.ax_min);
                }
            }
            best_eval.cost = std::numeric_limits<double>::infinity();
            best_eval.selected_plan_index = -1;
            best_eval.primitive_index = -1;
            best_eval.primitive_kind_index = -1;
            best_eval.speed_scale = 0.0;
            best_eval.terminal_n = nearest.n;
            best_eval.terminal_heading = heading_error;
            best_eval.min_margin = current_lateral_margin;
            best_eval.terminal_speed = raw_speed;
            best_eval.signed_progress = 0.0;
            best_eval.first_target_speed = raw_speed;
            best_eval.first_course_a_ref = course_a_ref_at(nearest.s);
            best_eval.first_course_target_gap = course_v_ref_at(nearest.s) - raw_speed;
            best_eval.first_dynamic_cap_gap = 0.0;
            best_eval.first_braking_corridor_active = false;
            best_eval.first_braking_corridor_v_lower = 0.0;
            best_eval.first_braking_corridor_v_upper = 0.0;
            best_eval.first_longitudinal_mode =
                hard_boundary_violation ? "hard_boundary" : "fallback_recovery";
            best_eval.first_healthy_underspeed = false;
            best_eval.first_accel_headroom =
                std::max(0.0, config_.ax_max - std::max(best_eval.first_ax, 0.0));
        }

        prev_delta_ = std::clamp(best_eval.first_delta, -steering_limit(), steering_limit());
        prev_ax_ = std::clamp(best_eval.first_ax, config_.ax_min, config_.ax_max);
        ++stats_.solve_count;
        stats_.evaluated_rollouts += evaluated;

        const auto end_time = std::chrono::steady_clock::now();
        const double solve_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count();
        stats_.total_solve_time_ms += solve_ms;
        stats_.max_solve_time_ms = std::max(stats_.max_solve_time_ms, solve_ms);

        last_debug_ = MPCDebugInfo{};
        last_debug_.target_tire_angle = prev_delta_;
        last_debug_.selected_ax = prev_ax_;
        last_debug_.target_speed_base = lookup_planned_target_speed(nearest.s);
        last_debug_.steering_speed_cap =
            compute_steering_speed_cap(nearest.s, current_tire_angle, raw_speed);
        last_debug_.track_margin_speed_cap =
            compute_track_margin_speed_cap(current_lateral_margin, last_debug_.target_speed_base);
        last_debug_.target_speed = best_eval.dynamic_speed_cap;
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
        last_debug_.attack_selected_primitive = best_eval.primitive_index;
        last_debug_.attack_selected_primitive_kind =
            primitive_kind_name(best_eval.primitive_kind_index);
        last_debug_.attack_selected_speed_scale = best_eval.speed_scale;
        last_debug_.attack_selected_target_speed = best_eval.first_target_speed;
        last_debug_.attack_selected_ax = best_eval.first_ax;
        last_debug_.course_v_ref = course_v_ref_at(nearest.s);
        last_debug_.course_n_ref = course_n_ref_at(nearest.s);
        last_debug_.course_kappa_ref = course_kappa_ref_at(nearest.s);
        last_debug_.course_speed_error = raw_speed - last_debug_.course_v_ref;
        last_debug_.course_lateral_error = nearest.n - last_debug_.course_n_ref;
        last_debug_.course_a_ref = best_eval.first_course_a_ref;
        last_debug_.course_target_gap = best_eval.first_course_target_gap;
        last_debug_.dynamic_cap_gap = best_eval.first_dynamic_cap_gap;
        last_debug_.braking_corridor_active = best_eval.first_braking_corridor_active;
        last_debug_.braking_corridor_v_lower = best_eval.first_braking_corridor_v_lower;
        last_debug_.braking_corridor_v_upper = best_eval.first_braking_corridor_v_upper;
        last_debug_.attack_residual_delta_v_ref = best_eval.first_residual_delta_v_ref;
        last_debug_.attack_residual_delta_steer_ff = best_eval.first_residual_delta_steer_ff;
        last_debug_.attack_residual_delta_ax_bias = best_eval.first_residual_delta_ax_bias;
        last_debug_.attack_residual_delta_heading_relax =
            best_eval.first_residual_delta_heading_relax;
        last_debug_.attack_residual_safety_gate_active =
            best_eval.first_residual_safety_gate_active;
        last_debug_.attack_raw_heading_error = best_eval.first_raw_heading_error;
        last_debug_.attack_effective_heading_error = best_eval.first_effective_heading_error;
        last_debug_.attack_ax_before_residual = best_eval.first_ax_before_residual;
        last_debug_.attack_ax_after_residual = best_eval.first_ax_after_residual;
        last_debug_.longitudinal_mode = best_eval.first_longitudinal_mode;
        last_debug_.healthy_underspeed = best_eval.first_healthy_underspeed;
        last_debug_.accel_headroom = best_eval.first_accel_headroom;
        last_debug_.track_phase = track_phase_name(best_eval.phase);
        last_debug_.phase_speed_bias = best_eval.phase_speed_bias;
        last_debug_.phase_margin_bias = best_eval.phase_margin_bias;
        last_debug_.phase_progress_bias = best_eval.phase_progress_bias;
        last_debug_.attack_segment_id = best_eval.segment_id;
        last_debug_.attack_allowed = best_eval.attack_allowed;
        last_debug_.attack_static_speed_cap = best_eval.static_speed_cap;
        last_debug_.attack_dynamic_speed_cap = best_eval.dynamic_speed_cap;
        last_debug_.attack_dynamic_base_speed = best_eval.dynamic_base_speed;
        last_debug_.attack_curvature_speed_cap = best_eval.curvature_speed_cap;
        last_debug_.attack_steering_speed_cap = best_eval.steering_speed_cap;
        last_debug_.attack_margin_speed_cap = best_eval.margin_speed_cap;
        last_debug_.attack_heading_speed_cap = best_eval.heading_speed_cap;
        last_debug_.attack_outward_speed_cap = best_eval.outward_speed_cap;
        last_debug_.attack_limiting_cap_reason = best_eval.limiting_cap_reason;
        last_debug_.attack_rollout_candidate_count = evaluated;
        last_debug_.attack_rollout_reject_count = rejected;
        last_debug_.attack_rollout_boundary_reject_count = boundary_rejected;
        last_debug_.attack_rollout_nonfinite_reject_count = nonfinite_rejected;
        last_debug_.attack_all_rejected = !std::isfinite(best_cost);
        last_debug_.attack_cost_tracking = best_eval.cost_tracking;
        last_debug_.attack_cost_speed = best_eval.cost_speed;
        last_debug_.attack_cost_boundary = best_eval.cost_boundary;
        last_debug_.attack_cost_control_smoothness = best_eval.cost_control_smoothness;
        last_debug_.attack_cost_progress = best_eval.cost_progress;
        last_debug_.attack_cost_primitive_bias = best_eval.cost_primitive_bias;
        last_debug_.attack_cost_overspeed = best_eval.cost_overspeed;
        last_debug_.attack_cost_underspeed = best_eval.cost_underspeed;
        last_debug_.attack_cost_time_progress = best_eval.cost_time_progress;
        last_debug_.attack_best_tracking_cost = best_kind_cost[0];
        last_debug_.attack_best_boundary_recovery_cost = best_kind_cost[1];
        last_debug_.attack_best_low_speed_pure_pursuit_cost = best_kind_cost[2];
        last_debug_.attack_best_speed_scale_065_cost = best_speed_scale_cost[0];
        last_debug_.attack_best_speed_scale_080_cost = best_speed_scale_cost[1];
        last_debug_.attack_best_speed_scale_095_cost = best_speed_scale_cost[2];
        last_debug_.attack_best_speed_scale_105_cost = best_speed_scale_cost[3];
        if (std::isfinite(best_eval.cost) && std::isfinite(best_speed_scale_095_eval.cost)) {
            last_debug_.attack_best_speed_scale_095_cost_delta =
                best_speed_scale_095_eval.cost - best_eval.cost;
            last_debug_.attack_best_speed_scale_095_tracking_delta =
                best_speed_scale_095_eval.cost_tracking - best_eval.cost_tracking;
            last_debug_.attack_best_speed_scale_095_speed_delta =
                best_speed_scale_095_eval.cost_speed - best_eval.cost_speed;
            last_debug_.attack_best_speed_scale_095_boundary_delta =
                best_speed_scale_095_eval.cost_boundary - best_eval.cost_boundary;
            last_debug_.attack_best_speed_scale_095_control_smoothness_delta =
                best_speed_scale_095_eval.cost_control_smoothness
              - best_eval.cost_control_smoothness;
            last_debug_.attack_best_speed_scale_095_progress_delta =
                best_speed_scale_095_eval.cost_progress - best_eval.cost_progress;
            last_debug_.attack_best_speed_scale_095_primitive_bias_delta =
                best_speed_scale_095_eval.cost_primitive_bias - best_eval.cost_primitive_bias;
        }
        last_debug_.torque_mode = vehicle_params_.steering.torque_mode;

        return convert_to_driver_input(prev_delta_, prev_ax_, raw_speed, apply_torque_actuator);
    }

    vehicle::DriverInput compute_outer_control(
        const vehicle::VehicleState& map_frame_vehicle_state,
        double current_time)
    {
        return compute_control(map_frame_vehicle_state, current_time, false);
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
        bicycle_params_ = vehicle::BicycleParams::from_vehicle_params(vehicle_params_);
        steering_actuator_config_ = make_steering_actuator_config();
        attack_speed_planner_config_.min_speed = config_.vx_min;
        attack_speed_planner_config_.max_speed = config_.vx_max;
        attack_speed_planner_config_.lateral_safety =
            config_.attack_speed_lateral_safety;
        attack_speed_planner_config_.braking_safety =
            config_.attack_speed_braking_safety;
        attack_speed_planner_config_.accel_limit =
            config_.attack_speed_accel_limit;
        attack_speed_planner_config_.external_speed_caps =
            config_.attack_speed_caps;
        attack_speed_planner_config_.external_speed_windows =
            config_.attack_speed_windows;
        course_attack_plan_ =
            CourseAttackPlan::build(
                track_,
                velocity_profile_,
                vehicle_params_,
                phase_profile_,
                line_planner_config_,
                attack_speed_planner_config_);
    }

    void set_residual_profile(AttackResidualProfile residual_profile) {
        residual_profile_ = std::move(residual_profile);
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
    enum class PrimitiveKind {
        Tracking,
        BoundaryRecovery,
        LowSpeedPurePursuit,
    };

    enum class RejectReason {
        None,
        Nonfinite,
        BoundaryMargin,
        PostBoundaryMargin,
    };

    struct TrackingPrimitive {
        PrimitiveKind kind = PrimitiveKind::Tracking;
        double preview_time = 0.25;     // [s]
        double min_lookahead = 2.0;     // [m]
        double lateral_gain = 0.12;     // [rad/m]
        double heading_gain = 0.45;     // [rad/rad]
        double kff_scale = 1.0;         // [-]
        double pure_pursuit_gain = 0.0; // [-]
        double rate_scale = 1.0;        // [-]
    };

    struct PhaseSchedule {
        TrackPhase phase = TrackPhase::Straight;
        double speed_bias = 1.0;
        double margin_bias = 0.0;
        double progress_bias = 1.0;
        double tracking_cost_scale = 1.0;
        double speed_cost_scale = 1.0;
        double boundary_cost_scale = 1.0;
        double progress_cost_scale = 1.0;
        bool allow_high_speed = false;
        bool suppress_high_speed = false;
        int segment_id = -1;
        bool attack_allowed = false;
        double static_speed_cap = 0.0;
    };

    struct AttackEvaluation {
        double cost = std::numeric_limits<double>::infinity();
        double first_delta = 0.0;
        double first_ax = 0.0;
        double first_target_speed = 0.0;
        double progress = 0.0;
        double progress_target = 0.0;
        double progress_deficit = 0.0;
        int selected_plan_index = -1;
        int primitive_index = -1;
        int primitive_kind_index = -1;
        double speed_scale = 0.0;
        RejectReason reject_reason = RejectReason::None;
        double terminal_n = 0.0;
        double terminal_heading = 0.0;
        double min_margin = std::numeric_limits<double>::infinity();
        double terminal_speed = 0.0;
        double signed_progress = 0.0;
        double cost_tracking = 0.0;
        double cost_speed = 0.0;
        double cost_boundary = 0.0;
        double cost_control_smoothness = 0.0;
        double cost_progress = 0.0;
        double cost_primitive_bias = 0.0;
        double cost_overspeed = 0.0;
        double cost_underspeed = 0.0;
        double cost_time_progress = 0.0;
        double first_course_a_ref = 0.0;
        double first_course_target_gap = 0.0;
        double first_dynamic_cap_gap = 0.0;
        bool first_braking_corridor_active = false;
        double first_braking_corridor_v_lower = 0.0;
        double first_braking_corridor_v_upper = 0.0;
        double first_residual_delta_v_ref = 0.0;
        double first_residual_delta_steer_ff = 0.0;
        double first_residual_delta_ax_bias = 0.0;
        double first_residual_delta_heading_relax = 0.0;
        bool first_residual_safety_gate_active = false;
        double first_raw_heading_error = 0.0;
        double first_effective_heading_error = 0.0;
        double first_ax_before_residual = 0.0;
        double first_ax_after_residual = 0.0;
        std::string first_longitudinal_mode = "unknown";
        bool first_healthy_underspeed = false;
        double first_accel_headroom = 0.0;
        TrackPhase phase = TrackPhase::Straight;
        double phase_speed_bias = 1.0;
        double phase_margin_bias = 0.0;
        double phase_progress_bias = 1.0;
        int segment_id = -1;
        bool attack_allowed = false;
        double static_speed_cap = 0.0;
        double dynamic_speed_cap = 0.0;
        double dynamic_base_speed = 0.0;
        double curvature_speed_cap = std::numeric_limits<double>::infinity();
        double steering_speed_cap = std::numeric_limits<double>::infinity();
        double margin_speed_cap = std::numeric_limits<double>::infinity();
        double heading_speed_cap = std::numeric_limits<double>::infinity();
        double outward_speed_cap = std::numeric_limits<double>::infinity();
        std::string limiting_cap_reason = "none";
    };

    struct SpeedCapBreakdown {
        double limited_speed = 0.0;
        double base_speed = 0.0;
        double course_v_ref = 0.0;
        double course_a_ref = 0.0;
        double course_target_gap = 0.0;
        double dynamic_cap_gap = 0.0;
        BrakingCorridorSpeedBand braking_corridor;
        double curvature_cap = std::numeric_limits<double>::infinity();
        double steering_cap = std::numeric_limits<double>::infinity();
        double margin_cap = std::numeric_limits<double>::infinity();
        double heading_cap = std::numeric_limits<double>::infinity();
        double outward_cap = std::numeric_limits<double>::infinity();
        std::string limiting_reason = "none";
    };

    struct LongitudinalCommand {
        double ax = 0.0;
        std::string mode = "track_speed";
        bool healthy_underspeed = false;
        double accel_headroom = 0.0;
        BrakingCorridorSpeedBand braking_corridor;
        double residual_delta_ax_bias = 0.0;
        double residual_delta_heading_relax = 0.0;
        bool residual_safety_gate_active = false;
        double raw_heading_error = 0.0;
        double effective_heading_error = 0.0;
        double ax_before_residual = 0.0;
        double ax_after_residual = 0.0;
    };

    struct ResidualSafetyState {
        bool active = false;
        double raw_abs_heading = 0.0;
        double effective_abs_heading = 0.0;
        double heading_relax = 0.0;
        double ax_bias = 0.0;
    };

    static constexpr double kPredictionSpeedFloor = 0.50; // [m/s], numerical rollout floor only
    static constexpr double kLowSpeedRecoverySpeed = 2.0; // [m/s], raw vehicle speed
    static constexpr double kBoundaryRiskProjectionTime = 0.40; // [s]

    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    static int primitive_kind_index(PrimitiveKind kind) {
        switch (kind) {
            case PrimitiveKind::Tracking:
                return 0;
            case PrimitiveKind::BoundaryRecovery:
                return 1;
            case PrimitiveKind::LowSpeedPurePursuit:
                return 2;
        }
        return -1;
    }

    static std::string primitive_kind_name(int kind_index) {
        switch (kind_index) {
            case 0:
                return "tracking";
            case 1:
                return "boundary_recovery";
            case 2:
                return "low_speed_pure_pursuit";
            case -1:
                return "fallback";
            default:
                return "unknown";
        }
    }

    double steering_limit() const {
        return std::max(1e-6, std::min(config_.delta_max, vehicle_params_.steering.max_angle));
    }

    void rebuild_planned_line_geometry_cache() {
        planned_line_geometry_profile_.clear();
        if (!line_planner_config_.enabled) {
            return;
        }

        const double length = track_.length();
        if (length <= 1e-6) {
            return;
        }

        constexpr double kGeometryCacheStep = 0.25;
        for (double s = 0.0; s < length; s += kGeometryCacheStep) {
            planned_line_geometry_profile_.push_back(
                compute_planned_line_geometry(track_, s, line_planner_config_));
        }
        planned_line_geometry_profile_.push_back(
            compute_planned_line_geometry(track_, length, line_planner_config_));
    }

    PlannedLineGeometry lookup_planned_line_geometry(double s) const {
        const double wrapped_s = wrap_track_s(s);
        if (planned_line_geometry_profile_.size() < 2U) {
            return compute_planned_line_geometry(track_, wrapped_s, line_planner_config_);
        }

        auto upper = std::lower_bound(
            planned_line_geometry_profile_.begin(),
            planned_line_geometry_profile_.end(),
            wrapped_s,
            [](const PlannedLineGeometry& geometry, double value) {
                return geometry.s < value;
            });

        if (upper == planned_line_geometry_profile_.begin()) {
            return *upper;
        }
        if (upper == planned_line_geometry_profile_.end()) {
            return planned_line_geometry_profile_.back();
        }

        const auto& lo = *(upper - 1);
        const auto& hi = *upper;
        const double ds = std::max(hi.s - lo.s, 1e-9);
        const double alpha = std::clamp((wrapped_s - lo.s) / ds, 0.0, 1.0);
        auto lerp = [alpha](double a, double b) {
            return a + (b - a) * alpha;
        };
        auto lerp_angle = [alpha](double a, double b) {
            return a + normalize_angle(b - a) * alpha;
        };

        PlannedLineGeometry geometry;
        geometry.s = wrapped_s;
        geometry.center_point = track_.get_point(wrapped_s);
        geometry.target_n = lerp(lo.target_n, hi.target_n);
        geometry.heading = lerp_angle(lo.heading, hi.heading);
        geometry.heading_offset = lerp_angle(lo.heading_offset, hi.heading_offset);
        geometry.curvature = lerp(lo.curvature, hi.curvature);
        geometry.dn_ds = lerp(lo.dn_ds, hi.dn_ds);
        geometry.ddn_ds2 = lerp(lo.ddn_ds2, hi.ddn_ds2);
        geometry.centerline_curvature = geometry.center_point.kappa;
        geometry.plan.target_n = lerp(lo.plan.target_n, hi.plan.target_n);
        geometry.plan.curvature_now = lerp(lo.plan.curvature_now, hi.plan.curvature_now);
        geometry.plan.curvature_ahead = lerp(lo.plan.curvature_ahead, hi.plan.curvature_ahead);
        geometry.plan.curvature_behind = lerp(lo.plan.curvature_behind, hi.plan.curvature_behind);
        geometry.plan.usable_half_width = lerp(lo.plan.usable_half_width, hi.plan.usable_half_width);
        return geometry;
    }

    PlannedLineGeometry compute_tracking_line_geometry(double s) const {
        const double wrapped_s = wrap_track_s(s);
        if (line_planner_config_.track_geometry) {
            auto geometry = lookup_planned_line_geometry(s);
            if (!course_attack_plan_.empty()) {
                geometry.target_n = course_attack_plan_.n_ref_at(wrapped_s);
                geometry.curvature = course_attack_plan_.kappa_ref_at(wrapped_s);
            }
            return geometry;
        }
        if (planned_line_geometry_profile_.size() >= 2U) {
            auto geometry = lookup_planned_line_geometry(s);
            geometry.target_n = course_n_ref_at(wrapped_s);
            geometry.heading = geometry.center_point.theta;
            geometry.heading_offset = 0.0;
            geometry.curvature = course_kappa_ref_at(wrapped_s);
            return geometry;
        }

        PlannedLineGeometry geometry;
        geometry.s = wrapped_s;
        geometry.center_point = track_.get_point(wrapped_s);
        geometry.centerline_curvature = geometry.center_point.kappa;
        geometry.plan = compute_boundary_line_plan(track_, wrapped_s, line_planner_config_);
        geometry.target_n = course_n_ref_at(wrapped_s);
        geometry.heading = geometry.center_point.theta;
        geometry.heading_offset = 0.0;
        geometry.curvature = course_kappa_ref_at(wrapped_s);
        return geometry;
    }

    double course_v_ref_at(double s) const {
        return course_attack_plan_.empty()
            ? apply_speed_residual(s, lookup_base_target_speed(s))
            : lookup_planned_target_speed(s);
    }

    double course_a_ref_at(double s) const {
        return course_attack_plan_.empty()
            ? 0.0
            : course_attack_plan_.a_ref_at(wrap_track_s(s));
    }

    double course_n_ref_at(double s) const {
        return course_attack_plan_.empty()
            ? 0.0
            : course_attack_plan_.n_ref_at(wrap_track_s(s));
    }

    double course_kappa_ref_at(double s) const {
        return course_attack_plan_.empty()
            ? track_.get_curvature(wrap_track_s(s))
            : course_attack_plan_.kappa_ref_at(wrap_track_s(s));
    }

    double tracking_curvature_at(double s) const {
        return compute_tracking_line_geometry(s).curvature;
    }

    AttackResidualCorrection residual_correction_at(double s) const {
        return residual_profile_.empty()
            ? AttackResidualCorrection{}
            : residual_profile_.correction_at(wrap_track_s(s));
    }

    double apply_speed_residual(double s, double target_speed) const {
        const auto correction = residual_correction_at(s);
        return std::clamp(
            target_speed + correction.delta_v_ref_mps,
            config_.vx_min,
            config_.vx_max);
    }

    double residual_steer_ff_at(double s) const {
        return residual_correction_at(s).delta_steer_ff_rad;
    }

    ResidualSafetyState residual_safety_state_at(
        double s,
        double abs_heading,
        double effective_margin,
        double outward_rate) const
    {
        ResidualSafetyState state;
        state.raw_abs_heading = std::max(0.0, abs_heading);
        state.effective_abs_heading = state.raw_abs_heading;
        const auto correction = residual_correction_at(s);

        const bool physically_safe =
            effective_margin > config_.soft_boundary_margin + 0.25
            && outward_rate < 0.20
            && state.raw_abs_heading < 0.55;
        if (!physically_safe) {
            return state;
        }

        state.heading_relax = std::clamp(
            correction.delta_heading_relax_rad,
            0.0,
            residual_profile_.max_delta_heading_relax_rad());
        state.ax_bias = std::clamp(
            correction.delta_ax_bias_mps2,
            -residual_profile_.max_delta_ax_bias_mps2(),
            residual_profile_.max_delta_ax_bias_mps2());
        state.effective_abs_heading =
            std::max(0.0, state.raw_abs_heading - state.heading_relax);
        state.active =
            state.heading_relax > 1e-9 || std::abs(state.ax_bias) > 1e-9;
        return state;
    }

    std::pair<double, double> planned_line_point(double s) const {
        const auto geometry = compute_tracking_line_geometry(s);
        const auto& point = geometry.center_point;
        const double normal_x = -std::sin(point.theta);
        const double normal_y = std::cos(point.theta);
        return {
            point.x + normal_x * geometry.target_n,
            point.y + normal_y * geometry.target_n,
        };
    }

    std::array<TrackingPrimitive, 7> build_tracking_primitives() const {
        return {{
            {PrimitiveKind::Tracking,           0.18, 2.0, 0.11, 0.42, 1.00, 0.00, 1.00},
            {PrimitiveKind::Tracking,           0.30, 3.5, 0.13, 0.50, 1.00, 0.00, 1.00},
            {PrimitiveKind::Tracking,           0.45, 5.0, 0.11, 0.55, 1.10, 0.00, 0.90},
            {PrimitiveKind::Tracking,           0.22, 2.5, 0.17, 0.68, 0.95, 0.00, 1.10},
            {PrimitiveKind::BoundaryRecovery,   0.28, 2.0, 0.28, 0.78, 0.85, 0.00, 1.15},
            {PrimitiveKind::LowSpeedPurePursuit,0.75, 1.5, 0.00, 0.00, 0.00, 1.15, 0.65},
            {PrimitiveKind::Tracking,           0.24, 2.0, 0.19, 0.92, 1.05, 0.00, 1.25},
        }};
    }

    std::array<double, 4> build_speed_scales() const {
        return {{0.65, 0.80, 0.95, 1.05}};
    }

    PhaseSchedule phase_schedule_at(double s) const {
        PhaseSchedule schedule;
        const double wrapped_s = wrap_track_s(s);
        const auto& sample = phase_profile_.sample_at(wrapped_s);
        const auto& course_sample = course_attack_plan_.sample_at(wrapped_s);
        schedule.phase = course_attack_plan_.empty() ? sample.phase : course_sample.phase;
        schedule.segment_id = course_attack_plan_.empty() ? sample.segment_id : course_sample.segment_id;
        schedule.attack_allowed =
            (course_attack_plan_.empty() ? sample.attack_allowed : course_sample.attack_allowed)
            || is_attack_phase(schedule.phase);
        const double planned_speed = lookup_planned_target_speed(s);
        schedule.static_speed_cap = planned_speed > 0.0
            ? planned_speed
            : course_sample.static_speed_cap > 0.0
                ? course_sample.static_speed_cap
                : lookup_base_target_speed(s);
        switch (schedule.phase) {
            case TrackPhase::Straight:
                schedule.speed_bias = 1.02;
                schedule.progress_bias = 1.05;
                schedule.speed_cost_scale = 0.50;
                schedule.progress_cost_scale = 0.95;
                schedule.allow_high_speed = true;
                break;
            case TrackPhase::Exit:
                schedule.speed_bias = 1.03;
                schedule.progress_bias = 1.10;
                schedule.speed_cost_scale = 0.60;
                schedule.progress_cost_scale = 0.90;
                schedule.allow_high_speed = true;
                break;
            case TrackPhase::Brake:
                schedule.speed_cost_scale = 0.90;
                schedule.suppress_high_speed = true;
                break;
            case TrackPhase::TurnIn:
                schedule.speed_cost_scale = 0.90;
                schedule.suppress_high_speed = true;
                break;
            case TrackPhase::Apex:
                schedule.speed_cost_scale = 0.90;
                schedule.suppress_high_speed = true;
                break;
            case TrackPhase::Complex:
                schedule.speed_cost_scale = 0.90;
                schedule.suppress_high_speed = true;
                break;
        }
        return schedule;
    }

    static bool is_speed_recovery_phase(TrackPhase phase) {
        return phase == TrackPhase::Straight || phase == TrackPhase::Exit;
    }

    static bool is_attack_phase(TrackPhase phase) {
        return phase == TrackPhase::Straight
            || phase == TrackPhase::Exit
            || phase == TrackPhase::Complex;
    }

    static bool is_high_speed_release_phase(TrackPhase phase) {
        return phase == TrackPhase::Straight;
    }

    double attack_profile_floor_scale(TrackPhase phase) const {
        switch (phase) {
            case TrackPhase::Straight:
                return config_.attack_floor_straight;
            case TrackPhase::Exit:
                return config_.attack_floor_exit;
            case TrackPhase::Complex:
                return config_.attack_floor_complex;
            case TrackPhase::TurnIn:
                return config_.attack_floor_turn_in;
            case TrackPhase::Apex:
                return config_.attack_floor_apex;
            case TrackPhase::Brake:
                return config_.attack_floor_brake;
        }
        return config_.attack_floor_complex;
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
        const double alpha = std::clamp((s - prev->first) / span, 0.0, 1.0);
        return std::clamp(
            prev->second + alpha * (it->second - prev->second),
            config_.vx_min,
            config_.vx_max);
    }

    double lookup_preview_min_target_speed(double s, double preview_distance) const {
        double target = lookup_base_target_speed(s);
        const double distance = std::max(preview_distance, 0.0);
        constexpr double kStep = 1.0;
        for (double ds = kStep; ds <= distance + 1e-12; ds += kStep) {
            target = std::min(target, lookup_base_target_speed(wrap_track_s(s + ds)));
        }
        return std::clamp(target, config_.vx_min, config_.vx_max);
    }

    double lookup_planned_target_speed(double s) const {
        const double base_speed = lookup_base_target_speed(s);
        if (!course_attack_plan_.empty()) {
            const double wrapped_s = wrap_track_s(s);
            const auto& sample = course_attack_plan_.sample_at(wrapped_s);
            const double planned_speed = course_attack_plan_.v_ref_at(wrapped_s);
            const bool hard_limited =
                sample.speed_reason != AttackSpeedLimitReason::Profile
                && sample.speed_reason != AttackSpeedLimitReason::Phase;
            const double profile_floor =
                base_speed * attack_profile_floor_scale(sample.phase);
            const double planned_with_floor = std::clamp(
                hard_limited ? planned_speed : std::max(planned_speed, profile_floor),
                config_.vx_min,
                config_.vx_max);
            return apply_speed_residual(wrapped_s, planned_with_floor);
        }
        return apply_speed_residual(s, base_speed);
    }

    double lookup_preview_min_planned_target_speed(double s, double preview_distance) const {
        if (!course_attack_plan_.empty()) {
            double target = lookup_planned_target_speed(s);
            const double distance = std::max(preview_distance, 0.0);
            constexpr double kStep = 1.0;
            for (double ds = kStep; ds <= distance + 1e-12; ds += kStep) {
                target = std::min(target, lookup_planned_target_speed(wrap_track_s(s + ds)));
            }
            return std::clamp(target, config_.vx_min, config_.vx_max);
        }
        return lookup_preview_min_target_speed(s, preview_distance);
    }

    double braking_corridor_decel() const {
        const double tire_limited = std::max(vehicle_params_.tire.mu_x_max, 0.1) * 9.81 * 0.20;
        const double actuator_limited = std::max(-config_.ax_min, 1e-6) * 0.20;
        const double nominal_decel = std::clamp(std::min(tire_limited, actuator_limited), 2.0, 4.0);
        return std::clamp(
            nominal_decel * std::max(config_.attack_braking_corridor_decel_scale, 0.0),
            1.0,
            6.0);
    }

    BrakingCorridorSpeedBand braking_corridor_speed_band_at(
        double s,
        double preview_distance) const
    {
        if (!course_attack_plan_.empty()) {
            const double current_reference = lookup_planned_target_speed(s);
            double upper = current_reference;
            const double distance = std::max(preview_distance, 0.0);
            const double decel = braking_corridor_decel();
            constexpr double kStep = 1.0;
            for (double ds = kStep; ds <= distance + 1e-12; ds += kStep) {
                const double future_speed = lookup_planned_target_speed(wrap_track_s(s + ds));
                upper = std::min(
                    upper,
                    braking_corridor_reachable_speed(
                        future_speed,
                        ds,
                        decel,
                        config_.vx_min,
                        config_.vx_max));
            }
            return make_braking_corridor_speed_band(
                current_reference,
                upper,
                config_.attack_braking_corridor_band_scale,
                config_.vx_min,
                config_.vx_max);
        }
        return BrakingCorridorSpeedBand{
            false,
            lookup_preview_min_target_speed(s, preview_distance),
            lookup_preview_min_target_speed(s, preview_distance),
        };
    }

    double lookup_braking_corridor_planned_target_speed(double s, double preview_distance) const {
        return braking_corridor_speed_band_at(s, preview_distance).v_upper;
    }

    double lookup_dynamic_base_target_speed(double s, double preview_distance) const {
        if (!course_attack_plan_.empty()) {
            return lookup_planned_target_speed(s);
        }
        return lookup_preview_min_target_speed(s, preview_distance);
    }

    double course_reference_speed_for_phase(
        double s,
        double /*current_speed*/,
        const PhaseSchedule& phase,
        double preview_distance) const
    {
        if (course_attack_plan_.empty()) {
            return lookup_dynamic_base_target_speed(s, preview_distance);
        }
        if (!config_.attack_braking_corridor_guard_only
                && (phase.phase == TrackPhase::Brake || phase.phase == TrackPhase::TurnIn)) {
            return lookup_braking_corridor_planned_target_speed(s, preview_distance);
        }
        return lookup_planned_target_speed(s);
    }

    double lookup_preview_min_static_speed_cap(double s, double preview_distance) const {
        double cap = phase_schedule_at(s).static_speed_cap;
        if (cap <= 0.0 || !std::isfinite(cap)) {
            cap = lookup_base_target_speed(s);
        }
        const double distance = std::max(preview_distance, 0.0);
        constexpr double kStep = 1.0;
        for (double ds = kStep; ds <= distance + 1e-12; ds += kStep) {
            const double step_cap = phase_schedule_at(wrap_track_s(s + ds)).static_speed_cap;
            if (std::isfinite(step_cap) && step_cap > 0.0) {
                cap = std::min(cap, step_cap);
            }
        }
        return std::clamp(cap, config_.vx_min, config_.vx_max);
    }

    double compute_actuator_aware_target_speed(
        double s,
        double current_tire_angle,
        double current_speed,
        double current_lateral_margin) const
    {
        const double preview_distance =
            std::clamp(std::max(current_speed, kPredictionSpeedFloor) * 1.10, 6.0, 32.0);
        const double base_speed = lookup_dynamic_base_target_speed(s, preview_distance);
        const double curvature_cap = compute_curvature_speed_cap(s, current_speed);
        const double steering_cap = compute_steering_speed_cap(s, current_tire_angle, current_speed);
        const double margin_cap = compute_track_margin_speed_cap(current_lateral_margin, base_speed);
        double capped_speed = base_speed;
        if (std::isfinite(curvature_cap)) {
            capped_speed = std::min(capped_speed, curvature_cap);
        }
        if (std::isfinite(steering_cap)) {
            capped_speed = std::min(capped_speed, steering_cap);
        }
        if (std::isfinite(margin_cap)) {
            capped_speed = std::min(capped_speed, margin_cap);
        }
        return std::clamp(capped_speed, config_.vx_min, config_.vx_max);
    }

    AttackEvaluation evaluate_rollout(
        const vehicle::VehicleState& initial,
        const TrackingPrimitive& primitive,
        int primitive_index,
        double speed_scale,
        int plan_index) const
    {
        AttackEvaluation eval;
        eval.selected_plan_index = plan_index;
        eval.primitive_index = primitive_index;
        eval.primitive_kind_index = primitive_kind_index(primitive.kind);
        eval.speed_scale = speed_scale;
        const PhaseSchedule initial_phase = phase_schedule_at(hint_s_);
        eval.phase = initial_phase.phase;
        eval.phase_speed_bias = initial_phase.speed_bias;
        eval.phase_margin_bias = initial_phase.margin_bias;
        eval.phase_progress_bias = initial_phase.progress_bias;
        eval.segment_id = initial_phase.segment_id;
        eval.attack_allowed = initial_phase.attack_allowed;
        eval.static_speed_cap = initial_phase.static_speed_cap;
        const double initial_speed = std::max(initial.vx, 0.0);
        double x = initial.x;
        double y = initial.y;
        double psi = initial.psi;
        double vx = std::clamp(
            std::max(initial.vx, kPredictionSpeedFloor),
            kPredictionSpeedFloor,
            config_.vx_max);
        double vy = initial.vy;
        double yaw_rate_state = initial.r;
        double hint_s = hint_s_;
        double prev_s = hint_s_ >= 0.0 ? hint_s_ : 0.0;
        double previous_target_delta = prev_delta_;
        double previous_ax = prev_ax_;
        double cost = 0.0;

        SteeringActuatorPredictionState predicted_steering{
            current_tire_angle_for_prediction(),
            vehicle_params_.steering.torque_mode
                ? last_vehicle_ddelta_sw_ / std::max(vehicle_params_.steering.R_steer, 1e-6)
                : 0.0,
        };

        const int steps = std::max(config_.N, 1);
        const double dt = control_dt();
        const double delta_rate = effective_tire_steer_rate() * primitive.rate_scale;
        bool high_speed_phase_penalty_applied = false;

        for (int k = 0; k < steps; ++k) {
            const auto nearest = track_query_.nearest(x, y, hint_s);
            hint_s = nearest.s;
            const PhaseSchedule step_phase = phase_schedule_at(nearest.s);
            const auto point = track_.get_point(nearest.s);
            const double centerline_heading_error = normalize_angle(psi - nearest.theta);
            const auto tracking_geometry = compute_tracking_line_geometry(nearest.s);
            const double tracking_heading_error =
                normalize_angle(psi - tracking_geometry.heading);
            const double lateral_margin = this->lateral_margin(point, nearest.n);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(psi)
                    || !std::isfinite(vx) || !std::isfinite(lateral_margin)) {
                eval.reject_reason = RejectReason::Nonfinite;
                eval.cost = std::numeric_limits<double>::infinity();
                return eval;
            }
            if (lateral_margin < -0.10) {
                eval.reject_reason = RejectReason::BoundaryMargin;
                eval.cost = std::numeric_limits<double>::infinity();
                return eval;
            }

            const double target_delta_unlimited = compute_feedback_target_delta(
                primitive,
                x,
                y,
                psi,
                vx,
                nearest,
                tracking_heading_error,
                lateral_margin);
            const double target_delta_raw = std::clamp(
                target_delta_unlimited,
                -steering_limit(),
                steering_limit());
            const double target_delta = std::clamp(
                target_delta_raw,
                previous_target_delta - std::max(delta_rate, 1e-6) * dt,
                previous_target_delta + std::max(delta_rate, 1e-6) * dt);

            if (vehicle_params_.steering.torque_mode
                    && config_.use_actuator_dynamics_in_prediction) {
                auto prediction_config = steering_actuator_config_;
                prediction_config.max_tire_steer_rate = std::max(delta_rate, 1e-6);
                predicted_steering = predict_steering_actuator_response(
                    prediction_config,
                    predicted_steering,
                    target_delta,
                    dt);
            } else {
                const double max_delta_step = std::max(delta_rate, 1e-6) * dt;
                const double rate_limited_step = std::clamp(
                    target_delta - predicted_steering.tire_angle,
                    -max_delta_step,
                    max_delta_step);
                predicted_steering.tire_angle += rate_limited_step;
                predicted_steering.tire_rate = rate_limited_step / std::max(dt, 1e-6);
            }
            const double predicted_delta = predicted_steering.tire_angle;

            const double outward_rate =
                outward_lateral_rate(nearest.n, centerline_heading_error, vx);
            const SpeedCapBreakdown target_speed_breakdown = compute_feedback_target_speed_breakdown(
                nearest.s,
                predicted_delta,
                vx,
                lateral_margin,
                tracking_heading_error,
                outward_rate,
                speed_scale,
                step_phase);
            const double target_speed = target_speed_breakdown.limited_speed;
            const LongitudinalCommand longitudinal_command = compute_feedback_ax_breakdown(
                nearest.s,
                target_speed_breakdown,
                target_speed,
                vx,
                lateral_margin,
                tracking_heading_error,
                outward_rate,
                step_phase);
            const double ax = longitudinal_command.ax;

            if (k == 0) {
                eval.first_delta = target_delta;
                eval.first_ax = ax;
                eval.first_target_speed = target_speed;
                eval.first_course_a_ref = target_speed_breakdown.course_a_ref;
                eval.first_course_target_gap = target_speed_breakdown.course_target_gap;
                eval.first_dynamic_cap_gap = target_speed_breakdown.dynamic_cap_gap;
                eval.first_braking_corridor_active =
                    target_speed_breakdown.braking_corridor.active;
                eval.first_braking_corridor_v_lower =
                    target_speed_breakdown.braking_corridor.v_lower;
                eval.first_braking_corridor_v_upper =
                    target_speed_breakdown.braking_corridor.v_upper;
                eval.first_residual_delta_v_ref =
                    residual_correction_at(nearest.s).delta_v_ref_mps;
                eval.first_residual_delta_steer_ff =
                    residual_steer_ff_at(wrap_track_s(nearest.s + std::max(
                        primitive.min_lookahead,
                        std::max(vx, kPredictionSpeedFloor) * primitive.preview_time)));
                eval.first_residual_delta_ax_bias =
                    longitudinal_command.residual_delta_ax_bias;
                eval.first_residual_delta_heading_relax =
                    longitudinal_command.residual_delta_heading_relax;
                eval.first_residual_safety_gate_active =
                    longitudinal_command.residual_safety_gate_active;
                eval.first_raw_heading_error = longitudinal_command.raw_heading_error;
                eval.first_effective_heading_error =
                    longitudinal_command.effective_heading_error;
                eval.first_ax_before_residual = longitudinal_command.ax_before_residual;
                eval.first_ax_after_residual = longitudinal_command.ax_after_residual;
                eval.first_longitudinal_mode = longitudinal_command.mode;
                eval.first_healthy_underspeed = longitudinal_command.healthy_underspeed;
                eval.first_accel_headroom = longitudinal_command.accel_headroom;
                eval.phase = step_phase.phase;
                eval.phase_speed_bias = step_phase.speed_bias;
                eval.phase_margin_bias = step_phase.margin_bias;
                eval.phase_progress_bias = step_phase.progress_bias;
                eval.segment_id = step_phase.segment_id;
                eval.attack_allowed = step_phase.attack_allowed;
                eval.static_speed_cap = step_phase.static_speed_cap;
                eval.dynamic_speed_cap = target_speed;
                eval.dynamic_base_speed = target_speed_breakdown.base_speed;
                eval.curvature_speed_cap = target_speed_breakdown.curvature_cap;
                eval.steering_speed_cap = target_speed_breakdown.steering_cap;
                eval.margin_speed_cap = target_speed_breakdown.margin_cap;
                eval.heading_speed_cap = target_speed_breakdown.heading_cap;
                eval.outward_speed_cap = target_speed_breakdown.outward_cap;
                eval.limiting_cap_reason = target_speed_breakdown.limiting_reason;
            }

            const double yaw_rate = yaw_rate_state;
            const auto next_state = predict_rollout_step(
                x,
                y,
                psi,
                vx,
                vy,
                yaw_rate_state,
                predicted_delta,
                ax,
                dt);
            x = next_state.x;
            y = next_state.y;
            psi = normalize_angle(next_state.psi);
            vx = std::clamp(next_state.vx, kPredictionSpeedFloor, config_.vx_max);
            vy = std::clamp(next_state.vy, -15.0, 15.0);
            yaw_rate_state = std::clamp(next_state.r, -8.0, 8.0);

            const auto after = track_query_.nearest(x, y, hint_s);
            hint_s = after.s;
            const PhaseSchedule after_phase = phase_schedule_at(after.s);
            const auto after_geometry = compute_tracking_line_geometry(after.s);
            const double signed_progress = signed_track_progress(prev_s, after.s);
            eval.signed_progress += signed_progress;
            eval.progress += std::max(0.0, signed_progress);
            prev_s = after.s;

            const auto after_point = track_.get_point(after.s);
            const double after_centerline_heading_error = normalize_angle(psi - after.theta);
            const double after_tracking_heading_error =
                normalize_angle(psi - after_geometry.heading);
            const double after_tracking_n = after.n - after_geometry.target_n;
            const double after_margin = this->lateral_margin(after_point, after.n);
            eval.terminal_n = after_tracking_n;
            eval.terminal_heading = after_tracking_heading_error;
            eval.terminal_speed = vx;
            eval.min_margin = std::min(eval.min_margin, after_margin);
            const double boundary_violation =
                std::max(0.0, config_.hard_boundary_margin - after_margin);
            const double soft_boundary_risk =
                std::max(0.0, config_.soft_boundary_margin - after_margin);
            const double after_outward_rate =
                outward_lateral_rate(after.n, after_centerline_heading_error, vx);
            const double projected_margin =
                after_margin
              - std::max(0.0, after_outward_rate) * kBoundaryRiskProjectionTime;
            const double boundary_risk = std::clamp(
                (config_.soft_boundary_margin - projected_margin)
                    / std::max(config_.soft_boundary_margin, 1e-3),
                0.0,
                1.0);
            const bool boundary_risk_active =
                projected_margin < config_.soft_boundary_margin;
            const double boundary_outward_rate =
                boundary_risk_active ? std::max(0.0, after_outward_rate) : 0.0;
            const double yaw_rate_error = yaw_rate - vx * after_geometry.curvature;
            const double delta_step_cmd = target_delta - previous_target_delta;
            const double ax_step_cmd = ax - previous_ax;
            const double terminal_scale = (k + 1 == steps) ? config_.terminal_weight : 1.0;

            const double tracking_cost = after_phase.tracking_cost_scale * (
                terminal_scale * (
                config_.w_y * after_tracking_n * after_tracking_n
              + config_.w_psi * after_tracking_heading_error * after_tracking_heading_error)
              + 0.08 * yaw_rate_error * yaw_rate_error);
            const double speed_cost_target = compute_feedback_target_speed(
                after.s,
                predicted_delta,
                vx,
                after_margin,
                after_tracking_heading_error,
                after_outward_rate,
                1.0,
                after_phase);
            const double speed_error = vx - speed_cost_target;
            const double overspeed = std::max(0.0, speed_error);
            const double underspeed = std::max(0.0, -speed_error);
            const double overspeed_cost =
                after_phase.speed_cost_scale
              * terminal_scale * config_.w_vx * 1.15 * overspeed * overspeed;
            const double underspeed_cost =
                after_phase.speed_cost_scale
              * terminal_scale * config_.w_vx * 1.75 * underspeed * underspeed;
            const double speed_cost = overspeed_cost + underspeed_cost;
            const double boundary_cost = after_phase.boundary_cost_scale * terminal_scale * (
                config_.hard_boundary_weight * boundary_violation * boundary_violation
              + config_.soft_boundary_weight * soft_boundary_risk * soft_boundary_risk
              + config_.outward_rate_weight
                    * boundary_outward_rate * boundary_outward_rate);
            const double control_cost =
                config_.w_delta * target_delta * target_delta
              + config_.w_ax * ax * ax
              + config_.w_delta_rate * delta_step_cmd * delta_step_cmd
              + config_.w_ax_rate * ax_step_cmd * ax_step_cmd;
            eval.cost_tracking += tracking_cost;
            eval.cost_speed += speed_cost;
            eval.cost_overspeed += overspeed_cost;
            eval.cost_underspeed += underspeed_cost;
            eval.cost_boundary += boundary_cost;
            eval.cost_control_smoothness += control_cost;
            cost += tracking_cost + speed_cost + boundary_cost + control_cost;
            if (!high_speed_phase_penalty_applied
                    && after_phase.suppress_high_speed
                    && speed_scale >= 0.949999) {
                const double primitive_cost = boundary_risk_active ? 500.0 : 0.0;
                eval.cost_primitive_bias += primitive_cost;
                cost += primitive_cost;
                high_speed_phase_penalty_applied = true;
            }
            if (boundary_risk_active) {
                if (primitive.kind == PrimitiveKind::BoundaryRecovery
                        && boundary_risk > 0.10) {
                    const double primitive_cost = -6.0 * boundary_risk;
                    eval.cost_primitive_bias += primitive_cost;
                    cost += primitive_cost;
                } else if (primitive.kind == PrimitiveKind::Tracking
                        && boundary_risk > 0.25) {
                    const double primitive_cost =
                        18.0 * boundary_risk * boundary_risk;
                    eval.cost_primitive_bias += primitive_cost;
                    cost += primitive_cost;
                }
            } else if (primitive.kind == PrimitiveKind::BoundaryRecovery) {
                constexpr double kInactiveBoundaryRecoveryCost = 5.0;
                eval.cost_primitive_bias += kInactiveBoundaryRecoveryCost;
                cost += kInactiveBoundaryRecoveryCost;
            }
            if (initial_speed < kLowSpeedRecoverySpeed
                    && vx < kLowSpeedRecoverySpeed
                    && after_margin > config_.hard_boundary_margin) {
                if (primitive.kind == PrimitiveKind::LowSpeedPurePursuit) {
                    eval.cost_primitive_bias -= 30.0;
                    cost -= 30.0;
                } else {
                    eval.cost_primitive_bias += 45.0;
                    cost += 45.0;
                }
            }
            if (!std::isfinite(after_margin)) {
                eval.reject_reason = RejectReason::Nonfinite;
                eval.cost = std::numeric_limits<double>::infinity();
                return eval;
            }
            if (after_margin < -0.10) {
                eval.reject_reason = RejectReason::PostBoundaryMargin;
                eval.cost = std::numeric_limits<double>::infinity();
                return eval;
            }
            if (signed_progress < -1e-6) {
                const double progress_cost = after_phase.progress_cost_scale
                    * 200.0 * signed_progress * signed_progress;
                eval.cost_progress += progress_cost;
                cost += progress_cost;
            }
            if (vx < kLowSpeedRecoverySpeed && after_margin > config_.hard_boundary_margin) {
                const double low_speed_gap = kLowSpeedRecoverySpeed - vx;
                const double progress_cost = after_phase.progress_cost_scale
                    * 12.0 * low_speed_gap * low_speed_gap;
                eval.cost_progress += progress_cost;
                cost += progress_cost;
            }

            eval.progress_target +=
                std::max(kPredictionSpeedFloor, lookup_planned_target_speed(after.s))
              * dt
              * std::clamp(config_.progress_target_fraction, 0.0, 1.0)
              * after_phase.progress_bias;
            previous_target_delta = target_delta;
            previous_ax = ax;
        }

        eval.progress_deficit = std::max(0.0, eval.progress_target - eval.progress);
        if (config_.terminal_progress_weight > 0.0) {
            const double progress_cost = initial_phase.progress_cost_scale
                * config_.terminal_progress_weight
                * eval.progress_deficit
                * eval.progress_deficit;
            eval.cost_progress += progress_cost;
            eval.cost_time_progress += progress_cost;
            cost += progress_cost;
        }
        if (eval.signed_progress < 0.35 * eval.progress_target) {
            const double shortfall = 0.35 * eval.progress_target - eval.signed_progress;
            const double progress_cost =
                initial_phase.progress_cost_scale * 20.0 * shortfall * shortfall;
            eval.cost_progress += progress_cost;
            eval.cost_time_progress += progress_cost;
            cost += progress_cost;
        }
        const double progress_reward =
            -0.08 * initial_phase.progress_bias * eval.progress;
        eval.cost_progress += progress_reward;
        cost += progress_reward;
        if (primitive.kind == PrimitiveKind::Tracking
                && speed_scale <= 0.800001
                && (is_attack_phase(initial_phase.phase)
                    || is_exit_or_straight_window(hint_s_, initial_speed))
                && initial_speed > 5.0
                && eval.min_margin > (
                    initial_phase.phase == TrackPhase::Complex ? 1.25 : 1.10)
                && std::abs(eval.terminal_heading) < (
                    initial_phase.phase == TrackPhase::Complex ? 0.14 : 0.16)
                && std::abs(eval.terminal_n) < (
                    initial_phase.phase == TrackPhase::Complex ? 0.70 : 0.85)
                && eval.signed_progress > 0.55 * eval.progress_target) {
            const double speed_gap = std::max(
                0.0,
                lookup_planned_target_speed(hint_s_) - initial_speed);
            const double severity = speed_scale <= 0.650001 ? 1.0 : 0.55;
            const double progress_underuse_penalty = severity * (
                45.0
                + 7.0 * std::max(0.0, eval.progress_deficit - 1.0)
                + 1.5 * speed_gap);
            eval.cost_progress += progress_underuse_penalty;
            eval.cost_time_progress += progress_underuse_penalty;
            cost += progress_underuse_penalty;
        }
        if (primitive.kind == PrimitiveKind::Tracking
                && (std::abs(speed_scale - 0.95) < 1e-6
                    || std::abs(speed_scale - 1.05) < 1e-6)
                && (is_speed_recovery_phase(initial_phase.phase)
                    || is_exit_or_straight_window(hint_s_, initial_speed))
                && initial_speed > 7.5
                && eval.min_margin > 1.30
                && std::abs(eval.terminal_heading) < 0.14
                && std::abs(eval.terminal_n) < 0.75
                && eval.progress_deficit > 2.50) {
            const double speed_gap = std::max(
                0.0,
                lookup_planned_target_speed(hint_s_) - initial_speed);
            const double exit_speed_reward =
                -std::min(
                    speed_scale > 1.0 ? 40.0 : 50.0,
                    18.0 + 2.5 * eval.progress_deficit + 1.5 * speed_gap);
            eval.cost_progress += exit_speed_reward;
            cost += exit_speed_reward;
        }
        if (primitive.kind == PrimitiveKind::Tracking
                && (std::abs(speed_scale - 0.95) < 1e-6
                    || std::abs(speed_scale - 1.05) < 1e-6)
                && initial_speed > 6.5
                && eval.min_margin > 1.05
                && std::abs(eval.terminal_heading) < 0.18
                && std::abs(eval.terminal_n) < 0.90
                && eval.signed_progress > 0.75 * eval.progress_target) {
            const double safe_attack_reward = speed_scale > 1.0 ? -45.0 : -75.0;
            eval.cost_progress += safe_attack_reward;
            cost += safe_attack_reward;
        }
        eval.cost = cost;
        return eval;
    }

    double compute_feedback_target_delta(
        const TrackingPrimitive& primitive,
        double x,
        double y,
        double psi,
        double speed,
        const track::NearestResult& nearest,
        double /*heading_error*/,
        double lateral_margin) const
    {
        const double lookahead = std::max(
            primitive.min_lookahead,
            std::max(speed, kPredictionSpeedFloor) * primitive.preview_time);
        const double preview_s = wrap_track_s(nearest.s + lookahead);
        if (primitive.kind == PrimitiveKind::LowSpeedPurePursuit
                && speed < kLowSpeedRecoverySpeed + 0.75) {
            return compute_pure_pursuit_delta(
                x,
                y,
                psi,
                preview_s,
                lookahead,
                primitive.pure_pursuit_gain);
        }

        const auto current_geometry = compute_tracking_line_geometry(nearest.s);
        const auto preview_geometry = compute_tracking_line_geometry(preview_s);
        const double tracking_lateral_error = nearest.n - current_geometry.target_n;
        const double tracking_heading_error = normalize_angle(psi - current_geometry.heading);
        const double preview_kappa = preview_geometry.curvature;
        double lateral_gain = primitive.lateral_gain;
        double heading_gain = primitive.heading_gain;
        double kff_scale = primitive.kff_scale;
        if (primitive.kind == PrimitiveKind::BoundaryRecovery) {
            const double margin_risk = std::clamp(
                (config_.soft_boundary_margin - lateral_margin)
                    / std::max(config_.soft_boundary_margin, 1e-3),
                0.0,
                1.0);
            lateral_gain *= 1.0 + 1.4 * margin_risk;
            heading_gain *= 1.0 + 0.8 * margin_risk;
            kff_scale *= 0.85;
        }
        const double residual_steer_ff = residual_steer_ff_at(preview_s);
        return std::clamp(
            kff_scale * required_tire_angle(preview_kappa, std::max(speed, 4.0))
          + lateral_gain * tracking_lateral_error
          + heading_gain * tracking_heading_error
          + residual_steer_ff,
            -steering_limit(),
            steering_limit());
    }

    double compute_feedback_target_speed(
        double s,
        double current_tire_angle,
        double current_speed,
        double lateral_margin,
        double heading_error,
        double outward_rate,
        double speed_scale,
        const PhaseSchedule& phase) const
    {
        return compute_feedback_target_speed_breakdown(
            s,
            current_tire_angle,
            current_speed,
            lateral_margin,
            heading_error,
            outward_rate,
            speed_scale,
            phase).limited_speed;
    }

    SpeedCapBreakdown compute_feedback_target_speed_breakdown(
        double s,
        double current_tire_angle,
        double current_speed,
        double lateral_margin,
        double heading_error,
        double outward_rate,
        double speed_scale,
        const PhaseSchedule& phase) const
    {
        SpeedCapBreakdown breakdown;
        const double preview_distance =
            std::clamp(std::max(current_speed, kPredictionSpeedFloor) * 1.00, 5.0, 28.0);
        const bool brake_corridor_phase =
            phase.phase == TrackPhase::Brake || phase.phase == TrackPhase::TurnIn;
        const BrakingCorridorSpeedBand corridor_band = brake_corridor_phase
            ? braking_corridor_speed_band_at(s, preview_distance)
            : BrakingCorridorSpeedBand{};
        const double reference_speed =
            (corridor_band.active && !config_.attack_braking_corridor_guard_only)
                ? corridor_band.v_upper
                : course_reference_speed_for_phase(s, current_speed, phase, preview_distance);
        const double course_speed = course_v_ref_at(s);
        const double base_speed = reference_speed
            * std::clamp(speed_scale, 0.1, 1.2)
            * phase.speed_bias;
        const double curvature_cap = compute_curvature_speed_cap(s, current_speed);
        const double steering_cap = compute_steering_speed_cap(s, current_tire_angle, current_speed);
        const double effective_margin = lateral_margin - phase.margin_bias;
        const double margin_cap = compute_track_margin_speed_cap(effective_margin, base_speed);
        breakdown.base_speed = base_speed;
        breakdown.course_v_ref = course_speed;
        breakdown.course_a_ref = course_a_ref_at(s);
        breakdown.course_target_gap = course_speed - current_speed;
        breakdown.curvature_cap = curvature_cap;
        breakdown.steering_cap = steering_cap;
        breakdown.margin_cap = margin_cap;
        breakdown.braking_corridor = corridor_band;
        double capped_speed = base_speed;
        breakdown.limiting_reason = "course_ref";
        auto apply_cap = [&](double cap, const char* reason) {
            if (std::isfinite(cap) && cap < capped_speed) {
                capped_speed = cap;
                breakdown.limiting_reason = reason;
            }
        };
        const double abs_heading = std::abs(heading_error);
        const auto residual_safety =
            residual_safety_state_at(s, abs_heading, effective_margin, outward_rate);
        const double safety_abs_heading = residual_safety.effective_abs_heading;
        const bool heading_risk = safety_abs_heading > 0.15;
        const bool margin_risk =
            effective_margin < config_.soft_boundary_margin
            || (outward_rate > 0.10
                && effective_margin < config_.soft_boundary_margin + 0.30);
        const bool complex_phase_risk =
            phase.phase == TrackPhase::Complex
            && (heading_risk
                || margin_risk
                || (outward_rate > 0.08
                    && effective_margin < config_.soft_boundary_margin + 0.55));
        if (course_attack_plan_.empty() || complex_phase_risk || heading_risk || margin_risk) {
            apply_cap(curvature_cap, "curvature");
        }
        apply_cap(steering_cap, "steering");
        apply_cap(margin_cap, "margin");

        double heading_cap = std::numeric_limits<double>::infinity();
        if (safety_abs_heading > 0.55) {
            heading_cap = 6.5;
        } else if (safety_abs_heading > 0.30) {
            heading_cap = 8.0;
        } else if (safety_abs_heading > 0.15) {
            heading_cap = 10.0;
        }
        breakdown.heading_cap = heading_cap;
        apply_cap(heading_cap, "heading");

        double outward_cap = std::numeric_limits<double>::infinity();
        if (outward_rate > 0.10 && effective_margin < config_.soft_boundary_margin) {
            const double risk = std::clamp(
                (config_.soft_boundary_margin - effective_margin)
                    / std::max(config_.soft_boundary_margin, 1e-3),
                0.0,
                1.0);
            outward_cap = 8.0 + 3.0 * (1.0 - risk);
        }
        breakdown.outward_cap = outward_cap;
        apply_cap(outward_cap, "outward");

        if (current_speed < kLowSpeedRecoverySpeed && lateral_margin > config_.hard_boundary_margin) {
            capped_speed = std::max(capped_speed, 3.0);
        }
        breakdown.limited_speed = std::clamp(capped_speed, config_.vx_min, config_.vx_max);
        breakdown.dynamic_cap_gap = std::max(0.0, course_speed - breakdown.limited_speed);
        if (breakdown.limited_speed <= config_.vx_min + 1e-9
                && capped_speed < config_.vx_min - 1e-9) {
            breakdown.limiting_reason = "vx_min";
        } else if (breakdown.limited_speed >= config_.vx_max - 1e-9
                && capped_speed > config_.vx_max + 1e-9) {
            breakdown.limiting_reason = "vx_max";
        }
        return breakdown;
    }

    LongitudinalCommand compute_feedback_ax_breakdown(
        double s,
        const SpeedCapBreakdown& target_speed_breakdown,
        double target_speed,
        double current_speed,
        double lateral_margin,
        double heading_error,
        double outward_rate,
        const PhaseSchedule& phase) const
    {
        LongitudinalCommand command;
        const double max_decel = std::max(-config_.ax_min, 1e-6);
        const double effective_margin = lateral_margin - phase.margin_bias;
        const double abs_heading = std::abs(heading_error);
        const auto residual_safety =
            residual_safety_state_at(s, abs_heading, effective_margin, outward_rate);
        const double safety_abs_heading = residual_safety.effective_abs_heading;
        command.residual_delta_heading_relax = residual_safety.heading_relax;
        command.residual_safety_gate_active = residual_safety.active;
        command.raw_heading_error = residual_safety.raw_abs_heading;
        command.effective_heading_error = safety_abs_heading;
        command.braking_corridor = target_speed_breakdown.braking_corridor;
        const bool safe_low_speed_recovery =
            current_speed < kLowSpeedRecoverySpeed
            && lateral_margin > -0.02
            && (lateral_margin > config_.hard_boundary_margin || safety_abs_heading < 0.35);
        if (safe_low_speed_recovery) {
            command.ax = std::clamp(1.0, config_.ax_min, config_.ax_max);
            command.mode = "low_speed_recovery";
            command.ax_before_residual = command.ax;
            command.ax_after_residual = command.ax;
            command.accel_headroom =
                std::max(0.0, config_.ax_max - std::max(command.ax, 0.0));
            return command;
        }
        if (effective_margin < config_.hard_boundary_margin) {
            command.ax = std::clamp(-0.55 * max_decel, config_.ax_min, config_.ax_max);
            command.mode = "hard_boundary";
            command.ax_before_residual = command.ax;
            command.ax_after_residual = command.ax;
            command.accel_headroom =
                std::max(0.0, config_.ax_max - std::max(command.ax, 0.0));
            return command;
        }

        const double course_speed = course_v_ref_at(s);
        const double course_speed_gap = course_speed - current_speed;
        const double course_accel = course_a_ref_at(s);
        const double speed_error = target_speed - current_speed;
        double accel_feedforward = course_accel;
        if (course_accel < 0.0) {
            accel_feedforward = speed_error > 0.15 ? 0.0 : 0.45 * course_accel;
        }
        double ax = accel_feedforward + 1.60 * speed_error;
        command.mode = speed_error < -0.30 ? "overspeed_brake" : "track_speed";
        if (std::abs(speed_error) < 0.15) {
            ax = accel_feedforward;
        }
        if (speed_error < -0.30) {
            ax = std::min(ax, 1.45 * speed_error);
        }
        const bool braking_corridor_active =
            target_speed_breakdown.braking_corridor.active
            && (phase.phase == TrackPhase::Brake || phase.phase == TrackPhase::TurnIn);
        const bool braking_corridor_safety_limited =
            safety_abs_heading > 0.15
            || effective_margin < config_.soft_boundary_margin
            || (outward_rate > 0.10
                && effective_margin < config_.soft_boundary_margin + 0.30)
            || target_speed_breakdown.limiting_reason == "steering"
            || target_speed_breakdown.limiting_reason == "margin"
            || target_speed_breakdown.limiting_reason == "heading"
            || target_speed_breakdown.limiting_reason == "outward";
        const bool braking_corridor_healthy =
            braking_corridor_active
            && !braking_corridor_safety_limited
            && effective_margin > config_.soft_boundary_margin + 0.25
            && safety_abs_heading < 0.12
            && outward_rate < 0.16;
        if (braking_corridor_active && !braking_corridor_safety_limited) {
            const double v_lower = target_speed_breakdown.braking_corridor.v_lower;
            const double v_upper = target_speed_breakdown.braking_corridor.v_upper;
            if (current_speed > v_upper + 0.25) {
                const double overspeed_gap = current_speed - v_upper;
                ax = std::min(
                    ax,
                    -std::max(config_.attack_braking_corridor_overspeed_brake_gain, 0.0)
                        * overspeed_gap);
                command.mode = "braking_corridor_overspeed";
            } else if (current_speed < v_lower && braking_corridor_healthy) {
                const double underspeed_gap = v_lower - current_speed;
                const double lower_band_ax = std::min(
                    0.28 * std::max(config_.ax_max, 0.0),
                    0.55 + 1.05 * underspeed_gap);
                ax = std::max(ax, lower_band_ax);
                command.mode = "braking_corridor_underspeed";
            } else if (current_speed >= v_lower - 1e-9 && current_speed <= v_upper + 1e-9) {
                const double mild_accel_cap = braking_corridor_healthy
                    ? 0.18
                        * std::max(config_.attack_braking_corridor_mild_accel_scale, 0.0)
                        * std::max(config_.ax_max, 0.0)
                    : 0.0;
                ax = std::clamp(ax, 0.0, mild_accel_cap);
                command.mode = mild_accel_cap > 1e-9
                    ? "braking_corridor_band"
                    : "braking_corridor_coast";
            }
        }
        const bool fast_recovery_underspeed =
            is_speed_recovery_phase(phase.phase)
            && course_speed_gap > 2.0
            && effective_margin > config_.soft_boundary_margin + 0.25
            && safety_abs_heading < 0.12
            && outward_rate < 0.20;
        const bool controlled_complex_underspeed =
            phase.phase == TrackPhase::Complex
            && course_speed_gap > 2.0
            && speed_error > 0.80
            && target_speed > course_speed - 1.50
            && effective_margin > config_.soft_boundary_margin + 0.55
            && safety_abs_heading < 0.12
            && outward_rate < 0.08;
        command.healthy_underspeed =
            fast_recovery_underspeed || controlled_complex_underspeed;
        double healthy_underspeed_min_ax = 0.0;
        if (command.healthy_underspeed) {
            healthy_underspeed_min_ax = std::max(config_.ax_max, 0.0)
                * (fast_recovery_underspeed ? 0.35 : 0.22);
            ax = std::max(ax, healthy_underspeed_min_ax);
            command.mode = "healthy_underspeed";
        }
        if (safety_abs_heading > 0.18) {
            ax = std::min(ax, 0.0);
            command.mode = "heading_safety";
        }
        if (safety_abs_heading > 0.30) {
            ax = std::min(ax, -0.20 * max_decel);
            command.mode = "heading_safety";
        }
        if (safety_abs_heading > 0.55) {
            ax = std::min(ax, -0.40 * max_decel);
            command.mode = "heading_safety";
        }
        if (outward_rate > 0.20 && effective_margin < config_.soft_boundary_margin) {
            const double risk = std::clamp(
                (config_.soft_boundary_margin - effective_margin)
                    / std::max(config_.soft_boundary_margin, 1e-3),
                0.0,
                1.0);
            ax = std::min(ax, -0.20 * max_decel * risk);
            command.mode = "outward_safety";
        }
        command.ax_before_residual = ax;
        double residual_ax_bias = residual_safety.ax_bias;
        if (speed_error < -0.15) {
            residual_ax_bias = std::min(0.0, residual_ax_bias);
        }
        if (safety_abs_heading > 0.30) {
            residual_ax_bias = std::min(0.0, residual_ax_bias);
        }
        if (residual_safety.active && std::abs(residual_ax_bias) > 1e-9) {
            ax += residual_ax_bias;
            command.residual_delta_ax_bias = residual_ax_bias;
            if (command.mode == "track_speed") {
                command.mode = "track_speed_residual";
            } else if (command.mode == "healthy_underspeed") {
                command.mode = "healthy_underspeed_residual";
            }
        }
        if (command.healthy_underspeed) {
            ax = std::max(ax, healthy_underspeed_min_ax);
        }
        if (std::abs(ax) < 0.05) {
            ax = 0.0;
        }
        command.ax = std::clamp(ax, config_.ax_min, config_.ax_max);
        command.ax_after_residual = command.ax;
        command.accel_headroom =
            std::max(0.0, config_.ax_max - std::max(command.ax, 0.0));
        return command;
    }

    double compute_pure_pursuit_delta(
        double x,
        double y,
        double psi,
        double target_s,
        double min_lookahead,
        double gain) const
    {
        const auto target = planned_line_point(target_s);
        const double dx = target.first - x;
        const double dy = target.second - y;
        const double alpha = normalize_angle(std::atan2(dy, dx) - psi);
        const double lookahead = std::max(min_lookahead, std::hypot(dx, dy));
        const double left_positive_delta =
            std::atan2(2.0 * wheelbase_ * std::sin(alpha), std::max(lookahead, 1e-6));
        return std::clamp(
            -std::max(gain, 0.0) * left_positive_delta,
            -steering_limit(),
            steering_limit());
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
                ? compute_torque_actuator_output(target_delta, config_.dt, 0.0).steering_torque
                : 0.0;
        } else {
            input.steering = target_delta;
        }

        if (ax >= 0.0) {
            input.throttle = std::clamp(
                ax / positive_accel_throttle_scale(),
                0.0,
                1.0);
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

    double positive_accel_throttle_scale() const {
        const double requested_scale = std::max(
            config_.attack_speed_accel_limit,
            0.60 * std::max(config_.ax_max, 1e-6));
        return std::clamp(
            requested_scale,
            1e-6,
            std::max(config_.ax_max, 1e-6));
    }

    SteeringActuatorOutput compute_torque_actuator_output(
        double target_delta,
        double dt,
        double T_SAT_total)
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
        actuator_config.max_tire_steer_rate = effective_tire_steer_rate();
        actuator_config.inner_loop_omega_n =
            std::max(config_.steering_inner_loop_omega_n, 1e-6);
        actuator_config.inner_loop_zeta =
            std::max(config_.steering_inner_loop_zeta, 0.0);

        const double I = std::max(vehicle_params_.steering.I_steer, 1e-6);
        const double C = std::max(vehicle_params_.steering.C_steer_torque, 0.0);
        actuator_config.Kp_inner =
            I * actuator_config.inner_loop_omega_n * actuator_config.inner_loop_omega_n;
        actuator_config.Kd_inner = std::max(
            2.0 * actuator_config.inner_loop_zeta * actuator_config.inner_loop_omega_n * I - C,
            0.0);
        actuator_config.Ki_inner = 0.10 * actuator_config.Kp_inner;
        actuator_config.sat_feedforward_gain = 1.0;
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
        constexpr double kNominalAuthority = (40.0 - 0.3) / 0.02;
        return std::clamp(authority / kNominalAuthority, 0.45, 1.35);
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
        constexpr double kTargetUndersteerGradient = 0.002; // [rad/(m/s^2)]
        const double ay = speed * speed * kappa;
        return std::clamp(
            -(wheelbase_ * kappa + kTargetUndersteerGradient * ay),
            -steering_limit(),
            steering_limit());
    }

    double predict_yaw_rate(double speed, double tire_angle) const {
        const double understeer_denominator =
            wheelbase_ + attack_rollout_understeer_gradient() * speed * speed;
        return -speed * std::tan(tire_angle) / std::max(understeer_denominator, 1e-6);
    }

    vehicle::BicycleState6DOF predict_rollout_step(
        double x,
        double y,
        double psi,
        double vx,
        double vy,
        double yaw_rate,
        double tire_angle,
        double ax,
        double dt) const
    {
        vehicle::BicycleModel6DOF model(bicycle_params_);
        const vehicle::BicycleState6DOF state{
            x,
            y,
            psi,
            std::clamp(vx, kPredictionSpeedFloor, config_.vx_max),
            std::clamp(vy, -15.0, 15.0),
            std::clamp(yaw_rate, -8.0, 8.0),
        };
        const vehicle::BicycleControl control{
            std::clamp(tire_angle, -steering_limit(), steering_limit()),
            std::clamp(ax, config_.ax_min, config_.ax_max),
        };
        return model.predict_step(state, control, std::clamp(dt, 0.001, 0.100));
    }

    double attack_rollout_understeer_gradient() const {
        return std::max(config_.attack_rollout_understeer_gradient, 0.0);
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
            0.35,
            1.50);
        const double preview_distance = std::clamp(
            std::max(current_speed, kPredictionSpeedFloor) * preview_time,
            4.0,
            40.0);

        double cap = std::numeric_limits<double>::infinity();
        constexpr double kStep = 1.0;
        constexpr double kDeadband = 0.018;
        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            const double sample_s = wrap_track_s(s + ds);
            const double kappa = tracking_curvature_at(sample_s);
            const double required = required_tire_angle(kappa, std::max(current_speed, kPredictionSpeedFloor));
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

    double compute_curvature_speed_cap(double s, double current_speed) const {
        const double preview_time = std::clamp(
            static_cast<double>(std::max(config_.N, 1)) * control_dt(),
            0.50,
            1.50);
        const double preview_distance = std::clamp(
            std::max(current_speed, kPredictionSpeedFloor) * preview_time,
            5.0,
            45.0);

        double max_abs_kappa = std::abs(tracking_curvature_at(s));
        constexpr double kStep = 1.0;
        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            max_abs_kappa = std::max(
                max_abs_kappa,
                std::abs(tracking_curvature_at(wrap_track_s(s + ds))));
        }
        if (max_abs_kappa <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }

        double cap_speed = std::clamp(
            std::max(current_speed, config_.vx_min),
            config_.vx_min,
            config_.vx_max);
        for (int iter = 0; iter < 6; ++iter) {
            const double lateral_accel_limit =
                attack_lateral_accel_limit(vehicle_params_, cap_speed)
              * std::clamp(config_.attack_speed_lateral_safety, 0.2, 1.0);
            cap_speed = std::sqrt(std::max(lateral_accel_limit / max_abs_kappa, 0.0));
        }
        return std::clamp(
            cap_speed,
            config_.vx_min,
            config_.vx_max);
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

    bool is_exit_or_straight_window(double s, double speed) const {
        const double preview_distance = std::clamp(
            std::max(speed, kPredictionSpeedFloor) * 0.90,
            6.0,
            24.0);
        const double k0 = std::abs(tracking_curvature_at(wrap_track_s(s)));
        const double k1 =
            std::abs(tracking_curvature_at(wrap_track_s(s + 0.50 * preview_distance)));
        const double k2 =
            std::abs(tracking_curvature_at(wrap_track_s(s + preview_distance)));
        const double max_abs_kappa = std::max(k0, std::max(k1, k2));
        if (max_abs_kappa < 0.025) {
            return true;
        }

        const bool curvature_releasing =
            k0 > 0.030
            && k1 <= k0 * 0.95 + 0.003
            && k2 <= k0 * 0.55
            && k2 < 0.040;
        const bool no_sharper_corner_ahead =
            k1 <= max_abs_kappa
            && k2 <= k0 + 0.010;
        return curvature_releasing && no_sharper_corner_ahead;
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
    TrackPhaseProfile phase_profile_;
    BoundaryLinePlannerConfig line_planner_config_;
    AttackResidualProfile residual_profile_;
    vehicle::VehicleParams vehicle_params_;
    vehicle::BicycleParams bicycle_params_ =
        vehicle::BicycleParams::from_vehicle_params(vehicle::VehicleParams::default_fsae());
    AttackSpeedPlannerConfig attack_speed_planner_config_;
    CourseAttackPlan course_attack_plan_;
    std::vector<laptime::VelocityPoint> velocity_profile_;
    std::vector<std::pair<double, double>> velocity_sv_;
    std::vector<PlannedLineGeometry> planned_line_geometry_profile_;

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
