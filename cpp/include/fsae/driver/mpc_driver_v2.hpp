#pragma once
/// @file mpc_driver_v2.hpp
/// @brief Side-by-side MPC v2 entry point for the general driver program.
///
/// MPC v2 intentionally lives next to the legacy sampling MPC while the
/// comparison gate is being built.  It preserves the laptime state contract:
/// callers pass a map-frame VehicleState, typically produced by
/// to_map_driver_state(...), and this class does not transform it again.

#include "mpc_driver.hpp"

namespace fsae::driver {

class MPCPathFollowingDriverV2 {
public:
    MPCPathFollowingDriverV2(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const control::MPCConfig& config = control::MPCConfig(),
        const vehicle::VehicleParams& vehicle_params = vehicle::VehicleParams::default_fsae())
        : delegate_(track, velocity_profile, make_v2_config(config, vehicle_params), vehicle_params)
    {}

    vehicle::DriverInput compute_control(
        const vehicle::VehicleState& map_frame_vehicle_state,
        double current_time)
    {
        return delegate_.compute_control(map_frame_vehicle_state, current_time);
    }

    vehicle::DriverInput compute_outer_control(
        const vehicle::VehicleState& map_frame_vehicle_state,
        double current_time)
    {
        return delegate_.compute_outer_control(map_frame_vehicle_state, current_time);
    }

    vehicle::DriverInput update_torque_actuator_control(
        vehicle::DriverInput input,
        double delta_sw,
        double ddelta_sw,
        double dt,
        double T_SAT_total = 0.0)
    {
        return delegate_.update_torque_actuator_control(input, delta_sw, ddelta_sw, dt, T_SAT_total);
    }

    const MPCStats& stats() const { return delegate_.stats(); }
    const MPCDebugInfo& last_debug() const { return delegate_.last_debug(); }
    double current_s() const { return delegate_.current_s(); }

    void set_vehicle_params(const vehicle::VehicleParams& vehicle_params) {
        delegate_.set_vehicle_params(vehicle_params);
    }

    void set_steering_state(double delta_sw, double ddelta_sw) {
        delegate_.set_steering_state(delta_sw, ddelta_sw);
    }

    void reset() {
        delegate_.reset();
    }

    static control::MPCConfig make_v2_config(
        control::MPCConfig config,
        const vehicle::VehicleParams& vehicle_params)
    {
        config.N = std::max(config.N, vehicle_params.steering.torque_mode ? 60 : 40);
        config.dt = std::clamp(config.dt, 0.015, 0.035);

        // Phase-3 starting point: keep a deterministic sampler but bias it
        // toward track-margin and actuator-feasible behavior. These are not
        // final tuning constants; they define the v2 comparison baseline.
        config.w_y = std::max(config.w_y, 8.0);
        config.w_psi = std::max(config.w_psi, 4.0);
        config.w_vx = std::max(config.w_vx, 0.8);
        config.w_delta = std::max(config.w_delta, 0.02);
        config.w_ax = std::max(config.w_ax, 0.02);
        config.w_delta_rate = std::max(config.w_delta_rate, 0.25);
        config.terminal_weight = std::max(config.terminal_weight, 5.0);
        config.terminal_progress_weight = std::max(config.terminal_progress_weight, 1.0);
        config.progress_target_fraction = std::clamp(config.progress_target_fraction, 0.70, 0.95);
        config.use_actuator_dynamics_in_prediction = true;
        config.use_track_margin_speed_cap = true;
        config.track_margin_speed_cap = 8.0;
        config.track_margin_speed_release = 1.20;
        config.hard_boundary_margin = 0.35;
        config.soft_boundary_margin = 1.00;
        config.hard_boundary_weight = std::max(config.hard_boundary_weight, 1500.0);
        config.soft_boundary_weight = std::max(config.soft_boundary_weight, 160.0);
        config.outward_rate_weight = std::max(config.outward_rate_weight, 80.0);

        if (vehicle_params.steering.torque_mode) {
            config.delta_max = std::min(config.delta_max, vehicle_params.steering.max_angle);
            config.delta_rate_max = std::min(config.delta_rate_max, 0.08);
        }
        return config;
    }

private:
    MPCPathFollowingDriver delegate_;
};

} // namespace fsae::driver
