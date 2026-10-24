#pragma once
/// @file fast_lap_vehicle.hpp
/// @brief Reduced-order lap evaluator backed by the existing 6DOF surrogate.

#include "lap_vehicle.hpp"
#include "vehicle_6dof.hpp"

#include <algorithm>
#include <array>
#include <string>

namespace fsae::vehicle {

class FastLapVehicle final : public LapVehicle {
public:
    explicit FastLapVehicle(const VehicleParams& params)
        : params_(params),
          vehicle_(params)
    {
        sync_state_from_internal(0.0);
    }

    const State14DOF& state() const override { return state_; }

    void reset() override {
        vehicle_.reset();
        tire_mz_.fill(0.0);
        sync_state_from_internal(0.0);
    }

    void set_initial_velocity(double vx) override {
        vehicle_.set_initial_velocity(vx);
        sync_state_from_internal(0.0);
    }

    void set_map_pose(double map_x, double map_y, double map_heading) override {
        auto& s = vehicle_.state();
        s.x = map_y;
        s.y = map_x;
        s.z = vehicle_.z_equilibrium();
        s.psi = M_PI / 2.0 - map_heading;
        sync_state_from_internal(0.0);
    }

    void step_adaptive(double dt, const DriverInput& input) override {
        DriverInput integrated = input;
        integrated.steering = clamp_tire_angle(input.steering);
        vehicle_.step(std::max(dt, 1e-6), integrated);
        sync_state_from_internal(dt);
    }

    bool is_drs_active() const override { return false; }
    const std::array<double, 4>& tire_Mz() const override { return tire_mz_; }

    std::string active_tire_runtime_description() const override {
        return "fast_6dof_tmeasy_v1";
    }

private:
    VehicleParams params_;
    Vehicle6DOF vehicle_;
    State14DOF state_{};
    std::array<double, 4> tire_mz_{};

    double clamp_tire_angle(double delta_tire) const {
        return std::clamp(
            delta_tire,
            -std::abs(params_.steering.max_angle),
            std::abs(params_.steering.max_angle));
    }

    void sync_state_from_internal(double dt) {
        (void)dt;
        const auto& src = vehicle_.state();
        state_ = State14DOF{};

        state_.x = src.x;
        state_.y = src.y;
        state_.z = src.z;
        state_.phi = src.phi;
        state_.theta = src.theta;
        state_.psi = src.psi;
        state_.vx = src.vx;
        state_.vy = src.vy;
        state_.vz = src.vz;
        state_.p = src.p;
        state_.q = src.q;
        state_.r = src.r;
        state_.omega = src.omega;
        state_.tire_xi = src.tire_xi;
        state_.tire_eta = src.tire_eta;
        state_.phi_twist = 0.0;
        state_.dphi_twist = 0.0;
        // Match the legacy angle-mode contract used by the full 14DOF backend.
        state_.delta_sw = 0.0;
        state_.ddelta_sw = 0.0;
    }
};

} // namespace fsae::vehicle
