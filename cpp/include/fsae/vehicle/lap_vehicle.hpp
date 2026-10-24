#pragma once
/// @file lap_vehicle.hpp
/// @brief Common vehicle interface for lap simulation backends.

#include "vehicle_14dof_new.hpp"

#include <array>
#include <cmath>
#include <string>

namespace fsae::vehicle {

class LapVehicle {
public:
    virtual ~LapVehicle() = default;

    virtual const State14DOF& state() const = 0;
    virtual void reset() = 0;
    virtual void set_initial_velocity(double vx) = 0;
    virtual void set_map_pose(double map_x, double map_y, double map_heading) = 0;
    virtual void step_adaptive(double dt, const DriverInput& input) = 0;
    virtual bool is_drs_active() const = 0;
    virtual const std::array<double, 4>& tire_Mz() const = 0;
    virtual std::string active_tire_runtime_description() const = 0;
};

class FullLapVehicle final : public LapVehicle {
public:
    explicit FullLapVehicle(const VehicleParams& params)
        : vehicle_(params) {}

    const State14DOF& state() const override { return vehicle_.state(); }
    void reset() override { vehicle_.reset(); }
    void set_initial_velocity(double vx) override { vehicle_.set_initial_velocity(vx); }

    void set_map_pose(double map_x, double map_y, double map_heading) override {
        vehicle_.state().x = map_y;
        vehicle_.state().y = map_x;
        vehicle_.state().z = 0.0;
        vehicle_.state().psi = M_PI / 2.0 - map_heading;
    }

    void step_adaptive(double dt, const DriverInput& input) override {
        vehicle_.step_adaptive(dt, input);
    }

    bool is_drs_active() const override { return vehicle_.is_drs_active(); }
    const std::array<double, 4>& tire_Mz() const override { return vehicle_.tire_Mz(); }

    std::string active_tire_runtime_description() const override {
        return vehicle_.active_tire_runtime_description();
    }

private:
    Vehicle14DOF_New vehicle_;
};

} // namespace fsae::vehicle
