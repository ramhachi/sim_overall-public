#pragma once
/// @file state_frame.hpp
/// @brief Coordinate transforms between the 14DOF SAE frame and map-frame drivers.

#include "../vehicle/vehicle_14dof_new.hpp"
#include "../vehicle/vehicle_state.hpp"

#include <algorithm>
#include <cmath>

namespace fsae::driver {

/// Convert the 14DOF SAE J670e state into the map frame used by MPC.
///
/// Vehicle14DOF uses inertial X=map Y, inertial Y=map X, body Y=right, and
/// clockwise-positive yaw. The map-frame preview models use body Y=left and
/// counter-clockwise-positive yaw, so both lateral velocity and yaw rate flip.
inline vehicle::VehicleState to_map_driver_state(const vehicle::State14DOF& state) {
    vehicle::VehicleState driver_state;
    driver_state.x = state.y;
    driver_state.y = state.x;
    driver_state.z = state.z;
    driver_state.phi = state.phi;
    driver_state.theta = state.theta;
    driver_state.psi = M_PI / 2.0 - state.psi;
    driver_state.vx = std::max(state.vx, 0.0);
    driver_state.vy = -state.vy;
    driver_state.vz = state.vz;
    driver_state.p = state.p;
    driver_state.q = state.q;
    driver_state.r = -state.r;
    return driver_state;
}

} // namespace fsae::driver
