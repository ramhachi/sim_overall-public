/// @file test_core_a_physics.cpp
/// @brief 14DOF runtime physics checks for the physics-precision epic (CORE-A).

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include <cmath>
#include <iostream>

using namespace fsae::vehicle;

namespace {

struct Snapshot {
    double vx, r, ay;
};

Snapshot run_constant_steer(const VehicleParams& p, double v0, double steer, double t_end) {
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(v0);
    DriverInput in;
    in.steering = steer;
    in.throttle = 0.0;
    in.gear = 3;
    const double dt = 0.001;
    for (int k = 0; k < static_cast<int>(t_end / dt); ++k) veh.step(dt, in);
    const auto& s = veh.state();
    return {s.vx, s.r, s.vx * s.r};
}

}  // namespace

/// Prints the headline numbers used for the before/after report.
TEST(CoreAReport, KeyNumbers) {
    const VehicleParams p = VehicleParams::default_fsae();

    const auto skid = run_constant_steer(p, 12.0, 0.06, 4.0);

    Vehicle14DOF_New step_veh(p);
    step_veh.set_initial_velocity(15.0);
    DriverInput in;
    in.gear = 3;
    double r_peak = 0.0;
    for (int k = 0; k < 1500; ++k) {
        in.steering = (k >= 200) ? 0.04 : 0.0;
        step_veh.step(0.001, in);
        r_peak = std::max(r_peak, std::abs(step_veh.state().r));
    }

    Vehicle14DOF_New acc_veh(p);
    acc_veh.set_initial_velocity(5.0);
    DriverInput acc;
    acc.throttle = 1.0;
    acc.gear = 1;
    for (int k = 0; k < 1000; ++k) acc_veh.step(0.001, acc);

    Vehicle14DOF_New brk_veh(p);
    brk_veh.set_initial_velocity(20.0);
    DriverInput brk;
    brk.brake = 0.6;
    brk.gear = 3;
    for (int k = 0; k < 1000; ++k) brk_veh.step(0.001, brk);

    std::cout << "[CORE-A] skidpad vx=" << skid.vx << " r=" << skid.r << " ay=" << skid.ay
              << "\n[CORE-A] step_steer r_peak=" << r_peak << " r_end=" << step_veh.state().r
              << "\n[CORE-A] accel_1s vx=" << acc_veh.state().vx
              << "\n[CORE-A] brake_1s vx=" << brk_veh.state().vx << std::endl;
    EXPECT_TRUE(std::isfinite(skid.ay));
}

// #3: planar EOM uses the full vehicle mass. In steady cornering vx*r = sum(Fy)/m_total.
TEST(CoreA_Mass, SkidpadLateralAccelUsesTotalMass) {
    const VehicleParams p = VehicleParams::default_fsae();
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(12.0);
    DriverInput in;
    in.steering = 0.03;
    in.gear = 3;
    for (int k = 0; k < 4000; ++k) veh.step(0.001, in);
    const auto& s = veh.state();
    double fy = 0.0;
    for (double f : veh.get_tire_Fy()) fy += f;
    const double m_total = p.mass.m_sprung + p.mass.m_unsprung[0] + p.mass.m_unsprung[1]
                         + p.mass.m_unsprung[2] + p.mass.m_unsprung[3];
    EXPECT_NEAR(fy / (s.vx * s.r), m_total, 0.03 * m_total);
}
