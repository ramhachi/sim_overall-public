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

// #38: roll gains are residuals; with a travel polynomial present they must not add again.
TEST(CoreA_KC, RollGainNotDoubleCountedWithTravelPolynomial) {
    VehicleParams p = VehicleParams::default_fsae();
    auto& kc = p.suspension.kc_corners[0];
    kc.valid = true;
    kc.kinematics.camber_vs_travel = {-0.02, -0.5, 0.0};
    kc.kinematics.toe_vs_travel = {0.001, 0.05, 0.0};
    kc.kinematics.roll_camber_gain = 0.8;
    kc.kinematics.roll_steer_gain = 0.1;
    Vehicle14DOF_New veh(p);
    const double phi = 0.02, travel = 0.6 * phi;  // pure roll: travel = t/2 * phi
    const std::array<double, 6> loads = {0, 0, 0, 0, 0, 0};
    EXPECT_NEAR(veh.compute_camber_angle(0, travel, 0.0, phi, loads), -0.02 - 0.5 * travel, 1e-12);
    EXPECT_NEAR(veh.compute_toe_angle(0, travel, 0.0, phi, loads), 0.001 + 0.05 * travel, 1e-12);

    // Pure steer (travel 0, roll 0) still reads the steer polynomial.
    kc.kinematics.toe_vs_steer = {0.0, 0.1, 0.0};
    Vehicle14DOF_New veh2(p);
    EXPECT_NEAR(veh2.compute_toe_angle(0, 0.0, 0.2, 0.0, loads), 0.001 + 0.02, 1e-12);

    // Without a travel polynomial the roll gain is the only roll path and still applies.
    kc.kinematics.camber_vs_travel = {0.0, 0.0, 0.0};
    Vehicle14DOF_New veh3(p);
    const double base = p.suspension.static_camber[0] + p.suspension.camber_gain[0] * travel;
    EXPECT_NEAR(veh3.compute_camber_angle(0, travel, 0.0, phi, loads), base + 0.8 * phi, 1e-12);
}

namespace {
// Settled front-axle tyre load and pitch under steady braking.
std::pair<double, double> braking_front_load(double anti_brake) {
    VehicleParams p = VehicleParams::default_fsae();
    p.suspension.kc_front.anti_brake_pct = anti_brake;
    p.suspension.kc_rear.anti_brake_pct = anti_brake;
    p.suspension.bw_alpha = 0.0;  // hysteresis would mask the spring/link split
    p.aero.CL_A = 0.0;            // ride-height-sensitive aero would confound the totals
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(25.0);
    DriverInput in;
    in.brake = 0.25;
    in.gear = 3;
    for (int k = 0; k < 1500; ++k) veh.step(0.001, in);
    const auto fz = veh.get_tire_Fz();
    return {fz[0] + fz[1], veh.state().theta};
}

// Settled lateral load-transfer moment and roll under steady cornering.
std::pair<double, double> cornering_transfer(double h_rc) {
    VehicleParams p = VehicleParams::default_fsae();
    p.suspension.kc_front.roll_center_height = h_rc;
    p.suspension.kc_rear.roll_center_height = h_rc;
    p.suspension.bw_alpha = 0.0;
    p.aero.CL_A = 0.0;
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(12.0);
    DriverInput in;
    in.steering = 0.04;
    in.gear = 3;
    for (int k = 0; k < 3000; ++k) veh.step(0.001, in);
    const auto fz = veh.get_tire_Fz();
    const double m = 0.5 * p.mass.track_f * (fz[0] - fz[1]) + 0.5 * p.mass.track_r * (fz[2] - fz[3]);
    return {m, veh.state().phi};
}
}  // namespace

// #9: anti geometry moves load transfer from springs to links but keeps its total.
TEST(CoreA_Anti, TotalLongitudinalTransferIndependentOfAnti) {
    const auto [fz0, pitch0] = braking_front_load(0.0);
    const auto [fz1, pitch1] = braking_front_load(0.6);
    EXPECT_NEAR(fz1, fz0, 0.02 * fz0);
    EXPECT_LT(std::abs(pitch1), 0.85 * std::abs(pitch0));  // less dive (anti acts per-wheel Fx)
}

// #9: RC height moves lateral transfer to the geometric path but keeps its total.
TEST(CoreA_Anti, TotalLateralTransferIndependentOfRollCentre) {
    const auto [m0, roll0] = cornering_transfer(0.0);
    const auto [m1, roll1] = cornering_transfer(0.08);
    EXPECT_NEAR(m1, m0, 0.03 * std::abs(m0));
    EXPECT_LT(std::abs(roll1), 0.85 * std::abs(roll0));  // less body roll
}

namespace {
double yaw_rate_with_compliance(double k_steer, double kc_toe_per_mz) {
    VehicleParams p = VehicleParams::default_fsae();
    p.steering.stiffness = k_steer;
    for (int i = 0; i < 2; ++i) {
        p.suspension.kc_corners[i].valid = true;
        p.suspension.kc_corners[i].compliance.toe_per_load[5] = kc_toe_per_mz;
    }
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(12.0);
    DriverInput in;
    in.steering = 0.05;
    in.gear = 3;
    for (int k = 0; k < 2000; ++k) veh.step(0.001, in);
    return veh.state().r;
}
}  // namespace

// #6: K&C Mz->toe and K_steer must not both act on the same Mz.
TEST(CoreA_ComplianceSteer, BothPathsEqualSinglePath) {
    const double both = yaw_rate_with_compliance(1500.0, 1.0 / 1500.0);
    const double kc_only = yaw_rate_with_compliance(1e12, 1.0 / 1500.0);
    EXPECT_NEAR(both, kc_only, 1e-9);
    const double ksteer_only = yaw_rate_with_compliance(1500.0, 0.0);
    EXPECT_NEAR(ksteer_only, kc_only, 0.02 * std::abs(kc_only));
}

// #36: once stopped, braked wheels hold omega ~ 0 (copysign used to chatter at T*dt/Iw ~ 1 rad/s).
TEST(CoreA_Brake, StopAndHoldWithoutChatter) {
    VehicleParams p = VehicleParams::default_fsae();
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(2.0);
    DriverInput in;
    in.brake = 0.5;
    in.gear = 3;
    double max_hold = 0.0;
    for (int k = 0; k < 3000; ++k) {
        veh.step(0.001, in);
        if (k >= 1000) {
            for (double w : veh.state().omega) max_hold = std::max(max_hold, std::abs(w));
        }
    }
    EXPECT_LT(max_hold, 0.01);
    EXPECT_LT(std::abs(veh.state().vx), 0.01);
}

// #36: balance is the front/rear line-pressure ratio; geometry is carried by A*r.
TEST(CoreA_Brake, BalanceIsPressureRatio) {
    VehicleParams p = VehicleParams::default_fsae();
    p.brake.balance = 0.6;
    Vehicle14DOF_New veh(p);
    const auto T = veh.compute_brake_torque_max(1e6);
    const auto& b = p.brake;
    const double pf = T[0] / (b.caliper_area[0] * b.mu_pad * b.effective_radius[0]);
    const double pr = T[2] / (b.caliper_area[2] * b.mu_pad * b.effective_radius[2]);
    EXPECT_NEAR(pf / (pf + pr), 0.6, 1e-12);
    EXPECT_NEAR(0.5 * (pf + pr), 1e6, 1e-6);

    // Pedal table + lag: pressure follows the table and lags with tau.
    p.brake.pedal_table = {0.0, 1.0};
    p.brake.pressure_table = {0.0, 4e6};
    p.brake.pressure_lag_s = 0.03;
    Vehicle14DOF_New lagged(p);
    EXPECT_NEAR(lagged.compute_brake_pressure_cmd(0.5), 2e6, 1e-6);
    p.brake.pressure_lag_s = 0.0;
    Vehicle14DOF_New instant(p);
    lagged.set_initial_velocity(15.0);
    instant.set_initial_velocity(15.0);
    DriverInput in;
    in.brake = 1.0;
    for (int k = 0; k < 30; ++k) {
        lagged.step(0.001, in);
        instant.step(0.001, in);
    }
    EXPECT_GT(lagged.state().omega[0], instant.state().omega[0] + 0.5);  // lagged pressure brakes less
}
