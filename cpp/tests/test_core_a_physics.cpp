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

// #3: planar EOM uses the full vehicle mass: sum(Fy_body) = m_total * (dvy/dt + r*vx - g_y).
TEST(CoreA_Mass, SkidpadLateralAccelUsesTotalMass) {
    VehicleParams p = VehicleParams::default_fsae();
    p.steering.ackermann = 0.0;  // both fronts at the same angle for the body-frame projection
    p.aero.CL_A = 0.0;
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(12.0);
    DriverInput in;
    in.steering = 0.03;
    in.gear = 3;
    for (int k = 0; k < 4000; ++k) veh.step(0.001, in);
    const double vy0 = veh.state().vy;
    veh.step(0.001, in);
    const auto& s = veh.state();
    const auto Fy = veh.get_tire_Fy_body();
    double fy_body = 0.0;
    for (int i = 0; i < 4; ++i) fy_body += Fy[i];
    const double g_y = 9.81 * std::sin(s.phi) * std::cos(s.theta);
    const double a_y = (s.vy - vy0) / 0.001 + s.r * s.vx - g_y;
    const double m_total = p.mass.m_sprung + p.mass.m_unsprung[0] + p.mass.m_unsprung[1]
                         + p.mass.m_unsprung[2] + p.mass.m_unsprung[3];
    EXPECT_NEAR(fy_body / a_y, m_total, 0.01 * m_total);  // was m_sprung (230 kg) before #3
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

namespace {
Vehicle14DOF_New launch(const VehicleParams& p, int steps, bool shift_mid) {
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(8.0);
    DriverInput in;
    in.throttle = 1.0;
    in.gear = 2;
    for (int k = 0; k < steps; ++k) {
        if (shift_mid && k == steps / 2) in.gear = 3;
        veh.step(0.001, in);
    }
    return veh;
}
}  // namespace

// #4: reflected engine/gearbox inertia slows rear-wheel spin-up and the launch.
TEST(CoreA_Driveline, ReflectedInertiaAndShiftCut) {
    VehicleParams p = VehicleParams::default_fsae();
    const auto base = launch(p, 50, false);
    p.powertrain.I_engine = 0.05;   // test value, not a measured CBR600 figure
    p.powertrain.I_gearbox = 0.01;
    const auto heavy = launch(p, 50, false);
    EXPECT_LT(heavy.state().omega[2], base.state().omega[2]);

    // Shift: drive torque is cut for shift_time_s after the gear change.
    p.powertrain.shift_time_s = 0.05;
    const auto shifting = launch(p, 60, true);   // gear change at 30 ms, still cut at 60 ms
    EXPECT_LE(shifting.get_tire_Fx()[2], 0.0);
    EXPECT_GT(heavy.get_tire_Fx()[2], 0.0);

    // mass.Iw as 4-element JSON array is accepted alongside the legacy scalar.
    const auto parsed = VehicleParams::from_json_string("{\"mass\": {\"Iw\": [0.3, 0.3, 0.5, 0.5]}}");
    EXPECT_DOUBLE_EQ(parsed.mass.Iw_corner[3], 0.5);
    EXPECT_DOUBLE_EQ(parsed.mass.Iw, 0.4);
}

namespace {
double coast_decel(double c_rr) {
    VehicleParams p = VehicleParams::default_fsae();
    p.tire.C_rr = c_rr;
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(15.0);
    DriverInput in;
    in.gear = 3;
    for (int k = 0; k < 500; ++k) veh.step(0.001, in);
    const double v0 = veh.state().vx;
    for (int k = 0; k < 500; ++k) veh.step(0.001, in);
    return (v0 - veh.state().vx) / 0.5;
}
}  // namespace

// #12: rolling resistance ~ C_rr*W shows up as extra coast-down deceleration,
// and a free-rolling wheel has kappa ~ 0 against the effective rolling radius.
TEST(CoreA_Tire, RollingResistanceAndEffectiveRadius) {
    const VehicleParams p = VehicleParams::default_fsae();
    const double extra = coast_decel(0.015) - coast_decel(0.0);
    EXPECT_NEAR(extra, 0.015 * 9.81, 0.25 * 0.015 * 9.81);

    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(15.0);
    DriverInput in;
    in.gear = 3;
    for (int k = 0; k < 500; ++k) veh.step(0.001, in);
    const double Fz = veh.get_tire_Fz()[0];
    const double Re = p.tire.radius - Fz / (3.0 * p.tire.k_tire);
    const double kappa = (veh.state().omega[0] * Re - veh.state().vx) / veh.state().vx;
    EXPECT_LT(std::abs(kappa), 2e-3);
}

// #20: one low-speed slip definition (virtual velocity) is continuous through rest.
TEST(CoreA_Tire, CommonSlipIsContinuousAtLowSpeed) {
    VehicleParams p = VehicleParams::default_fsae();
    Vehicle14DOF_New veh(p);
    double prev_kappa = 0.0, prev_alpha = 0.0;
    for (int k = 0; k <= 200; ++k) {
        const double vx = 0.01 * k;
        const double omega_r = 0.5 + 0.7 * vx;
        const auto s = veh.common_slip(vx, 0.1 + 0.2 * vx, omega_r);
        EXPECT_TRUE(std::isfinite(s.transport));
        EXPECT_TRUE(std::isfinite(s.kappa));
        EXPECT_TRUE(std::isfinite(s.alpha_rad));
        EXPECT_GE(s.transport, p.tire.v_N - 1e-12);
        if (k > 0) {
            EXPECT_LT(std::abs(s.kappa - prev_kappa), 0.05);
            EXPECT_LT(std::abs(s.alpha_rad - prev_alpha), 0.05);
        }
        prev_kappa = s.kappa;
        prev_alpha = s.alpha_rad;
    }
    const auto rest = veh.common_slip(0.0, 0.0, 0.0);
    EXPECT_NEAR(rest.kappa, 0.0, 1e-12);
    EXPECT_NEAR(rest.alpha_rad, 0.0, 1e-12);
}

// #21: the full contact force is rotated; a cambered vertical load feeds body x/y.
TEST(CoreA_Tire, CamberedVerticalLoadProjectsToBodyPlane) {
    VehicleParams zero = VehicleParams::default_fsae();
    zero.suspension.static_camber = {0.0, 0.0, 0.0, 0.0};
    VehicleParams cambered = zero;
    cambered.suspension.static_camber = {0.05, 0.05, 0.0, 0.0};

    Vehicle14DOF_New flat(zero), tilted(cambered);
    flat.reset();
    tilted.reset();
    DriverInput in;
    in.gear = 0;
    flat.step(0.001, in);
    tilted.step(0.001, in);

    double fy_flat = 0.0, fy_tilted = 0.0;
    for (int i = 0; i < 4; ++i) {
        fy_flat += flat.get_tire_Fy_body()[i];
        fy_tilted += tilted.get_tire_Fy_body()[i];
    }
    EXPECT_GT(std::abs(fy_tilted - fy_flat), 10.0);
}

// #2: strict artifact validation lists missing production groups.
TEST(CoreA_Params, StrictArtifactValidation) {
    const char* complete = R"({
        "brake": {}, "mass": {"Iw": 0.5},
        "tire": {"k_tire": 100000.0, "relaxation": {}},
        "suspension": {"bump_stop_gap": 0.03,
            "kc": {"front": {}, "rear": {}, "corners": {}}},
        "powertrain": {"engine_rpm_points": [1000, 6000],
                       "engine_torque_curve": [10, 20]}
    })";
    const auto ok = VehicleParams::from_json_string(complete, true);
    EXPECT_TRUE(ok.missing_required_groups().empty());

    const auto partial = VehicleParams::from_json_string(R"({"brake": {}})");
    EXPECT_EQ(partial.missing_required_groups().size(), 6u);
    EXPECT_THROW(VehicleParams::from_json_string(R"({"brake": {}})", true),
                 std::runtime_error);
}

// #11: per-corner damper dyno LUT replaces the Bouc-Wen force and parses from JSON.
TEST(CoreA_Damper, DynoLutIsPerCornerAndParsed) {
    VehicleParams p = VehicleParams::default_fsae();
    p.suspension.damper_lut_velocity = {-0.5, 0.0, 0.5};
    p.suspension.damper_lut_force[0] = {50.0, 100.0, 150.0};
    p.suspension.damper_lut_force[1] = {40.0, 80.0, 120.0};
    p.suspension.damper_lut_force[2] = {30.0, 60.0, 90.0};
    p.suspension.damper_lut_force[3] = {20.0, 40.0, 60.0};
    Vehicle14DOF_New veh(p);
    veh.step(1e-4, DriverInput{});
    const double mr = p.suspension.motion_ratio[0];
    EXPECT_NEAR(veh.get_bw_forces()[0], mr * 100.0, 0.1 * mr * 100.0);
    EXPECT_NEAR(veh.get_bw_forces()[1], p.suspension.motion_ratio[1] * 80.0,
                0.1 * p.suspension.motion_ratio[1] * 80.0);

    const auto parsed = VehicleParams::from_json_string(R"({
        "suspension": {"damper_lut": {
            "velocity_mps": [-0.5, 0.0, 0.5],
            "force_N": {"FL": [1, 2, 3], "RR": [4, 5, 6]}
        }}
    })");
    EXPECT_EQ(parsed.suspension.damper_lut_velocity.size(), 3u);
    EXPECT_EQ(parsed.suspension.damper_lut_force[0].size(), 3u);
    EXPECT_EQ(parsed.suspension.damper_lut_force[3].size(), 3u);
    EXPECT_EQ(parsed.suspension.damper_lut_force[1].size(), 0u);
}

// #7: angle-mode steering is clamped by steering.max_angle.
TEST(CoreA_Params, AngleModeClampsToMaxAngle) {
    VehicleParams p = VehicleParams::default_fsae();
    p.steering.torque_mode = false;
    p.steering.max_angle = 0.2;
    Vehicle14DOF_New limited(p), over(p);
    limited.set_initial_velocity(10.0);
    over.set_initial_velocity(10.0);
    DriverInput a, b;
    a.gear = 0;
    b.gear = 0;
    a.steering = 0.2;
    b.steering = 1.0;
    for (int k = 0; k < 100; ++k) {
        limited.step(0.001, a);
        over.step(0.001, b);
    }
    EXPECT_NEAR(limited.state().r, over.state().r, 1e-12);
}

// #7: load compliance moves the contact patch; caster compliance changes kingpin torque.
TEST(CoreA_Params, LoadComplianceIsConsumed) {
    auto run_track = [](double track_per_fz) {
        VehicleParams p = VehicleParams::default_fsae();
        p.suspension.kc_corners[0].compliance.track_per_load[2] = track_per_fz;
        Vehicle14DOF_New veh(p);
        veh.set_initial_velocity(15.0);
        DriverInput in;
        in.gear = 0;
        in.steering = 0.03;
        for (int k = 0; k < 200; ++k) veh.step(0.001, in);
        return veh.get_tire_Fx_body()[0];
    };
    EXPECT_GT(std::abs(run_track(1e-3) - run_track(0.0)), 1e-3);

    auto run_wheelbase = [](double wheelbase_per_fz) {
        VehicleParams p = VehicleParams::default_fsae();
        p.suspension.kc_corners[0].compliance.wheelbase_per_load[2] = wheelbase_per_fz;
        Vehicle14DOF_New veh(p);
        veh.set_initial_velocity(15.0);
        DriverInput in;
        in.gear = 0;
        in.steering = 0.03;
        for (int k = 0; k < 200; ++k) veh.step(0.001, in);
        return veh.get_tire_Fy_body()[0];
    };
    EXPECT_GT(std::abs(run_wheelbase(1e-3) - run_wheelbase(0.0)), 1e-3);

    auto run_caster = [](double caster_per_fz) {
        VehicleParams p = VehicleParams::default_fsae();
        p.suspension.kc_corners[0].mechanical_trail = 0.03;
        p.suspension.kc_corners[0].compliance.caster_per_load[2] = caster_per_fz;
        Vehicle14DOF_New veh(p);
        veh.set_initial_velocity(15.0);
        DriverInput in;
        in.gear = 0;
        in.steering = 0.03;
        for (int k = 0; k < 200; ++k) veh.step(0.001, in);
        return veh.get_kingpin_torque()[0];
    };
    EXPECT_GT(std::abs(run_caster(1e-3) - run_caster(0.0)), 1e-6);
}

// #8: dense K&C samples are parsed and take precedence over the quadratic surrogate.
TEST(CoreA_KC, DenseSamplesTakePrecedenceOverQuadratic) {
    VehicleParams p = VehicleParams::default_fsae();
    auto& kc = p.suspension.kc_corners[0];
    kc.valid = true;
    kc.kinematics.camber_vs_travel = {0.5, 0.0, 0.0};  // misleading surrogate
    kc.kinematics.toe_vs_travel = {0.5, 0.0, 0.0};
    kc.samples.travel_m.clear();
    kc.samples.camber_rad.clear();
    kc.samples.toe_rad.clear();
    for (int mm = -30; mm <= 30; mm += 5) {
        const double x = 0.001 * mm;
        kc.samples.travel_m.push_back(x);
        kc.samples.camber_rad.push_back(-0.02 + 0.5 * x + 40.0 * x * x);
        kc.samples.toe_rad.push_back(0.001 + 0.1 * x);
    }
    Vehicle14DOF_New veh(p);
    const std::array<double, 6> loads = {0, 0, 0, 0, 0, 0};
    EXPECT_NEAR(veh.compute_camber_angle(0, 0.01, 0.0, 0.0, loads),
                -0.02 + 0.005 + 40.0 * 1e-4, 1e-12);
    EXPECT_NEAR(veh.compute_toe_angle(0, 0.01, 0.0, 0.0, loads), 0.001 + 0.001, 1e-12);
    // Mid-knot interpolation stays within 0.05 deg of the generating quadratic.
    for (double x = -0.025; x <= 0.025; x += 0.001) {
        const double expected = -0.02 + 0.5 * x + 40.0 * x * x;
        EXPECT_NEAR(veh.compute_camber_angle(0, x, 0.0, 0.0, loads), expected, 8.7e-4);
    }
}

TEST(CoreA_KC, DenseSamplesJsonParsing) {
    const auto p = VehicleParams::from_json_string(R"({
        "suspension": {"kc": {"corners": {"FL": {"sample_values": {
            "travel_m": [-0.01, 0.0, 0.01],
            "camber_rad": [-0.1, 0.0, 0.1],
            "steer_rad": [-0.2, 0.2],
            "steer_camber_rad": [-0.01, 0.01]
        }}}}}
    })");
    EXPECT_EQ(p.suspension.kc_corners[0].samples.travel_m.size(), 3u);
    EXPECT_EQ(p.suspension.kc_corners[0].samples.camber_rad.size(), 3u);
    EXPECT_EQ(p.suspension.kc_corners[0].samples.steer_camber_rad.size(), 2u);
}

namespace {
State14DOF run_bumpy(const VehicleParams& p) {
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(12.0);
    DriverInput in;
    in.gear = 3;
    for (int k = 0; k < 1500; ++k) {
        in.steering = (k > 200 && k < 700) ? 0.06 : -0.03;
        in.brake = (k > 900 && k < 1100) ? 0.4 : 0.0;
        veh.step(0.001, in);
    }
    return veh.state();
}
}  // namespace

// #10: a constant MR(travel) table reproduces the constant-MR model.
TEST(CoreA_MotionRatio, ConstantTableMatchesConstantMR) {
    VehicleParams p = VehicleParams::default_fsae();
    const auto ref = run_bumpy(p);
    p.suspension.motion_ratio_table_travel = {-0.05, 0.0, 0.05};
    for (int i = 0; i < 4; ++i) {
        const double mr = p.suspension.motion_ratio[i];
        p.suspension.motion_ratio_table[i] = {mr, mr, mr};
    }
    const auto tab = run_bumpy(p);
    EXPECT_NEAR(tab.phi, ref.phi, 1e-9);
    EXPECT_NEAR(tab.r, ref.r, 1e-9);
    EXPECT_NEAR(tab.z, ref.z, 1e-9);
}

// #10: spring stroke integrates MR exactly; bump/rebound gaps and rates are separate.
TEST(CoreA_MotionRatio, RisingRateSpringAndAsymmetricBumpStop) {
    VehicleParams p = VehicleParams::default_fsae();
    p.suspension.motion_ratio_table_travel = {0.0, 0.1};
    for (int i = 0; i < 4; ++i) p.suspension.motion_ratio_table[i] = {0.8, 1.0};
    p.suspension.bump_stop_gap_rebound = 0.05;
    p.suspension.bump_stop_k_rebound = 20000.0;
    Vehicle14DOF_New veh(p);
    EXPECT_NEAR(veh.motion_ratio_at(0, 0.05), 0.9, 1e-12);
    EXPECT_NEAR(veh.spring_stroke(0, 0.1), 0.09, 1e-12);       // trapezoid 0.1*(0.8+1.0)/2
    EXPECT_NEAR(veh.spring_stroke(0, -0.01), -0.008, 1e-12);   // constant extrapolation
    EXPECT_NEAR(veh.compute_bumpstop_force(0.04, 0), 100000.0 * 0.01, 1e-9);
    EXPECT_NEAR(veh.compute_bumpstop_force(-0.04, 0), 0.0, 1e-12);
    EXPECT_NEAR(veh.compute_bumpstop_force(-0.06, 0), -20000.0 * 0.01, 1e-9);
}

// #10: report the effect of applying MR to dampers/LuGre/bump (default on).
TEST(CoreA_MotionRatio, ReportDamperMotionRatioEffect) {
    VehicleParams p = VehicleParams::default_fsae();
    const auto on = run_bumpy(p);
    p.suspension.mr_applies_to_dampers = false;
    const auto off = run_bumpy(p);
    std::cout << "[CORE-A] MR->dampers on: phi=" << on.phi << " r=" << on.r << " z=" << on.z
              << " | off: phi=" << off.phi << " r=" << off.r << " z=" << off.z << std::endl;
    EXPECT_NE(on.phi, off.phi);
}

// #14: road step input. Single wheel: initial unsprung acceleration = k_t*h/m_u.
// All four wheels: the whole car settles exactly h higher (quasi-static quarter-car solution).
TEST(CoreA_Road, QuarterCarStepResponse) {
    const VehicleParams p = VehicleParams::default_fsae();
    const double h = 0.01;  // road rises 10 mm -> z_road = -h (Z down)

    Vehicle14DOF_New one(p);
    const double z_u0 = one.state().z_u[0];
    one.set_road_input({-h, 0.0, 0.0, 0.0});
    one.step(1e-4, DriverInput{});
    const double a_u = one.state().vz_u[0] / 1e-4;
    EXPECT_NEAR(a_u, -p.tire.k_tire * h / p.mass.m_unsprung[0], 0.05 * p.tire.k_tire * h / p.mass.m_unsprung[0]);
    EXPECT_LT(one.state().z_u[0], z_u0);

    VehicleParams q = p;  // remove hysteresis/friction so the quasi-static solution is unique
    q.suspension.bw_alpha = 0.0;
    q.suspension.lugre_sigma0 = 0.0;  // LuGre reduces to viscous sigma1*v
    Vehicle14DOF_New all(q);
    const double z0 = all.state().z;
    const auto fz0 = all.get_tire_Fz();
    all.set_road_input({-h, -h, -h, -h});
    for (int k = 0; k < 4000; ++k) all.step(0.001, DriverInput{});
    EXPECT_NEAR(all.state().z - z0, -h, 0.02 * h);
    EXPECT_NEAR(all.get_tire_Fz()[0], fz0[0], 0.01 * fz0[0]);
}

// #15: starting at speed, the per-corner equilibrium already carries the aero load (no bounce).
TEST(CoreA_Equilibrium, NoStartupBounceAtSpeed) {
    const VehicleParams p = VehicleParams::default_fsae();
    Vehicle14DOF_New veh(p);
    veh.set_initial_velocity(15.0);
    DriverInput in;
    in.gear = 0;  // neutral: isolate equilibrium from engine braking (#34)
    double vz = 0.0, q = 0.0;
    for (int k = 0; k < 300; ++k) {
        veh.step(0.001, in);
        vz = std::max(vz, std::abs(veh.state().vz));
        q = std::max(q, std::abs(veh.state().q));
    }
    EXPECT_LT(vz, 0.02);  // was 0.048 m/s with the single-value equilibrium (now ~0.009)
    EXPECT_LT(q, 0.005);  // was 0.023 rad/s (now ~0.001)
}
