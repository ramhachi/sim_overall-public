/// @file test_fast_lap_vehicle.cpp
/// @brief Reduced-order lap evaluator tests.

#include <gtest/gtest.h>

#include "fsae/driver/state_frame.hpp"
#include "fsae/vehicle/fast_lap_vehicle.hpp"
#include "fsae/vehicle/vehicle_params.hpp"

#include <cmath>

using namespace fsae::vehicle;

TEST(FastLapVehicleTest, PreservesMapPoseContract) {
    VehicleParams params = VehicleParams::default_fsae();
    FastLapVehicle vehicle(params);

    vehicle.reset();
    vehicle.set_initial_velocity(12.0);
    vehicle.set_map_pose(18.0, -4.5, 0.35);

    const auto& state = vehicle.state();
    EXPECT_NEAR(state.y, 18.0, 1e-12);
    EXPECT_NEAR(state.x, -4.5, 1e-12);
    EXPECT_NEAR(state.psi, M_PI / 2.0 - 0.35, 1e-12);
    EXPECT_NEAR(state.vx, 12.0, 1e-12);

    const auto map_state = fsae::driver::to_map_driver_state(state);
    EXPECT_NEAR(map_state.x, 18.0, 1e-12);
    EXPECT_NEAR(map_state.y, -4.5, 1e-12);
    EXPECT_NEAR(map_state.psi, 0.35, 1e-12);
}

TEST(FastLapVehicleTest, AcceleratesAndStaysFinite) {
    VehicleParams params = VehicleParams::default_fsae();
    FastLapVehicle vehicle(params);

    vehicle.reset();
    vehicle.set_map_pose(0.0, 0.0, 0.0);
    vehicle.set_initial_velocity(8.0);

    DriverInput input;
    input.throttle = 0.7;
    input.gear = 3;

    for (int i = 0; i < 80; ++i) {
        vehicle.step_adaptive(0.01, input);
    }

    const auto& state = vehicle.state();
    EXPECT_TRUE(std::isfinite(state.x));
    EXPECT_TRUE(std::isfinite(state.y));
    EXPECT_TRUE(std::isfinite(state.psi));
    EXPECT_TRUE(std::isfinite(state.vx));
    EXPECT_GT(state.speed(), 8.0);
    EXPECT_EQ(vehicle.active_tire_runtime_description(), "fast_6dof_tmeasy_v1");
}

TEST(FastLapVehicleTest, RightTurnProducesNegativeMapYawRate) {
    VehicleParams params = VehicleParams::default_fsae();
    FastLapVehicle vehicle(params);

    vehicle.reset();
    vehicle.set_map_pose(0.0, 0.0, 0.0);
    vehicle.set_initial_velocity(12.0);

    DriverInput input;
    input.steering = 0.04;
    input.throttle = 0.2;

    for (int i = 0; i < 40; ++i) {
        vehicle.step_adaptive(0.01, input);
    }

    const auto map_state = fsae::driver::to_map_driver_state(vehicle.state());
    EXPECT_LT(map_state.r, 0.0);
    EXPECT_GT(vehicle.state().r, 0.0);
}

TEST(FastLapVehicleTest, AngleModeKeepsLegacySteeringWheelContract) {
    VehicleParams params = VehicleParams::default_fsae();
    FastLapVehicle vehicle(params);

    vehicle.reset();
    vehicle.set_map_pose(0.0, 0.0, 0.0);
    vehicle.set_initial_velocity(10.0);

    DriverInput input;
    input.steering = 0.08;
    input.throttle = 0.1;

    for (int i = 0; i < 20; ++i) {
        vehicle.step_adaptive(0.01, input);
    }

    EXPECT_DOUBLE_EQ(vehicle.state().delta_sw, 0.0);
    EXPECT_DOUBLE_EQ(vehicle.state().ddelta_sw, 0.0);
}

// Issue #30: semi-implicit tire deflection keeps dt = 10 ms stable where v_c·dt/L is large
// (explicit RK4 stability limit ≈ 2.8) and from near rest.
TEST(FastLapVehicleTest, TireDeflectionStableAtTenMsHighAndLowSpeed) {
    VehicleParams params = VehicleParams::default_fsae();
    for (double v0 : {0.3, 2.0, 32.0}) {
        FastLapVehicle vehicle(params);
        vehicle.reset();
        vehicle.set_map_pose(0.0, 0.0, 0.0);
        vehicle.set_initial_velocity(v0);
        DriverInput input;
        input.steering = 0.02;
        input.throttle = 0.4;
        input.gear = 2;
        for (int i = 0; i < 300; ++i) vehicle.step_adaptive(0.01, input);
        const auto& s = vehicle.state();
        EXPECT_TRUE(std::isfinite(s.vx) && std::isfinite(s.r)) << "v0=" << v0;
        for (int w = 0; w < 4; ++w) {
            EXPECT_LT(std::abs(s.tire_xi[w]), 0.2) << "v0=" << v0 << " wheel " << w;
            EXPECT_LT(std::abs(s.tire_eta[w]), 0.2) << "v0=" << v0 << " wheel " << w;
        }
        EXPECT_GT(s.vx, v0 * 0.5) << "v0=" << v0;
    }
}

// Issue #30: the surrogate uses the same AeroMap4D as the 14DOF (direct_load mode honoured).
TEST(FastLapVehicleTest, UsesAeroMap4D) {
    auto coast = [](VehicleParams params) {
        FastLapVehicle vehicle(params);
        vehicle.reset();
        vehicle.set_map_pose(0.0, 0.0, 0.0);
        vehicle.set_initial_velocity(25.0);
        DriverInput input;
        for (int i = 0; i < 100; ++i) vehicle.step_adaptive(0.01, input);
        return vehicle.state().vx;
    };
    VehicleParams coeff = VehicleParams::default_fsae();
    VehicleParams direct = coeff;
    direct.aero.map_mode = "direct_load";
    direct.aero.Fx_ref_total = -10.0 * 0.5 * coeff.aero.rho * coeff.aero.CD_A
                             * coeff.aero.reference_speed * coeff.aero.reference_speed;
    EXPECT_LT(coast(direct), coast(coeff) - 0.5);  // 10x drag map must slow the coast-down
}
