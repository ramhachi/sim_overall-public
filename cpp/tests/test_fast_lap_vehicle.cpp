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
