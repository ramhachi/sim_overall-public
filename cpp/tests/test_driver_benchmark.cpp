/// @file test_driver_benchmark.cpp
/// @brief Tests for vehicle capability benchmark and attack-driver profile contract.

#include <gtest/gtest.h>

#include <fsae/driver/driver_benchmark.hpp>

using namespace fsae;

TEST(DriverCapabilityProfileTest, JsonRoundTripPreservesAttackOverrides) {
    driver::DriverCapabilityProfile profile;
    profile.effective_mu_y = 1.35;
    profile.stable_corner_speed_mps = 10.0;
    profile.attack_overrides.attack_speed_lateral_safety = 0.64;
    profile.attack_overrides.attack_floor_complex = 0.82;
    profile.attack_overrides.attack_w_psi = 2.5;

    const auto json = driver::driver_capability_profile_to_json(profile);
    const auto parsed = driver::driver_capability_profile_from_json(json);

    EXPECT_DOUBLE_EQ(parsed.effective_mu_y, 1.35);
    EXPECT_DOUBLE_EQ(parsed.stable_corner_speed_mps, 10.0);
    EXPECT_DOUBLE_EQ(parsed.attack_overrides.attack_speed_lateral_safety, 0.64);
    EXPECT_DOUBLE_EQ(parsed.attack_overrides.attack_floor_complex, 0.82);
    EXPECT_DOUBLE_EQ(parsed.attack_overrides.attack_w_psi, 2.5);
}

TEST(DriverCapabilityProfileTest, AppliesAttackOverridesToMPCConfig) {
    driver::AttackDriverOverrides overrides;
    overrides.attack_speed_lateral_safety = 0.62;
    overrides.attack_speed_braking_safety = 0.77;
    overrides.attack_speed_accel_limit = 4.2;
    overrides.attack_floor_turn_in = 0.71;
    overrides.attack_w_psi = 2.7;
    overrides.attack_w_ax_rate = 0.0005;

    control::MPCConfig config;
    driver::apply_attack_overrides(overrides, config);

    EXPECT_DOUBLE_EQ(config.attack_speed_lateral_safety, 0.62);
    EXPECT_DOUBLE_EQ(config.attack_speed_braking_safety, 0.77);
    EXPECT_DOUBLE_EQ(config.attack_speed_accel_limit, 4.2);
    EXPECT_DOUBLE_EQ(config.attack_floor_turn_in, 0.71);
    EXPECT_DOUBLE_EQ(config.w_psi, 2.7);
    EXPECT_DOUBLE_EQ(config.w_ax_rate, 0.0005);
}

TEST(DriverCapabilityProfileTest, HealthyCapabilityDoesNotLowerAttackFloors) {
    driver::DriverCapabilityProfile profile;
    profile.effective_mu_y = 1.30;
    profile.effective_mu_x_accel = 0.80;
    profile.effective_mu_x_brake = 1.35;
    profile.slalom_stable_speed_mps = 11.0;
    profile.yaw_response_time_s = 0.14;
    profile.max_steering_lag_ratio = 0.15;

    const auto overrides = driver::make_attack_overrides_from_capability(profile);

    EXPECT_TRUE(std::isnan(overrides.attack_speed_lateral_safety));
    EXPECT_TRUE(std::isnan(overrides.attack_speed_braking_safety));
    EXPECT_TRUE(std::isnan(overrides.attack_speed_accel_limit));
    EXPECT_TRUE(std::isnan(overrides.attack_floor_complex));
    EXPECT_TRUE(std::isnan(overrides.attack_floor_turn_in));
    EXPECT_TRUE(std::isnan(overrides.attack_floor_apex));
}

TEST(DriverCapabilityProfileTest, WeakSlalomCapabilityAddsConservativeAttackFloors) {
    driver::DriverCapabilityProfile profile;
    profile.effective_mu_y = 1.00;
    profile.effective_mu_x_accel = 0.55;
    profile.effective_mu_x_brake = 1.00;
    profile.slalom_stable_speed_mps = 7.0;
    profile.yaw_response_time_s = 0.42;
    profile.max_steering_lag_ratio = 0.30;

    const auto overrides = driver::make_attack_overrides_from_capability(profile);

    EXPECT_TRUE(std::isfinite(overrides.attack_floor_complex));
    EXPECT_TRUE(std::isfinite(overrides.attack_floor_turn_in));
    EXPECT_TRUE(std::isfinite(overrides.attack_speed_lateral_safety));
    EXPECT_TRUE(std::isfinite(overrides.attack_speed_braking_safety));
    EXPECT_LT(overrides.attack_floor_complex, 0.86);
    EXPECT_LT(overrides.attack_floor_turn_in, 0.84);
}

TEST(DriverBenchmarkTest, QuickBenchmarkProducesFiniteCapabilityProfile) {
    auto params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.I_steer = 0.02;
    params.suspension.K_chassis = 2000.0;

    driver::DriverBenchmarkConfig config;
    config.quick = true;
    config.dt = 0.004;

    const auto profile = driver::run_driver_capability_benchmarks(params, config);

    EXPECT_GT(profile.effective_mu_y, 0.1);
    EXPECT_GT(profile.stable_corner_speed_mps, 0.0);
    EXPECT_GT(profile.effective_mu_x_brake, 0.1);
    EXPECT_TRUE(std::isfinite(profile.yaw_response_time_s));
    EXPECT_TRUE(std::isfinite(profile.steering_response_time_s));
    EXPECT_GT(profile.slalom_stable_speed_mps, 0.0);
}
