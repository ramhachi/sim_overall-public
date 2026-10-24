/// @file test_rl_control_safety.cpp
/// @brief RL control safety helper tests.

#include <gtest/gtest.h>

#include "fsae/driver/gear_selection.hpp"
#include "fsae/driver/rl_control_safety.hpp"

#include <cstdlib>
#include <string>

namespace {

void restore_env_var(const char* name, bool had_value, const std::string& value) {
#if defined(_WIN32)
    if (had_value) {
        _putenv_s(name, value.c_str());
    } else {
        _putenv_s(name, "");
    }
#else
    if (had_value) {
        setenv(name, value.c_str(), 1);
    } else {
        unsetenv(name);
    }
#endif
}

void clear_env_var(const char* name) {
#if defined(_WIN32)
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

class ScopedEnvVar {
public:
    explicit ScopedEnvVar(const char* name) : name_(name) {
        const char* current = std::getenv(name_);
        if (current != nullptr) {
            had_value_ = true;
            value_ = current;
        }
    }

    ~ScopedEnvVar() {
        restore_env_var(name_, had_value_, value_);
    }

private:
    const char* name_;
    bool had_value_ = false;
    std::string value_;
};

}  // namespace

TEST(RLControlSafetyTest, SpeedLimitsEnabledByDefault) {
    ScopedEnvVar env_guard("FOPT_DISABLE_RL_DRIVER_SPEED_LIMITS");
    clear_env_var("FOPT_DISABLE_RL_DRIVER_SPEED_LIMITS");

    EXPECT_FALSE(fsae::driver::rl_driver_speed_limits_disabled());
}

TEST(RLControlSafetyTest, SteeringSlewLimitSuppressesHardReversal) {
    const double filtered = fsae::driver::smooth_rl_steering_command(-20.0, 10.0, 25.0);

    EXPECT_GT(filtered, 0.0);
    EXPECT_LT(filtered, 10.0);
}

TEST(RLControlSafetyTest, SteeringSlewLimitAllowsSameDirectionRamp) {
    const double filtered = fsae::driver::smooth_rl_steering_command(25.0, 0.0, 25.0);

    EXPECT_GT(filtered, 0.0);
    EXPECT_LT(filtered, 25.0);
}

TEST(RLControlSafetyTest, TorqueModeDoesNotAddHiddenSlewLimitByDefault) {
    const double filtered = fsae::driver::smooth_rl_steering_command(
        25.0, 0.0, 25.0, true);

    EXPECT_DOUBLE_EQ(filtered, 25.0);
}

TEST(RLControlSafetyTest, TorqueModeUsesExplicitSlewLimitWhenConfigured) {
    const double filtered = fsae::driver::smooth_rl_steering_command(
        25.0, 0.0, 25.0, true, 0.020, 80.0);

    EXPECT_NEAR(filtered, 1.6, 1e-12);
}

TEST(DriverParameterLinkageTest, GearSelectionUsesVehicleRatiosAndTireRadius) {
    auto params = fsae::vehicle::VehicleParams::default_fsae();
    params.tire.radius = 0.24;
    params.powertrain.primary_ratio = 2.0;
    params.powertrain.final_ratio = 4.0;
    params.powertrain.engine_rpm_points = {1000.0, 6000.0, 12000.0};
    params.powertrain.gear_ratios = {3.0, 2.0, 1.0};

    EXPECT_EQ(fsae::driver::select_gear_from_vehicle_params(2.0, params), 1);
    EXPECT_EQ(fsae::driver::select_gear_from_vehicle_params(12.0, params), 2);
    EXPECT_EQ(fsae::driver::select_gear_from_vehicle_params(24.0, params), 3);

    params.tire.radius = 0.48;
    EXPECT_EQ(fsae::driver::select_gear_from_vehicle_params(24.0, params), 2);
}

TEST(RLControlSafetyTest, CurvatureLimitUsesTireMuForBrakeDemand) {
    fsae::track::CurvatureBasedTrack track(
        std::vector<std::pair<double, double>>{{0.0, 0.05}, {100.0, 0.05}});

    fsae::track::FrenetConverter high_mu_converter(track);
    fsae::track::FrenetConverter low_mu_converter(track);

    fsae::vehicle::State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 25.0;

    auto high_mu_params = fsae::vehicle::VehicleParams::default_fsae();
    high_mu_params.tire.mu_x_max = 2.0;
    high_mu_params.tire.mu_y_max = 2.0;

    auto low_mu_params = high_mu_params;
    low_mu_params.tire.mu_x_max = 0.7;
    low_mu_params.tire.mu_y_max = 0.7;

    fsae::vehicle::DriverInput high_mu_input;
    high_mu_input.throttle = 1.0;
    high_mu_input.brake = 0.0;

    fsae::vehicle::DriverInput low_mu_input = high_mu_input;

    fsae::driver::apply_rl_curvature_speed_limit(
        state, track, high_mu_converter, high_mu_input, high_mu_params);
    fsae::driver::apply_rl_curvature_speed_limit(
        state, track, low_mu_converter, low_mu_input, low_mu_params);

    EXPECT_GT(low_mu_input.brake, high_mu_input.brake);
}
