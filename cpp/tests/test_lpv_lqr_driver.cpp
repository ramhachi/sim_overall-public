#include <gtest/gtest.h>

#include "fsae/driver/lpv_lqr_driver.hpp"
#include "fsae/track/track_model.hpp"

#include <cmath>

namespace {

using fsae::driver::PreviewLPVLQRDriver;
using fsae::laptime::VelocityPoint;
using fsae::vehicle::VehicleParams;
using fsae::vehicle::VehicleState;

std::vector<VelocityPoint> constant_profile(double length, double speed) {
    return {
        {0.0, speed, 0.0, 0.0, 0.0},
        {length, speed, 0.0, 0.0, 0.0},
    };
}

TEST(LPVLQRDriver, GainTableIsFiniteAndStableAcrossSpeedSchedule) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 12.0));

    EXPECT_GT(driver.gain_table_size(), 10u);
    EXPECT_EQ(driver.gain_table_size(), driver.actuator_gain_table_size());
    for (double speed = 3.0; speed <= 40.0; speed += 0.5) {
        EXPECT_TRUE(driver.gain_at_speed(speed).allFinite());
        EXPECT_TRUE(driver.actuator_gain_at_speed(speed).allFinite());
        EXPECT_LT(driver.closed_loop_spectral_radius(speed), 1.0)
            << "speed=" << speed;
        EXPECT_LT(driver.actuator_closed_loop_spectral_radius(speed), 1.0)
            << "actuator speed=" << speed;
    }
}

TEST(LPVLQRDriver, SteadyStateFeedforwardUsesDriverSignConvention) {
    const auto track = fsae::track::create_circle_track(30.0, 4.0);
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0));

    const auto left = driver.steady_state_reference(10.0, 1.0 / 30.0);
    const auto right = driver.steady_state_reference(10.0, -1.0 / 30.0);
    EXPECT_GT(left.steering, 0.0);   // internal SAE convention: left positive
    EXPECT_LT(right.steering, 0.0);
    EXPECT_NEAR(left.steering, -right.steering, 1e-10);
}

TEST(LPVLQRDriver, LeftLateralErrorCommandsRightSteering) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0));
    VehicleState state;
    state.x = 10.0;
    state.y = 0.5;  // map left of center
    state.psi = 0.0;
    state.vx = 10.0;

    const auto input = driver.compute_outer_control(state, 0.0);
    EXPECT_GT(input.steering, 0.0);  // driver convention: right positive
    EXPECT_TRUE(std::isfinite(input.throttle));
    EXPECT_TRUE(std::isfinite(input.brake));
}

TEST(LPVLQRDriver, VehicleParameterChangesRegenerateTheSchedule) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    const auto profile = constant_profile(track.length(), 12.0);
    VehicleParams nominal = VehicleParams::default_fsae();
    VehicleParams heavy = nominal;
    heavy.mass.m_total *= 1.20;
    heavy.mass.m_sprung *= 1.20;
    heavy.mass.Izz *= 1.20;

    PreviewLPVLQRDriver nominal_driver(track, profile, {}, nominal);
    PreviewLPVLQRDriver heavy_driver(track, profile, {}, heavy);
    EXPECT_GT((nominal_driver.gain_at_speed(15.0)
             - heavy_driver.gain_at_speed(15.0)).norm(), 1e-5);
}

TEST(LPVLQRDriver, TorqueModeKeepsAngleTargetOutsideInnerLoop) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0),
        {},
        params);
    VehicleState state;
    state.x = 10.0;
    state.y = 0.5;
    state.vx = 10.0;

    auto input = driver.compute_outer_control(state, 0.0);
    EXPECT_DOUBLE_EQ(input.steering, 0.0);
    input = driver.update_torque_actuator_control(input, 0.0, 0.0, 0.001, 0.0);
    EXPECT_GT(input.steering, 0.0);
    EXPECT_LE(input.steering, params.steering.T_driver_max);
}

TEST(LPVLQRDriver, ActuatorStateFeedbackOpposesMeasuredSteeringError) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0),
        {},
        params);
    VehicleState state;
    state.x = 10.0;
    state.vx = 10.0;

    // The measured wheel is steered right while a straight target requires
    // zero. The augmented controller must command left to unwind it.
    driver.set_steering_state(0.10 * params.steering.R_steer, 0.0);
    driver.compute_outer_control(state, 0.0);
    EXPECT_LT(driver.last_debug().target_tire_angle, 0.0);
}

TEST(LPVLQRDriver, RawLqrRequestRespectsPhysicalRackAngle) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.max_angle = 0.31;
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0),
        {},
        params);
    VehicleState state;
    state.x = 10.0;
    state.y = 5.0;
    state.vx = 10.0;

    driver.compute_outer_control(state, 0.0);
    EXPECT_LE(std::abs(driver.last_debug().raw_target_tire_angle), 0.31 + 1e-12);
}

TEST(LPVLQRDriver, SteeringBandwidthChangesAugmentedGainSchedule) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    const auto profile = constant_profile(track.length(), 12.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    fsae::driver::LPVLQRConfig fast_config;
    fsae::driver::LPVLQRConfig slow_config;
    fast_config.steering_inner_loop_omega_n = 30.0;
    slow_config.steering_inner_loop_omega_n = 15.0;

    PreviewLPVLQRDriver fast(track, profile, fast_config, params);
    PreviewLPVLQRDriver slow(track, profile, slow_config, params);
    EXPECT_GT((fast.actuator_gain_at_speed(15.0)
             - slow.actuator_gain_at_speed(15.0)).norm(), 1e-4);
}

TEST(LPVLQRDriver, SteeringPredictionUsesDriverSignAndPreservesEquilibrium) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0));

    const auto equilibrium = driver.predict_steering_state(0.08, 0.0, 0.08, 0.025);
    EXPECT_NEAR(equilibrium.first, 0.08, 1e-12);
    EXPECT_NEAR(equilibrium.second, 0.0, 1e-12);

    const auto right_step = driver.predict_steering_state(0.0, 0.0, 0.10, 0.025);
    EXPECT_GT(right_step.first, 0.0);
    EXPECT_GT(right_step.second, 0.0);
}

TEST(LPVLQRDriver, OneStepPredictionIsComparedAtTheNextOuterTick) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0),
        {},
        params);
    VehicleState state;
    state.x = 10.0;
    state.vx = 10.0;

    driver.set_steering_state(0.0, 0.0);
    driver.compute_outer_control(state, 0.0);
    const auto first_debug = driver.last_debug();
    ASSERT_TRUE(first_debug.steering_prediction_valid);
    EXPECT_FALSE(first_debug.steering_prediction_error_valid);

    driver.set_steering_state(
        first_debug.predicted_tire_angle * params.steering.R_steer,
        first_debug.predicted_tire_rate * params.steering.R_steer);
    driver.compute_outer_control(state, 0.025);
    EXPECT_TRUE(driver.last_debug().steering_prediction_error_valid);
    EXPECT_EQ(driver.stats().actuator_prediction_count, 1);
    EXPECT_NEAR(driver.stats().actuator_prediction_rmse(), 0.0, 1e-12);
}

TEST(LPVLQRDriver, NonFiniteSteeringMeasurementTriggersFiniteBrakeFallback) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    auto params = VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    PreviewLPVLQRDriver driver(
        track,
        constant_profile(track.length(), 10.0),
        {},
        params);
    fsae::vehicle::DriverInput input;
    input.throttle = 0.7;
    input = driver.update_torque_actuator_control(
        input,
        std::numeric_limits<double>::quiet_NaN(),
        0.0,
        0.001,
        0.0);
    EXPECT_TRUE(std::isfinite(input.steering));
    EXPECT_DOUBLE_EQ(input.steering, 0.0);
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    EXPECT_GE(input.brake, 0.35);
    EXPECT_EQ(driver.stats().fail_count, 1);
}

TEST(LPVLQRDriver, SpeedPreviewLeavesBrakingDistanceToSlowPoint) {
    const auto track = fsae::track::create_acceleration_track(100.0);
    const std::vector<VelocityPoint> profile = {
        {0.0, 15.0, 0.0, 0.0, 0.0},
        {30.0, 15.0, 0.0, 0.0, 0.0},
        {31.0, 5.0, 0.0, 0.0, 0.0},
        {track.length(), 5.0, 0.0, 0.0, 0.0},
    };
    PreviewLPVLQRDriver driver(track, profile);
    VehicleState state;
    state.x = 20.0;
    state.vx = 10.0;
    driver.compute_outer_control(state, 0.0);
    // The 5 m/s point is ~11 m ahead: sqrt(5^2 + 2*4*11) ~ 10.7 m/s, not 5.
    const double target = driver.last_debug().target_speed;
    EXPECT_GT(target, 9.5);
    EXPECT_LT(target, 15.0);
}

TEST(LPVLQRDriver, CurvatureCapFollowsVehicleGgvLateralLimit) {
    const auto track = fsae::track::create_circle_track(20.0, 6.0);
    auto ggv = std::make_shared<fsae::laptime::GGVMap>();
    ggv->v = {0.0, 40.0};
    ggv->ay_max = {10.0, 10.0};
    ggv->r = {0.0, 1.0};
    ggv->ax_max = {{5.0, 0.0}, {5.0, 0.0}};
    ggv->ax_min = {{10.0, 0.0}, {10.0, 0.0}};
    fsae::driver::LPVLQRConfig config;
    config.ggv = ggv;
    PreviewLPVLQRDriver driver(track, constant_profile(track.length(), 30.0), config);
    const auto start = track.get_point(10.0);
    VehicleState state;
    state.x = start.x;
    state.y = start.y;
    state.psi = start.theta;
    state.vx = 12.0;
    driver.compute_outer_control(state, 0.0);
    // Constant curvature 1/20: the cap is the GGV corner speed sqrt(10 * 20).
    EXPECT_NEAR(driver.last_debug().steering_speed_cap, std::sqrt(200.0), 0.05);
}

TEST(AnalyticSafetyFilter, ProjectsRateAndRecoversTowardCenter) {
    fsae::driver::AnalyticSafetyConfig config;
    config.max_tire_angle = 0.35;
    config.max_tire_rate = 1.0;
    fsae::driver::AnalyticSafetyFilter filter(config);
    const auto safe = filter.apply(
        {-0.30, 12.0, 12.0},
        {1.0, 0.20, 0.20, 10.0, 0.0, 0.0},
        0.025);

    EXPECT_TRUE(safe.track_recovery_active);
    EXPECT_GT(safe.target_tire_angle, 0.0); // left error -> right recovery
    EXPECT_LE(safe.target_tire_angle, 0.025 + 1e-12);
    EXPECT_LT(safe.target_speed, 12.0);
}

TEST(AnalyticSafetyFilter, NonFiniteRequestUsesDeterministicFallback) {
    fsae::driver::AnalyticSafetyFilter filter({});
    const auto safe = filter.apply(
        {std::numeric_limits<double>::quiet_NaN(), 12.0, 12.0},
        {0.0, 0.0, 2.0, 10.0, 0.0, 0.0},
        0.025);
    EXPECT_TRUE(safe.finite_fallback_active);
    EXPECT_TRUE(std::isfinite(safe.target_tire_angle));
    EXPECT_DOUBLE_EQ(safe.target_speed, 1.0);
}

TEST(AnalyticSafetyFilter, StabilityEnvelopeCapsSpeed) {
    fsae::driver::AnalyticSafetyFilter filter({});
    const auto safe = filter.apply(
        {0.0, 15.0, 15.0},
        {0.0, 0.0, 2.0, 12.0, 0.20, 0.0},
        0.025);
    EXPECT_DOUBLE_EQ(safe.target_speed, 8.0);
    EXPECT_DOUBLE_EQ(safe.stability_speed_cap, 8.0);
}

} // namespace
