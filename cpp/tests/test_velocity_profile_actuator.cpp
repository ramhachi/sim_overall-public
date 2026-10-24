/// @file test_velocity_profile_actuator.cpp
/// @brief Steering-actuator-aware velocity profile tests.

#include <gtest/gtest.h>

#include "fsae/laptime/velocity_profile.hpp"
#include "fsae/track/track_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace fsae;

namespace {

laptime::VehiclePerformance fast_profile_perf() {
    laptime::VehiclePerformance perf;
    perf.max_speed = 35.0;
    perf.min_speed = 2.0;
    perf.safety_factor = 1.0;
    perf.mu_lat = 3.0;
    perf.mu_lon = 3.0;
    perf.cl_a = 0.0;
    perf.cd_a = 0.0;
    perf.max_lon_accel = 8.0 * 9.81;
    perf.max_lon_decel = 8.0 * 9.81;
    return perf;
}

double speed_at(const std::vector<laptime::VelocityPoint>& profile, double s) {
    if (profile.empty()) {
        return 0.0;
    }
    for (size_t i = 0; i + 1 < profile.size(); ++i) {
        const auto& p0 = profile[i];
        const auto& p1 = profile[i + 1];
        if (s >= p0.s && s <= p1.s) {
            const double ds = p1.s - p0.s;
            const double alpha = ds > 1e-9 ? (s - p0.s) / ds : 0.0;
            return p0.v + alpha * (p1.v - p0.v);
        }
    }
    return profile.back().v;
}

double steering_cap_at(const std::vector<laptime::VelocityPoint>& profile, double s) {
    if (profile.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const auto nearest = std::min_element(
        profile.begin(),
        profile.end(),
        [s](const auto& lhs, const auto& rhs) {
            return std::abs(lhs.s - s) < std::abs(rhs.s - s);
        });
    return nearest->steering_speed_cap;
}

} // namespace

TEST(VelocityProfileActuatorTest, CurvatureStepIsCappedBeforeCornerEntry) {
    const auto track = track::create_straight_corner_track(30.0, 20.0, 4.5);

    auto open_perf = fast_profile_perf();
    laptime::VelocityProfileGenerator open_generator(open_perf);
    const auto open_profile = open_generator.generate(track, 1.0);

    auto limited_perf = fast_profile_perf();
    limited_perf.use_steering_actuator_limits = true;
    limited_perf.wheelbase = 1.55;
    limited_perf.max_tire_steer_rate = 0.25;
    limited_perf.steering_preview_distance = 12.0;
    limited_perf.steering_deadband = 0.0;
    limited_perf.steering_speed_safety_factor = 1.0;
    laptime::VelocityProfileGenerator limited_generator(limited_perf);
    const auto limited_profile = limited_generator.generate(track, 1.0);

    const double open_entry_speed = speed_at(open_profile, 25.0);
    const double limited_entry_speed = speed_at(limited_profile, 25.0);

    EXPECT_LT(limited_entry_speed, open_entry_speed - 5.0);
    EXPECT_LT(limited_entry_speed, 20.0);
}

TEST(VelocityProfileActuatorTest, ConstantCurvatureIsNotArtificiallyReduced) {
    const auto track = track::create_circle_track(20.0, 4.5);

    auto open_perf = fast_profile_perf();
    open_perf.max_speed = 22.0;
    laptime::VelocityProfileGenerator open_generator(open_perf);
    const auto open_profile = open_generator.generate(track, 1.0);

    auto limited_perf = open_perf;
    limited_perf.use_steering_actuator_limits = true;
    limited_perf.max_tire_steer_rate = 0.25;
    limited_perf.steering_preview_distance = 12.0;
    limited_perf.steering_deadband = 0.0;
    laptime::VelocityProfileGenerator limited_generator(limited_perf);
    const auto limited_profile = limited_generator.generate(track, 1.0);

    ASSERT_EQ(limited_profile.size(), open_profile.size());
    for (size_t i = 0; i < open_profile.size(); ++i) {
        EXPECT_NEAR(limited_profile[i].v, open_profile[i].v, 1e-9);
    }
}

TEST(VelocityProfileActuatorTest, CurvatureOverrideDrivesCornerSpeedLimit) {
    const auto track = track::create_circle_track(10.0, 4.5);

    auto centerline_perf = fast_profile_perf();
    centerline_perf.mu_lat = 1.0;
    centerline_perf.cl_a = 0.0;
    laptime::VelocityProfileGenerator centerline_generator(centerline_perf);
    const auto centerline_profile = centerline_generator.generate(track, 1.0);

    auto override_perf = centerline_perf;
    for (double s = 0.0; s <= track.length() + 1e-9; s += 1.0) {
        override_perf.curvature_override.emplace_back(std::min(s, track.length()), 0.0);
    }
    laptime::VelocityProfileGenerator override_generator(override_perf);
    const auto override_profile = override_generator.generate(track, 1.0);

    EXPECT_GT(speed_at(override_profile, 20.0), speed_at(centerline_profile, 20.0) + 10.0);
    EXPECT_NEAR(override_profile.front().kappa, 0.0, 1e-12);
}

TEST(VelocityProfileActuatorTest, LowSteeringTorqueAuthorityTightensSpeedCap) {
    const auto track = track::create_straight_corner_track(30.0, 20.0, 4.5);

    auto nominal_perf = fast_profile_perf();
    nominal_perf.use_steering_actuator_limits = true;
    nominal_perf.max_tire_steer_rate = 0.35;
    nominal_perf.steering_preview_distance = 12.0;
    nominal_perf.steering_deadband = 0.0;
    nominal_perf.steering_speed_safety_factor = 1.0;
    laptime::VelocityProfileGenerator nominal_generator(nominal_perf);
    const auto nominal_profile = nominal_generator.generate(track, 1.0);

    auto weak_perf = nominal_perf;
    weak_perf.steering_inertia = 0.04;
    weak_perf.steering_driver_torque_max = 10.0;
    laptime::VelocityProfileGenerator weak_generator(weak_perf);
    const auto weak_profile = weak_generator.generate(track, 1.0);

    EXPECT_LT(speed_at(weak_profile, 25.0), speed_at(nominal_profile, 25.0) - 2.0);
}

TEST(VelocityProfileActuatorTest, StatefulSteeringLagCapsGradualCurvatureRamp) {
    const track::CurvatureBasedTrack ramp_track({
        {0.0, 0.0},
        {30.0, 0.08},
        {60.0, 0.08},
    }, 0.0, 0.0, 0.0, 4.5);

    auto open_perf = fast_profile_perf();
    laptime::VelocityProfileGenerator open_generator(open_perf);
    const auto open_profile = open_generator.generate(ramp_track, 1.0);

    auto limited_perf = fast_profile_perf();
    limited_perf.use_steering_actuator_limits = true;
    limited_perf.max_tire_steer_rate = 0.04;
    limited_perf.steering_preview_distance = 12.0;
    limited_perf.steering_deadband = 0.0;
    limited_perf.steering_speed_safety_factor = 1.0;
    laptime::VelocityProfileGenerator limited_generator(limited_perf);
    const auto limited_profile = limited_generator.generate(ramp_track, 1.0);

    EXPECT_LT(speed_at(limited_profile, 25.0), speed_at(open_profile, 25.0) - 4.0)
        << "Accumulated steering lag through a curvature ramp should cap entry speed";
}

TEST(VelocityProfileActuatorTest, LowerSteeringBandwidthTightensCornerEntrySpeedCap) {
    const auto track = track::create_straight_corner_track(30.0, 20.0, 4.5);

    auto fast_response_perf = fast_profile_perf();
    fast_response_perf.use_steering_actuator_limits = true;
    fast_response_perf.max_tire_steer_rate = 5.0;
    fast_response_perf.steering_preview_distance = 12.0;
    fast_response_perf.steering_deadband = 0.0;
    fast_response_perf.steering_speed_safety_factor = 1.0;
    fast_response_perf.steering_response_omega_n = 80.0;
    laptime::VelocityProfileGenerator fast_response_generator(fast_response_perf);
    const auto fast_response_profile = fast_response_generator.generate(track, 1.0);

    auto slow_response_perf = fast_response_perf;
    slow_response_perf.steering_response_omega_n = 10.0;
    laptime::VelocityProfileGenerator slow_response_generator(slow_response_perf);
    const auto slow_response_profile = slow_response_generator.generate(track, 1.0);

    EXPECT_LT(speed_at(slow_response_profile, 25.0), speed_at(fast_response_profile, 25.0) - 2.0);
}

TEST(VelocityProfileActuatorTest, UndersteerDemandTightensHighSpeedCornerEntryCap) {
    const auto track = track::create_straight_corner_track(30.0, 20.0, 4.5);

    auto neutral_perf = fast_profile_perf();
    neutral_perf.use_steering_actuator_limits = true;
    neutral_perf.max_tire_steer_rate = 0.60;
    neutral_perf.steering_preview_distance = 12.0;
    neutral_perf.steering_deadband = 0.0;
    neutral_perf.steering_speed_safety_factor = 1.0;
    neutral_perf.steering_understeer_gradient = 0.0;
    laptime::VelocityProfileGenerator neutral_generator(neutral_perf);
    const auto neutral_profile = neutral_generator.generate(track, 1.0);

    auto understeer_perf = neutral_perf;
    understeer_perf.steering_understeer_gradient = 0.01;
    laptime::VelocityProfileGenerator understeer_generator(understeer_perf);
    const auto understeer_profile = understeer_generator.generate(track, 1.0);

    EXPECT_LT(speed_at(understeer_profile, 25.0), speed_at(neutral_profile, 25.0) - 1.0);
}

TEST(VelocityProfileActuatorTest, TimeBasedPreviewLooksFurtherAheadAtHighSpeed) {
    const auto track = track::create_straight_corner_track(30.0, 20.0, 4.5);

    auto low_speed_perf = fast_profile_perf();
    low_speed_perf.use_steering_actuator_limits = true;
    low_speed_perf.max_speed = 10.0;
    low_speed_perf.steering_preview_time = 0.30;
    low_speed_perf.steering_min_preview_distance = 3.0;
    low_speed_perf.steering_max_preview_distance = 25.0;
    laptime::VelocityProfileGenerator low_speed_generator(low_speed_perf);
    const auto low_speed_profile = low_speed_generator.generate(track, 1.0);

    auto high_speed_perf = low_speed_perf;
    high_speed_perf.max_speed = 35.0;
    laptime::VelocityProfileGenerator high_speed_generator(high_speed_perf);
    const auto high_speed_profile = high_speed_generator.generate(track, 1.0);

    EXPECT_FALSE(std::isfinite(steering_cap_at(low_speed_profile, 22.0)));
    EXPECT_TRUE(std::isfinite(steering_cap_at(high_speed_profile, 22.0)));
}

TEST(VelocityProfileActuatorTest, LoadSensitivityReducesHighDownforceCornerSpeed) {
    const auto track = track::create_circle_track(20.0, 4.5);

    auto insensitive_perf = fast_profile_perf();
    insensitive_perf.max_lat_accel = 100.0;
    insensitive_perf.mu_lat = 2.0;
    insensitive_perf.cl_a = 5.0;
    insensitive_perf.max_speed = 35.0;
    laptime::VelocityProfileGenerator insensitive_generator(insensitive_perf);
    const auto insensitive_profile = insensitive_generator.generate(track, 1.0);

    auto sensitive_perf = insensitive_perf;
    sensitive_perf.use_load_sensitive_lateral_limit = true;
    sensitive_perf.tire_nominal_load = 700.0;
    sensitive_perf.tire_load_sensitivity = -0.0003;
    laptime::VelocityProfileGenerator sensitive_generator(sensitive_perf);
    const auto sensitive_profile = sensitive_generator.generate(track, 1.0);

    EXPECT_LT(speed_at(sensitive_profile, 20.0), speed_at(insensitive_profile, 20.0) - 1.0);
}

TEST(VelocityProfileActuatorTest, LoadTransferReducesCornerSpeedForHighCgNarrowTrack) {
    const auto track = track::create_circle_track(16.0, 4.5);

    auto low_transfer_perf = fast_profile_perf();
    low_transfer_perf.max_lat_accel = 100.0;
    low_transfer_perf.mu_lat = 2.0;
    low_transfer_perf.cl_a = 0.0;
    low_transfer_perf.use_load_sensitive_lateral_limit = true;
    low_transfer_perf.tire_nominal_load = 700.0;
    low_transfer_perf.tire_load_sensitivity = -0.0003;
    low_transfer_perf.cg_height = 0.20;
    low_transfer_perf.track_front = 1.60;
    low_transfer_perf.track_rear = 1.60;
    laptime::VelocityProfileGenerator low_transfer_generator(low_transfer_perf);
    const auto low_transfer_profile = low_transfer_generator.generate(track, 1.0);

    auto high_transfer_perf = low_transfer_perf;
    high_transfer_perf.cg_height = 0.45;
    high_transfer_perf.track_front = 1.10;
    high_transfer_perf.track_rear = 1.10;
    laptime::VelocityProfileGenerator high_transfer_generator(high_transfer_perf);
    const auto high_transfer_profile = high_transfer_generator.generate(track, 1.0);

    EXPECT_LT(speed_at(high_transfer_profile, 20.0), speed_at(low_transfer_profile, 20.0) - 0.3);
}

TEST(VelocityProfileActuatorTest, ExplicitLateralAccelerationLimitIsEnforced) {
    const auto track = track::create_circle_track(20.0, 4.5);

    auto perf = fast_profile_perf();
    perf.mu_lat = 3.0;
    perf.cl_a = 5.0;
    perf.max_lat_accel = 8.0;
    perf.enforce_max_lat_accel = true;
    laptime::VelocityProfileGenerator generator(perf);
    const auto profile = generator.generate(track, 1.0);

    EXPECT_LE(speed_at(profile, 20.0), std::sqrt(perf.max_lat_accel * 20.0) + 1e-9);
}
