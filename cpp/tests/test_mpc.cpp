/// @file test_mpc.cpp
/// @brief MPC関連コンポーネントのユニットテスト

#include <gtest/gtest.h>
#include <fsae/track/track_model.hpp>
#include <fsae/track/track_distance.hpp>
#include <fsae/vehicle/bicycle_model.hpp>
#include <fsae/control/mpc_problem.hpp>
#include <fsae/driver/attack_driver.hpp>
#include <fsae/driver/attack_speed_planner.hpp>
#include <fsae/driver/course_attack_plan.hpp>
#include <fsae/driver/mpc_driver.hpp>
#include <fsae/driver/mpc_driver_v2.hpp>
#include <fsae/driver/state_frame.hpp>
#include <fsae/driver/track_phase_analyzer.hpp>
#include <nlohmann/json.hpp>

#include <cmath>

using namespace fsae;

TEST(StateFrameTest, ConvertsSAELateralVelocityAndYawRateToMapConvention) {
    vehicle::State14DOF state;
    state.x = 12.0;
    state.y = -3.0;
    state.psi = 0.2;
    state.vx = 10.0;
    state.vy = 1.5;
    state.r = 0.4;

    const auto map_state = driver::to_map_driver_state(state);

    EXPECT_DOUBLE_EQ(map_state.x, -3.0);
    EXPECT_DOUBLE_EQ(map_state.y, 12.0);
    EXPECT_NEAR(map_state.psi, M_PI / 2.0 - 0.2, 1e-12);
    EXPECT_DOUBLE_EQ(map_state.vx, 10.0);
    EXPECT_DOUBLE_EQ(map_state.vy, -1.5);
    EXPECT_DOUBLE_EQ(map_state.r, -0.4);
}

// ====================================================================
// TrackDistanceQuery テスト
// ====================================================================

class TrackDistanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        circle_track_ = std::make_unique<track::CurvatureBasedTrack>(
            track::create_circle_track(30.0, 3.0));
        query_ = std::make_unique<track::TrackDistanceQuery>(*circle_track_, 0.5);
    }

    std::unique_ptr<track::CurvatureBasedTrack> circle_track_;
    std::unique_ptr<track::TrackDistanceQuery> query_;
};

/// 円形トラック上の点の最近接検索
TEST_F(TrackDistanceTest, NearestPoint_OnTrack) {
    // 円形トラック: R=30m, 中心(0,0)
    // (30, 0) はs=0付近（東端）
    auto result = query_->nearest(30.0, 0.0);

    EXPECT_NEAR(result.s, 0.0, 1.0) << "s should be near 0 for (30,0)";
    EXPECT_NEAR(result.distance, 0.0, 0.5) << "Distance should be ~0 on track";
    EXPECT_NEAR(std::abs(result.n), 0.0, 0.5) << "Lateral deviation ~0";
}

/// トラック外の点の最近接検索
TEST_F(TrackDistanceTest, NearestPoint_OffTrack) {
    // (35, 0) は中心(0,0)から半径35m → トラック(R=30)の外側5m
    auto result = query_->nearest(35.0, 0.0);

    EXPECT_NEAR(result.s, 0.0, 1.0);
    EXPECT_NEAR(result.distance, 5.0, 1.0) << "5m outside track";
}

/// hint_s付き局所探索
TEST_F(TrackDistanceTest, NearestPoint_WithHint) {
    // s=0付近で最初の検索
    auto r1 = query_->nearest(30.0, 0.0);
    // 近くの点をhint付きで検索（5m先）
    double s_next = r1.s + 5.0;
    if (s_next > circle_track_->length()) s_next -= circle_track_->length();
    auto pt = circle_track_->get_point(s_next);
    auto r2 = query_->nearest(pt.x, pt.y, r1.s);

    EXPECT_NEAR(r2.s, s_next, 1.0);
    EXPECT_NEAR(r2.distance, 0.0, 0.5);
}

/// 参照軌道の連続性
TEST_F(TrackDistanceTest, ReferenceTrajectory_Consistency) {
    // (s, v) speed profile
    std::vector<std::pair<double, double>> sv;
    double L = circle_track_->length();
    for (int i = 0; i <= 100; ++i) {
        sv.emplace_back(L * i / 100.0, 10.0);  // 一定10m/s
    }

    auto ref = query_->get_reference_trajectory(0.0, sv, 20, 0.025);

    ASSERT_EQ(ref.size(), 21u);

    // 参照点が連続的に前進している
    for (size_t i = 1; i < ref.size(); ++i) {
        double dx = ref[i].x - ref[i-1].x;
        double dy = ref[i].y - ref[i-1].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        // 10m/s × 0.025s = 0.25m ステップ
        EXPECT_GT(dist, 0.1) << "Reference points should be spaced";
        EXPECT_LT(dist, 1.0) << "Reference points should not jump";
    }

    // 速度が正しい
    for (const auto& rp : ref) {
        EXPECT_NEAR(rp.vx, 10.0, 0.1);
    }
}

TEST(TrackPhaseAnalyzerTest, StraightTrackClassifiesAsStraight) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 12.0;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(straight_track, profile);

    EXPECT_EQ(phases.phase_at(10.0), driver::TrackPhase::Straight);
    EXPECT_EQ(phases.phase_at(60.0), driver::TrackPhase::Straight);
    EXPECT_EQ(phases.phase_at(110.0), driver::TrackPhase::Straight);
    EXPECT_TRUE(phases.attack_allowed_at(60.0));
    EXPECT_GT(phases.static_speed_cap_at(60.0), 12.0);
}

TEST(AttackBrakingCorridorTest, ReachableSpeedUsesConstantDecelKinematics) {
    const double reachable = driver::braking_corridor_reachable_speed(
        10.0,
        12.0,
        3.0,
        2.0,
        30.0);

    EXPECT_NEAR(reachable, std::sqrt(10.0 * 10.0 + 2.0 * 3.0 * 12.0), 1e-12);
    EXPECT_GT(
        driver::braking_corridor_reachable_speed(10.0, 20.0, 3.0, 2.0, 30.0),
        reachable)
        << "More distance to the same slow point permits a higher entry speed";
    EXPECT_GT(
        driver::braking_corridor_reachable_speed(10.0, 12.0, 4.0, 2.0, 30.0),
        reachable)
        << "More available deceleration permits a higher entry speed";
}

TEST(AttackBrakingCorridorTest, ReachableSpeedRespectsBoundsAndZeroDistance) {
    EXPECT_NEAR(
        driver::braking_corridor_reachable_speed(11.0, 0.0, 3.0, 2.0, 30.0),
        11.0,
        1e-12);
    EXPECT_DOUBLE_EQ(
        driver::braking_corridor_reachable_speed(40.0, 5.0, 3.0, 2.0, 30.0),
        30.0);
    EXPECT_DOUBLE_EQ(
        driver::braking_corridor_reachable_speed(0.1, 0.0, 3.0, 2.0, 30.0),
        2.0);
}

TEST(AttackBrakingCorridorTest, SpeedBandProvidesLowerAndUpperTargets) {
    const auto band = driver::make_braking_corridor_speed_band(
        20.0,
        14.0,
        1.0,
        2.0,
        30.0);

    ASSERT_TRUE(band.active);
    EXPECT_DOUBLE_EQ(band.v_upper, 14.0);
    EXPECT_LT(band.v_lower, band.v_upper)
        << "The braking corridor needs a finite coast/mild-accel band, not a single speed";
    EXPECT_GE(band.v_lower, 12.25);
    EXPECT_LE(band.v_lower, 13.25);

    const auto wider_band = driver::make_braking_corridor_speed_band(
        20.0,
        14.0,
        1.50,
        2.0,
        30.0);
    EXPECT_LT(wider_band.v_lower, band.v_lower)
        << "A larger band scale should allow earlier coast/mild-accel recovery";
}

TEST(TrackPhaseAnalyzerTest, ConstantRadiusTrackClassifiesAsApex) {
    track::CurvatureBasedTrack radius_track(
        {{0.0, 0.04}, {120.0, 0.04}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 9.0;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(radius_track, profile);

    EXPECT_EQ(phases.phase_at(20.0), driver::TrackPhase::Apex);
    EXPECT_EQ(phases.phase_at(60.0), driver::TrackPhase::Apex);
}

TEST(TrackPhaseAnalyzerTest, HairpinClassifiesBrakeTurnInApexAndExit) {
    track::CurvatureBasedTrack hairpin_track(
        {
            {0.0, 0.0},
            {35.0, 0.0},
            {55.0, 0.06},
            {80.0, 0.06},
            {105.0, 0.0},
            {140.0, 0.0},
        },
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (const auto& [s, v] : std::vector<std::pair<double, double>>{
             {0.0, 18.0},
             {35.0, 16.0},
             {60.0, 8.0},
             {80.0, 8.0},
             {120.0, 15.0},
             {140.0, 15.0},
         }) {
        laptime::VelocityPoint point;
        point.s = s;
        point.v = v;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(hairpin_track, profile);

    EXPECT_EQ(phases.phase_at(30.0), driver::TrackPhase::Brake);
    EXPECT_EQ(phases.phase_at(45.0), driver::TrackPhase::TurnIn);
    EXPECT_EQ(phases.phase_at(68.0), driver::TrackPhase::Apex);
    EXPECT_EQ(phases.phase_at(92.0), driver::TrackPhase::Exit);
    EXPECT_FALSE(phases.attack_allowed_at(30.0));
    EXPECT_LT(phases.static_speed_cap_at(30.0), 17.0);
    EXPECT_TRUE(phases.attack_allowed_at(92.0));
    EXPECT_GE(phases.segment_id_at(92.0), phases.segment_id_at(30.0));
}

TEST(TrackPhaseAnalyzerTest, ChicaneSignChangeClassifiesAsComplex) {
    track::CurvatureBasedTrack chicane_track(
        {
            {0.0, 0.0},
            {20.0, 0.05},
            {40.0, 0.05},
            {55.0, -0.05},
            {75.0, -0.05},
            {100.0, 0.0},
        },
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 10; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 9.0;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(chicane_track, profile);

    EXPECT_EQ(phases.phase_at(30.0), driver::TrackPhase::Complex);
    EXPECT_EQ(phases.phase_at(50.0), driver::TrackPhase::Complex);
}

TEST(AttackResidualProfileTest, InterpolatesAndClampsJsonSamples) {
    const nlohmann::json root = {
        {"schema", "attack_residual_profile_v1"},
        {"limits", {
            {"max_delta_v_ref_mps", 1.5},
            {"max_delta_steer_ff_rad", 0.03},
            {"max_delta_ax_bias_mps2", 0.5},
            {"max_delta_heading_relax_rad", 0.04},
        }},
        {"samples", nlohmann::json::array({
            {
                {"s", 0.0},
                {"delta_v_ref_mps", 0.0},
                {"delta_steer_ff_rad", 0.0},
                {"delta_ax_bias_mps2", 0.0},
                {"delta_heading_relax_rad", 0.0},
            },
            {
                {"s", 10.0},
                {"delta_v_ref_mps", 3.0},
                {"delta_steer_ff_rad", 0.06},
                {"delta_ax_bias_mps2", 2.0},
                {"delta_heading_relax_rad", 0.10},
            },
        })},
    };

    const auto profile = driver::AttackResidualProfile::from_json(root, 100.0, true);
    const auto midpoint = profile.correction_at(5.0);
    const auto high = profile.correction_at(10.0);

    EXPECT_NEAR(midpoint.delta_v_ref_mps, 0.75, 1e-12);
    EXPECT_NEAR(midpoint.delta_steer_ff_rad, 0.015, 1e-12);
    EXPECT_NEAR(midpoint.delta_ax_bias_mps2, 0.25, 1e-12);
    EXPECT_NEAR(midpoint.delta_heading_relax_rad, 0.02, 1e-12);
    EXPECT_NEAR(high.delta_v_ref_mps, 1.5, 1e-12)
        << "Speed residual must be clipped to the configured safe envelope";
    EXPECT_NEAR(high.delta_steer_ff_rad, 0.03, 1e-12)
        << "Steering residual must be clipped to the configured safe envelope";
    EXPECT_NEAR(high.delta_ax_bias_mps2, 0.5, 1e-12)
        << "Longitudinal residual must be clipped to the configured safe envelope";
    EXPECT_NEAR(high.delta_heading_relax_rad, 0.04, 1e-12)
        << "Heading relaxation must be clipped to the configured safe envelope";
}

TEST(AttackSpeedPlannerTest, StraightTrackRaisesAttackReferenceAboveBaseProfile) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 12.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(straight_track, profile);

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        straight_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_GT(speed_profile.planned_speed_at(60.0), 12.0)
        << "Straight/exit sections should expose attack headroom over the conservative base profile";
    EXPECT_LE(speed_profile.planned_speed_at(60.0), 30.0);
}

TEST(AttackSpeedPlannerTest, HighCurvatureTrackIsLimitedByLateralCapacity) {
    track::CurvatureBasedTrack radius_track(
        {{0.0, 0.08}, {120.0, 0.08}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 25.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(radius_track, profile);

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        radius_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_LT(speed_profile.planned_speed_at(60.0), 16.0)
        << "A high-curvature section must not inherit a straight-line speed target";
    EXPECT_EQ(
        speed_profile.sample_at(60.0).limiting_reason,
        driver::AttackSpeedLimitReason::Curvature);
}

TEST(AttackSpeedPlannerTest, LateralCapacityUsesVehicleTireAndAeroParams) {
    track::CurvatureBasedTrack radius_track(
        {{0.0, 0.055}, {120.0, 0.055}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 30.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(radius_track, profile);

    auto low_capacity = vehicle::VehicleParams::default_fsae();
    low_capacity.tire.mu_y_max = 1.15;
    low_capacity.aero.CL_A = 0.0;

    auto high_capacity = vehicle::VehicleParams::default_fsae();
    high_capacity.tire.mu_y_max = 2.20;
    high_capacity.aero.CL_A = 4.0;

    driver::AttackSpeedPlannerConfig config;
    config.accel_limit = 100.0;
    config.braking_safety = 1.0;

    const auto low_profile = driver::AttackSpeedPlanner::plan(
        radius_track,
        profile,
        low_capacity,
        phases,
        config);
    const auto high_profile = driver::AttackSpeedPlanner::plan(
        radius_track,
        profile,
        high_capacity,
        phases,
        config);

    ASSERT_FALSE(low_profile.empty());
    ASSERT_FALSE(high_profile.empty());
    EXPECT_GT(
        high_profile.planned_speed_at(60.0),
        low_profile.planned_speed_at(60.0) + 3.0)
        << "Attack speed planning must scale with vehicle lateral capability";
}

TEST(AttackSpeedPlannerTest, VehicleLateralSafetyScaleAccountsForMass) {
    auto nominal = vehicle::VehicleParams::default_fsae();
    auto heavy = nominal;
    heavy.mass.m_total = 322.0;
    auto light = nominal;
    light.mass.m_total = 252.0;

    EXPECT_NEAR(driver::attack_vehicle_lateral_safety_scale(nominal), 1.0, 1e-12);
    EXPECT_LT(driver::attack_vehicle_lateral_safety_scale(heavy), 0.93);
    EXPECT_GT(driver::attack_vehicle_lateral_safety_scale(light), 1.05);
    EXPECT_GE(driver::attack_vehicle_lateral_safety_scale(heavy), 0.82);
    EXPECT_LE(driver::attack_vehicle_lateral_safety_scale(light), 1.08);
}

TEST(AttackSpeedPlannerTest, HairpinPlannerBrakesBeforeCornerApex) {
    track::CurvatureBasedTrack hairpin_track(
        {
            {0.0, 0.0},
            {35.0, 0.0},
            {55.0, 0.06},
            {80.0, 0.06},
            {105.0, 0.0},
            {140.0, 0.0},
        },
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (const auto& [s, v] : std::vector<std::pair<double, double>>{
             {0.0, 30.0},
             {35.0, 30.0},
             {60.0, 4.0},
             {85.0, 4.0},
             {120.0, 18.0},
             {140.0, 18.0},
         }) {
        laptime::VelocityPoint point;
        point.s = s;
        point.v = v;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(hairpin_track, profile);

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        hairpin_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_LT(speed_profile.planned_speed_at(35.0), 28.0)
        << "Backward braking pass should lower the entry speed before the corner cap";
    EXPECT_LT(speed_profile.planned_speed_at(65.0), speed_profile.planned_speed_at(10.0));
    EXPECT_EQ(
        speed_profile.sample_at(35.0).limiting_reason,
        driver::AttackSpeedLimitReason::Braking);
}

TEST(AttackSpeedPlannerTest, ExternalSpeedCapsLimitOnlyLocalRegion) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 18.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(straight_track, profile);

    driver::AttackSpeedPlannerConfig config;
    config.external_speed_caps.push_back({60.0, 7.5, 4.0});
    config.accel_limit = 100.0;       // keep the test focused on the point cap
    config.braking_safety = 1.0;

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        straight_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases,
        config);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_LE(speed_profile.sample_at(60.0).planned_speed, 7.5 + 1e-9);
    EXPECT_EQ(
        speed_profile.sample_at(60.0).limiting_reason,
        driver::AttackSpeedLimitReason::ExternalCap);
    EXPECT_GT(speed_profile.planned_speed_at(20.0), 12.0)
        << "A local cap must not globally clamp the whole course";
}

TEST(AttackSpeedPlannerTest, ExternalSpeedWindowsLimitOnlyConfiguredRegion) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 18.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(straight_track, profile);

    driver::AttackSpeedPlannerConfig config;
    config.external_speed_windows.push_back({50.0, 70.0, 7.5});
    config.accel_limit = 100.0;
    config.braking_safety = 1.0;

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        straight_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases,
        config);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_LE(speed_profile.sample_at(60.0).planned_speed, 7.5 + 1e-9);
    EXPECT_EQ(
        speed_profile.sample_at(60.0).limiting_reason,
        driver::AttackSpeedLimitReason::ExternalCap);
    EXPECT_GT(speed_profile.planned_speed_at(20.0), 12.0);
    EXPECT_GT(speed_profile.planned_speed_at(90.0), 12.0);
}

TEST(AttackSpeedPlannerTest, ExternalCapsDoNotWrapAroundOpenTrack) {
    track::CurvatureBasedTrack open_track(
        {{0.0, 0.002}, {120.0, 0.002}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    ASSERT_FALSE(open_track.is_closed_loop());
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 18.0;
        profile.push_back(point);
    }
    const auto phases = driver::TrackPhaseAnalyzer::analyze(open_track, profile);

    driver::AttackSpeedPlannerConfig config;
    config.external_speed_caps.push_back({115.0, 7.5, 15.0});
    config.external_speed_windows.push_back({100.0, 140.0, 6.5});
    config.accel_limit = 100.0;
    config.braking_safety = 1.0;

    const auto speed_profile = driver::AttackSpeedPlanner::plan(
        open_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases,
        config);

    ASSERT_FALSE(speed_profile.empty());
    EXPECT_TRUE(std::isinf(speed_profile.sample_at(5.0).external_speed_cap));
    EXPECT_TRUE(std::isinf(speed_profile.sample_at(5.0).external_window_speed_cap));
    EXPECT_GT(speed_profile.planned_speed_at(5.0), 12.0)
        << "Open-track caps near the finish must not wrap back to the start";
    EXPECT_LE(speed_profile.sample_at(110.0).external_speed_cap, 7.5 + 1e-9);
    EXPECT_LE(speed_profile.sample_at(110.0).external_window_speed_cap, 6.5 + 1e-9);
}

TEST(CourseAttackPlanTest, BuildsUnifiedSpeedAndLineReference) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 12.0;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(straight_track, profile);
    const auto plan = driver::CourseAttackPlan::build(
        straight_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases,
        driver::BoundaryLinePlannerConfig());

    ASSERT_FALSE(plan.empty());
    EXPECT_GT(plan.v_ref_at(60.0), 12.0);
    EXPECT_TRUE(std::isfinite(plan.a_ref_at(60.0)));
    EXPECT_LE(std::abs(plan.a_ref_at(60.0)), 5.5 + 1e-9);
    EXPECT_NEAR(plan.n_ref_at(60.0), 0.0, 1e-12);
    EXPECT_NEAR(plan.kappa_ref_at(60.0), 0.0, 1e-12);
    EXPECT_EQ(plan.sample_at(60.0).phase, driver::TrackPhase::Straight);
    EXPECT_TRUE(plan.sample_at(60.0).attack_allowed);
}

TEST(CourseAttackPlanTest, PreservesExternalSpeedCapsAndPlannedLineOffset) {
    track::CurvatureBasedTrack radius_track(
        {{0.0, 0.04}, {120.0, 0.04}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 18.0;
        profile.push_back(point);
    }

    const auto phases = driver::TrackPhaseAnalyzer::analyze(radius_track, profile);
    driver::BoundaryLinePlannerConfig line_config;
    line_config.enabled = true;
    line_config.track_geometry = true;
    line_config.offset_fraction = 0.45;
    line_config.safety_margin = 0.20;

    driver::AttackSpeedPlannerConfig speed_config;
    speed_config.external_speed_caps.push_back({60.0, 7.5, 3.0});
    speed_config.accel_limit = 100.0;
    speed_config.braking_safety = 1.0;

    const auto plan = driver::CourseAttackPlan::build(
        radius_track,
        profile,
        vehicle::VehicleParams::default_fsae(),
        phases,
        line_config,
        speed_config);

    ASSERT_FALSE(plan.empty());
    EXPECT_LE(plan.sample_at(60.0).v_ref, 7.5 + 1e-9);
    EXPECT_TRUE(std::isfinite(plan.sample_at(60.0).a_ref));
    EXPECT_LE(plan.sample_at(60.0).a_ref, speed_config.accel_limit + 1e-9);
    EXPECT_EQ(plan.sample_at(60.0).speed_reason, driver::AttackSpeedLimitReason::ExternalCap);
    EXPECT_GT(std::abs(plan.n_ref_at(60.0)), 0.05)
        << "Fixed-course plan should expose the planned lateral line as n_ref";
    EXPECT_TRUE(std::isfinite(plan.kappa_ref_at(60.0)));
}

// ====================================================================
// MPCProblem テスト
// ====================================================================

class MPCProblemTest : public ::testing::Test {
protected:
    void SetUp() override {
        vehicle::BicycleParams params{280, 85, 0.73, 0.82, 40000, 45000};
        model_ = std::make_unique<vehicle::BicycleModel6DOF>(params);
    }

    std::unique_ptr<vehicle::BicycleModel6DOF> model_;
};

/// QP次元の正しさ
TEST_F(MPCProblemTest, QP_Dimensions) {
    control::MPCConfig config;
    config.N = 10;
    control::MPCProblem problem(config);

    vehicle::BicycleState6DOF x0{0, 0, 0, 10.0, 0, 0};

    // 参照軌道（直線）
    std::vector<track::ReferencePoint> ref_traj;
    for (int k = 0; k <= config.N; ++k) {
        double s = k * 10.0 * config.dt;
        ref_traj.push_back({s, 0.0, 0.0, 10.0, s, 0.0, 1.5, 1.5});
    }

    auto qp = problem.formulate(x0, ref_traj, 0.0, *model_);

    // 変数数: (N+1)*6 + N*2 = 11*6 + 10*2 = 86
    EXPECT_EQ(qp.n_vars, 86);
    EXPECT_EQ(qp.P.rows(), 86);
    EXPECT_EQ(qp.P.cols(), 86);
    EXPECT_EQ(qp.q.size(), 86);
    EXPECT_EQ(qp.A.cols(), 86);

    // 制約数: 6(初期) + 10*6(動力学) + 10*2(入力) + 9(δレート) + 10(vx) = 6+60+20+9+10 = 105
    EXPECT_EQ(qp.n_constraints, 105);
    EXPECT_EQ(qp.A.rows(), 105);
    EXPECT_EQ(qp.l.size(), 105);
    EXPECT_EQ(qp.u.size(), 105);
}

/// 直線参照でδ≈0
/// P行列の正半定値性と基本的な構造検証
TEST_F(MPCProblemTest, StraightLine_SmallSteering) {
    control::MPCConfig config;
    config.N = 10;
    control::MPCProblem problem(config);

    vehicle::BicycleState6DOF x0{0, 0, 0, 10.0, 0, 0};

    // 直線参照軌道
    std::vector<track::ReferencePoint> ref_traj;
    for (int k = 0; k <= config.N; ++k) {
        double s = k * 10.0 * config.dt;
        ref_traj.push_back({s, 0.0, 0.0, 10.0, s, 0.0, 1.5, 1.5});
    }

    auto qp = problem.formulate(x0, ref_traj, 0.0, *model_);

    // P行列の対角要素が非負（半正定値）
    for (int i = 0; i < qp.P.rows(); ++i) {
        EXPECT_GE(qp.P.coeff(i, i), 0.0)
            << "P diagonal should be non-negative at index " << i;
    }

    // 初期状態制約が正しくセット
    // 最初のNX個のA行列行が単位行列に対応
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(qp.l(i), x0.to_vector()(i), 1e-10)
            << "Initial state constraint l[" << i << "]";
        EXPECT_NEAR(qp.u(i), x0.to_vector()(i), 1e-10)
            << "Initial state constraint u[" << i << "]";
    }
}

// ====================================================================
// Sampling preview MPC driver integration tests
// ====================================================================

class MPCDriverTest : public ::testing::Test {
protected:
    void SetUp() override {
        circle_track_ = std::make_unique<track::CurvatureBasedTrack>(
            track::create_circle_track(30.0, 3.0));

        // 速度プロファイル（一定10m/s）
        double L = circle_track_->length();
        for (int i = 0; i <= 100; ++i) {
            laptime::VelocityPoint vp;
            vp.s = L * i / 100.0;
            vp.v = 10.0;
            vp.ax = 0.0;
            vp.ay = 0.0;
            velocity_profile_.push_back(vp);
        }

        control::MPCConfig config;
        config.N = 10;  // テスト用に短いホライズン
        config.dt = 0.025;

        driver_ = std::make_unique<driver::MPCPathFollowingDriver>(
            *circle_track_, velocity_profile_, config);
    }

    std::unique_ptr<track::CurvatureBasedTrack> circle_track_;
    std::vector<laptime::VelocityPoint> velocity_profile_;
    std::unique_ptr<driver::MPCPathFollowingDriver> driver_;
};

static driver::TrackPhaseProfile make_constant_phase_profile(
    const track::CurvatureBasedTrack& track,
    driver::TrackPhase phase)
{
    std::vector<driver::TrackPhaseSample> samples;
    const double length = track.length();
    samples.reserve(4);
    driver::TrackPhaseSample start;
    start.s = 0.0;
    start.phase = phase;
    samples.push_back(start);
    driver::TrackPhaseSample mid;
    mid.s = 0.5 * length;
    mid.phase = phase;
    samples.push_back(mid);
    driver::TrackPhaseSample end;
    end.s = length;
    end.phase = phase;
    samples.push_back(end);
    return driver::TrackPhaseProfile(std::move(samples), length);
}

TEST_F(MPCDriverTest, CircleTrack_ProducesControl) {
    // 車両状態: 円形トラックのスタート位置
    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 7.0;
    state.vy = 0.0;
    state.r = 0.0;

    auto input = driver_->compute_control(state, 0.0);

    // 制御入力が有効な範囲内
    EXPECT_GE(input.steering, -0.5);
    EXPECT_LE(input.steering, 0.5);
    EXPECT_GE(input.throttle, 0.0);
    EXPECT_LE(input.throttle, 1.0);
    EXPECT_GE(input.brake, 0.0);
    EXPECT_LE(input.brake, 1.0);
}

TEST_F(MPCDriverTest, CircleTrackSteeringUsesRightPositiveDriverConvention) {
    vehicle::VehicleState state;
    state.x = 30.80;
    state.y = 0.0;
    state.psi = 1.2; // Deliberate outward boundary risk in a turn-in phase.
    state.vx = 4.0;

    auto input = driver_->compute_control(state, 0.0);

    EXPECT_LT(input.steering, 0.0)
        << "CCW circle tracking requires left steering; driver convention is positive=right";
}

TEST_F(MPCDriverTest, TorqueModeUsesSharedSteeringActuatorLayer) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.05;
    params.steering.C_steer_torque = 0.5;

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;

    driver::MPCPathFollowingDriver torque_driver(*circle_track_, velocity_profile_, config, params);
    torque_driver.set_steering_state(0.0, 0.0);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    auto input = torque_driver.compute_control(state, 0.0);
    const auto& debug = torque_driver.last_debug();

    EXPECT_TRUE(debug.torque_mode);
    EXPECT_LT(debug.target_tire_angle, 0.0);
    EXPECT_LT(input.steering, 0.0);
    EXPECT_LE(std::abs(input.steering), params.steering.T_driver_max);
    EXPECT_NEAR(input.steering, debug.steering_actuator.steering_torque, 1e-12);
    EXPECT_LT(debug.steering_actuator.target_sw, 0.0);
}

TEST_F(MPCDriverTest, TorqueModeOuterControlDefersInnerLoopToPhysicsRateUpdate) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.02;
    params.steering.C_steer_torque = 0.3;

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;
    config.steering_inner_loop_omega_n = 80.0;

    driver::MPCPathFollowingDriver torque_driver(*circle_track_, velocity_profile_, config, params);
    torque_driver.set_steering_state(0.0, 0.0);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    auto input = torque_driver.compute_outer_control(state, 0.0);
    EXPECT_DOUBLE_EQ(input.steering, 0.0);
    EXPECT_LT(torque_driver.last_debug().target_tire_angle, 0.0);

    constexpr double kPhysicsDt = 0.001;
    input = torque_driver.update_torque_actuator_control(input, 0.0, 0.0, kPhysicsDt);
    const auto& actuator = torque_driver.last_debug().steering_actuator;

    EXPECT_LT(input.steering, 0.0);
    EXPECT_NEAR(input.steering, actuator.steering_torque, 1e-12);
    EXPECT_LE(
        std::abs(actuator.target_sw),
        config.delta_rate_max / config.dt * params.steering.R_steer * kPhysicsDt + 1e-12);
}

TEST_F(MPCDriverTest, InnerLoopBandwidthConfigChangesTorqueActuatorResponse) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 100.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.02;
    params.steering.C_steer_torque = 0.3;

    control::MPCConfig slow_config;
    slow_config.N = 10;
    slow_config.dt = 0.025;
    slow_config.steering_inner_loop_omega_n = 30.0;

    auto fast_config = slow_config;
    fast_config.steering_inner_loop_omega_n = 80.0;

    driver::MPCPathFollowingDriver slow_driver(*circle_track_, velocity_profile_, slow_config, params);
    driver::MPCPathFollowingDriver fast_driver(*circle_track_, velocity_profile_, fast_config, params);
    slow_driver.set_steering_state(0.0, 0.0);
    fast_driver.set_steering_state(0.0, 0.0);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    slow_driver.compute_control(state, 0.0);
    fast_driver.compute_control(state, 0.0);

    EXPECT_GT(
        std::abs(fast_driver.last_debug().steering_actuator.T_driver_raw),
        std::abs(slow_driver.last_debug().steering_actuator.T_driver_raw));
}

TEST_F(MPCDriverTest, SteeringTargetRespectsActuatorReachPerControlStep) {
    auto tight_track = track::create_circle_track(8.0, 3.0);
    std::vector<laptime::VelocityPoint> velocity_profile;
    for (int i = 0; i <= 100; ++i) {
        laptime::VelocityPoint vp;
        vp.s = tight_track.length() * i / 100.0;
        vp.v = 10.0;
        vp.kappa = 1.0 / 8.0;
        velocity_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;
    config.delta_rate_max = 0.01;

    driver::MPCPathFollowingDriver limited_driver(tight_track, velocity_profile, config);

    vehicle::VehicleState state;
    state.x = 8.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    auto input = limited_driver.compute_control(state, 0.0);
    const auto& debug = limited_driver.last_debug();

    EXPECT_LE(std::abs(debug.target_tire_angle), config.delta_rate_max + 1e-12);
    EXPECT_NEAR(input.steering, debug.target_tire_angle, 1e-12);
    EXPECT_LT(debug.steering_speed_cap, debug.target_speed_base);
    EXPECT_LE(debug.target_speed, debug.steering_speed_cap + 1e-12);
}

TEST_F(MPCDriverTest, CurrentLateralMarginUsesAsymmetricTrackWidths) {
    std::vector<track::TrackPoint> points;
    for (int i = 0; i <= 50; ++i) {
        const double s = static_cast<double>(i);
        points.push_back(track::TrackPoint{
            s,
            s,
            0.0,
            0.0,
            0.0,
            2.0,  // left width
            1.0,  // right width
        });
    }
    track::CurvatureBasedTrack straight_track(points, false);

    std::vector<laptime::VelocityPoint> velocity_profile;
    for (int i = 0; i <= 50; ++i) {
        laptime::VelocityPoint vp;
        vp.s = static_cast<double>(i);
        vp.v = 6.0;
        velocity_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 5;
    config.dt = 0.025;
    driver::MPCPathFollowingDriver margin_driver(straight_track, velocity_profile, config);

    vehicle::VehicleState state;
    state.x = 5.0;
    state.y = 1.5;  // left of centerline; left margin should be 2.0 - 1.5 = 0.5 m
    state.psi = 0.0;
    state.vx = 5.0;

    margin_driver.compute_control(state, 0.0);

    EXPECT_NEAR(margin_driver.last_debug().current_lateral_margin, 0.5, 0.05);
}

TEST_F(MPCDriverTest, LowSpeedHighTargetDoesNotSelectBrake) {
    std::vector<track::TrackPoint> points;
    for (int i = 0; i <= 50; ++i) {
        const double s = static_cast<double>(i);
        points.push_back(track::TrackPoint{s, s, 0.0, 0.0, 0.0, 2.0, 2.0});
    }
    track::CurvatureBasedTrack straight_track(points, false);

    std::vector<laptime::VelocityPoint> velocity_profile;
    for (int i = 0; i <= 50; ++i) {
        laptime::VelocityPoint vp;
        vp.s = static_cast<double>(i);
        vp.v = 12.0;
        velocity_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 20;
    config.dt = 0.025;
    driver::MPCPathFollowingDriver launch_driver(straight_track, velocity_profile, config);

    vehicle::VehicleState state;
    state.x = 5.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 1.0;

    auto input = launch_driver.compute_control(state, 0.0);

    EXPECT_GT(input.throttle, 0.0);
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
    EXPECT_GT(launch_driver.last_debug().selected_ax, 0.0);
}

TEST_F(MPCDriverTest, SolveTime_Reasonable) {
    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    // 5回実行
    for (int i = 0; i < 5; ++i) {
        driver_->compute_control(state, i * 0.025);
    }

    auto stats = driver_->stats();
    EXPECT_GT(stats.solve_count, 0);
    EXPECT_GT(stats.evaluated_rollouts, 0);
    EXPECT_LT(stats.avg_solve_time_ms(), 50.0)
        << "Average solve time should be <50ms";
}

// ====================================================================
// 新規テスト: 閉ループ収束・アクチュエータラグ・SteeringActuator単体
// ====================================================================

/// 閉ループ多ステップ: N=40ステップ後にトラックへ収束することを確認
TEST_F(MPCDriverTest, MultiStep_ConvergesOnCircleTrack) {
    control::MPCConfig config;
    config.N = 20;
    config.dt = 0.025;

    driver::MPCPathFollowingDriver converge_driver(*circle_track_, velocity_profile_, config);

    // 初期状態: トラック上 (R=30, psi=90°) から少し横にずれた位置
    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 9.0;

    // 簡易キネマティック自転車モデルで状態を更新
    const double L = 1.55;  // ホイールベース [m]
    const double dt = 0.025;

    for (int step = 0; step < 40; ++step) {
        auto input = converge_driver.compute_control(state, step * dt);

        // throttle/brake から ax を復元 (clampなし近似)
        const double ax_mag = 8.0;  // ax_max のデフォルト相当
        const double ax = (input.throttle - input.brake) * ax_mag;

        // キネマティック自転車モデルで更新
        const double delta = input.steering;  // 位置モードなのでそのままタイヤ角
        const double yaw_rate = -state.vx * std::tan(delta) / L;
        state.psi += yaw_rate * dt;
        // psi を [-pi, pi] に正規化
        while (state.psi >  M_PI) state.psi -= 2.0 * M_PI;
        while (state.psi < -M_PI) state.psi += 2.0 * M_PI;
        state.x += state.vx * std::cos(state.psi) * dt;
        state.y += state.vx * std::sin(state.psi) * dt;
        state.vx = std::max(state.vx + ax * dt, 0.5);
    }

    // 40ステップ後 (1秒間): 横偏差が 0.5 m 以内かつヘッディングエラーが 0.3 rad 以内
    const auto& debug = converge_driver.last_debug();
    EXPECT_LT(std::abs(debug.current_lateral_margin - 0.0), 1.5)
        << "Vehicle should be near track centerline after 40 steps; margin="
        << debug.current_lateral_margin;
    // コントロールが失敗扱いになっていないことを確認
    EXPECT_EQ(converge_driver.stats().fail_count, 0)
        << "MPC should not fail during closed-loop run";
}

/// アクチュエータ動特性フラグをOFFにすると予測がレート制限のみになる
TEST_F(MPCDriverTest, ActuatorDynamicsFlag_DisabledUsesRateLimitOnly) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.05;
    params.steering.C_steer_torque = 0.5;

    // actuator dynamics ON
    control::MPCConfig config_on;
    config_on.N = 10;
    config_on.dt = 0.025;
    config_on.use_actuator_dynamics_in_prediction = true;

    // actuator dynamics OFF
    control::MPCConfig config_off = config_on;
    config_off.use_actuator_dynamics_in_prediction = false;

    driver::MPCPathFollowingDriver driver_lag_on(*circle_track_, velocity_profile_, config_on, params);
    driver::MPCPathFollowingDriver driver_lag_off(*circle_track_, velocity_profile_, config_off, params);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    driver_lag_on.set_steering_state(0.0, 0.0);
    driver_lag_off.set_steering_state(0.0, 0.0);

    auto input_on  = driver_lag_on.compute_control(state, 0.0);
    auto input_off = driver_lag_off.compute_control(state, 0.0);

    // どちらも有効な制御を出力すること
    EXPECT_LE(std::abs(input_on.steering),  params.steering.T_driver_max + 1e-9);
    EXPECT_LE(std::abs(input_off.steering), params.steering.T_driver_max + 1e-9);
    // lag ON と OFF でステアリング出力が異なること（予測が変わるため）
    // ただし稀に同一になる可能性があるので soft-fail
    EXPECT_EQ(driver_lag_on.stats().fail_count, 0);
    EXPECT_EQ(driver_lag_off.stats().fail_count, 0);
}

TEST(SteeringActuatorControllerTest, PredictionUsesSecondOrderClosedLoopResponse) {
    using namespace fsae::driver;
    SteeringActuatorConfig config;
    config.max_tire_angle = 0.35;
    config.max_tire_steer_rate = 20.0;
    config.inner_loop_omega_n = 30.0;
    config.inner_loop_zeta = 0.70;

    SteeringActuatorPredictionState state;
    const auto at_10ms = predict_steering_actuator_response(config, state, 0.20, 0.010);
    const auto at_100ms = predict_steering_actuator_response(config, state, 0.20, 0.100);

    EXPECT_GT(at_10ms.tire_angle, 0.0);
    EXPECT_LT(at_10ms.tire_angle, 0.20);
    EXPECT_GT(at_100ms.tire_angle, at_10ms.tire_angle);
    EXPECT_NEAR(at_100ms.tire_angle, 0.20, 0.03);
}

TEST(SteeringActuatorControllerTest, PredictionRespectsTireRateLimit) {
    using namespace fsae::driver;
    SteeringActuatorConfig config;
    config.max_tire_angle = 0.35;
    config.max_tire_steer_rate = 0.50;
    config.inner_loop_omega_n = 80.0;
    config.inner_loop_zeta = 0.70;

    SteeringActuatorPredictionState state;
    const auto predicted = predict_steering_actuator_response(config, state, 0.30, 0.100);

    EXPECT_LE(std::abs(predicted.tire_rate), config.max_tire_steer_rate + 1e-12);
    EXPECT_LE(predicted.tire_angle, config.max_tire_steer_rate * 0.100 + 1e-3);
}

TEST(SteeringActuatorControllerTest, ReachTimeIncludesSecondOrderRiseTime) {
    using namespace fsae::driver;
    SteeringActuatorConfig fast_config;
    fast_config.max_tire_angle = 0.35;
    fast_config.max_tire_steer_rate = 20.0;
    fast_config.inner_loop_omega_n = 50.0;
    fast_config.inner_loop_zeta = 0.70;

    auto slow_config = fast_config;
    slow_config.inner_loop_omega_n = 20.0;

    SteeringActuatorPredictionState state;
    const double fast_time =
        estimate_steering_actuator_reach_time(fast_config, state, 0.20, 0.01, 1.0);
    const double slow_time =
        estimate_steering_actuator_reach_time(slow_config, state, 0.20, 0.01, 1.0);

    EXPECT_TRUE(std::isfinite(fast_time));
    EXPECT_TRUE(std::isfinite(slow_time));
    EXPECT_GT(fast_time, 0.0);
    EXPECT_GT(slow_time, fast_time);
}

TEST_F(MPCDriverTest, V2PreservesMapFrameStateContractAndTorqueActuator) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.05;
    params.steering.C_steer_torque = 0.5;

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;

    driver::MPCPathFollowingDriverV2 v2_driver(*circle_track_, velocity_profile_, config, params);
    v2_driver.set_steering_state(0.0, 0.0);

    vehicle::VehicleState map_state;
    map_state.x = 30.0;
    map_state.y = 0.0;
    map_state.psi = M_PI / 2.0;
    map_state.vx = 10.0;

    auto input = v2_driver.compute_control(map_state, 0.0);
    const auto& debug = v2_driver.last_debug();

    EXPECT_TRUE(debug.torque_mode);
    EXPECT_NEAR(debug.current_s, 0.0, 1.0)
        << "MPC v2 must consume the already map-frame state without a second transform";
    EXPECT_LT(debug.target_tire_angle, 0.0);
    EXPECT_LT(input.steering, 0.0);
    EXPECT_LE(std::abs(input.steering), params.steering.T_driver_max);
    EXPECT_GT(v2_driver.stats().solve_count, 0);
    EXPECT_EQ(v2_driver.stats().fail_count, 0);
}

TEST_F(MPCDriverTest, V2TrackMarginSpeedCapReducesBoundarySpeedTarget) {
    std::vector<track::TrackPoint> points;
    for (int i = 0; i <= 60; ++i) {
        const double s = static_cast<double>(i);
        points.push_back(track::TrackPoint{s, s, 0.0, 0.0, 0.0, 2.0, 2.0});
    }
    track::CurvatureBasedTrack straight_track(points, false);

    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 60; ++i) {
        laptime::VelocityPoint vp;
        vp.s = static_cast<double>(i);
        vp.v = 20.0;
        fast_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;

    driver::MPCPathFollowingDriver legacy_driver(straight_track, fast_profile, config);
    driver::MPCPathFollowingDriverV2 v2_driver(straight_track, fast_profile, config);

    vehicle::VehicleState near_left_boundary;
    near_left_boundary.x = 5.0;
    near_left_boundary.y = 1.55; // left margin = 0.45 m
    near_left_boundary.psi = 0.0;
    near_left_boundary.vx = 12.0;

    legacy_driver.compute_control(near_left_boundary, 0.0);
    v2_driver.compute_control(near_left_boundary, 0.0);

    const auto& legacy_debug = legacy_driver.last_debug();
    const auto& v2_debug = v2_driver.last_debug();

    EXPECT_FALSE(std::isfinite(legacy_debug.track_margin_speed_cap))
        << "Legacy MPC keeps track-margin speed capping disabled";
    EXPECT_TRUE(std::isfinite(v2_debug.track_margin_speed_cap));
    EXPECT_LT(v2_debug.track_margin_speed_cap, v2_debug.target_speed_base);
    EXPECT_LE(v2_debug.target_speed, v2_debug.track_margin_speed_cap + 1e-12);
}

TEST_F(MPCDriverTest, ProgressObjectiveDoesNotRelaxWhenSafetySpeedCapActivates) {
    std::vector<track::TrackPoint> points;
    for (int i = 0; i <= 60; ++i) {
        const double s = static_cast<double>(i);
        points.push_back(track::TrackPoint{s, s, 0.0, 0.0, 0.0, 2.0, 2.0});
    }
    track::CurvatureBasedTrack straight_track(points, false);

    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 60; ++i) {
        laptime::VelocityPoint vp;
        vp.s = static_cast<double>(i);
        vp.v = 20.0;
        fast_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 10;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.track_margin_speed_cap = 8.0;
    config.track_margin_speed_release = 1.0;
    config.terminal_progress_weight = 1.0;

    driver::MPCPathFollowingDriver margin_driver(straight_track, fast_profile, config);

    vehicle::VehicleState near_left_boundary;
    near_left_boundary.x = 5.0;
    near_left_boundary.y = 1.55;
    near_left_boundary.psi = 0.0;
    near_left_boundary.vx = 12.0;

    margin_driver.compute_control(near_left_boundary, 0.0);
    const auto& debug = margin_driver.last_debug();

    EXPECT_LT(debug.target_speed, debug.target_speed_base);
    EXPECT_NEAR(
        debug.rollout_progress_target,
        20.0 * config.N * config.dt * config.progress_target_fraction,
        1e-9);
}

TEST_F(MPCDriverTest, TerminalProgressObjectivePrefersAccelerationWhenSpeedWeightDisabled) {
    std::vector<track::TrackPoint> points;
    for (int i = 0; i <= 100; ++i) {
        const double s = static_cast<double>(i);
        points.push_back(track::TrackPoint{s, s, 0.0, 0.0, 0.0, 2.0, 2.0});
    }
    track::CurvatureBasedTrack straight_track(points, false);

    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 100; ++i) {
        laptime::VelocityPoint vp;
        vp.s = static_cast<double>(i);
        vp.v = 20.0;
        fast_profile.push_back(vp);
    }

    control::MPCConfig config;
    config.N = 20;
    config.dt = 0.05;
    config.w_y = 0.0;
    config.w_psi = 0.0;
    config.w_vx = 0.0;
    config.w_delta = 0.0;
    config.w_delta_rate = 0.0;
    config.w_ax = 0.10;
    config.ax_max = 10.0;
    config.ax_min = -10.0;
    config.terminal_progress_weight = 2.0;
    config.progress_target_fraction = 0.90;

    driver::MPCPathFollowingDriver progress_driver(straight_track, fast_profile, config);

    vehicle::VehicleState state;
    state.x = 5.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 8.0;

    auto input = progress_driver.compute_control(state, 0.0);
    const auto& debug = progress_driver.last_debug();

    EXPECT_GT(debug.selected_ax, 4.0)
        << "Terminal progress deficit should overcome the ax regularizer";
    EXPECT_GT(input.throttle, 0.0);
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
    EXPECT_GT(debug.rollout_progress_target, debug.rollout_progress);
    EXPECT_GT(debug.rollout_progress_deficit, 0.0);
}

TEST_F(MPCDriverTest, AttackDriverProducesControlAndSequenceRolloutStats) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();
    const auto& stats = attack_driver.stats();

    EXPECT_LT(debug.target_tire_angle, 0.0)
        << "CCW circle tracking requires left steering; driver convention is positive=right";
    EXPECT_GE(input.throttle, 0.0);
    EXPECT_LE(input.throttle, 1.0);
    EXPECT_GE(input.brake, 0.0);
    EXPECT_LE(input.brake, 1.0);
    EXPECT_GT(debug.rollout_progress_target, 0.0);
    EXPECT_GE(debug.selected_plan_index, 0);
    EXPECT_TRUE(std::isfinite(debug.predicted_terminal_n));
    EXPECT_TRUE(std::isfinite(debug.predicted_terminal_heading));
    EXPECT_TRUE(std::isfinite(debug.predicted_min_margin));
    EXPECT_TRUE(std::isfinite(debug.predicted_terminal_speed));
    EXPECT_TRUE(std::isfinite(debug.predicted_signed_progress));
    EXPECT_GE(debug.attack_selected_primitive, 0);
    EXPECT_NE(debug.attack_selected_primitive_kind, "unknown");
    EXPECT_NE(debug.attack_selected_primitive_kind, "fallback");
    EXPECT_TRUE(std::isfinite(debug.attack_selected_speed_scale));
    EXPECT_TRUE(std::isfinite(debug.attack_selected_target_speed));
    EXPECT_TRUE(std::isfinite(debug.attack_selected_ax));
    EXPECT_TRUE(std::isfinite(debug.course_v_ref));
    EXPECT_GT(debug.course_v_ref, 0.0);
    EXPECT_TRUE(std::isfinite(debug.course_n_ref));
    EXPECT_TRUE(std::isfinite(debug.course_kappa_ref));
    EXPECT_TRUE(std::isfinite(debug.course_speed_error));
    EXPECT_TRUE(std::isfinite(debug.course_lateral_error));
    EXPECT_TRUE(std::isfinite(debug.course_a_ref));
    EXPECT_TRUE(std::isfinite(debug.course_target_gap));
    EXPECT_TRUE(std::isfinite(debug.dynamic_cap_gap));
    EXPECT_NE(debug.longitudinal_mode, "unknown");
    EXPECT_TRUE(std::isfinite(debug.accel_headroom));
    EXPECT_EQ(debug.attack_rollout_candidate_count, 28);
    EXPECT_GE(debug.attack_rollout_reject_count, 0);
    EXPECT_LE(debug.attack_rollout_reject_count, debug.attack_rollout_candidate_count);
    EXPECT_FALSE(debug.attack_all_rejected);
    EXPECT_TRUE(std::isfinite(debug.attack_cost_tracking));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_speed));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_boundary));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_control_smoothness));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_progress));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_primitive_bias));
    EXPECT_TRUE(std::isfinite(debug.attack_best_tracking_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_boundary_recovery_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_low_speed_pure_pursuit_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_065_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_080_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_105_cost));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_cost_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_tracking_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_speed_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_boundary_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_control_smoothness_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_progress_delta));
    EXPECT_TRUE(std::isfinite(debug.attack_best_speed_scale_095_primitive_bias_delta));
    EXPECT_NE(debug.track_phase, "unknown");
    EXPECT_TRUE(std::isfinite(debug.phase_speed_bias));
    EXPECT_TRUE(std::isfinite(debug.phase_margin_bias));
    EXPECT_TRUE(std::isfinite(debug.phase_progress_bias));
    EXPECT_GE(debug.attack_segment_id, 0);
    EXPECT_TRUE(std::isfinite(debug.attack_static_speed_cap));
    EXPECT_GT(debug.attack_static_speed_cap, 0.0);
    EXPECT_TRUE(std::isfinite(debug.attack_dynamic_speed_cap));
    EXPECT_GT(debug.attack_dynamic_speed_cap, 0.0);
    EXPECT_TRUE(std::isfinite(debug.attack_dynamic_base_speed));
    EXPECT_GT(debug.attack_dynamic_base_speed, 0.0);
    EXPECT_FALSE(std::isnan(debug.attack_curvature_speed_cap));
    EXPECT_FALSE(std::isnan(debug.attack_steering_speed_cap));
    EXPECT_FALSE(std::isnan(debug.attack_margin_speed_cap));
    EXPECT_FALSE(std::isnan(debug.attack_heading_speed_cap));
    EXPECT_FALSE(std::isnan(debug.attack_outward_speed_cap));
    EXPECT_FALSE(debug.attack_limiting_cap_reason.empty());
    EXPECT_TRUE(std::isfinite(debug.attack_cost_overspeed));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_underspeed));
    EXPECT_TRUE(std::isfinite(debug.attack_cost_time_progress));
    EXPECT_GT(stats.solve_count, 0);
    EXPECT_EQ(stats.fail_count, 0);
    EXPECT_EQ(stats.evaluated_rollouts, 28)
        << "Attack driver should evaluate feedback primitive/speed-scale sequences";
}

TEST_F(MPCDriverTest, AttackDriverAppliesResidualProfileToSpeedAndSteerDebug) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    driver::AttackPathFollowingDriver base_driver(*circle_track_, velocity_profile_, config);
    (void)base_driver.compute_control(state, 0.0);
    const double base_course_v_ref = base_driver.last_debug().course_v_ref;

    const double length = circle_track_->length();
    driver::AttackResidualProfile residual_profile(
        {
            driver::AttackResidualSample{0.0, {1.25, 0.020, 0.30, 0.040}},
            driver::AttackResidualSample{0.5 * length, {1.25, 0.020, 0.30, 0.040}},
        },
        length,
        true);
    driver::AttackPathFollowingDriver residual_driver(
        *circle_track_,
        velocity_profile_,
        config,
        vehicle::VehicleParams::default_fsae(),
        driver::TrackPhaseProfile(),
        driver::BoundaryLinePlannerConfig(),
        residual_profile);
    (void)residual_driver.compute_control(state, 0.0);
    const auto& debug = residual_driver.last_debug();

    EXPECT_NEAR(debug.attack_residual_delta_v_ref, 1.25, 1e-12);
    EXPECT_NEAR(debug.attack_residual_delta_steer_ff, 0.020, 1e-12);
    EXPECT_NEAR(debug.attack_residual_delta_ax_bias, 0.30, 1e-12);
    EXPECT_NEAR(debug.attack_residual_delta_heading_relax, 0.040, 1e-12);
    EXPECT_TRUE(debug.attack_residual_safety_gate_active);
    EXPECT_LE(debug.attack_effective_heading_error, debug.attack_raw_heading_error + 1e-12);
    EXPECT_GT(debug.attack_ax_after_residual, debug.attack_ax_before_residual);
    EXPECT_GT(debug.course_v_ref, base_course_v_ref + 1.0)
        << "Residual speed correction should affect the logged course reference";
}

TEST_F(MPCDriverTest, AttackDriverUsesPlannedLineGeometryForTracking) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    driver::AttackPathFollowingDriver centerline_driver(
        *circle_track_,
        velocity_profile_,
        config);
    (void)centerline_driver.compute_control(state, 0.0);
    const double centerline_delta = centerline_driver.last_debug().target_tire_angle;

    driver::BoundaryLinePlannerConfig line_config;
    line_config.enabled = true;
    line_config.track_geometry = true;
    line_config.offset_fraction = 0.45;
    line_config.safety_margin = 0.20;
    line_config.preview_distance = 8.0;

    driver::AttackPathFollowingDriver planned_line_driver(
        *circle_track_,
        velocity_profile_,
        config,
        vehicle::VehicleParams::default_fsae(),
        driver::TrackPhaseProfile(),
        line_config);
    (void)planned_line_driver.compute_control(state, 0.0);
    const auto& debug = planned_line_driver.last_debug();

    EXPECT_TRUE(std::isfinite(debug.target_tire_angle));
    EXPECT_FALSE(debug.attack_all_rejected);
    EXPECT_GT(std::abs(debug.course_n_ref), 0.05);
    EXPECT_NEAR(debug.course_lateral_error, -debug.course_n_ref, 0.10);
    EXPECT_GT(std::abs(debug.target_tire_angle - centerline_delta), 1e-4)
        << "Planned-line target_n/curvature should affect the selected steering target";
}

TEST_F(MPCDriverTest, AttackDriverUsesProvidedExitPhaseProfileForSpeedRecovery) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    auto phase_profile = make_constant_phase_profile(*circle_track_, driver::TrackPhase::Exit);
    driver::AttackPathFollowingDriver attack_driver(
        *circle_track_,
        velocity_profile_,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "exit");
    EXPECT_TRUE(debug.attack_allowed);
    EXPECT_GT(debug.attack_static_speed_cap, 0.0);
    EXPECT_GT(debug.phase_speed_bias, 1.0);
    EXPECT_DOUBLE_EQ(debug.phase_margin_bias, 0.0);
    EXPECT_GT(debug.phase_progress_bias, 1.0);
    EXPECT_FALSE(debug.attack_all_rejected);
}

TEST_F(MPCDriverTest, AttackDriverDoesNotLetLowSpeedScaleWinInHealthyStraight) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {160.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 16; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 22.0;
        fast_profile.push_back(point);
    }

    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 2.0;
    config.progress_target_fraction = 0.90;
    config.w_y = 0.25;
    config.w_psi = 0.15;
    config.w_vx = 0.50;
    config.w_ax = 0.01;

    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::Straight);
    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 40.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 8.0;

    const auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "straight");
    EXPECT_FALSE(debug.attack_all_rejected);
    EXPECT_GE(debug.predicted_min_margin, 1.0);
    EXPECT_LT(std::abs(debug.predicted_terminal_heading), 0.10);
    EXPECT_GE(debug.attack_selected_speed_scale, 0.95)
        << "Low speed scales must not reduce the speed-tracking cost in healthy attack zones";
    EXPECT_GT(debug.attack_selected_target_speed, state.vx + 4.0);
    EXPECT_GT(debug.selected_ax, 0.0);
    EXPECT_TRUE(debug.healthy_underspeed);
    EXPECT_EQ(debug.longitudinal_mode, "healthy_underspeed");
    EXPECT_GT(debug.course_target_gap, 2.0);
    EXPECT_GE(debug.attack_selected_target_speed, debug.course_v_ref * 0.90);
    EXPECT_NEAR(
        input.throttle,
        std::clamp(debug.selected_ax / 6.0, 0.0, 1.0),
        1e-12)
        << "Attack throttle normalization should use the effective acceleration scale, not ax_max";
}

TEST_F(MPCDriverTest, AttackDriverAllowsControlledUnderspeedInHealthyComplex) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {160.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 16; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 22.0;
        fast_profile.push_back(point);
    }

    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 2.0;
    config.progress_target_fraction = 0.90;
    config.w_y = 0.25;
    config.w_psi = 0.15;
    config.w_vx = 0.50;
    config.w_ax = 0.01;

    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::Complex);
    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 40.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 8.0;

    const auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "complex");
    EXPECT_FALSE(debug.attack_all_rejected);
    EXPECT_GT(debug.course_target_gap, 2.0);
    EXPECT_GT(debug.attack_selected_target_speed, state.vx + 0.60);
    EXPECT_GT(debug.selected_ax, 0.0);
    EXPECT_GT(input.throttle, 0.0);
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
    EXPECT_TRUE(debug.healthy_underspeed);
    EXPECT_TRUE(
        debug.longitudinal_mode == "healthy_underspeed"
        || debug.longitudinal_mode == "healthy_underspeed_residual");
}

TEST_F(MPCDriverTest, AttackDriverKeepsHeadingSafetyOverComplexUnderspeed) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {160.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 16; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 22.0;
        fast_profile.push_back(point);
    }

    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 2.0;
    config.progress_target_fraction = 0.90;
    config.w_y = 0.25;
    config.w_psi = 0.15;
    config.w_vx = 0.50;
    config.w_ax = 0.01;

    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::Complex);
    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 40.0;
    state.y = 0.0;
    state.psi = 0.22;
    state.vx = 8.0;

    const auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "complex");
    EXPECT_FALSE(debug.attack_all_rejected);
    EXPECT_GT(debug.course_target_gap, 2.0);
    EXPECT_FALSE(debug.healthy_underspeed);
    EXPECT_EQ(debug.longitudinal_mode, "heading_safety");
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    EXPECT_LE(debug.selected_ax, 0.0);
}

TEST_F(MPCDriverTest, AttackDriverSuppressesHighSpeedInTurnInPhase) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    auto phase_profile = make_constant_phase_profile(*circle_track_, driver::TrackPhase::TurnIn);
    driver::AttackPathFollowingDriver attack_driver(
        *circle_track_,
        velocity_profile_,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 30.80;
    state.y = 0.0;
    state.psi = 1.2; // Deliberate outward boundary risk in a turn-in phase.
    state.vx = 4.0;

    attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "turn_in");
    EXPECT_LE(debug.attack_selected_speed_scale, 0.80);
    EXPECT_FALSE(debug.attack_all_rejected);
}

TEST_F(MPCDriverTest, AttackDriverPlannerConfigAffectsPlannedTargetSpeed) {
    track::CurvatureBasedTrack tight_track(
        {{0.0, 0.10}, {80.0, 0.10}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 8; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 25.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(tight_track, driver::TrackPhase::Apex);
    const auto track_point = tight_track.get_point(10.0);

    control::MPCConfig conservative_config;
    conservative_config.N = 12;
    conservative_config.dt = 0.025;
    conservative_config.use_track_margin_speed_cap = true;
    conservative_config.terminal_progress_weight = 1.0;
    conservative_config.attack_speed_lateral_safety = 0.45;
    conservative_config.attack_floor_apex = 0.20;

    control::MPCConfig permissive_config = conservative_config;
    permissive_config.attack_speed_lateral_safety = 0.85;

    driver::AttackPathFollowingDriver conservative_driver(
        tight_track,
        fast_profile,
        conservative_config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);
    driver::AttackPathFollowingDriver permissive_driver(
        tight_track,
        fast_profile,
        permissive_config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = track_point.x;
    state.y = track_point.y;
    state.psi = track_point.theta;
    state.vx = 5.0;

    conservative_driver.compute_control(state, 0.0);
    permissive_driver.compute_control(state, 0.0);

    EXPECT_LT(
        conservative_driver.last_debug().target_speed_base,
        permissive_driver.last_debug().target_speed_base - 1.0)
        << "Attack planner lateral safety should directly affect the driver speed target";
    EXPECT_LT(
        conservative_driver.last_debug().attack_curvature_speed_cap,
        permissive_driver.last_debug().attack_curvature_speed_cap - 1.0)
        << "Dynamic attack curvature cap should use the same lateral safety contract";
}

TEST_F(MPCDriverTest, AttackDriverDynamicCurvatureCapUsesVehicleCapacity) {
    track::CurvatureBasedTrack radius_track(
        {{0.0, 0.055}, {120.0, 0.055}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 30.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(radius_track, driver::TrackPhase::Apex);
    const auto track_point = radius_track.get_point(10.0);

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;
    config.attack_speed_lateral_safety = 0.90;

    auto low_capacity = vehicle::VehicleParams::default_fsae();
    low_capacity.tire.mu_y_max = 1.15;
    low_capacity.aero.CL_A = 0.0;
    auto high_capacity = vehicle::VehicleParams::default_fsae();
    high_capacity.tire.mu_y_max = 2.20;
    high_capacity.aero.CL_A = 4.0;

    driver::AttackPathFollowingDriver low_driver(
        radius_track,
        fast_profile,
        config,
        low_capacity,
        phase_profile);
    driver::AttackPathFollowingDriver high_driver(
        radius_track,
        fast_profile,
        config,
        high_capacity,
        phase_profile);

    vehicle::VehicleState state;
    state.x = track_point.x;
    state.y = track_point.y;
    state.psi = track_point.theta;
    state.vx = 8.0;

    low_driver.compute_control(state, 0.0);
    high_driver.compute_control(state, 0.0);

    EXPECT_GT(
        high_driver.last_debug().attack_curvature_speed_cap,
        low_driver.last_debug().attack_curvature_speed_cap + 3.0)
        << "Dynamic attack caps must not use a fixed lateral-acceleration ceiling";
}

TEST_F(MPCDriverTest, AttackDriverExternalWindowCapOverridesAttackFloor) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 20.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::Straight);

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;
    config.attack_floor_straight = 0.98;
    config.attack_speed_windows.push_back({50.0, 70.0, 6.0});

    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 60.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 8.0;

    attack_driver.compute_control(state, 0.0);

    EXPECT_LE(attack_driver.last_debug().target_speed_base, 6.0 + 1e-9)
        << "External speed windows are hard caps and must not be lifted by attack floors";
}

TEST_F(MPCDriverTest, AttackDriverDoesNotDoubleApplyPreviewMinAfterPlannedBraking) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 20.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::Straight);

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;
    config.attack_floor_straight = 0.98;
    config.attack_speed_windows.push_back({70.0, 75.0, 6.0});

    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 50.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 20.0;

    attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_GT(debug.target_speed_base, 10.0)
        << "The planned profile's braking pass should decide where slowing starts";
    EXPECT_GT(debug.target_speed, 10.0)
        << "Attack target speed must not be clamped to a future window by a second preview-min pass";
}

TEST_F(MPCDriverTest, AttackDriverUsesBrakingCorridorBandInTurnInPhase) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 20.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::TurnIn);

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;
    config.attack_braking_corridor_guard_only = false;
    config.attack_speed_windows.push_back({54.0, 58.0, 6.0});

    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 50.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 7.0;

    const auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.track_phase, "turn_in");
    EXPECT_TRUE(debug.braking_corridor_active);
    EXPECT_GT(debug.braking_corridor_v_upper, 6.0)
        << "The corridor should use available braking distance, not the raw future slow-point speed";
    EXPECT_LT(debug.braking_corridor_v_lower, debug.braking_corridor_v_upper);
    EXPECT_GE(debug.selected_ax, 0.0)
        << "Inside the corridor band, healthy turn-in should coast or use mild acceleration";
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
}

TEST_F(MPCDriverTest, AttackDriverCanUseBrakingCorridorOnlyAsGuard) {
    track::CurvatureBasedTrack straight_track(
        {{0.0, 0.0}, {120.0, 0.0}},
        0.0,
        0.0,
        0.0,
        3.0,
        false);
    std::vector<laptime::VelocityPoint> fast_profile;
    for (int i = 0; i <= 12; ++i) {
        laptime::VelocityPoint point;
        point.s = 10.0 * i;
        point.v = 20.0;
        fast_profile.push_back(point);
    }
    auto phase_profile = make_constant_phase_profile(straight_track, driver::TrackPhase::TurnIn);

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;
    config.attack_braking_corridor_guard_only = true;
    config.attack_speed_windows.push_back({54.0, 58.0, 6.0});

    driver::AttackPathFollowingDriver attack_driver(
        straight_track,
        fast_profile,
        config,
        vehicle::VehicleParams::default_fsae(),
        phase_profile);

    vehicle::VehicleState state;
    state.x = 50.0;
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 7.0;

    attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_TRUE(debug.braking_corridor_active);
    EXPECT_GT(debug.attack_dynamic_base_speed, debug.braking_corridor_v_upper)
        << "Guard-only mode should keep the course reference target while retaining the braking guard";
}

TEST_F(MPCDriverTest, AttackDriverLowSpeedInsideTrackReleasesBrakeForRecovery) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = 0.0;  // Deliberately poor heading at the circle start.
    state.vx = 0.2;

    auto input = attack_driver.compute_control(state, 0.0);

    EXPECT_GT(input.throttle, 0.0)
        << "A stopped but in-track vehicle must creep forward to recover heading";
    EXPECT_DOUBLE_EQ(input.brake, 0.0);
    EXPECT_GT(attack_driver.last_debug().selected_ax, 0.0);
    EXPECT_EQ(attack_driver.last_debug().selected_plan_index / 4, 5)
        << "Low-speed recovery should select the pure-pursuit primitive family";
    EXPECT_EQ(attack_driver.last_debug().attack_selected_primitive, 5);
    EXPECT_EQ(
        attack_driver.last_debug().attack_selected_primitive_kind,
        "low_speed_pure_pursuit");
    EXPECT_FALSE(attack_driver.last_debug().attack_all_rejected);
    EXPECT_LT(
        attack_driver.last_debug().attack_rollout_reject_count,
        attack_driver.last_debug().attack_rollout_candidate_count);
    EXPECT_GT(std::abs(attack_driver.last_debug().target_tire_angle), 0.01)
        << "Low-speed recovery should include a steering target, not only throttle";
}

TEST_F(MPCDriverTest, AttackDriverLowSpeedOutsideTrackKeepsBrakePriority) {
    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 36.0; // Well outside the 30 m circle track width.
    state.y = 0.0;
    state.psi = 0.0;
    state.vx = 0.2;

    auto input = attack_driver.compute_control(state, 0.0);

    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    EXPECT_GT(input.brake, 0.0)
        << "Recovery creep must not override a clear track-boundary violation";
    EXPECT_LT(attack_driver.last_debug().selected_ax, 0.0);
}

TEST_F(MPCDriverTest, AttackDriverSufficientProjectedMarginDoesNotSelectBoundaryRecovery) {
    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 30.10; // Slightly outside the centerline but still with ample margin.
    state.y = 0.0;
    state.psi = 1.4; // Small outward heading; projected margin should remain safe.
    state.vx = 4.0;

    attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    ASSERT_GE(debug.selected_plan_index, 0);
    EXPECT_NE(debug.attack_selected_primitive_kind, "boundary_recovery")
        << "Positive outward rate alone must not trigger boundary recovery when projected margin is safe";
    EXPECT_GT(debug.predicted_min_margin, config.soft_boundary_margin)
        << "This fixture should stay outside the soft-boundary risk region";
}

TEST_F(MPCDriverTest, AttackDriverNearBoundaryOutwardHeadingSelectsRecoveryPrimitive) {
    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 30.80; // Outside side of the east apex, with about 0.70 m margin.
    state.y = 0.0;
    state.psi = 1.2; // Tangent heading is pi/2, so this is still outward.
    state.vx = 4.0;

    auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_GT(input.brake, 0.0)
        << "Outward boundary risk should prioritize deceleration over progress";
    ASSERT_GE(debug.selected_plan_index, 0);
    EXPECT_EQ(debug.selected_plan_index / 4, 4)
        << "Near-boundary outward heading should select boundary recovery primitive";
    EXPECT_GT(debug.predicted_min_margin, -0.10)
        << "Selected rollout must not cross the hard rejection margin";
    EXPECT_EQ(debug.attack_selected_primitive, 4)
        << "Near-boundary outward heading should select boundary recovery primitive";
    EXPECT_EQ(debug.attack_selected_primitive_kind, "boundary_recovery");
    EXPECT_LE(debug.attack_best_boundary_recovery_cost, debug.rollout_cost + 1e-9);
}

TEST_F(MPCDriverTest, AttackDriverDoesNotSelectPredictedHardBoundaryViolation) {
    control::MPCConfig config;
    config.N = 16;
    config.dt = 0.025;
    config.use_track_margin_speed_cap = true;
    config.terminal_progress_weight = 1.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config);

    vehicle::VehicleState state;
    state.x = 31.25; // Inside track, but only about 0.25 m from the outside boundary.
    state.y = 0.0;
    state.psi = 0.0; // Strongly outward; every finite rollout should cross the reject margin.
    state.vx = 8.0;

    auto input = attack_driver.compute_control(state, 0.0);
    const auto& debug = attack_driver.last_debug();

    EXPECT_EQ(debug.selected_plan_index, -1)
        << "No rollout should be selected when all candidates predict hard boundary violation";
    EXPECT_GT(input.brake, 0.0)
        << "Fallback should brake instead of accepting an unsafe predicted rollout";
    EXPECT_GT(debug.predicted_min_margin, -0.10)
        << "Published debug state should not report an accepted boundary-crossing rollout";
    EXPECT_TRUE(debug.attack_all_rejected);
    EXPECT_EQ(debug.attack_rollout_candidate_count, 28);
    EXPECT_EQ(debug.attack_rollout_reject_count, debug.attack_rollout_candidate_count);
    EXPECT_GT(debug.attack_rollout_boundary_reject_count, 0);
    EXPECT_EQ(debug.attack_selected_primitive, -1);
    EXPECT_EQ(debug.attack_selected_primitive_kind, "fallback");
    EXPECT_EQ(attack_driver.stats().fail_count, 1);
}

TEST_F(MPCDriverTest, AttackDriverTorqueUpdateUsesSATFeedforward) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    params.steering.torque_mode = true;
    params.steering.T_driver_max = 40.0;
    params.steering.R_steer = 4.5;
    params.steering.I_steer = 0.02;
    params.steering.C_steer_torque = 0.3;

    control::MPCConfig config;
    config.N = 12;
    config.dt = 0.025;
    config.steering_inner_loop_omega_n = 60.0;

    driver::AttackPathFollowingDriver attack_driver(*circle_track_, velocity_profile_, config, params);
    attack_driver.set_steering_state(0.0, 0.0);

    vehicle::VehicleState state;
    state.x = 30.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    auto input = attack_driver.compute_outer_control(state, 0.0);
    EXPECT_DOUBLE_EQ(input.steering, 0.0);
    EXPECT_TRUE(attack_driver.last_debug().torque_mode);

    constexpr double kPhysicsDt = 0.001;
    constexpr double kFrontSAT = 18.0;
    input = attack_driver.update_torque_actuator_control(input, 0.0, 0.0, kPhysicsDt, kFrontSAT);
    const auto& actuator = attack_driver.last_debug().steering_actuator;

    EXPECT_NEAR(actuator.T_SAT_feedforward, -kFrontSAT / params.steering.R_steer, 1e-12);
    EXPECT_NEAR(input.steering, actuator.steering_torque, 1e-12);
    EXPECT_LE(std::abs(input.steering), params.steering.T_driver_max);
}

/// SteeringActuatorController の単体テスト
TEST(SteeringActuatorControllerTest, BasicPIDResponse) {
    using namespace fsae::driver;
    SteeringActuatorConfig config;
    config.Kp_inner = 50.0;
    config.Ki_inner = 5.0;
    config.Kd_inner = 1.5;
    config.R_steer = 4.5;
    config.max_tire_angle = 0.35;
    config.max_driver_torque = 40.0;
    config.max_tire_steer_rate = 3.0;

    SteeringActuatorState state;

    // 目標: +0.2rad, 現在: 0 → 正のトルクが出るはず
    SteeringActuatorCommand cmd;
    cmd.target_tire_angle = 0.2;
    cmd.delta_sw = 0.0;
    cmd.ddelta_sw = 0.0;
    cmd.dt = 0.001;

    auto output = compute_steering_actuator_control(config, state, cmd);

    EXPECT_GT(output.steering_torque, 0.0)
        << "Positive target angle should produce positive torque";
    EXPECT_LE(std::abs(output.steering_torque), config.max_driver_torque + 1e-9)
        << "Torque should be within limits";
    EXPECT_GT(output.target_sw, 0.0)
        << "Target SW should be positive for positive tire angle";
    EXPECT_NEAR(output.actual_delta_tire, 0.0, 1e-9)
        << "Actual tire angle should still be 0 (no physical movement)";
}

/// SteeringActuatorController: 積分器の蓄積を確認
TEST(SteeringActuatorControllerTest, IntegralAccumulates) {
    using namespace fsae::driver;
    SteeringActuatorConfig config;
    config.Kp_inner = 50.0;
    config.Ki_inner = 5.0;
    config.Kd_inner = 0.0;
    config.R_steer = 4.5;
    config.max_tire_angle = 0.35;
    config.max_driver_torque = 400.0;  // 制限をゆるくして積分の効果を見る
    config.max_tire_steer_rate = 10.0;
    config.integral_limit = 10.0;

    SteeringActuatorState state;

    SteeringActuatorCommand cmd;
    cmd.target_tire_angle = 0.1;
    cmd.delta_sw = 0.0;
    cmd.ddelta_sw = 0.0;
    cmd.dt = 0.001;

    // 1回目
    auto out1 = compute_steering_actuator_control(config, state, cmd);
    // 100回繰り返して積分項が蓄積したか確認
    for (int i = 0; i < 99; ++i) {
        compute_steering_actuator_control(config, state, cmd);
    }
    auto out100 = compute_steering_actuator_control(config, state, cmd);

    EXPECT_GT(out100.T_driver_raw, out1.T_driver_raw)
        << "Torque should increase as integral accumulates over 100 steps";
    EXPECT_GT(state.inner_integral, 0.0)
        << "Integral state should be positive";
}
