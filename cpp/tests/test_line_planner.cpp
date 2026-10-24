#include <gtest/gtest.h>

#include "fsae/driver/line_planner.hpp"
#include "fsae/track/track_model.hpp"

using namespace fsae;

TEST(LinePlannerTest, DisabledPlannerTargetsCenterline) {
    auto track = track::create_straight_corner_track(20.0, 10.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = false;

    const auto plan = driver::compute_boundary_line_plan(track, 20.0, config);
    EXPECT_DOUBLE_EQ(plan.target_n, 0.0);
    EXPECT_GT(plan.curvature_now, 0.0);
}

TEST(LinePlannerTest, PlansOutsideEntryAndInsideApexForLeftCorner) {
    auto track = track::create_straight_corner_track(20.0, 10.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = true;
    config.safety_margin = 0.25;
    config.offset_fraction = 0.8;
    config.preview_distance = 8.0;
    config.smoothing_distance = 1.0;

    const auto entry = driver::compute_boundary_line_plan(track, 12.0, config);
    const auto apex = driver::compute_boundary_line_plan(track, 25.0, config);

    EXPECT_LT(entry.target_n, -0.2) << "Left-corner entry should start on the outside/right.";
    EXPECT_GT(apex.target_n, 0.2) << "Left-corner apex should move to the inside/left.";
}

TEST(LinePlannerTest, PlansInsideForRightCornerApex) {
    auto track = track::create_circle_cw_track(20.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = true;
    config.safety_margin = 0.25;
    config.offset_fraction = 0.8;
    config.preview_distance = 8.0;
    config.smoothing_distance = 1.0;

    const auto apex = driver::compute_boundary_line_plan(track, 20.0, config);
    EXPECT_LT(apex.target_n, -0.2) << "Right-corner apex should move to the inside/right.";
}

TEST(LinePlannerTest, PlannedLineCurvatureProfileIsFinite) {
    auto track = track::create_straight_corner_track(20.0, 10.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = true;
    config.safety_margin = 0.25;
    config.offset_fraction = 0.4;

    const auto profile = driver::compute_planned_line_curvature_profile(track, config, 1.0);
    ASSERT_GT(profile.size(), 10U);
    bool saw_corner_curvature = false;
    for (const auto& sample : profile) {
        EXPECT_TRUE(std::isfinite(sample.first));
        EXPECT_TRUE(std::isfinite(sample.second));
        saw_corner_curvature = saw_corner_curvature || std::abs(sample.second) > 0.01;
    }
    EXPECT_TRUE(saw_corner_curvature);
}

TEST(LinePlannerTest, PlannedLineGeometryUsesOffsetPathCurvature) {
    auto track = track::create_circle_track(20.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = true;
    config.safety_margin = 0.25;
    config.offset_fraction = 0.8;
    config.preview_distance = 8.0;
    config.smoothing_distance = 1.0;

    const auto geometry = driver::compute_planned_line_geometry(track, 20.0, config);

    EXPECT_GT(geometry.target_n, 0.2) << "Left-corner apex should offset to the inside.";
    EXPECT_NEAR(geometry.heading_offset, 0.0, 0.02);
    EXPECT_GT(geometry.curvature, track.get_curvature(20.0))
        << "Inside offset on a left circle should tighten the planned-line curvature.";
    EXPECT_TRUE(std::isfinite(geometry.curvature));
}

TEST(LinePlannerTest, RespectsSafetyMargin) {
    auto track = track::create_straight_corner_track(20.0, 10.0, 3.0);
    driver::BoundaryLinePlannerConfig config;
    config.enabled = true;
    config.safety_margin = 0.4;
    config.offset_fraction = 1.0;
    config.preview_distance = 8.0;

    const auto plan = driver::compute_boundary_line_plan(track, 25.0, config);
    EXPECT_LE(std::abs(plan.target_n), 1.1 + 1e-9);
}
