/// @file test_velocity_profile_ggv.cpp
/// @brief Lap planner physics checks (Issues #23, #24, #26-#29).

#include <gtest/gtest.h>

#include "fsae/laptime/velocity_profile.hpp"
#include "fsae/track/track_model.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace fsae;

namespace {

laptime::VehiclePerformance base_perf() {
    laptime::VehiclePerformance perf;
    perf.max_speed = 35.0;
    perf.mass = 280.0;
    perf.mu_lat = 1.6;
    perf.cl_a = 2.5;
    perf.cd_a = 1.0;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.6 * 9.81;
    return perf;
}

/// Closed loop: 60 m straight, 180° R=8 hairpin, 60 m straight, 180° R=20 sweeper.
track::CurvatureBasedTrack stadium_track() {
    std::vector<std::pair<double, double>> k;
    const double l1 = 60.0, l2 = M_PI * 8.0, l3 = 60.0, l4 = M_PI * 20.0;
    for (double s = 0.0; s <= l1 + l2 + l3 + l4 + 1e-9; s += 0.5) {
        double kap = 0.0;
        if (s > l1 && s <= l1 + l2) kap = 1.0 / 8.0;
        else if (s > l1 + l2 + l3) kap = 1.0 / 20.0;
        k.emplace_back(s, kap);
    }
    return track::CurvatureBasedTrack(k, 0.0, 0.0, 0.0, 4.0, true);
}

/// Max longitudinal accel implied by the profile vs the limits it was planned with.
void expect_realizable(const std::vector<laptime::VelocityPoint>& p,
                       const laptime::VehiclePerformance& perf) {
    const double drag_max = 0.5 * perf.rho * perf.cd_a * perf.max_speed * perf.max_speed / perf.mass;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) {
        EXPECT_LE(p[i].ax, perf.max_lon_accel + 1e-6) << "s=" << p[i].s;
        EXPECT_GE(p[i].ax, -(perf.max_lon_decel + drag_max) - 1e-6) << "s=" << p[i].s;
    }
}

}  // namespace

// Issue #29: closed-loop seam is periodic and realizable (no post-pass linear blend).
TEST(LapPlanner, ClosedLoopSeamIsPeriodicAndRealizable) {
    auto perf = base_perf();
    auto track = stadium_track();
    ASSERT_TRUE(track.is_closed_loop());
    laptime::VelocityProfileGenerator gen(perf);
    const auto p = gen.generate(track, 1.0, 6.0);
    ASSERT_GT(p.size(), 10u);
    EXPECT_NEAR(p.front().v, p.back().v, 1e-9);
    expect_realizable(p, perf);
    // The seam sits mid-sweeper exit → straight, so the car must already be fast at s=0.
    EXPECT_GT(p.front().v, 10.0);
}

// Issue #29: open track end speed is a backward-pass boundary, not a 10 m linear taper.
TEST(LapPlanner, OpenTrackTerminalSpeedIsBackwardBoundary) {
    auto perf = base_perf();
    auto track = track::create_acceleration_track(75.0);
    ASSERT_FALSE(track.is_closed_loop());
    laptime::VelocityProfileGenerator gen(perf);
    const auto free_end = gen.generate(track, 1.0, 0.0);
    const auto stop_end = gen.generate(track, 1.0, 0.0, 5.0);
    EXPECT_GT(free_end.back().v, 20.0);  // no forced slowdown at the finish line
    EXPECT_NEAR(stop_end.back().v, 5.0, 1e-9);
    expect_realizable(stop_end, perf);
}
