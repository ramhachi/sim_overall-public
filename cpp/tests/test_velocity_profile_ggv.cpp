/// @file test_velocity_profile_ggv.cpp
/// @brief Lap planner physics checks (Issues #23, #24, #26-#29).

#include <gtest/gtest.h>

#include "fsae/laptime/velocity_profile.hpp"
#include "fsae/track/track_model.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
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

// Issue #23: a GGV tabulated from the closed-form friction-ellipse model reproduces the
// closed-form profile within 2 % (bilinear interpolation, JSON round trip).
TEST(LapPlanner, SyntheticEllipseGGVReproducesClosedForm) {
    auto perf = base_perf();
    const double g = 9.81;
    auto drag = [&](double v) { return 0.5 * perf.rho * perf.cd_a * v * v / perf.mass; };
    laptime::GGVMap ggv;
    ggv.source = "synthetic friction ellipse";
    for (double v = 0.0; v <= 40.0 + 1e-9; v += 1.0) ggv.v.push_back(v);
    for (double r = 0.0; r <= 1.0 + 1e-9; r += 0.05) ggv.r.push_back(std::min(r, 1.0));
    for (double v : ggv.v) {
        ggv.ay_max.push_back(perf.mu_lat * (perf.mass * g + 0.5 * perf.rho * perf.cl_a * v * v) / perf.mass);
        std::vector<double> acc, brk;
        for (double r : ggv.r) {
            const double e = std::sqrt(std::max(0.0, 1.0 - r * r));
            acc.push_back(std::max(0.0, perf.max_lon_accel * e - drag(v)));
            brk.push_back(perf.max_lon_decel * e + drag(v));
        }
        ggv.ax_max.push_back(acc);
        ggv.ax_min.push_back(brk);
    }
    const auto loaded = laptime::GGVMap::from_json(ggv.to_json());

    auto track = stadium_track();
    const auto closed_form = laptime::VelocityProfileGenerator(perf).generate(track);
    auto ggv_perf = perf;
    ggv_perf.ggv = std::make_shared<const laptime::GGVMap>(loaded);
    const auto tabulated = laptime::VelocityProfileGenerator(ggv_perf).generate(track);

    ASSERT_EQ(closed_form.size(), tabulated.size());
    double worst = 0.0;
    for (std::size_t i = 0; i < closed_form.size(); ++i) {
        worst = std::max(worst, std::abs(tabulated[i].v / closed_form[i].v - 1.0));
    }
    const double t_cf = laptime::compute_lap_time(closed_form);
    const double t_ggv = laptime::compute_lap_time(tabulated);
    EXPECT_LT(worst, 0.02) << "pointwise speed error";
    EXPECT_NEAR(t_ggv / t_cf, 1.0, 0.02) << "closed-form " << t_cf << " s vs GGV " << t_ggv << " s";
}

// Issue #26: one explicit margin, applied once to the lateral limit (v ∝ sqrt(m)); legacy
// factors default to 1.0.
TEST(LapPlanner, SingleFrictionMarginAppliedOnce) {
    laptime::VehiclePerformance defaults;
    EXPECT_DOUBLE_EQ(defaults.friction_margin, laptime::kDefaultFrictionMargin);
    EXPECT_DOUBLE_EQ(defaults.safety_factor, 1.0);
    EXPECT_DOUBLE_EQ(defaults.steering_speed_safety_factor, 1.0);

    auto perf = base_perf();
    perf.cl_a = 0.0;
    auto track = track::create_circle_track(20.0);
    perf.friction_margin = 1.0;
    const double v_full = laptime::VelocityProfileGenerator(perf).generate(track)[50].v;
    perf.friction_margin = 0.81;
    const double v_m = laptime::VelocityProfileGenerator(perf).generate(track)[50].v;
    EXPECT_NEAR(v_full, std::sqrt(perf.mu_lat * 9.81 * 20.0), 1e-6);
    EXPECT_NEAR(v_m / v_full, 0.9, 1e-6);
}

// Issue #24: standing start — no min_speed floor on the straight, no warmup ramp.
TEST(LapPlanner, AccelerationEventIsStandingStart) {
    auto perf = base_perf();
    perf.cd_a = 0.0;  // isolate the launch from drag
    auto track = track::create_acceleration_track(75.0);
    laptime::VelocityProfileGenerator gen(perf);
    const auto p = gen.generate(track, 1.0, 0.0);
    ASSERT_GT(p.size(), 70u);
    EXPECT_DOUBLE_EQ(p.front().v, 0.0);
    // v(s) = sqrt(2 a s) from rest: no 0.6 g warmup ramp, no 5 m/s floor
    for (double s : {1.0, 5.0, 10.0}) {
        const auto& q = p[static_cast<std::size_t>(s)];
        EXPECT_NEAR(q.v, std::sqrt(2.0 * perf.max_lon_accel * s), 1e-6) << "s=" << s;
    }
    // ax continuous through 0→5 m/s (constant traction-limited launch)
    for (std::size_t i = 0; i < 8; ++i) {
        EXPECT_NEAR(p[i].ax, perf.max_lon_accel, 1e-6) << "i=" << i;
    }
}
