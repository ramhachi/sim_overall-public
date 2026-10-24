/// @file test_velocity_profile_io.cpp
/// @brief Contract tests for frozen station-based velocity profiles.

#include <gtest/gtest.h>

#include "fsae/laptime/velocity_profile_io.hpp"
#include "fsae/track/track_model.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

using namespace fsae;

namespace {

class TemporaryJsonFile {
public:
    TemporaryJsonFile() {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() /
            ("fsae_velocity_profile_" + std::to_string(nonce) + ".json");
    }
    ~TemporaryJsonFile() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

std::vector<laptime::VelocityPoint> sample_closed_profile(double length) {
    return {
        {0.0, 8.0, 0.5, 3.2, 0.05, 15.0},
        {0.5 * length, 12.0, -0.4, -2.0, -0.04,
         std::numeric_limits<double>::infinity()},
        {length, 8.0, 0.5, 3.2, 0.05, 15.0},
    };
}

} // namespace

TEST(VelocityProfileIoTest, RoundTripPreservesProfileAndFingerprint) {
    const auto track = track::create_circle_track(20.0, 4.5);
    const auto points = sample_closed_profile(track.length());
    const auto artifact = laptime::make_velocity_profile_artifact(points, "circle", track);
    TemporaryJsonFile file;

    laptime::save_velocity_profile_artifact(artifact, file.path().string());
    const auto loaded = laptime::load_velocity_profile_artifact(
        file.path().string(), "circle", track);

    ASSERT_EQ(loaded.points.size(), points.size());
    EXPECT_EQ(loaded.profile_fingerprint, artifact.profile_fingerprint);
    EXPECT_EQ(loaded.track_geometry_fingerprint, artifact.track_geometry_fingerprint);
    for (std::size_t index = 0; index < points.size(); ++index) {
        EXPECT_DOUBLE_EQ(loaded.points[index].s, points[index].s);
        EXPECT_DOUBLE_EQ(loaded.points[index].v, points[index].v);
        EXPECT_DOUBLE_EQ(loaded.points[index].ax, points[index].ax);
        EXPECT_DOUBLE_EQ(loaded.points[index].ay, points[index].ay);
        EXPECT_DOUBLE_EQ(loaded.points[index].kappa, points[index].kappa);
    }
    EXPECT_TRUE(std::isinf(loaded.points[1].steering_speed_cap));
}

TEST(VelocityProfileIoTest, RejectsInvalidStationAndSpeedContracts) {
    const auto track = track::create_circle_track(20.0, 4.5);
    auto points = sample_closed_profile(track.length());
    points[1].s = points[0].s;
    EXPECT_THROW(
        laptime::validate_velocity_profile_points(points, track.length(), true),
        std::runtime_error);

    points = sample_closed_profile(track.length());
    points[1].v = -1.0;
    EXPECT_THROW(
        laptime::validate_velocity_profile_points(points, track.length(), true),
        std::runtime_error);

    points = sample_closed_profile(track.length());
    points[1].ax = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        laptime::validate_velocity_profile_points(points, track.length(), true),
        std::runtime_error);

    points = sample_closed_profile(track.length());
    points[1].steering_speed_cap = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        laptime::validate_velocity_profile_points(points, track.length(), true),
        std::runtime_error);
}

TEST(VelocityProfileIoTest, RejectsClosedLoopSeamDiscontinuity) {
    const auto track = track::create_circle_track(20.0, 4.5);
    auto points = sample_closed_profile(track.length());
    points.back().v += 0.1;
    EXPECT_THROW(
        laptime::make_velocity_profile_artifact(points, "circle", track),
        std::runtime_error);
}

TEST(VelocityProfileIoTest, RejectsTrackNameLengthAndGeometryMismatch) {
    const auto track = track::create_circle_track(20.0, 4.5);
    const auto artifact = laptime::make_velocity_profile_artifact(
        sample_closed_profile(track.length()), "circle", track);
    TemporaryJsonFile file;
    laptime::save_velocity_profile_artifact(artifact, file.path().string());

    EXPECT_THROW(
        laptime::load_velocity_profile_artifact(file.path().string(), "circle_cw", track),
        std::runtime_error);

    const auto different_length = track::create_circle_track(21.0, 4.5);
    EXPECT_THROW(
        laptime::load_velocity_profile_artifact(
            file.path().string(), "circle", different_length),
        std::runtime_error);

    const auto different_width = track::create_circle_track(20.0, 5.0);
    EXPECT_THROW(
        laptime::load_velocity_profile_artifact(
            file.path().string(), "circle", different_width),
        std::runtime_error);
}

TEST(VelocityProfileIoTest, DetectsArtifactPointTampering) {
    const auto track = track::create_circle_track(20.0, 4.5);
    auto artifact = laptime::make_velocity_profile_artifact(
        sample_closed_profile(track.length()), "circle", track);
    auto root = laptime::velocity_profile_artifact_to_json(artifact);
    root["points"][1]["speed_mps"] = 9.0;
    TemporaryJsonFile file;
    {
        std::ofstream output(file.path());
        output << root.dump(2) << '\n';
    }

    EXPECT_THROW(
        laptime::load_velocity_profile_artifact(file.path().string(), "circle", track),
        std::runtime_error);
}
