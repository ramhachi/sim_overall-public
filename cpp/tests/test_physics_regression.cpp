/// @file test_physics_regression.cpp
/// @brief SAE J670e sign, symmetry, and contact-kinematics regression tests.

#include <gtest/gtest.h>
#include "fsae/vehicle/body_moments.hpp"
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include <array>
#include <cmath>

using namespace fsae::vehicle;

namespace {

std::array<Eigen::Vector3d, 4> zero_positions() {
    return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

std::array<double, 4> zeros() {
    return {0.0, 0.0, 0.0, 0.0};
}

} // namespace

TEST(BodyMoments, RightTurnLateralForceRollsLeft) {
    auto pos = zero_positions();
    pos[0] = Eigen::Vector3d(0.7, -0.6, 0.0);
    pos[1] = Eigen::Vector3d(0.7, 0.6, 0.0);
    auto fx = zeros();
    auto fy = zeros();
    fy[0] = 1000.0;
    fy[1] = 1000.0;
    auto fs = zeros();
    auto roll_arm = std::array<double, 4>{0.30, 0.30, 0.30, 0.30};
    auto pitch_arm = zeros();
    auto mz = zeros();

    const auto moments = compute_body_moment_contributions(
        pos, fx, fy, fs, roll_arm, pitch_arm, mz);
    // Contact force below the CG: Mx = -r_z * Fy for each wheel.
    EXPECT_NEAR(moments.Mx, -600.0, 1e-12);
    EXPECT_NEAR(moments.My, 0.0, 1e-12);
}

TEST(BodyMoments, ForwardForceAtContactPatchPitchesNoseUp) {
    auto pos = zero_positions();
    pos[0] = Eigen::Vector3d(1.0, 0.0, 0.0);
    auto fx = zeros();
    fx[0] = 1000.0;
    auto fy = zeros();
    auto fs = zeros();
    auto roll_arm = zeros();
    auto pitch_arm = std::array<double, 4>{0.25, 0.25, 0.25, 0.25};
    auto mz = zeros();

    const auto moments = compute_body_moment_contributions(
        pos, fx, fy, fs, roll_arm, pitch_arm, mz);
    EXPECT_NEAR(moments.My, 250.0, 1e-12);
}

TEST(BodyMoments, LeftRightMirrorFlipsRollAndYaw) {
    auto pos = zero_positions();
    pos[0] = Eigen::Vector3d(0.7, -0.6, 0.0);
    pos[1] = Eigen::Vector3d(0.7, 0.6, 0.0);
    auto fx = zeros();
    auto fy_right = zeros();
    fy_right[0] = 1000.0;
    fy_right[1] = 1000.0;
    auto fy_left = zeros();
    fy_left[0] = -1000.0;
    fy_left[1] = -1000.0;
    auto fs = zeros();
    auto roll_arm = std::array<double, 4>{0.3, 0.3, 0.3, 0.3};
    auto pitch_arm = zeros();
    auto mz = zeros();

    const auto right = compute_body_moment_contributions(
        pos, fx, fy_right, fs, roll_arm, pitch_arm, mz);
    const auto left = compute_body_moment_contributions(
        pos, fx, fy_left, fs, roll_arm, pitch_arm, mz);
    EXPECT_NEAR(right.Mx, -left.Mx, 1e-12);
    EXPECT_NEAR(right.Mz, -left.Mz, 1e-12);
}

TEST(BodyMoments, TireMzEntersBodyYaw) {
    auto pos = zero_positions();
    pos[0] = Eigen::Vector3d(0.7, -0.6, 0.0);
    auto fx = zeros();
    auto fy = zeros();
    auto fs = zeros();
    auto roll_arm = zeros();
    auto pitch_arm = zeros();
    auto mz = zeros();
    mz[0] = 12.0;

    const auto moments = compute_body_moment_contributions(
        pos, fx, fy, fs, roll_arm, pitch_arm, mz);
    EXPECT_NEAR(moments.Mz, 12.0, 1e-12);
}

TEST(ContactKinematics, IncludesPitchAndRollVelocityTerms) {
    const Eigen::Vector3d v(10.0, 1.0, 0.0);
    const Eigen::Vector3d omega(0.01, 0.02, 0.10);
    const Eigen::Vector3d r(0.7, 0.6, 0.28);
    const Eigen::Vector3d contact = compute_contact_patch_velocity(v, omega, r);

    EXPECT_NEAR(contact.x(), 10.0 + 0.02 * 0.28 - 0.10 * 0.6, 1e-12);
    EXPECT_NEAR(contact.y(), 1.0 + 0.10 * 0.7 - 0.01 * 0.28, 1e-12);
    EXPECT_NEAR(contact.z(), 0.0 + 0.01 * 0.6 - 0.02 * 0.7, 1e-12);
}

TEST(ArbRegression, AxleRollStiffnessUsesHalfCouple) {
    VehicleParams params = VehicleParams::default_fsae();
    params.suspension.k_arb_f = 1000.0;
    params.suspension.k_arb_r = 0.0;
    params.mass.track_f = 1.2;
    params.mass.track_r = 1.2;
    Vehicle14DOF_New vehicle(params);

    const auto forces = vehicle.compute_arb_forces({0.012, -0.012, 0.0, 0.0});
    const double roll_angle = (0.012 - (-0.012)) / 1.2;
    const double roll_moment = 1.2 * forces[0];
    EXPECT_NEAR(roll_moment, 1000.0 * roll_angle, 1e-9);
    EXPECT_NEAR(forces[0], -forces[1], 1e-12);
}

TEST(EulerIntegration, ClampedHighPitchStaysFinite) {
    VehicleParams params = VehicleParams::default_fsae();
    Vehicle14DOF_New vehicle(params);
    vehicle.state().theta = 1.55;
    vehicle.state().vx = 10.0;
    vehicle.step(0.001, {});
    EXPECT_TRUE(std::isfinite(vehicle.state().theta));
    EXPECT_TRUE(std::isfinite(vehicle.state().q));
    EXPECT_TRUE(std::isfinite(vehicle.state().vx));
}

// #13: wheel gyroscopic moment signs (SAE, forward rolling omega > 0).
TEST(BodyMoments, WheelGyroRollsOutwardAndSteersIntoLean) {
    const double Iw = 0.5, omega = 88.0;
    const auto yaw = compute_wheel_gyro_moment(Iw, omega, 0.0, Eigen::Vector3d(0, 0, 1.0));
    EXPECT_NEAR(yaw.x(), -Iw * omega, 1e-9);  // right turn: roll moment toward the outside (left)
    const auto roll = compute_wheel_gyro_moment(Iw, omega, 0.0, Eigen::Vector3d(0.5, 0, 0));
    EXPECT_NEAR(roll.z(), Iw * omega * 0.5, 1e-9);  // roll right -> kingpin torque steers right
}
