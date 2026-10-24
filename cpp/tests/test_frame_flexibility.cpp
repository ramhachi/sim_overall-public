/// @file test_frame_flexibility.cpp
/// @brief フレーム柔軟性モデルのテスト（Phase A1）

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include <cmath>
#include <algorithm>

using namespace fsae::vehicle;

class FrameFlexibilityTest : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        params = VehicleParams::default_fsae();
        params.suspension.K_chassis = 2000.0;
        params.suspension.C_chassis = 5.0;
        params.suspension.I_twist = 1.5;
    }
};

/// Test 1: 退化テスト - K_chassis→∞で剛体モードと一致
TEST_F(FrameFlexibilityTest, RigidFrameDegeneration) {
    params.suspension.K_chassis = 1e6;

    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    input.steering = 0.2;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;
    double max_phi_twist = 0.0;

    // 車両が加速して定常状態になるまで走行
    for (int i = 0; i < 3000; ++i) {
        vehicle.step(dt, input);
        max_phi_twist = std::max(max_phi_twist, std::abs(vehicle.get_frame_twist()));
    }

    EXPECT_LT(max_phi_twist, 0.005)
        << "Rigid frame should have minimal twist (max: " << max_phi_twist << " rad)";
}

/// Test 2: 柔軟フレームでの測定可能なねじり
TEST_F(FrameFlexibilityTest, MeasurableTwist) {
    params.suspension.K_chassis = 2000.0;

    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    input.steering = 0.2;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 5000; ++i) {
        vehicle.step(dt, input);
    }

    double phi_twist_deg = vehicle.get_frame_twist() * 180.0 / M_PI;
    // This is a nonzero-twist regression guard, not a calibration target.
    EXPECT_GT(std::abs(phi_twist_deg), 0.005) << "Frame should experience measurable twist";
    EXPECT_LT(std::abs(phi_twist_deg), 10.0) << "Frame twist should be realistic (< 10°)";
}

/// Test 3: ロール剛性配分の変化
TEST_F(FrameFlexibilityTest, RollStiffnessDistribution) {
    VehicleParams rigid_params = params;
    rigid_params.suspension.K_chassis = 1e6;
    Vehicle14DOF_New rigid_vehicle(rigid_params);

    VehicleParams flex_params = params;
    flex_params.suspension.K_chassis = 1500.0;
    Vehicle14DOF_New flex_vehicle(flex_params);

    DriverInput input;
    input.steering = 0.3;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 3000; ++i) {
        rigid_vehicle.step(dt, input);
        flex_vehicle.step(dt, input);
    }

    auto [phi_front_flex, phi_rear_flex] = flex_vehicle.get_split_roll_angles();
    auto [phi_front_rigid, phi_rear_rigid] = rigid_vehicle.get_split_roll_angles();

    double roll_diff_flex = std::abs(phi_front_flex - phi_rear_flex);
    double roll_diff_rigid = std::abs(phi_front_rigid - phi_rear_rigid);

    EXPECT_GT(roll_diff_flex, roll_diff_rigid)
        << "Flexible frame should have larger front-rear roll difference"
        << " (flex: " << roll_diff_flex << ", rigid: " << roll_diff_rigid << ")";

    EXPECT_GT(std::abs(flex_vehicle.get_frame_twist()), 5e-5)
        << "Flexible frame should have measurable twist";
}

/// Test 4: 数値安定性
TEST_F(FrameFlexibilityTest, NumericalStability) {
    params.suspension.K_chassis = 2000.0;

    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    input.steering = 0.15;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 10000; ++i) {
        vehicle.step(dt, input);

        double phi_twist = vehicle.get_frame_twist();
        double dphi_twist = vehicle.get_frame_twist_rate();

        ASSERT_FALSE(std::isnan(phi_twist))
            << "Frame twist became NaN at step " << i;
        ASSERT_FALSE(std::isinf(phi_twist))
            << "Frame twist became Inf at step " << i;
        ASSERT_FALSE(std::isnan(dphi_twist))
            << "Frame twist rate became NaN at step " << i;
    }

    SUCCEED() << "10-second simulation completed without NaN/Inf";
}

/// Test 5: 後方互換性
TEST_F(FrameFlexibilityTest, BackwardCompatibility) {
    VehicleParams default_params = VehicleParams::default_fsae();

    Vehicle14DOF_New vehicle(default_params);
    DriverInput input;

    input.steering = 0.2;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;
    double max_phi_twist = 0.0;

    for (int i = 0; i < 3000; ++i) {
        vehicle.step(dt, input);
        max_phi_twist = std::max(max_phi_twist, std::abs(vehicle.get_frame_twist()));
    }

    EXPECT_LT(max_phi_twist, 0.005)
        << "Default rigid-frame assumption should be preserved (max twist: "
        << max_phi_twist << " rad)";
}
