/// @file test_phase_a1_a2_integration.cpp
/// @brief 柔軟フレーム（Phase A1）とトルクステアリング（Phase A2）の統合テスト

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include <cmath>
#include <algorithm>

using namespace fsae::vehicle;

class IntegrationTest : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        params = VehicleParams::default_fsae();
        
        // 柔軟フレームの有効化 (Phase A1)
        params.suspension.K_chassis = 2000.0;
        params.suspension.C_chassis = 5.0;
        params.suspension.I_twist = 1.5;
        
        // トルクステアリングの有効化 (Phase A2)
        params.steering.torque_mode = true;
        params.steering.I_steer = 0.05;
        params.steering.R_steer = 4.5;
        params.steering.T_fric = 1.5;
        params.steering.C_steer_torque = 0.5;
        params.steering.T_driver_max = 25.0;
    }
};

/// Test 1: 柔軟フレームとトルクステアリングの統合定常特性
TEST_F(IntegrationTest, FlexibleFrameWithTorqueSteering) {
    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    // ステアリング入力（トルクモード: Nm）とスロットルを定常想定で入力
    input.steering = 15.0;  // 15Nmの左旋回トルク
    input.throttle = 0.4;
    input.gear = 3;

    double dt = 0.001;

    // 定常状態になるまで走行（約4秒）
    for (int i = 0; i < 4000; ++i) {
        vehicle.step(dt, input);
    }

    // 期待: トルク操舵時も柔軟フレーム状態が有限かつ測定可能に応答すること
    double phi_twist = std::abs(vehicle.get_frame_twist());
    EXPECT_GT(phi_twist, 1e-5) << "Frame should have measurable twist";
    EXPECT_LT(phi_twist, 0.02) << "Frame twist should remain bounded";

    // 期待: ステアリングホイール角が計算されていること
    double delta_sw = vehicle.get_steering_wheel_angle();
    EXPECT_GT(std::abs(delta_sw), 0.1) << "Steering wheel should be rotated";
    
    // 期待: 横加速度が適度に出ていること
    // (14DOFのAPIに依存するが、定常旋回中のはず)
}

/// Test 2: ステアリング操作時の遅延とねじれの連成確認
TEST_F(IntegrationTest, DynamicResponse) {
    Vehicle14DOF_New vehicle(params);
    DriverInput input;
    
    input.throttle = 0.3;
    input.gear = 3;
    double dt = 0.001;
    
    // 直進
    input.steering = 0.0;
    for (int i = 0; i < 2000; ++i) vehicle.step(dt, input);
    
    // ステップ入力
    input.steering = 25.0;
    
    // ステップ直後の応答（100ms経過時）
    for (int i = 0; i < 100; ++i) vehicle.step(dt, input);
    
    double phi_twist_early = vehicle.get_frame_twist();
    double delta_sw_early = vehicle.get_steering_wheel_angle();
    
    double max_abs_phi_twist = std::abs(phi_twist_early);

    // さらに500ms経過時
    for (int i = 0; i < 500; ++i) vehicle.step(dt, input);
    
    double phi_twist_late = vehicle.get_frame_twist();
    double delta_sw_late = vehicle.get_steering_wheel_angle();
    max_abs_phi_twist = std::max(max_abs_phi_twist, std::abs(phi_twist_late));
    
    // 時間経過とともにステアリングが切れ込み、フレームねじり状態も有限に応答
    EXPECT_GT(std::abs(delta_sw_late), std::abs(delta_sw_early));
    EXPECT_GT(max_abs_phi_twist, 1e-5);
    EXPECT_LT(max_abs_phi_twist, 0.02);
}
