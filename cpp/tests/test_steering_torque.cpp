/// @file test_steering_torque.cpp
/// @brief ステアリング操舵トルクモデルのテスト（Phase A2）

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include "fsae/vehicle/steering_torque.hpp"
#include <cmath>
#include <algorithm>

using namespace fsae::vehicle;

class SteeringTorqueTest : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        params = VehicleParams::default_fsae();
        // Phase A2パラメータ
        params.steering.torque_mode = true;
        params.steering.I_steer = 0.05;
        params.steering.R_steer = 4.5;
        params.steering.T_fric = 1.5;
        params.steering.C_steer_torque = 0.5;
        params.steering.T_driver_max = 25.0;
    }
};

/// Test 1: トルク平衡関数の単体テスト
TEST(SteeringTorqueFunction, TorqueBalance) {
    double I_steer = 0.05;
    double T_driver = 10.0;
    double T_SAT = 5.0;
    double R_steer = 4.5;
    double delta_sw = 0.1;
    double ddelta_sw = 0.0;
    double T_fric = 1.5;
    double C_steer = 0.5;

    double accel = compute_steering_acceleration(
        I_steer, T_driver, T_SAT, R_steer,
        delta_sw, ddelta_sw, T_fric, C_steer);

    // T_net = T_driver + T_SAT/R_steer - T_fric·sign(0) - C_steer·0
    // T_net = 10 + 5/4.5 - 0 = 11.111
    // accel = 11.111 / 0.05 = 222.22
    EXPECT_NEAR(accel, 222.22, 1.0);
}

/// Test 2: ステアリング比の変換テスト
TEST(SteeringTorqueFunction, SteeringRatioConversion) {
    double R_steer = 4.5;
    double delta_sw = 0.45;  // ハンドル角 [rad]

    double delta_tire = steering_wheel_to_tire_angle(delta_sw, R_steer);
    EXPECT_NEAR(delta_tire, 0.1, 1e-6);  // 0.45 / 4.5 = 0.1

    double delta_sw_back = tire_angle_to_steering_wheel(delta_tire, R_steer);
    EXPECT_NEAR(delta_sw_back, delta_sw, 1e-6);
}

/// Test 3: トルクモードでの車両シミュレーション
TEST_F(SteeringTorqueTest, TorqueModeSimulation) {
    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    // ドライバートルク入力（正: 左旋回）
    input.steering = 10.0;  // トルクモードでは [Nm]
    input.throttle = 0.3;
    input.gear = 3;

    double dt = 0.001;

    // 1秒間走行
    for (int i = 0; i < 1000; ++i) {
        vehicle.step(dt, input);
    }

    // ステアリングホイール角が変化していることを確認
    double delta_sw = vehicle.get_steering_wheel_angle();
    EXPECT_GT(std::abs(delta_sw), 0.01)
        << "Steering wheel should rotate under driver torque";
}

/// Test 4: トルク飽和テスト
TEST_F(SteeringTorqueTest, TorqueSaturation) {
    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    // 過大なトルク入力
    input.steering = 100.0;  // >> T_driver_max (25 Nm)
    input.throttle = 0.2;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 1000; ++i) {
        vehicle.step(dt, input);
    }

    // 飽和により、角加速度は有限値
    double ddelta_sw = vehicle.get_steering_wheel_rate();
    EXPECT_LT(std::abs(ddelta_sw), 1000.0)
        << "Steering rate should be limited by torque saturation";
}

/// Test 5: 角度モード（後方互換性）
TEST_F(SteeringTorqueTest, AngleModeBackwardCompatibility) {
    params.steering.torque_mode = false;  // 角度モード

    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    input.steering = 0.2;  // 角度モードでは [rad]
    input.throttle = 0.4;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 1000; ++i) {
        vehicle.step(dt, input);
    }

    // 角度モードではステアリングホイール角は更新されない
    double delta_sw = vehicle.get_steering_wheel_angle();
    EXPECT_NEAR(delta_sw, 0.0, 0.001)
        << "Angle mode should not update steering wheel state";
}

/// Test 6: 数値安定性
TEST_F(SteeringTorqueTest, NumericalStability) {
    Vehicle14DOF_New vehicle(params);
    DriverInput input;

    input.steering = 15.0;
    input.throttle = 0.5;
    input.gear = 3;

    double dt = 0.001;

    for (int i = 0; i < 10000; ++i) {
        vehicle.step(dt, input);

        double delta_sw = vehicle.get_steering_wheel_angle();
        double ddelta_sw = vehicle.get_steering_wheel_rate();

        ASSERT_FALSE(std::isnan(delta_sw))
            << "Steering wheel angle became NaN at step " << i;
        ASSERT_FALSE(std::isinf(delta_sw))
            << "Steering wheel angle became Inf at step " << i;
        ASSERT_FALSE(std::isnan(ddelta_sw))
            << "Steering wheel rate became NaN at step " << i;
    }

    SUCCEED() << "10-second simulation completed without NaN/Inf";
}
