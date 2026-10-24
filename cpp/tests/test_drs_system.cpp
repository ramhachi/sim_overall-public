// tests/test_drs_system.cpp
// DRS (Drag Reduction System) の単体テスト・統合テスト

#include <gtest/gtest.h>
#include "fsae/vehicle/drs_system.hpp"
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include "fsae/physics/aero/aero_map_4d.hpp"
#include <cmath>

using namespace fsae::vehicle;
using namespace fsae::physics::aero;

// ======== ユニットテスト: DRSSystem ========

class DRSSystemTest : public ::testing::Test {
protected:
    DRSConfig config;
    std::unique_ptr<DRSSystem> drs;

    void SetUp() override {
        config.min_speed_for_activation = 20.0;   // 20 m/s
        config.max_lateral_accel = 0.5 * 9.81;    // 約5 m/s²
        config.max_steering_angle = 0.05;          // 0.05 rad
        config.activation_delay = 0.1;             // 0.1 s
        drs = std::make_unique<DRSSystem>(config);
    }

    // 安全な状態でアップデートするヘルパー
    bool update_safe(bool button, double dt = 0.01) {
        double safe_speed = 25.0;   // 20 m/s 以上
        double safe_vx = 25.0;
        double safe_yaw_rate = 0.0; // 横G=0
        double safe_steer = 0.0;    // 舵角=0
        return drs->update(button, safe_speed, safe_vx * safe_yaw_rate, safe_steer, dt);
    }

    // 低速でアップデートするヘルパー
    bool update_slow(bool button, double dt = 0.01) {
        double low_speed = 15.0;    // 20 m/s 未満
        double vx = 15.0;
        double yaw_rate = 0.0;
        double steer = 0.0;
        return drs->update(button, low_speed, vx * yaw_rate, steer, dt);
    }
};

// ======== 1. 初期状態チェック ========
TEST_F(DRSSystemTest, InitialState) {
    // 初期状態はINACTIVE（DRSオフ）
    EXPECT_FALSE(drs->is_active());

    // 統計初期値はゼロ
    auto stats = drs->get_stats();
    EXPECT_EQ(stats.activation_count, 0);
    EXPECT_NEAR(stats.total_active_time, 0.0, 1e-10);
    EXPECT_NEAR(stats.avg_speed_when_active, 0.0, 1e-10);
}

// ======== 2. 起動遅延チェック ========
TEST_F(DRSSystemTest, ActivationDelay) {
    const double dt = 0.001;     // 1ms タイムステップ（浮動小数点問題を避ける）
    const double delay = 0.1;    // 100ms 遅延

    // 遅延前は起動しない: 初回(INACTIVE→ACTIVATING)を含めて遅延分以下はfalse
    // 途中でACTIVEになっていないことを確認
    int steps_before = static_cast<int>((delay / dt) * 0.9);  // 遅延の90%分
    for (int i = 0; i < steps_before; ++i) {
        bool result = update_safe(true, dt);
        EXPECT_FALSE(result) << "Step " << i << ": DRS should still be ACTIVATING (before delay)";
    }
    EXPECT_FALSE(drs->is_active()) << "DRS should not be active before delay completes";

    // 遅延を超えるまで更新し、いつかACTIVEになる
    bool activated = false;
    for (int i = 0; i < 200; ++i) {  // 最大200ステップ (delay の2倍)
        bool result = update_safe(true, dt);
        if (result) {
            activated = true;
            break;
        }
    }
    EXPECT_TRUE(activated) << "DRS should become ACTIVE after activation delay";
    EXPECT_TRUE(drs->is_active()) << "DRS should be ACTIVE";
}

// ======== 3. 速度閾値チェック ========
TEST_F(DRSSystemTest, SpeedThresholdCheck) {
    // 低速では起動できない
    for (int i = 0; i < 20; ++i) {
        bool result = update_slow(true, 0.01);
        EXPECT_FALSE(result) << "DRS should not activate below speed threshold";
    }

    // 速度がしきい値以上になるとACTIVATINGに移行する
    drs->reset();

    // 高速でボタンを押し、遅延後にACTIVEになる
    double safe_speed = 25.0;
    double dt = 0.01;
    for (int i = 0; i < 15; ++i) {
        drs->update(true, safe_speed, 0.0, 0.0, dt);
    }
    EXPECT_TRUE(drs->is_active());
}

// ======== 4. 横加速度チェック ========
TEST_F(DRSSystemTest, LateralAccelCheck) {
    // コーナリング中（高横G）では起動できない
    double high_lateral_g = config.max_lateral_accel + 1.0;  // しきい値超過
    // lateral_accel = |vx * yaw_rate|
    // vx=25, yaw_rate = high_lateral_g / vx
    double vx = 25.0;
    double yaw_rate = high_lateral_g / vx;

    for (int i = 0; i < 20; ++i) {
        bool result = drs->update(true, vx, high_lateral_g, 0.0, 0.01);
        EXPECT_FALSE(result) << "DRS should not activate under high lateral G";
    }

    // 横Gが小さければ起動できる
    drs->reset();
    double safe_yaw_rate = 0.1;  // 低い横G
    for (int i = 0; i < 15; ++i) {
        drs->update(true, 25.0, 25.0 * safe_yaw_rate, 0.0, 0.01);
    }
    EXPECT_TRUE(drs->is_active());
}

// ======== 5. ステアリング角チェック ========
TEST_F(DRSSystemTest, SteeringAngleCheck) {
    // ステアリング角が大きいと起動できない
    double large_steer = config.max_steering_angle + 0.01;  // しきい値超過

    for (int i = 0; i < 20; ++i) {
        bool result = drs->update(true, 25.0, 0.0, large_steer, 0.01);
        EXPECT_FALSE(result) << "DRS should not activate with large steering angle";
    }

    // ステアリング角が小さければ起動できる
    drs->reset();
    double small_steer = config.max_steering_angle - 0.01;  // しきい値以下
    for (int i = 0; i < 15; ++i) {
        drs->update(true, 25.0, 0.0, small_steer, 0.01);
    }
    EXPECT_TRUE(drs->is_active());
}

// ======== 6. 即座の停止チェック ========
TEST_F(DRSSystemTest, InstantDeactivation) {
    // DRSをACTIVEにする
    for (int i = 0; i < 15; ++i) {
        update_safe(true, 0.01);
    }
    ASSERT_TRUE(drs->is_active()) << "DRS should be ACTIVE before deactivation test";

    // ボタンを離すと即座に停止
    bool result = update_safe(false, 0.01);
    EXPECT_FALSE(result) << "DRS should deactivate immediately when button is released";

    // 安全条件違反でも即座に停止
    // 再起動
    for (int i = 0; i < 15; ++i) {
        update_safe(true, 0.01);
    }
    ASSERT_TRUE(drs->is_active());

    // 低速になると即座に停止
    bool result2 = update_slow(true, 0.01);  // ボタンは押したまま、でも低速
    EXPECT_FALSE(result2) << "DRS should deactivate immediately on safety violation";
}

// ======== 7. 統計情報チェック ========
TEST_F(DRSSystemTest, Statistics) {
    const double dt = 0.01;
    const double active_duration = 0.5;   // 0.5秒間ACTIVE
    const double speed = 28.0;

    // DRSをACTIVEにする（遅延: 0.1s = 10ステップ）
    for (int i = 0; i < 15; ++i) {
        drs->update(true, speed, 0.0, 0.0, dt);
    }
    ASSERT_TRUE(drs->is_active());

    // 0.5秒間ACTIVE状態を維持
    int active_steps = static_cast<int>(active_duration / dt);
    for (int i = 0; i < active_steps; ++i) {
        drs->update(true, speed, 0.0, 0.0, dt);
    }

    // 統計を確認
    auto stats = drs->get_stats();
    EXPECT_EQ(stats.activation_count, 1) << "Should have activated once";
    EXPECT_NEAR(stats.total_active_time, active_duration, 0.05)
        << "Total active time should be approximately " << active_duration << "s";
    EXPECT_NEAR(stats.avg_speed_when_active, speed, 1.0)
        << "Average speed should be close to " << speed << " m/s";

    // ボタン解除後、再起動で2回目のカウント
    drs->update(false, speed, 0.0, 0.0, dt);
    ASSERT_FALSE(drs->is_active());

    for (int i = 0; i < 15; ++i) {
        drs->update(true, speed, 0.0, 0.0, dt);
    }
    EXPECT_EQ(drs->get_stats().activation_count, 2) << "Should have activated twice";
}

// ======== 統合テスト: Vehicle14DOF_New + DRS ========

class DRSIntegrationTest : public ::testing::Test {
protected:
    VehicleParams params;
    std::unique_ptr<Vehicle14DOF_New> vehicle;

    void SetUp() override {
        params = VehicleParams();
        vehicle = std::make_unique<Vehicle14DOF_New>(params);
        vehicle->set_initial_velocity(25.0);  // 25 m/s (速度閾値以上)
    }
};

// ======== 8. 空力係数の変化チェック ========
TEST_F(DRSIntegrationTest, AeroForceReduction) {
    // DRSなしの空力力を計算
    AeroMap4D<double> aero_map;
    double velocity = 25.0;
    double rh_front = 40.0, rh_rear = 55.0;
    double roll = 0.0, yaw = 0.0;

    auto forces_off = aero_map.compute_forces(velocity, rh_front, rh_rear, roll, yaw, false);
    auto forces_on  = aero_map.compute_forces(velocity, rh_front, rh_rear, roll, yaw, true);

    // DRS ONで空力係数が減少
    // CL_on = CL_off * 0.54 → Fz_on = Fz_off * 0.54 (ダウンフォース減少)
    // CD_on = CD_off * 0.74 → Fx_on = Fx_off * 0.74 (ドラッグ減少)
    EXPECT_NEAR(forces_on.Fz / forces_off.Fz, 0.54, 1e-6)
        << "DRS should reduce downforce (CL) to 54% of normal";
    EXPECT_NEAR(forces_on.Fx / forces_off.Fx, 0.74, 1e-6)
        << "DRS should reduce drag (CD) to 74% of normal";

    // Fyは変化しない（サイドフォース）
    EXPECT_NEAR(forces_on.Fy, forces_off.Fy, 1e-6)
        << "Side force should not change with DRS";

    // リアウイングのみの DRS: フロント DF は不変、CoP は前進（旧実装は CoP 固定）
    EXPECT_NEAR(forces_on.Fz_front, forces_off.Fz_front, 1e-6);
    EXPECT_GT(aero_map.compute(rh_front, rh_rear, roll, yaw, true).CoP_x,
              aero_map.compute(rh_front, rh_rear, roll, yaw, false).CoP_x);
}

// ======== 8b. DRS 目標値（比率・CoP シフト）は AeroMapParams で明示 ========
TEST(DRSAeroTargets, RatiosAndCopShiftFromParams) {
    AeroMap4D<double> aero_map;
    aero_map.params().drs_CL_ratio = 0.6;
    aero_map.params().drs_CD_ratio = 0.8;
    aero_map.params().drs_CoP_x_shift = 0.05;
    const auto off = aero_map.compute(40, 55, 0, 0, false);
    const auto on = aero_map.compute(40, 55, 0, 0, true);
    EXPECT_NEAR(on.CL / off.CL, 0.6, 1e-12);
    EXPECT_NEAR(on.CD / off.CD, 0.8, 1e-12);
    EXPECT_NEAR(on.CoP_x - off.CoP_x, 0.05, 1e-12);
}

// ======== 8c. 安全判定は車体横加速度 ay（旧 |vx·r| は非推奨オーバーロードで委譲） ========
TEST_F(DRSSystemTest, UsesBodyLateralAccel) {
    // 定常でない旋回: |vx·r| は小さいが実 ay は大きい → 起動不可
    for (int i = 0; i < 20; ++i) {
        EXPECT_FALSE(drs->update(true, 25.0, config.max_lateral_accel + 0.5, 0.0, 0.01));
    }
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    drs->reset();
    for (int i = 0; i < 20; ++i) {
        EXPECT_FALSE(drs->update(true, 25.0, 25.0, (config.max_lateral_accel + 0.5) / 25.0, 0.0, 0.01))
            << "deprecated overload delegates with ay = vx*r";
    }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

// ======== 8d. フラップ 1 次遅れ（τ=0 は二値） ========
TEST_F(DRSSystemTest, FlapFirstOrderBlend) {
    for (int i = 0; i < 15; ++i) update_safe(true, 0.01);
    ASSERT_TRUE(drs->is_active());
    EXPECT_DOUBLE_EQ(drs->flap_position(), 1.0) << "tau = 0 -> binary flap";

    DRSConfig slow = config;
    slow.flap_tau = 0.1;
    DRSSystem lag(slow);
    for (int i = 0; i < 20 && !lag.is_active(); ++i) lag.update(true, 25.0, 0.0, 0.0, 0.01);  // 0.1 s delay
    ASSERT_TRUE(lag.is_active());
    const double first = lag.flap_position();
    EXPECT_GT(first, 0.0);
    EXPECT_LT(first, 0.5);
    for (int i = 0; i < 100; ++i) lag.update(true, 25.0, 0.0, 0.0, 0.01);
    EXPECT_NEAR(lag.flap_position(), 1.0, 1e-3);
    lag.update(false, 25.0, 0.0, 0.0, 0.01);
    EXPECT_FALSE(lag.is_active());
    EXPECT_GT(lag.flap_position(), 0.5) << "flap closes with lag after deactivation";
}

// ======== 9. 速度閾値以下での非起動チェック（実シミュレーション） ========
TEST_F(DRSIntegrationTest, NoActivationBelowSpeedThreshold) {
    // 低速車両を作成
    Vehicle14DOF_New slow_vehicle(params);
    slow_vehicle.set_initial_velocity(10.0);  // 10 m/s (閾値以下)

    // DRSボタンを押してもACTIVEにならない
    DriverInput input;
    input.throttle = 0.5;
    input.drs_button = true;  // DRSボタン押す
    input.gear = 3;

    for (int i = 0; i < 100; ++i) {
        slow_vehicle.step(0.001, input);  // 0.1秒シミュレーション
    }

    EXPECT_FALSE(slow_vehicle.is_drs_active())
        << "DRS should not activate below speed threshold (10 m/s < 20 m/s)";
}

// ======== 10. トルク操舵時のDRS安全判定は実舵角で行う ========
TEST_F(DRSIntegrationTest, TorqueModeUsesActualTireSteerForSafetyCheck) {
    VehicleParams torque_params = params;
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 20.0;
    torque_params.steering.T_fric = 0.0;
    torque_params.steering.C_steer_torque = 0.0;
    torque_params.steering.R_steer = 4.5;

    Vehicle14DOF_New torque_vehicle(torque_params);
    torque_vehicle.set_initial_velocity(25.0);

    DriverInput input;
    input.throttle = 0.0;
    input.steering = 0.25;  // [Nm], old DRS check incorrectly treated this as rad and blocked.
    input.drs_button = true;
    input.gear = 3;

    for (int i = 0; i < 140; ++i) {
        torque_vehicle.step(0.001, input);
    }

    EXPECT_LT(std::abs(torque_vehicle.get_tire_angle()), torque_vehicle.drs_system().config().max_steering_angle);
    EXPECT_TRUE(torque_vehicle.is_drs_active())
        << "Torque-mode DRS safety check should use actual tire steer angle, not steering torque";
}

// ======== 11. リセット後の状態確認 ========
TEST_F(DRSSystemTest, ResetClearsState) {
    // DRSをACTIVEにする
    for (int i = 0; i < 15; ++i) {
        update_safe(true, 0.01);
    }
    ASSERT_TRUE(drs->is_active());

    // リセット
    drs->reset();

    // 状態がクリアされる
    EXPECT_FALSE(drs->is_active());
    auto stats = drs->get_stats();
    EXPECT_EQ(stats.activation_count, 0);
    EXPECT_NEAR(stats.total_active_time, 0.0, 1e-10);
}
