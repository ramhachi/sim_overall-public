// tests/test_pou.cpp
// POU (Pushrod on Upright) システムの単体テスト

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include "fsae/vehicle/vehicle_state.hpp"
#include <cmath>

using namespace fsae::vehicle;

// ======== テストフィクスチャ ========
class POUKinematicsTest : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        // 基本パラメータの設定（デフォルト値を使用）
        params = VehicleParams();
    }
};

// ======== 1. ゼロ舵角でPOU効果なし ========
TEST_F(POUKinematicsTest, ZeroSteerNoPOUEffect) {
    Vehicle14DOF_New vehicle(params);

    double steer_angle = 0.0;
    int wheel_idx = 0;  // FL

    // POU変位計算
    double pou_disp = vehicle.compute_pou_displacement(wheel_idx, steer_angle);
    EXPECT_NEAR(pou_disp, 0.0, 1e-10);  // ゼロ舵角でPOU変位=0

    // キャンバー計算（POU係数がデフォルト0なので変化なし）
    double susp_travel = 0.0;
    double camber = vehicle.compute_camber_angle(wheel_idx, susp_travel, steer_angle);
    EXPECT_NEAR(camber, params.suspension.static_camber[wheel_idx], 1e-10);
}

// ======== 2. ジャッキアップ変位の計算 ========
TEST_F(POUKinematicsTest, JackUpDisplacement) {
    params.suspension.pou_jack_c1[0] = 0.01;   // 1cm/rad
    params.suspension.pou_jack_c2[0] = 0.05;   // 5cm/rad²

    Vehicle14DOF_New vehicle(params);

    double steer_angle = 0.2;  // 0.2 rad (~11.5°)
    int wheel_idx = 0;  // FL

    // 期待値: Δz = 0.01*0.2 + 0.05*0.2² = 0.002 + 0.002 = 0.004 m (4mm)
    double expected_disp = 0.01 * steer_angle + 0.05 * steer_angle * steer_angle;
    double pou_disp = vehicle.compute_pou_displacement(wheel_idx, steer_angle);

    EXPECT_NEAR(pou_disp, expected_disp, 1e-9);
    EXPECT_NEAR(pou_disp, 0.004, 1e-9);
}

// ======== 3. ステアリング誘発キャンバー ========
TEST_F(POUKinematicsTest, SteeringInducedCamber) {
    params.suspension.pou_camber_c1[0] = -0.05;   // -0.05 rad/rad
    params.suspension.pou_camber_c2[0] = -0.10;   // -0.10 rad/rad²

    Vehicle14DOF_New vehicle(params);

    double steer_angle = 0.3;  // 0.3 rad (~17.2°)
    int wheel_idx = 0;  // FL
    double susp_travel = 0.0;  // ストロークなし

    // 期待値:
    // 静的: -0.017 (static_camber[0])
    // POU: -0.05*0.3 + (-0.10)*0.3² = -0.015 - 0.009 = -0.024 rad
    // 合計: -0.017 - 0.024 = -0.041 rad
    double expected_camber = params.suspension.static_camber[wheel_idx]
                           + (-0.05 * steer_angle + (-0.10) * steer_angle * steer_angle);
    double camber = vehicle.compute_camber_angle(wheel_idx, susp_travel, steer_angle);

    EXPECT_NEAR(camber, expected_camber, 1e-9);
    EXPECT_NEAR(camber, -0.041, 1e-9);
}

// ======== 4. 後方互換性（POU係数=0でデフォルト動作） ========
TEST_F(POUKinematicsTest, BackwardCompatibility) {
    // POU係数を全てゼロに設定（デフォルト値）
    Vehicle14DOF_New vehicle(params);

    double steer_angle = 0.2;
    double susp_travel = 0.01;  // 10mm compression
    int wheel_idx = 0;

    // POU変位はゼロ
    double pou_disp = vehicle.compute_pou_displacement(wheel_idx, steer_angle);
    EXPECT_NEAR(pou_disp, 0.0, 1e-10);

    // キャンバーは従来の計算と一致（静的 + ゲイン×ストローク）
    double expected_camber = params.suspension.static_camber[wheel_idx]
                           + params.suspension.camber_gain[wheel_idx] * susp_travel;
    double camber = vehicle.compute_camber_angle(wheel_idx, susp_travel, steer_angle);

    EXPECT_NEAR(camber, expected_camber, 1e-10);
}

// ======== 統合テスト用フィクスチャ ========
class POUIntegrationTest : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        params = VehicleParams();
    }
};

// ======== 5. 静的ジャッキアップテスト（据え切り） ========
TEST_F(POUIntegrationTest, StaticJackUpTest) {
    // フロント輪にPOUジャッキアップを設定
    params.suspension.pou_jack_c1[0] = 0.015;  // FL: 15mm/rad
    params.suspension.pou_jack_c1[1] = 0.015;  // FR: 15mm/rad

    Vehicle14DOF_New vehicle(params);
    VehicleState state = VehicleState();
    state.vx = 0.0;  // 停車状態

    DriverInput input;
    input.steering = 0.3;  // 0.3 rad (~17°) 据え切り
    input.throttle = 0.0;
    input.brake = 0.0;

    // 平衡状態から少しステップして収束させる
    const double dt = 0.001;
    for (int i = 0; i < 100; ++i) {
        vehicle.step(dt, input);
    }

    // POU変位を取得
    auto pou_disp = vehicle.get_pou_displacement();

    // フロント輪（FL, FR）にPOU効果があることを確認
    // Ackermann幾何により各輪の実際の舵角は入力と若干異なるため、許容誤差を緩和
    double expected_disp_front = 0.015 * input.steering;  // ~4.5mm (近似値)
    EXPECT_NEAR(pou_disp[0], expected_disp_front, 0.001);  // FL: ±1mm許容
    EXPECT_NEAR(pou_disp[1], expected_disp_front, 0.001);  // FR: ±1mm許容

    // リア輪（RL, RR）はPOU効果なし（係数=0）
    EXPECT_NEAR(pou_disp[2], 0.0, 1e-9);  // RL
    EXPECT_NEAR(pou_disp[3], 0.0, 1e-9);  // RR
}

// ======== 6. スキッドパッド比較（POU有無） ========
TEST_F(POUIntegrationTest, SkidpadComparison) {
    // POU設定: フロントのみ
    params.suspension.pou_jack_c1[0] = 0.010;   // FL: 10mm/rad
    params.suspension.pou_jack_c1[1] = 0.010;   // FR: 10mm/rad
    params.suspension.pou_camber_c1[0] = -0.04; // FL: -0.04 rad/rad (~2.3°/rad)
    params.suspension.pou_camber_c1[1] = 0.04;  // FR: +0.04 rad/rad (ミラー対称)

    Vehicle14DOF_New vehicle(params);
    VehicleState state = VehicleState();
    state.vx = 10.0;  // 10 m/s
    state.vy = 0.0;

    DriverInput input;
    input.steering = 0.15;  // 定常円旋回
    input.throttle = 0.3;
    input.brake = 0.0;

    // 1秒間シミュレーション
    const double dt = 0.001;
    for (int i = 0; i < 1000; ++i) {
        vehicle.step(dt, input);
    }

    // POU変位とキャンバーを取得
    auto pou_disp = vehicle.get_pou_displacement();
    auto pou_camber = vehicle.get_pou_camber();

    // フロント輪にPOU効果が現れることを確認
    EXPECT_GT(std::abs(pou_disp[0]), 0.001);     // FL: 1mm以上の変位
    EXPECT_GT(std::abs(pou_camber[0]), 0.001);   // FL: 0.001 rad以上の変化

    // 車両が安定して走行（NaN/Infなし）
    const auto& s = vehicle.state();
    EXPECT_FALSE(std::isnan(s.vx));
    EXPECT_FALSE(std::isinf(s.vx));
    EXPECT_FALSE(std::isnan(s.vy));
    EXPECT_FALSE(std::isinf(s.vy));
}

// ======== 7. 数値安定性テスト ========
TEST_F(POUIntegrationTest, NumericalStability) {
    // 極端なPOU係数でも発散しないことを確認
    params.suspension.pou_jack_c1[0] = 0.050;    // 大きめ: 50mm/rad
    params.suspension.pou_camber_c1[0] = -0.10;  // 大きめ: -0.1 rad/rad

    Vehicle14DOF_New vehicle(params);
    VehicleState state = VehicleState();
    state.vx = 15.0;

    DriverInput input;
    input.steering = 0.25;  // 大きめの舵角
    input.throttle = 0.5;
    input.brake = 0.0;

    // 2秒間シミュレーション
    const double dt = 0.001;
    for (int i = 0; i < 2000; ++i) {
        vehicle.step(dt, input);
    }

    // NaN/Infが発生していないことを確認
    const auto& s = vehicle.state();
    EXPECT_FALSE(std::isnan(s.x));
    EXPECT_FALSE(std::isnan(s.vx));
    EXPECT_FALSE(std::isnan(s.vy));
    EXPECT_FALSE(std::isnan(s.z));
    EXPECT_FALSE(std::isinf(s.vx));
    EXPECT_FALSE(std::isinf(s.vy));

    // タイヤ力も有限値
    auto Fz = vehicle.get_tire_Fz();
    for (int i = 0; i < 4; ++i) {
        EXPECT_FALSE(std::isnan(Fz[i]));
        EXPECT_FALSE(std::isinf(Fz[i]));
        EXPECT_GT(Fz[i], 0.0);  // 垂直力は正
    }
}
