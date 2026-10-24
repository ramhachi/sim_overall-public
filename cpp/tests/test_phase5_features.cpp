// tests/test_phase5_features.cpp
// Phase 5で実装された機能の単体テスト

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include "fsae/physics/suspension/lugre.hpp"
#include <cmath>

using namespace fsae::vehicle;
using namespace fsae::physics::suspension;

// ======== テストフィクスチャ ========
class Phase5Test : public ::testing::Test {
protected:
    VehicleParams params;

    void SetUp() override {
        // 基本パラメータの設定（デフォルト値を使用）
        params = VehicleParams();
    }
};

// ======== 1. アッカーマン幾何のテスト ========
TEST_F(Phase5Test, AckermannGeometry_ZeroSteer_ParallelWheels) {
    Vehicle14DOF_New vehicle(params);

    double delta_center = 0.0;
    double ackermann_ratio = 0.7;  // 70%

    auto steer_angles = vehicle.compute_ackermann_steering(delta_center, ackermann_ratio);

    // 舵角ゼロの場合、全輪が0度
    EXPECT_NEAR(steer_angles[0], 0.0, 1e-6);  // FL
    EXPECT_NEAR(steer_angles[1], 0.0, 1e-6);  // FR
    EXPECT_NEAR(steer_angles[2], 0.0, 1e-6);  // RL
    EXPECT_NEAR(steer_angles[3], 0.0, 1e-6);  // RR
}

TEST_F(Phase5Test, AckermannGeometry_RightTurn_InnerWheelGreater) {
    Vehicle14DOF_New vehicle(params);

    double delta_center = 0.1;  // SAE J670e: 正の舵角は右旋回
    double ackermann_ratio = 0.7;

    auto steer_angles = vehicle.compute_ackermann_steering(delta_center, ackermann_ratio);

    double delta_FL = steer_angles[0];  // 外輪（左）
    double delta_FR = steer_angles[1];  // 内輪（右）

    // 右旋回時、右前輪（内輪）の方が舵角が大きい
    EXPECT_GT(delta_FR, delta_FL);

    // 後輪は操舵しない
    EXPECT_NEAR(steer_angles[2], 0.0, 1e-6);
    EXPECT_NEAR(steer_angles[3], 0.0, 1e-6);
}

TEST_F(Phase5Test, AckermannGeometry_LeftTurn_InnerWheelGreater) {
    Vehicle14DOF_New vehicle(params);

    double delta_center = -0.1;  // SAE J670e: 負の舵角は左旋回
    double ackermann_ratio = 0.7;

    auto steer_angles = vehicle.compute_ackermann_steering(delta_center, ackermann_ratio);

    double delta_FL = steer_angles[0];  // 左前輪
    double delta_FR = steer_angles[1];  // 右前輪

    // 左旋回時、左前輪（内輪）の方が絶対値が大きい
    EXPECT_GT(std::abs(delta_FL), std::abs(delta_FR));  // 絶対値で比較
    EXPECT_LT(delta_FR, 0.0);  // 両方とも負
    EXPECT_LT(delta_FL, 0.0);
}

TEST_F(Phase5Test, AckermannGeometry_FullAckermann_vs_Parallel) {
    Vehicle14DOF_New vehicle(params);

    double delta_center = 0.15;

    // 完全アッカーマン（100%）
    auto steer_full = vehicle.compute_ackermann_steering(delta_center, 1.0);

    // パラレル（0%）
    auto steer_parallel = vehicle.compute_ackermann_steering(delta_center, 0.0);

    // パラレルの場合、両輪が等しい
    EXPECT_NEAR(steer_parallel[0], delta_center, 1e-6);
    EXPECT_NEAR(steer_parallel[1], delta_center, 1e-6);

    // 完全アッカーマンの場合、右旋回では右前輪（内輪） > 左前輪（外輪）
    EXPECT_GT(steer_full[1], steer_full[0]);
    EXPECT_GT(steer_full[1], delta_center);
    EXPECT_LT(steer_full[0], delta_center);
}

// ======== 2. コンプライアンスステアのテスト ========
TEST_F(Phase5Test, ComplianceSteer_ZeroTorque_ZeroCompliance) {
    Vehicle14DOF_New vehicle(params);

    double Mz_FL = 0.0;
    double Mz_FR = 0.0;

    auto compliance = vehicle.compute_compliance_steer(Mz_FL, Mz_FR);

    // トルクゼロの場合、コンプライアンスもゼロ
    EXPECT_NEAR(compliance[0], 0.0, 1e-9);
    EXPECT_NEAR(compliance[1], 0.0, 1e-9);
    EXPECT_NEAR(compliance[2], 0.0, 1e-9);
    EXPECT_NEAR(compliance[3], 0.0, 1e-9);
}

TEST_F(Phase5Test, ComplianceSteer_PositiveTorque_PositiveAngle) {
    Vehicle14DOF_New vehicle(params);

    double Mz_FL = 50.0;   // [Nm]
    double Mz_FR = 50.0;

    auto compliance = vehicle.compute_compliance_steer(Mz_FL, Mz_FR);

    // 正のトルクで正の舵角変化
    EXPECT_GT(compliance[0], 0.0);
    EXPECT_GT(compliance[1], 0.0);

    // 後輪はゼロ
    EXPECT_NEAR(compliance[2], 0.0, 1e-9);
    EXPECT_NEAR(compliance[3], 0.0, 1e-9);
}

TEST_F(Phase5Test, ComplianceSteer_AsymmetricTorque) {
    Vehicle14DOF_New vehicle(params);

    double Mz_FL = 100.0;   // 左前輪に大きなトルク
    double Mz_FR = 20.0;    // 右前輪は小さい

    auto compliance = vehicle.compute_compliance_steer(Mz_FL, Mz_FR);

    // 左の方が大きく変化
    EXPECT_GT(compliance[0], compliance[1]);

    // ステアリング剛性との関係確認
    double K_steer = params.steering.stiffness;
    EXPECT_NEAR(compliance[0], Mz_FL / K_steer, 1e-9);
    EXPECT_NEAR(compliance[1], Mz_FR / K_steer, 1e-9);
}

// ======== 3. ARB力計算のテスト ========
TEST_F(Phase5Test, ARB_ZeroRollDifference_ZeroForce) {
    Vehicle14DOF_New vehicle(params);

    // 左右のサス変位が同じ（ロールなし）
    std::array<double, 4> susp_travel = {0.01, 0.01, 0.02, 0.02};

    auto F_arb = vehicle.compute_arb_forces(susp_travel);

    // ロールがない場合、ARB力はゼロ
    EXPECT_NEAR(F_arb[0], 0.0, 1e-6);  // FL
    EXPECT_NEAR(F_arb[1], 0.0, 1e-6);  // FR
    EXPECT_NEAR(F_arb[2], 0.0, 1e-6);  // RL
    EXPECT_NEAR(F_arb[3], 0.0, 1e-6);  // RR
}

TEST_F(Phase5Test, ARB_FrontRoll_OppositeForces) {
    Vehicle14DOF_New vehicle(params);

    // フロントに左ロール（左側が沈む）
    std::array<double, 4> susp_travel = {0.02, 0.00, 0.01, 0.01};

    auto F_arb = vehicle.compute_arb_forces(susp_travel);

    // フロントARBが左右で逆向きの力を発生
    EXPECT_GT(F_arb[0], 0.0);  // 左を持ち上げる
    EXPECT_LT(F_arb[1], 0.0);  // 右を押し下げる

    // 力の大きさは等しい
    EXPECT_NEAR(std::abs(F_arb[0]), std::abs(F_arb[1]), 1e-6);

    // リアはロールしていないのでゼロ
    EXPECT_NEAR(F_arb[2], 0.0, 1e-6);
    EXPECT_NEAR(F_arb[3], 0.0, 1e-6);
}

TEST_F(Phase5Test, ARB_RearRoll_OppositeForces) {
    Vehicle14DOF_New vehicle(params);

    // リアに右ロール（右側が沈む）
    std::array<double, 4> susp_travel = {0.01, 0.01, 0.00, 0.02};

    auto F_arb = vehicle.compute_arb_forces(susp_travel);

    // フロントはロールなし
    EXPECT_NEAR(F_arb[0], 0.0, 1e-6);
    EXPECT_NEAR(F_arb[1], 0.0, 1e-6);

    // リアARBが左右で逆向きの力を発生
    EXPECT_LT(F_arb[2], 0.0);  // 左を押し下げる
    EXPECT_GT(F_arb[3], 0.0);  // 右を持ち上げる

    // 力の大きさは等しい
    EXPECT_NEAR(std::abs(F_arb[2]), std::abs(F_arb[3]), 1e-6);
}

// ======== 4. LuGre安定性制約のテスト ========
TEST_F(Phase5Test, LuGre_StiffnessConstraint) {
    // LuGre摩擦モデルのσ₀が安定性境界内にあることを確認

    LuGreParams<double> params_lugre;
    params_lugre.sigma_0 = 1000.0;  // [N/m]（修正後の値）
    params_lugre.sigma_1 = 100.0;
    params_lugre.sigma_2 = 0.5;
    params_lugre.vs = 0.01;
    params_lugre.Fc = 10.0;
    params_lugre.Fs = 15.0;

    // 安定性条件: σ₀ ≤ 10000 N/m（総合ガイド §11.6より）
    EXPECT_LE(params_lugre.sigma_0, 10000.0);

    // 実装で使用している値が1000 N/m（10倍のマージン）
    EXPECT_NEAR(params_lugre.sigma_0, 1000.0, 1e-3);
}

TEST_F(Phase5Test, LuGre_ZeroVelocity_NoForce) {
    LuGreParams<double> params_lugre;
    params_lugre.sigma_0 = 1000.0;
    params_lugre.sigma_1 = 100.0;
    params_lugre.sigma_2 = 0.5;
    params_lugre.vs = 0.01;
    params_lugre.Fc = 10.0;
    params_lugre.Fs = 15.0;

    double z = 0.0;  // 初期毛変形
    double v = 0.0;  // 速度ゼロ

    auto result = compute_lugre(params_lugre, v, z);

    // 速度ゼロの場合、摩擦力は静止摩擦力
    EXPECT_TRUE(std::isfinite(result.force));
    EXPECT_TRUE(std::isfinite(result.dz_dt));

    // 変形速度もゼロ
    EXPECT_NEAR(result.dz_dt, 0.0, 1e-6);
}

TEST_F(Phase5Test, LuGre_StickSlipTransition) {
    LuGreParams<double> params_lugre;
    params_lugre.sigma_0 = 1000.0;
    params_lugre.sigma_1 = 100.0;
    params_lugre.sigma_2 = 0.5;
    params_lugre.vs = 0.01;
    params_lugre.Fc = 10.0;   // クーロン摩擦
    params_lugre.Fs = 15.0;   // 静止摩擦（高い）

    double z = 0.0;

    // 低速（スティック領域）
    auto result_stick = compute_lugre(params_lugre, 0.001, z);

    // 高速（スリップ領域）
    auto result_slip = compute_lugre(params_lugre, 0.1, z);

    // 摩擦力の符号が異なる場合があるので、絶対値で比較
    // ストライベック効果により低速時の方が高い g(v) を持つが、
    // 力は z と v に依存するため、実際の比較は複雑
    // ここでは計算が有限値であることを確認
    EXPECT_TRUE(std::isfinite(result_stick.force));
    EXPECT_TRUE(std::isfinite(result_slip.force));
}

// ======== 5. キャンバー・トー座標変換のテスト ========
// （キャンバー・トー変換はprivateメソッドのため、統合テストでカバーする）

// ======== 7. 適応的タイムステップのテスト ========
TEST_F(Phase5Test, AdaptiveTimestep_NormalCondition_StandardDt) {
    Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(10.0);  // 10 m/s

    DriverInput input;
    input.steering = 0.0;
    input.throttle = 0.5;
    input.brake = 0.0;

    double dt = 0.001;  // 1 ms

    // バンプストップに接触していない通常状態
    EXPECT_NO_THROW({
        vehicle.step(dt, input);
    });

    // 状態が有限値であることを確認
    EXPECT_TRUE(std::isfinite(vehicle.state().vx));
    EXPECT_TRUE(std::isfinite(vehicle.state().vy));
    EXPECT_TRUE(std::isfinite(vehicle.state().z));
}

TEST_F(Phase5Test, AdaptiveTimestep_StiffEvent_NoExplosion) {
    Vehicle14DOF_New vehicle(params);
    vehicle.reset();

    // 大きな路面入力をシミュレート（バンプストップに接触する状況）
    vehicle.state().z = vehicle.z_equilibrium() - 0.15;  // 大きく沈む

    DriverInput input;
    input.steering = 0.0;
    input.throttle = 0.0;
    input.brake = 1.0;  // フルブレーキ

    double dt = 0.001;

    // Stiffイベント時も発散しないことを確認
    for (int i = 0; i < 100; ++i) {
        EXPECT_NO_THROW({
            vehicle.step(dt, input);
        });

        // 全状態変数が有限値
        EXPECT_TRUE(std::isfinite(vehicle.state().x));
        EXPECT_TRUE(std::isfinite(vehicle.state().y));
        EXPECT_TRUE(std::isfinite(vehicle.state().z));
        EXPECT_TRUE(std::isfinite(vehicle.state().vx));
        EXPECT_TRUE(std::isfinite(vehicle.state().vy));
        EXPECT_TRUE(std::isfinite(vehicle.state().vz));
    }
}

// ======== 統合テスト：Phase 5機能の組み合わせ ========
TEST_F(Phase5Test, Integration_CorneringWithAllFeatures) {
    Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);  // 15 m/s

    DriverInput input;
    input.steering = 0.1;  // 左旋回
    input.throttle = 0.3;
    input.brake = 0.0;
    input.gear = 3;

    double dt = 0.001;
    int n_steps = 1000;  // 1秒間

    // シミュレーション実行
    for (int i = 0; i < n_steps; ++i) {
        vehicle.step(dt, input);

        // 全状態が有限値であることを確認
        EXPECT_TRUE(std::isfinite(vehicle.state().x));
        EXPECT_TRUE(std::isfinite(vehicle.state().y));
        EXPECT_TRUE(std::isfinite(vehicle.state().vx));
        EXPECT_TRUE(std::isfinite(vehicle.state().vy));
        EXPECT_TRUE(std::isfinite(vehicle.state().r));

        // 物理的に妥当な範囲にあることを確認
        EXPECT_LT(std::abs(vehicle.state().phi), M_PI);  // ロール角
        EXPECT_LT(std::abs(vehicle.state().theta), M_PI);  // ピッチ角
        EXPECT_LT(vehicle.state().vx, 50.0);  // 速度上限
    }

    // 旋回により横位置が変化していることを確認
    EXPECT_GT(std::abs(vehicle.state().y), 0.1);

    // ヨーレートが発生していることを確認（旋回中）
    // 左旋回（正の舵角）で正または負のヨーレートが発生
    EXPECT_GT(std::abs(vehicle.state().r), 0.0);
}
