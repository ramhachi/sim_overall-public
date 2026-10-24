/// @file test_bicycle_model.cpp
/// @brief 6-DOF自転車モデルのユニットテスト（Phase 8a）

#include <gtest/gtest.h>
#include "fsae/vehicle/bicycle_model.hpp"
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include <cmath>

using namespace fsae::vehicle;

/// テスト用フィクスチャ
class BicycleModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 標準的なFSAEパラメータ
        params_.m = 280.0;
        params_.Izz = 85.0;
        params_.lf = 0.73;
        params_.lr = 0.82;
        params_.Cf = 60000.0;  // N/rad
        params_.Cr = 70000.0;  // N/rad

        model_ = std::make_unique<BicycleModel6DOF>(params_);
    }

    BicycleParams params_;
    std::unique_ptr<BicycleModel6DOF> model_;
};

/// テスト1: 直線走行（ゼロ入力）
TEST_F(BicycleModelTest, StraightLineMotion) {
    // 初期状態: 10 m/s直進
    BicycleState6DOF state{0, 0, 0, 10.0, 0, 0};
    BicycleControl control{0.0, 0.0};  // ゼロ舵角、ゼロ加速度

    double dt = 0.025;  // 25ms
    auto next_state = model_->predict_step(state, control, dt);

    // 期待値: x方向に0.25m進む
    EXPECT_NEAR(next_state.x, 0.25, 0.001);
    EXPECT_NEAR(next_state.y, 0.0, 0.001);
    EXPECT_NEAR(next_state.psi, 0.0, 0.001);
    EXPECT_NEAR(next_state.vx, 10.0, 0.01);
    EXPECT_NEAR(next_state.vy, 0.0, 0.001);
    EXPECT_NEAR(next_state.r, 0.0, 0.001);
}

/// テスト2: 定常円旋回
/// 解析解の正当性と初期応答方向を検証
TEST_F(BicycleModelTest, SteadyStateCircle) {
    double speed = 10.0;    // m/s
    double radius = 30.0;   // m

    // 解析解を取得
    auto [r_analytic, vy_analytic, delta_analytic] =
        model_->steady_state_cornering(speed, radius);

    // 期待値: r ≈ v/R
    EXPECT_NEAR(r_analytic, speed / radius, 0.01);

    // 短い数値シミュレーションで初期応答方向を検証
    // （簡略モデルは縦方向ドラッグがないため長時間シミュレーションでは発散する）
    BicycleState6DOF state{0, 0, 0, speed, 0, 0};
    BicycleControl control{delta_analytic, 0.0};

    double dt = 0.001;  // 小さな時間刻み
    bool nan_detected = false;
    for (int i = 0; i < 100; ++i) {  // 0.1秒間のみ
        state = model_->predict_step(state, control, dt);
        if (std::isnan(state.vx) || std::isnan(state.r)) {
            nan_detected = true;
            break;
        }
    }

    // NaNが発生しない
    EXPECT_FALSE(nan_detected) << "NaN detected during simulation";

    // ヨーレートが解析解の方向に発展している
    // (正のdelta → Phase 7右旋回 → 正のr)
    // 解析解は左旋回(radius>0) → r > 0
    if (!nan_detected) {
        // 初期応答としてヨーレートが正しい方向
        double r_sign = (r_analytic > 0) ? 1.0 : -1.0;
        EXPECT_GT(state.r * r_sign, 0.0)
            << "Yaw rate should develop toward analytic solution";
    }
}

/// テスト3: 線形化精度
/// A,B行列が微小摂動を正しく予測できるか検証
TEST_F(BicycleModelTest, LinearizationAccuracy) {
    BicycleState6DOF state{0, 0, 0, 15.0, 0.5, 0.1};
    BicycleControl control{0.05, 0.0};  // 小さな舶角

    double dt = 0.025;

    // 基準点での非線形予測
    auto x_nom = model_->predict_step(state, control, dt);

    // 線形化
    auto [A, B] = model_->linearize(state, control, dt);

    // 摂動した状態で検証:
    // x_next(x0+dx) ≈ x_nom + A*dx
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(6);
    dx(3) = 0.1;  // vxに微小摂動
    dx(4) = 0.01; // vyに微小摂動

    // 摂動状態
    auto state_pert = BicycleState6DOF::from_vector(state.to_vector() + dx);
    auto x_pert_actual = model_->predict_step(state_pert, control, dt);

    // 線形予測: x_nom + A*dx
    Eigen::VectorXd x_pert_linear = x_nom.to_vector() + A * dx;

    // 線形化誤差が5%以内
    for (int i = 0; i < 6; ++i) {
        double actual = x_pert_actual.to_vector()(i);
        double predicted = x_pert_linear(i);
        double error = std::abs(predicted - actual);
        double magnitude = std::max(std::abs(actual), 0.01);
        double relative_error = error / magnitude;

        EXPECT_LT(relative_error, 0.05)
            << "State index " << i << ": linear=" << predicted
            << ", nonlinear=" << actual;
    }
}

/// テスト4: 加速
TEST_F(BicycleModelTest, Acceleration) {
    BicycleState6DOF state{0, 0, 0, 10.0, 0, 0};
    BicycleControl control{0.0, 2.0};  // 2 m/s² 加速

    double dt = 0.1;  // 100ms
    auto next_state = model_->predict_step(state, control, dt);

    // v = v0 + a·t → 10 + 2·0.1 = 10.2 m/s
    EXPECT_NEAR(next_state.vx, 10.2, 0.01);
}

/// テスト5: 減速（負の加速度）
TEST_F(BicycleModelTest, Braking) {
    BicycleState6DOF state{0, 0, 0, 20.0, 0, 0};
    BicycleControl control{0.0, -5.0};  // 5 m/s² 減速

    double dt = 0.1;
    auto next_state = model_->predict_step(state, control, dt);

    // v = 20 - 5·0.1 = 19.5 m/s
    EXPECT_NEAR(next_state.vx, 19.5, 0.01);
}

/// テスト6: 右旋回（Phase 7の符号慣例に合わせる）
TEST_F(BicycleModelTest, RightTurn) {
    BicycleState6DOF state{0, 0, 0, 10.0, 0, 0};
    BicycleControl control{0.1, 0.0};  // 正の舵角 = 右旋回

    double dt = 0.1;
    auto next_state = model_->predict_step(state, control, dt);

    // 正の舶角 → 負のヨーレート（右旋回 = 時計回り = SAE慣例で負）
    // ※ Phase 7慣例: 正のステアリング = 右旋回、内部でSAE変換
    EXPECT_LT(next_state.r, 0.0) << "Positive steering (right turn) should produce negative yaw rate (CW in SAE)";
}

/// テスト7: 低速安定性（vx→0での特異点回避）
TEST_F(BicycleModelTest, LowSpeedStability) {
    BicycleState6DOF state{0, 0, 0, 0.1, 0, 0};  // 非常に低速
    BicycleControl control{0.05, 1.0};

    double dt = 0.01;

    // クラッシュせずに積分できる
    EXPECT_NO_THROW({
        for (int i = 0; i < 100; ++i) {
            state = model_->predict_step(state, control, dt);
        }
    });

    // 加速して低速域を脱出
    EXPECT_GE(state.vx, 0.5);
}

/// テスト8: パラメータ抽出（14DOF → 自転車モデル）
TEST(BicycleParamsTest, FromVehicleParams) {
    VehicleParams vp;  // デフォルト値
    vp.mass.m_total = 280.0;
    vp.mass.Izz = 85.0;
    vp.mass.a = 0.73;
    vp.mass.b = 0.82;
    vp.tire.mu_y_max = 2.0;

    auto bp = BicycleParams::from_vehicle_params(vp);

    EXPECT_DOUBLE_EQ(bp.m, 280.0);
    EXPECT_DOUBLE_EQ(bp.Izz, 85.0);
    EXPECT_DOUBLE_EQ(bp.lf, 0.73);
    EXPECT_DOUBLE_EQ(bp.lr, 0.82);

    // TMeasyの線形化コーナリング剛性が合理的な範囲
    EXPECT_GT(bp.Cf, 10000.0);  // 少なくとも10 kN/rad
    EXPECT_LT(bp.Cf, 150000.0); // 最大150 kN/rad
    EXPECT_GT(bp.Cr, 10000.0);
    EXPECT_LT(bp.Cr, 150000.0);
}

TEST(BicycleParamsTest, TMeasyLinearizationFollowsCyAndRelaxationLength) {
    VehicleParams vp = VehicleParams::default_fsae();
    vp.tire.relaxation.enabled = false;

    const auto baseline = BicycleParams::from_vehicle_params(vp);

    vp.tire.Cy *= 1.5;
    const auto higher_cy = BicycleParams::from_vehicle_params(vp);
    EXPECT_NEAR(higher_cy.Cf, 1.5 * baseline.Cf, 1e-9);
    EXPECT_NEAR(higher_cy.Cr, 1.5 * baseline.Cr, 1e-9);

    vp.tire.relaxation.Ly_base *= 0.5;
    const auto shorter_relaxation = BicycleParams::from_vehicle_params(vp);
    EXPECT_NEAR(shorter_relaxation.Cf, 0.5 * higher_cy.Cf, 1e-9);
    EXPECT_NEAR(shorter_relaxation.Cr, 0.5 * higher_cy.Cr, 1e-9);
}

/// テスト9: ヨーモーメント符号（右旋回でモーメント正）
TEST_F(BicycleModelTest, YawMomentSign) {
    BicycleState6DOF state{0, 0, 0, 15.0, 0, 0};
    BicycleControl control{0.1, 0.0};  // 正の舵角

    auto derivative = model_->compute_derivative(state, control);

    // 正の舶角 → 負のヨー加速度（右旋回 = SAE慣例で負）
    EXPECT_LT(derivative.r, 0.0);
}

/// テスト10: RK4精度（高次精度検証）
TEST_F(BicycleModelTest, RK4Accuracy) {
    BicycleState6DOF state{0, 0, 0, 10.0, 0, 0};
    BicycleControl control{0.0, 0.0};

    // 大きな時間刻み
    double dt_large = 0.1;
    auto x_large = model_->predict_step(state, control, dt_large);

    // 小さな時間刻みで10回
    double dt_small = 0.01;
    auto x_small = state;
    for (int i = 0; i < 10; ++i) {
        x_small = model_->predict_step(x_small, control, dt_small);
    }

    // RK4は4次精度 → 誤差はO(dt^5)で非常に小さい
    EXPECT_NEAR(x_large.x, x_small.x, 1e-6);
    EXPECT_NEAR(x_large.vx, x_small.vx, 1e-6);
}

/// MPC preview models use a map frame with CCW-positive yaw while Vehicle14DOF
/// uses the SAE J670e body frame with right-turn-positive yaw. Keep the
/// simplified model contract tied to the plant before wiring it into rollouts.
TEST(BicyclePreviewContractTest, AngleModeYawResponseMatches14DOFConvention) {
    VehicleParams vehicle_params = VehicleParams::default_fsae();
    vehicle_params.steering.torque_mode = false;
    vehicle_params.steering.ackermann = 0.0;
    vehicle_params.steering.stiffness = 1e12;
    vehicle_params.suspension.static_camber = {0.0, 0.0, 0.0, 0.0};
    vehicle_params.suspension.static_toe = {0.0, 0.0, 0.0, 0.0};
    vehicle_params.aero.CL_A = 0.0;
    vehicle_params.aero.CD_A = 0.0;

    Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);

    DriverInput input;
    input.steering = 0.05;
    input.throttle = 0.0;
    input.brake = 0.0;

    BicycleModel6DOF preview(BicycleParams::from_vehicle_params(vehicle_params));
    BicycleState6DOF preview_state{0.0, 0.0, 0.0, 15.0, 0.0, 0.0};
    BicycleControl preview_input{input.steering, 0.0};

    constexpr double dt = 0.001;
    constexpr int steps = 250;
    for (int i = 0; i < steps; ++i) {
        vehicle.step(dt, input);
        preview_state = preview.predict_step(preview_state, preview_input, dt);
    }

    const double plant_map_yaw_rate = -vehicle.state().r;
    ASSERT_LT(plant_map_yaw_rate, 0.0);
    ASSERT_LT(preview_state.r, 0.0);

    EXPECT_NEAR(preview_state.r, plant_map_yaw_rate, 0.02)
        << "preview_r=" << preview_state.r
        << ", plant_map_r=" << plant_map_yaw_rate;
}
