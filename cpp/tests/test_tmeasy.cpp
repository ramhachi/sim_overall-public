// tests/test_tmeasy.cpp
#include <gtest/gtest.h>
#include "fsae/physics/tire/tmeasy.hpp"
#include <cmath>

using namespace fsae::physics::tire;

class TMeasyTest : public ::testing::Test {
protected:
    TMeasyParams<double> params;
    
    void SetUp() override {
        params.mu_x_max = 1.6;
        params.mu_y_max = 1.8;
        params.Fz_nom = 1500.0;
        params.Lx = 0.15;
        params.Ly = 0.25;
        params.v_N = 0.5;
    }
};

TEST_F(TMeasyTest, ZeroSlipProducesZeroForce) {
    TMeasyState<double> state{0.0, 0.0};
    double Fz = 1500.0;
    double vsx = 0.0, vsy = 0.0, vcx = 10.0;
    
    auto out = compute_tmeasy(params, state, Fz, vsx, vsy, vcx);
    
    EXPECT_NEAR(out.Fx, 0.0, 1e-6);
    EXPECT_NEAR(out.Fy, 0.0, 1e-6);
}

TEST_F(TMeasyTest, LateralSlipProducesLateralForce) {
    TMeasyState<double> state{0.0, 0.01};  // 横変形あり
    double Fz = 1500.0;
    double vsx = 0.0, vsy = 0.5, vcx = 10.0;
    
    auto out = compute_tmeasy(params, state, Fz, vsx, vsy, vcx);
    
    // 横力が発生していること
    EXPECT_LT(out.Fy, 0.0);  // 負（右スリップで左向き力）
    EXPECT_NEAR(out.Fx, 0.0, 100.0);  // 縦力は小さい
}

TEST_F(TMeasyTest, ZeroSpeedDoesNotDiverge) {
    TMeasyState<double> state{0.0, 0.01};
    double Fz = 1500.0;
    double vsx = 0.0, vsy = 0.1, vcx = 0.0;  // 速度ゼロ！
    
    // 特異点が発生しないことを確認
    EXPECT_NO_THROW({
        auto out = compute_tmeasy(params, state, Fz, vsx, vsy, vcx);
        EXPECT_TRUE(std::isfinite(out.Fx));
        EXPECT_TRUE(std::isfinite(out.Fy));
        EXPECT_TRUE(std::isfinite(out.dxi_dt));
        EXPECT_TRUE(std::isfinite(out.deta_dt));
    });
}

TEST_F(TMeasyTest, LoadSensitivity) {
    TMeasyState<double> state{0.0, 0.02};
    double vsx = 0.0, vsy = 0.5, vcx = 10.0;
    
    // 低荷重
    auto out_low = compute_tmeasy(params, state, 500.0, vsx, vsy, vcx);
    // 高荷重
    auto out_high = compute_tmeasy(params, state, 2500.0, vsx, vsy, vcx);
    
    // 高荷重の方が力は大きいが、摩擦係数は低下
    EXPECT_GT(std::abs(out_high.Fy), std::abs(out_low.Fy));
    
    // 正規化すると低荷重の方が高い摩擦係数
    double mu_low = std::abs(out_low.Fy) / 500.0;
    double mu_high = std::abs(out_high.Fy) / 2500.0;
    EXPECT_GT(mu_low, mu_high);
}

TEST_F(TMeasyTest, NominalLoadUsesPeakFrictionCoefficient) {
    TMeasyState<double> state{0.0, 1.0};

    auto out = compute_tmeasy(params, state, params.Fz_nom, 0.0, 0.0, 10.0);

    EXPECT_NEAR(std::abs(out.Fy), params.mu_y_max * params.Fz_nom, 1e-9);
}

TEST_F(TMeasyTest, LoadSensitivityUsesNominalLoadAsReference) {
    TMeasyState<double> state{0.0, 1.0};
    constexpr double low_load = 500.0;
    constexpr double high_load = 2500.0;

    auto out_low = compute_tmeasy(params, state, low_load, 0.0, 0.0, 10.0);
    auto out_high = compute_tmeasy(params, state, high_load, 0.0, 0.0, 10.0);

    const double expected_mu_low =
        params.mu_y_max + params.dmu_dFz * (low_load - params.Fz_nom);
    const double expected_mu_high =
        params.mu_y_max + params.dmu_dFz * (high_load - params.Fz_nom);
    EXPECT_NEAR(std::abs(out_low.Fy), expected_mu_low * low_load, 1e-9);
    EXPECT_NEAR(std::abs(out_high.Fy), expected_mu_high * high_load, 1e-9);
}

// #22: Cy is a carcass stiffness [N/m]; the steady-state cornering stiffness
// 2·Cy·Ly must reproduce the measured 22436 N/rad (the old Cy=22000 gave 11000).
TEST_F(TMeasyTest, SteadyStateCorneringStiffnessMatchesMeasured) {
    const double alpha = 1e-3;  // [rad]
    const double vcx = 10.0;
    TMeasyState<double> state{0.0, params.Ly * std::tan(alpha)};  // deta_dt = 0
    auto out = compute_tmeasy(params, state, params.Fz_nom, 0.0, vcx * std::tan(alpha), vcx);
    EXPECT_NEAR(out.deta_dt, 0.0, 1e-12);
    EXPECT_NEAR(std::abs(out.Fy) / alpha, 22436.0, 0.01 * 22436.0);
}

TEST_F(TMeasyTest, BatchComputation) {
    double xi[4] = {0, 0.01, 0.02, 0.03};
    double eta[4] = {0.01, 0.02, 0.03, 0.04};
    double Fz[4] = {1000, 1500, 2000, 2500};
    double vsx[4] = {0, 0, 0, 0};
    double vsy[4] = {0.5, 0.5, 0.5, 0.5};
    double vcx[4] = {10, 10, 10, 10};
    
    double Fx_out[4], Fy_out[4], dxi_out[4], deta_out[4];
    
    compute_tmeasy_batch(params, xi, eta, Fz, vsx, vsy, vcx,
                         Fx_out, Fy_out, dxi_out, deta_out);
    
    // 全て有限値であること
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(std::isfinite(Fx_out[i]));
        EXPECT_TRUE(std::isfinite(Fy_out[i]));
    }
}
