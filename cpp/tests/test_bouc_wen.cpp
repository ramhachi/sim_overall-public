// tests/test_bouc_wen.cpp
#include <gtest/gtest.h>
#include "fsae/physics/suspension/bouc_wen.hpp"
#include <cmath>

using namespace fsae::physics::suspension;

class BoucWenTest : public ::testing::Test {
protected:
    BoucWenParams<double> params;
    
    void SetUp() override {
        params.c0 = 2000.0;
        params.k0 = 0.0;
        params.alpha = 5000.0;
        params.A = 1.0;
        params.beta = 500.0;
        params.gamma = 100.0;
        params.n = 2.0;
    }
};

TEST_F(BoucWenTest, ZeroVelocityZeroForce) {
    double x = 0.0, v = 0.0, z = 0.0;
    
    double force = compute_bouc_wen_force(params, x, v, z);
    
    EXPECT_NEAR(force, 0.0, 1e-6);
}

TEST_F(BoucWenTest, LinearDampingAtLowVelocity) {
    double x = 0.0, z = 0.0;
    double v = 0.01;  // 低速
    
    double force = compute_bouc_wen_force(params, x, v, z);
    
    // 粘性減衰項が支配的
    EXPECT_NEAR(force, params.c0 * v, 1.0);
}

TEST_F(BoucWenTest, HysteresisVariableEvolution) {
    double v = 0.1;
    double z = 0.0;
    
    // 正の速度で履歴変数が増加
    double dz = compute_bouc_wen_dz(params, v, z);
    EXPECT_GT(dz, 0.0);
    
    // 履歴変数が大きくなると増加率が減少
    z = 0.5;
    double dz2 = compute_bouc_wen_dz(params, v, z);
    EXPECT_LT(dz2, dz);
}

TEST_F(BoucWenTest, VelocityReversalHysteresis) {
    // 正速度→負速度の反転でヒステリシスを確認
    // 注意: z=0ではヒステリシスは現れない。zが蓄積された状態でテストする必要がある。
    double x = 0.01;
    double z = 0.5;  // 非ゼロの履歴変数（正方向に動いた後の状態を模擬）
    
    // 正方向に動き続ける場合
    double v_pos = 0.1;
    auto out_pos = compute_bouc_wen(params, x, v_pos, z);
    
    // 反転（同じ変位、同じz状態で逆速度）
    double v_neg = -0.1;
    auto out_neg = compute_bouc_wen(params, x, v_neg, z);
    
    // zが非ゼロの場合、alpha*z項により力の絶対値が異なる
    // F = c0*v + k0*x + alpha*z
    // F_pos = 2000*0.1 + 0 + 5000*0.5 = 200 + 2500 = 2700
    // F_neg = 2000*(-0.1) + 0 + 5000*0.5 = -200 + 2500 = 2300
    // |F_pos| = 2700, |F_neg| = 2300 → 異なる！
    EXPECT_NE(std::abs(out_pos.force), std::abs(out_neg.force));
    
    // より具体的に：正方向の力が大きい
    EXPECT_GT(std::abs(out_pos.force), std::abs(out_neg.force));
}
