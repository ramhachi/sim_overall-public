// tests/test_rotation.cpp
#include <gtest/gtest.h>
#include "fsae/math/rotation.hpp"
#include <cmath>

using namespace fsae::math;

TEST(RotationTest, IdentityRotation) {
    EulerAngles<double> euler{0.0, 0.0, 0.0};
    auto R = euler_to_rotation_matrix(euler);
    
    // 単位行列に近いはず
    EXPECT_NEAR(R(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(R(1, 1), 1.0, 1e-10);
    EXPECT_NEAR(R(2, 2), 1.0, 1e-10);
    EXPECT_NEAR(R(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(R(0, 2), 0.0, 1e-10);
}

TEST(RotationTest, YawRotation90) {
    EulerAngles<double> euler{0.0, 0.0, M_PI / 2.0};  // 90度ヨー
    auto R = euler_to_rotation_matrix(euler);
    
    // X軸がY軸方向を向く
    EXPECT_NEAR(R(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(R(0, 1), 1.0, 1e-10);
}

TEST(RotationTest, GravityInBody) {
    EulerAngles<double> euler{0.0, 0.0, 0.0};
    auto g = gravity_in_body(euler, 9.81);
    
    // SAE座標系：Z下向きが正
    EXPECT_NEAR(g(0), 0.0, 1e-10);
    EXPECT_NEAR(g(1), 0.0, 1e-10);
    EXPECT_NEAR(g(2), 9.81, 1e-10);
}

TEST(RotationTest, PitchAffectsGravity) {
    EulerAngles<double> euler{0.0, M_PI / 6.0, 0.0};  // 30度ピッチ
    auto g = gravity_in_body(euler, 9.81);
    
    // X方向に重力成分が現れる
    EXPECT_LT(g(0), 0.0);  // 前下がり→負のX成分
    EXPECT_GT(g(2), 0.0);  // Z成分は減少するが正
}
