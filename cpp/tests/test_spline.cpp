// tests/test_spline.cpp
#include <gtest/gtest.h>
#include "fsae/math/spline.hpp"
#include <cmath>
#include <vector>

using namespace fsae::math;

class SplineTest : public ::testing::Test {
protected:
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> y = {0.0, 1.0, 4.0, 9.0, 16.0};  // y = x^2
};

TEST_F(SplineTest, InterpolatesAtKnots) {
    CubicSpline<double> spline;
    spline.build(x, y);
    
    // ノット点では正確に一致
    for (size_t i = 0; i < x.size(); ++i) {
        EXPECT_NEAR(spline(x[i]), y[i], 1e-10);
    }
}

TEST_F(SplineTest, InterpolatesBetweenKnots) {
    CubicSpline<double> spline;
    spline.build(x, y);
    
    // 中間点でも妥当な値
    double mid = spline(1.5);
    EXPECT_GT(mid, 1.0);  // y(1) = 1
    EXPECT_LT(mid, 4.0);  // y(2) = 4
    EXPECT_NEAR(mid, 2.25, 0.5);  // y = x^2 → 1.5^2 = 2.25
}

TEST_F(SplineTest, DerivativeIsReasonable) {
    CubicSpline<double> spline;
    spline.build(x, y);
    
    // 微分値（y = x^2 → y' = 2x）
    double deriv_at_2 = spline.derivative(2.0);
    EXPECT_NEAR(deriv_at_2, 4.0, 1.0);  // 2x = 4 at x=2
}

TEST_F(SplineTest, ClampsOutOfRange) {
    CubicSpline<double> spline;
    spline.build(x, y);
    
    // 範囲外はクランプされる
    double below = spline(-1.0);
    double above = spline(10.0);
    
    EXPECT_TRUE(std::isfinite(below));
    EXPECT_TRUE(std::isfinite(above));
}
