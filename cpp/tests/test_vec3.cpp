// tests/test_vec3.cpp
#include <gtest/gtest.h>
#include "fsae/math/vec3.hpp"
#include <cmath>

using namespace fsae::math;

TEST(Vec3Test, DefaultConstructor) {
    Vec3d v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

TEST(Vec3Test, ParameterizedConstructor) {
    Vec3d v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, 2.0);
    EXPECT_DOUBLE_EQ(v.z, 3.0);
}

TEST(Vec3Test, Addition) {
    Vec3d a(1.0, 2.0, 3.0);
    Vec3d b(4.0, 5.0, 6.0);
    Vec3d c = a + b;
    
    EXPECT_DOUBLE_EQ(c.x, 5.0);
    EXPECT_DOUBLE_EQ(c.y, 7.0);
    EXPECT_DOUBLE_EQ(c.z, 9.0);
}

TEST(Vec3Test, DotProduct) {
    Vec3d a(1.0, 0.0, 0.0);
    Vec3d b(0.0, 1.0, 0.0);
    
    EXPECT_DOUBLE_EQ(a.dot(b), 0.0);
    EXPECT_DOUBLE_EQ(a.dot(a), 1.0);
}

TEST(Vec3Test, CrossProduct) {
    Vec3d x = Vec3d::unit_x();
    Vec3d y = Vec3d::unit_y();
    Vec3d z = x.cross(y);
    
    EXPECT_DOUBLE_EQ(z.x, 0.0);
    EXPECT_DOUBLE_EQ(z.y, 0.0);
    EXPECT_DOUBLE_EQ(z.z, 1.0);
}

TEST(Vec3Test, Norm) {
    Vec3d v(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

TEST(Vec3Test, Normalize) {
    Vec3d v(3.0, 4.0, 0.0);
    Vec3d n = v.normalized();
    
    EXPECT_NEAR(n.norm(), 1.0, 1e-10);
    EXPECT_DOUBLE_EQ(n.x, 0.6);
    EXPECT_DOUBLE_EQ(n.y, 0.8);
}
