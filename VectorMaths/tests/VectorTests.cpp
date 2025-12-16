/**
 * @file VectorTests.cpp
 * @brief Unit tests for Vec2, Vec3, and Vec4 classes
 */

#include <gtest/gtest.h>
#include "Vector.hpp"
#include <cmath>

// ========== Vec2 Tests ==========

TEST(Vec2Test, DefaultConstructor) {
    Vec2 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
}

TEST(Vec2Test, ParameterisedConstructor) {
    Vec2 v(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v.x, 3.0f);
    EXPECT_FLOAT_EQ(v.y, 4.0f);
}

TEST(Vec2Test, Addition) {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);
    Vec2 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 4.0f);
    EXPECT_FLOAT_EQ(c.y, 6.0f);
}

TEST(Vec2Test, Subtraction) {
    Vec2 a(5.0f, 7.0f);
    Vec2 b(2.0f, 3.0f);
    Vec2 c = a - b;
    EXPECT_FLOAT_EQ(c.x, 3.0f);
    EXPECT_FLOAT_EQ(c.y, 4.0f);
}

TEST(Vec2Test, ScalarMultiplication) {
    Vec2 v(2.0f, 3.0f);
    Vec2 result = v * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST(Vec2Test, ScalarDivision) {
    Vec2 v(4.0f, 6.0f);
    Vec2 result = v / 2.0f;
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
}

TEST(Vec2Test, Length) {
    Vec2 v(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vec2Test, LengthSquared) {
    Vec2 v(3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v.lengthSquared(), 25.0f);
}

TEST(Vec2Test, Normalize) {
    Vec2 v(3.0f, 4.0f);
    Vec2 normalized = v.normalised();
    EXPECT_NEAR(normalized.length(), 1.0f, 0.0001f);
    EXPECT_FLOAT_EQ(normalized.x, 0.6f);
    EXPECT_FLOAT_EQ(normalized.y, 0.8f);
}

TEST(Vec2Test, DotProduct) {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);
    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 11.0f); // 1*3 + 2*4 = 11
}

TEST(Vec2Test, CrossProduct) {
    Vec2 a(2.0f, 0.0f);
    Vec2 b(0.0f, 3.0f);
    float cross = a.cross(b);
    EXPECT_FLOAT_EQ(cross, 6.0f); // 2*3 - 0*0 = 6
}

TEST(Vec2Test, Equality) {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(1.0f, 2.0f);
    Vec2 c(1.0f, 3.0f);
    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TEST(Vec2Test, Lerp) {
    Vec2 a(0.0f, 0.0f);
    Vec2 b(10.0f, 10.0f);
    Vec2 mid = Vec2::lerp(a, b, 0.5f);
    EXPECT_FLOAT_EQ(mid.x, 5.0f);
    EXPECT_FLOAT_EQ(mid.y, 5.0f);
}

TEST(Vec2Test, Distance) {
    Vec2 a(0.0f, 0.0f);
    Vec2 b(3.0f, 4.0f);
    float dist = Vec2::distance(a, b);
    EXPECT_FLOAT_EQ(dist, 5.0f);
}

// ========== Vec3 Tests ==========

TEST(Vec3Test, DefaultConstructor) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Test, ParameterisedConstructor) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Test, Addition) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    Vec3 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 5.0f);
    EXPECT_FLOAT_EQ(c.y, 7.0f);
    EXPECT_FLOAT_EQ(c.z, 9.0f);
}

TEST(Vec3Test, Length) {
    Vec3 v(1.0f, 2.0f, 2.0f);
    EXPECT_FLOAT_EQ(v.length(), 3.0f);
}

TEST(Vec3Test, Normalize) {
    Vec3 v(3.0f, 0.0f, 4.0f);
    Vec3 normalized = v.normalised();
    EXPECT_NEAR(normalized.length(), 1.0f, 0.0001f);
}

TEST(Vec3Test, DotProduct) {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);
    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 32.0f); // 1*4 + 2*5 + 3*6 = 32
}

TEST(Vec3Test, CrossProduct) {
    Vec3 a(1.0f, 0.0f, 0.0f);
    Vec3 b(0.0f, 1.0f, 0.0f);
    Vec3 cross = a.cross(b);
    EXPECT_FLOAT_EQ(cross.x, 0.0f);
    EXPECT_FLOAT_EQ(cross.y, 0.0f);
    EXPECT_FLOAT_EQ(cross.z, 1.0f);
}

TEST(Vec3Test, CrossProductRightHandRule) {
    // X cross Y should give Z
    Vec3 x(1.0f, 0.0f, 0.0f);
    Vec3 y(0.0f, 1.0f, 0.0f);
    Vec3 z = x.cross(y);
    EXPECT_NEAR(z.x, 0.0f, 0.0001f);
    EXPECT_NEAR(z.y, 0.0f, 0.0001f);
    EXPECT_NEAR(z.z, 1.0f, 0.0001f);
}

// ========== Vec4 Tests ==========

TEST(Vec4Test, DefaultConstructor) {
    Vec4 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
    EXPECT_FLOAT_EQ(v.w, 0.0f);
}

TEST(Vec4Test, ParameterisedConstructor) {
    Vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
    EXPECT_FLOAT_EQ(v.w, 4.0f);
}

TEST(Vec4Test, Addition) {
    Vec4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 b(5.0f, 6.0f, 7.0f, 8.0f);
    Vec4 c = a + b;
    EXPECT_FLOAT_EQ(c.x, 6.0f);
    EXPECT_FLOAT_EQ(c.y, 8.0f);
    EXPECT_FLOAT_EQ(c.z, 10.0f);
    EXPECT_FLOAT_EQ(c.w, 12.0f);
}

TEST(Vec4Test, DotProduct) {
    Vec4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vec4 b(5.0f, 6.0f, 7.0f, 8.0f);
    float dot = a.dot(b);
    EXPECT_FLOAT_EQ(dot, 70.0f); // 1*5 + 2*6 + 3*7 + 4*8 = 70
}
