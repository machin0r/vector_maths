/**
 * @file QuaternionTests.cpp
 * @brief Unit tests for Quaternion class
 */

#include <gtest/gtest.h>
#include "Quaternion.hpp"
#include "Vector.hpp"
#include "Matrix.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST(QuaternionTest, DefaultConstructor) {
    Quaternion q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionTest, ParameterConstructor) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 2.0f);
    EXPECT_FLOAT_EQ(q.y, 3.0f);
    EXPECT_FLOAT_EQ(q.z, 4.0f);
}

TEST(QuaternionTest, Multiplication) {
    Quaternion q1(1, 0, 1, 0);
    Quaternion q2(1, 0.5f, 0.5f, 0.75f);

    Quaternion res = q1 * q2;

    EXPECT_FLOAT_EQ(res.w, 0.5f);
    EXPECT_FLOAT_EQ(res.x, -0.25f);
    EXPECT_FLOAT_EQ(res.y, 1.5f);
    EXPECT_FLOAT_EQ(res.z, 1.25f);
}

TEST(QuaternionTest, LengthAndNormalisation) {
    Quaternion q(2, 0, 0, 0);
    EXPECT_FLOAT_EQ(q.length(), 2.0f);

    Quaternion n = q.normalised();
    EXPECT_NEAR(n.length(), 1.0f, 1e-6f);
    EXPECT_FLOAT_EQ(n.w, 1.0f);
    EXPECT_FLOAT_EQ(n.x, 0.0f);
    EXPECT_FLOAT_EQ(n.y, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 0.0f);
}

TEST(QuaternionTest, ConjugateAndInverse) {
    Quaternion q(1, 2, 3, 4);
    Quaternion inv = q.inverse();
    Quaternion id = q * inv;

    EXPECT_NEAR(id.w, 1.0f, 1e-6f);
    EXPECT_NEAR(id.x, 0.0f, 1e-6f);
    EXPECT_NEAR(id.y, 0.0f, 1e-6f);
    EXPECT_NEAR(id.z, 0.0f, 1e-6f);
}

TEST(QuaternionTest, FromEulerAndToEuler) {
    float pitch = M_PI / 4; // 45 degrees
    float yaw = M_PI / 6; // 30 degrees
    float roll = M_PI / 3; // 60 degrees

    Quaternion q = Quaternion::fromEulerAngles(pitch, yaw, roll);
    Vec3 euler = q.toEulerAngles();
    Quaternion q2 = Quaternion::fromEulerAngles(
        euler.y, euler.z, euler.x
    );

    EXPECT_NEAR((q - q2).length(), 0.0f, 1e-5f);
}

TEST(QuaternionTest, FromAxisAngleAndToAxisAngle) {
    Vec3 axis(0, 1, 0);
    float angle = M_PI / 2; // 90 degrees

    Quaternion q = Quaternion::fromAxisAngle(axis, angle);
    AxisAngle aa = q.toAxisAngle();

    EXPECT_NEAR(aa.angle, angle, 1e-5f);
    EXPECT_NEAR(aa.axis.x, axis.x, 1e-5f);
    EXPECT_NEAR(aa.axis.y, axis.y, 1e-5f);
    EXPECT_NEAR(aa.axis.z, axis.z, 1e-5f);
}

TEST(QuaternionTest, RotateVector) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2); // 90 deg around Z
    Vec3 v(1, 0, 0);

    Vec3 rotated = q.rotateVector(v);

    EXPECT_NEAR(rotated.x, 0.0f, 1e-5f);
    EXPECT_NEAR(rotated.y, 1.0f, 1e-5f);
    EXPECT_NEAR(rotated.z, 0.0f, 1e-5f);
}

TEST(QuaternionTest, ToRotationMatrix) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2);
    Mat4 m = q.toRotationMatrix();

    // Basic sanity: last row should be [0 0 0 1]
    EXPECT_FLOAT_EQ(m.m[12], 0.0f);
    EXPECT_FLOAT_EQ(m.m[13], 0.0f);
    EXPECT_FLOAT_EQ(m.m[14], 0.0f);
    EXPECT_FLOAT_EQ(m.m[15], 1.0f);
}

TEST(QuaternionTest, Slerp) {
    Quaternion q1 = Quaternion::fromAxisAngle(Vec3(0, 1, 0), 0.0f);
    Quaternion q2 = Quaternion::fromAxisAngle(Vec3(0, 1, 0), M_PI);

    Quaternion mid = Quaternion::slerp(q1, q2, 0.5f);

    AxisAngle aa = mid.toAxisAngle();
    EXPECT_NEAR(aa.angle, M_PI / 2, 1e-5f);
}
