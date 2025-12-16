/**
 * @file MatrixTests.cpp
 * @brief Unit tests for Mat3 and Mat4 classes
 */

#include <gtest/gtest.h>
#include "Matrix.hpp"
#include <cmath>


// ========== Mat3 Tests ==========

TEST(Mat3Test, DefaultConstructor) {
    Mat3 m;
    for (int i = 0; i < 9; i++) {
        if (i == 0 || i == 4 || i == 8) {
            EXPECT_FLOAT_EQ(m.m[i], 1.0f);
        }
        else {
            EXPECT_FLOAT_EQ(m.m[i], 0.0f);
        }
    }
}

TEST(Mat3Test, ParameterisedConstructor) {
    float values[9] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
    Mat3 m(values);

    for (int i = 0; i < 9; i++) {
        EXPECT_FLOAT_EQ(m.m[i], static_cast<float>(i));
    }
}

TEST(Mat3Test, ScalarMultiplication) {
    float values[9] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };
    Mat3 m(values);
    Mat3 res = m * 2.0f;
    EXPECT_FLOAT_EQ(res.m[0], 2.0f);
    EXPECT_FLOAT_EQ(res.m[1], 4.0f);
    EXPECT_FLOAT_EQ(res.m[2], 6.0f);
    EXPECT_FLOAT_EQ(res.m[3], 8.0f);
    EXPECT_FLOAT_EQ(res.m[4], 10.0f);
    EXPECT_FLOAT_EQ(res.m[5], 12.0f);
    EXPECT_FLOAT_EQ(res.m[6], 14.0f);
    EXPECT_FLOAT_EQ(res.m[7], 16.0f);
    EXPECT_FLOAT_EQ(res.m[8], 18.0f);

    Mat3 res2 = 3.0f * m;
    EXPECT_FLOAT_EQ(res2.m[0], 3.0f);
    EXPECT_FLOAT_EQ(res2.m[1], 6.0f);
    EXPECT_FLOAT_EQ(res2.m[2], 9.0f);
    EXPECT_FLOAT_EQ(res2.m[3], 12.0f);
    EXPECT_FLOAT_EQ(res2.m[4], 15.0f);
    EXPECT_FLOAT_EQ(res2.m[5], 18.0f);
    EXPECT_FLOAT_EQ(res2.m[6], 21.0f);
    EXPECT_FLOAT_EQ(res2.m[7], 24.0f);
    EXPECT_FLOAT_EQ(res2.m[8], 27.0f);
}

TEST(Mat3Test, ScalarDivision) {
    float values[9] = { 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f, 16.0f, 18.0f };
    Mat3 m(values);
    Mat3 res = m / 2.0f;
    EXPECT_FLOAT_EQ(res.m[0], 1.0f);
    EXPECT_FLOAT_EQ(res.m[1], 2.0f);
    EXPECT_FLOAT_EQ(res.m[2], 3.0f);
    EXPECT_FLOAT_EQ(res.m[3], 4.0f);
    EXPECT_FLOAT_EQ(res.m[4], 5.0f);
    EXPECT_FLOAT_EQ(res.m[5], 6.0f);
    EXPECT_FLOAT_EQ(res.m[6], 7.0f);
    EXPECT_FLOAT_EQ(res.m[7], 8.0f);
    EXPECT_FLOAT_EQ(res.m[8], 9.0f);
}


TEST(Mat3Test, MatrixMultiplication) {
    float values1[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 }; // Col-major: [1 4 7; 2 5 8; 3 6 9]
    float values2[9] = { 9, 8, 7, 6, 5, 4, 3, 2, 1 }; // Col-major: [9 6 3; 8 5 2; 7 4 1]
    Mat3 m1(values1);
    Mat3 m2(values2);
    Mat3 res = m1 * m2;

    // Column 0 (first column of result)
    // R0C0 = 1*9 + 4*8 + 7*7 = 9 + 32 + 49 = 90
    // R1C0 = 2*9 + 5*8 + 8*7 = 18 + 40 + 56 = 114
    // R2C0 = 3*9 + 6*8 + 9*7 = 27 + 48 + 63 = 138

    // Column 1 (second column of result)
    // R0C1 = 1*6 + 4*5 + 7*4 = 6 + 20 + 28 = 54
    // R1C1 = 2*6 + 5*5 + 8*4 = 12 + 25 + 32 = 69
    // R2C1 = 3*6 + 6*5 + 9*4 = 18 + 30 + 36 = 84

    // Column 2 (third column of result)
    // R0C2 = 1*3 + 4*2 + 7*1 = 3 + 8 + 7 = 18
    // R1C2 = 2*3 + 5*2 + 8*1 = 6 + 10 + 8 = 24
    // R2C2 = 3*3 + 6*2 + 9*1 = 9 + 12 + 9 = 30

    EXPECT_FLOAT_EQ(res.m[0], 90.0f);   // row 0, col 0
    EXPECT_FLOAT_EQ(res.m[1], 114.0f);  // row 1, col 0
    EXPECT_FLOAT_EQ(res.m[2], 138.0f);  // row 2, col 0
    EXPECT_FLOAT_EQ(res.m[3], 54.0f);   // row 0, col 1
    EXPECT_FLOAT_EQ(res.m[4], 69.0f);   // row 1, col 1
    EXPECT_FLOAT_EQ(res.m[5], 84.0f);   // row 2, col 1
    EXPECT_FLOAT_EQ(res.m[6], 18.0f);   // row 0, col 2
    EXPECT_FLOAT_EQ(res.m[7], 24.0f);   // row 1, col 2
    EXPECT_FLOAT_EQ(res.m[8], 30.0f);   // row 2, col 2
}

TEST(Mat3Test, Transpose) {
    float values[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 }; // Col-major: [1 4 7; 2 5 8; 3 6 9]
    Mat3 m(values);
    Mat3 transposed = m.transpose();

    // Expected transpose (col-major): [1 2 3; 4 5 6; 7 8 9] (which is row-major of original)
    EXPECT_FLOAT_EQ(transposed.m[0], 1.0f);
    EXPECT_FLOAT_EQ(transposed.m[1], 4.0f);
    EXPECT_FLOAT_EQ(transposed.m[2], 7.0f);
    EXPECT_FLOAT_EQ(transposed.m[3], 2.0f);
    EXPECT_FLOAT_EQ(transposed.m[4], 5.0f);
    EXPECT_FLOAT_EQ(transposed.m[5], 8.0f);
    EXPECT_FLOAT_EQ(transposed.m[6], 3.0f);
    EXPECT_FLOAT_EQ(transposed.m[7], 6.0f);
    EXPECT_FLOAT_EQ(transposed.m[8], 9.0f);
}

TEST(Mat3Test, AtMethod) {
    float values[9] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 }; // Col-major: [0 3 6; 1 4 7; 2 5 8]
    Mat3 m(values);

    EXPECT_FLOAT_EQ(m.at(0, 0), 0.0f);
    EXPECT_FLOAT_EQ(m.at(1, 0), 1.0f);
    EXPECT_FLOAT_EQ(m.at(2, 0), 2.0f);
    EXPECT_FLOAT_EQ(m.at(0, 1), 3.0f);
    EXPECT_FLOAT_EQ(m.at(1, 1), 4.0f);
    EXPECT_FLOAT_EQ(m.at(2, 1), 5.0f);
    EXPECT_FLOAT_EQ(m.at(0, 2), 6.0f);
    EXPECT_FLOAT_EQ(m.at(1, 2), 7.0f);
    EXPECT_FLOAT_EQ(m.at(2, 2), 8.0f);
}

TEST(Mat3Test, Determinant) {
    // Example 1: Identity matrix
    Mat3 identity; // Default constructor makes identity
    EXPECT_FLOAT_EQ(Mat3::determinant(identity), 1.0f);

    // Example 2: Simple matrix
    float values1[9] = { 1, 0, 0, 0, 2, 0, 0, 0, 3 }; // Col-major: [1 0 0; 0 2 0; 0 0 3]
    Mat3 m1(values1);
    EXPECT_FLOAT_EQ(Mat3::determinant(m1), 6.0f); // 1*(2*3 - 0*0) = 6

    // Example 3: Non-zero determinant
    float values2[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 }; // Col-major: [1 4 7; 2 5 8; 3 6 9]
    Mat3 m2(values2);
    // Determinant of [1 4 7; 2 5 8; 3 6 9] should be 0
    EXPECT_FLOAT_EQ(Mat3::determinant(m2), 0.0f);
}

TEST(Mat3Test, Equality) {
    float values1[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    float values2[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    float values3[9] = { 9, 8, 7, 6, 5, 4, 3, 2, 1 };

    Mat3 m1(values1);
    Mat3 m2(values2);
    Mat3 m3(values3);

    EXPECT_TRUE(m1 == m2);
    EXPECT_FALSE(m1 == m3);
    EXPECT_TRUE(m1 != m3);
    EXPECT_FALSE(m1 != m2);
}

TEST(Mat3Test, VectorMultiplication) {
    Mat3 m; // Identity matrix
    Vec3 v(1.0f, 2.0f, 3.0f);

    Vec3 res = m * v;

    EXPECT_FLOAT_EQ(res.x, 1.0f);
    EXPECT_FLOAT_EQ(res.y, 2.0f);
    EXPECT_FLOAT_EQ(res.z, 3.0f);
}

// ========== Mat4 Tests ==========

TEST(Mat4Test, DefaultConstructor) {
    Mat4 m;
    for (int i = 0; i < 16; i++) {
        if (i == 0 || i == 5 || i == 10 || i == 15) {
            EXPECT_FLOAT_EQ(m.m[i], 1.0f);
        }
        else {
            EXPECT_FLOAT_EQ(m.m[i], 0.0f);
        }
    }
}

TEST(Mat4Test, ParameterisedConstructor) {
    float values[16];
    for (int i = 0; i < 16; i++) values[i] = static_cast<float>(i);
    Mat4 m(values);

    for (int i = 0; i < 16; i++) {
        EXPECT_FLOAT_EQ(m.m[i], static_cast<float>(i));
    }
}

TEST(Mat4Test, MatrixMultiplication) {
    float values1[16] = {
        1, 2, 3, 4,
        5, 6, 7, 8,
        9,10,11,12,
       13,14,15,16
    };
    float values2[16] = {
        16,15,14,13,
        12,11,10,9,
        8,7,6,5,
        4,3,2,1
    };
    Mat4 m1(values1);
    Mat4 m2(values2);

    Mat4 res = m1 * m2;

    // Column 0
    EXPECT_FLOAT_EQ(res.m[0], 386.0f);
    EXPECT_FLOAT_EQ(res.m[1], 444.0f);
    EXPECT_FLOAT_EQ(res.m[2], 502.0f);
    EXPECT_FLOAT_EQ(res.m[3], 560.0f);

    // Column 1
    EXPECT_FLOAT_EQ(res.m[4], 274.0f);
    EXPECT_FLOAT_EQ(res.m[5], 316.0f);
    EXPECT_FLOAT_EQ(res.m[6], 358.0f);
    EXPECT_FLOAT_EQ(res.m[7], 400.0f);

    // Column 2
    EXPECT_FLOAT_EQ(res.m[8], 162.0f);
    EXPECT_FLOAT_EQ(res.m[9], 188.0f);
    EXPECT_FLOAT_EQ(res.m[10], 214.0f);
    EXPECT_FLOAT_EQ(res.m[11], 240.0f);

    // Column 3
    EXPECT_FLOAT_EQ(res.m[12], 50.0f);
    EXPECT_FLOAT_EQ(res.m[13], 60.0f);
    EXPECT_FLOAT_EQ(res.m[14], 70.0f);
    EXPECT_FLOAT_EQ(res.m[15], 80.0f);
}

TEST(Mat4Test, Transpose) {
    float values[16] = {
        1,2,3,4,
        5,6,7,8,
        9,10,11,12,
        13,14,15,16
    };
    Mat4 m(values);
    Mat4 t = m.transpose();

    EXPECT_FLOAT_EQ(t.m[0], 1.0f);
    EXPECT_FLOAT_EQ(t.m[1], 5.0f);
    EXPECT_FLOAT_EQ(t.m[2], 9.0f);
    EXPECT_FLOAT_EQ(t.m[3], 13.0f);
    EXPECT_FLOAT_EQ(t.m[4], 2.0f);
    EXPECT_FLOAT_EQ(t.m[5], 6.0f);
    EXPECT_FLOAT_EQ(t.m[6], 10.0f);
    EXPECT_FLOAT_EQ(t.m[7], 14.0f);
    EXPECT_FLOAT_EQ(t.m[8], 3.0f);
    EXPECT_FLOAT_EQ(t.m[9], 7.0f);
    EXPECT_FLOAT_EQ(t.m[10], 11.0f);
    EXPECT_FLOAT_EQ(t.m[11], 15.0f);
    EXPECT_FLOAT_EQ(t.m[12], 4.0f);
    EXPECT_FLOAT_EQ(t.m[13], 8.0f);
    EXPECT_FLOAT_EQ(t.m[14], 12.0f);
    EXPECT_FLOAT_EQ(t.m[15], 16.0f);
}

TEST(Mat4Test, AtMethod) {
    float values[16];
    for (int i = 0;i < 16;i++) values[i] = static_cast<float>(i);
    Mat4 m(values);

    for (int r = 0;r < 4;r++) {
        for (int c = 0;c < 4;c++) {
            EXPECT_FLOAT_EQ(m.at(r, c), values[c * 4 + r]);
        }
    }
}

TEST(Mat4Test, DeterminantAndInverse) {
    // Identity matrix determinant = 1
    Mat4 m;
    EXPECT_FLOAT_EQ(Mat4::determinant(m), 1.0f);

    // Non-invertible matrix
    float values[16] = {
        1,0,0,0,
        0,1,0,0,
        0,0,0,0,
        0,0,0,1
    };
    Mat4 singular(values);
    EXPECT_FLOAT_EQ(Mat4::determinant(singular), 0.0f);

    // Inverse of identity = identity
    Mat4 inv = m.inverse();
    for (int i = 0;i < 16;i++) {
        EXPECT_FLOAT_EQ(inv.m[i], m.m[i]);
    }
}

TEST(Mat4Test, VectorMultiplication) {
    Mat4 m; // Identity
    Vec4 v(1, 2, 3, 4);
    Vec4 res = m * v;

    EXPECT_FLOAT_EQ(res.x, 1.0f);
    EXPECT_FLOAT_EQ(res.y, 2.0f);
    EXPECT_FLOAT_EQ(res.z, 3.0f);
    EXPECT_FLOAT_EQ(res.w, 4.0f);
}

TEST(Mat4Test, Translation) {
    Mat4 m; // Identity
    Vec3 t(1, 2, 3);
    Mat4 translated = m.translation(t);

    EXPECT_FLOAT_EQ(translated.m[12], 1.0f);
    EXPECT_FLOAT_EQ(translated.m[13], 2.0f);
    EXPECT_FLOAT_EQ(translated.m[14], 3.0f);
    EXPECT_FLOAT_EQ(translated.m[15], 1.0f); // Homogeneous coordinate
}

TEST(Mat4Test, Scaling) {
    Mat4 m; // Identity
    Vec3 s(2, 3, 4);
    Mat4 scaled = m.scale(s);

    EXPECT_FLOAT_EQ(scaled.m[0], 2.0f);
    EXPECT_FLOAT_EQ(scaled.m[5], 3.0f);
    EXPECT_FLOAT_EQ(scaled.m[10], 4.0f);
    EXPECT_FLOAT_EQ(scaled.m[15], 1.0f);
}

TEST(Mat4Test, LookAt) {
    Mat4 m;
    Vec3 eye(0, 0, 0);
    Vec3 target(0, 0, -1);
    Vec3 up(0, 1, 0);

    Mat4 view = m.lookAt(eye, target, up);

    // Only basic sanity check: last row should be [0,0,0,1]
    EXPECT_FLOAT_EQ(view.m[12], 0.0f);
    EXPECT_FLOAT_EQ(view.m[13], 0.0f);
    EXPECT_FLOAT_EQ(view.m[14], 0.0f);
    EXPECT_FLOAT_EQ(view.m[15], 1.0f);
}
