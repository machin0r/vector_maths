#pragma once
#include "Vector.hpp"

class Mat4;

class Quaternion {
public:
    float w, x, y, z;

    // Constructors
    Quaternion();
    Quaternion(float w, float x, float y, float z);

    // Basic operations
    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator*(const Quaternion& q) const; // Quaternion multiplication
    Quaternion operator*(float scalar) const;
    Quaternion operator/(float scalar) const;

    // Utility functions
    float length() const;

    Quaternion normalise() const;
    inline Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion inverse() const;

    // Conversion functions
    Mat4 toRotationMatrix() const;
    Vec3 toEulerAngles() const;
    static Quaternion fromEulerAngles(float pitch, float yaw, float roll);

    // Rotation functions
    // Vec3 rotateVector(const Vec3& v) const;

    // Interpolation
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t);
};