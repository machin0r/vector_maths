#pragma once
#include "Vector.hpp"

class Mat3;
class Mat4;

class Quaternion {
public:
    float w, x, y, z;

    // Constructors
    Quaternion();
    Quaternion(float w, float x, float y, float z);

    // Basic operations
    template<typename T>
    inline Quaternion operator*(const T scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    template<typename T>
    inline Quaternion operator/(const T scalar) const {
        return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
    }

    template<typename T>
    friend inline Quaternion operator*(const T scalar, const Quaternion& q) {
        return q * scalar;
    }

    template<typename T>
    friend inline Quaternion operator/(const T scalar, const Quaternion& q) {
        return q / scalar;
    }

    Quaternion operator+(const Quaternion q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    Quaternion operator*(const Quaternion& q) const; // Quaternion multiplication

    // Utility functions
    float length() const;

    Quaternion normalised() const;
    inline Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion inverse() const;

    // Conversion functions
    Mat4 toRotationMatrix() const;
    Vec3 toEulerAngles() const;
    static Quaternion fromEulerAngles(float pitch, float yaw, float roll);
    static Quaternion fromRotationMatrix(Mat3 rotMat);


    // Rotation functions
    Vec3 rotateVector(const Vec3& v) const;

    // Interpolation
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t);
};