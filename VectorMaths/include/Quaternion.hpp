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

    inline bool operator==(const Quaternion& q) const {
        // Mathematically, quaternions q and -q represent the same rotation.
        float epsilon = 0.0001f;
        bool same = std::abs(w - q.w) < epsilon && std::abs(x - q.x) < epsilon &&
            std::abs(y - q.y) < epsilon && std::abs(z - q.z) < epsilon;
        bool opposite = std::abs(w + q.w) < epsilon && std::abs(x + q.x) < epsilon &&
            std::abs(y + q.y) < epsilon && std::abs(z + q.z) < epsilon;
        return same || opposite;
    }

    inline bool operator!=(const Quaternion& q) const {
        return !(*this == q);
    }

    friend inline std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        return os << "Quat(" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << ")";
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