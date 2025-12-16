/**
 * @file Quaternion.hpp
 * @brief Quaternion class for 3D rotations
 *
 * Provides quaternion operations including rotation, interpolation (slerp),
 * and conversions to/from matrices and euler angles. Quaternions avoid
 * gimbal lock and provide smooth interpolation between rotations.
 */

#pragma once
#include "Vector.hpp"

#include <cassert>

class Mat3;
class Mat4;

/**
 * @brief Axis-angle representation of a rotation
 *
 * Stores a rotation as an axis (normalized Vec3) and angle in radians.
 * This is often the most intuitive way to specify rotations.
 */
struct AxisAngle {
    Vec3 axis;    ///< Rotation axis (should be normalized)
    float angle;  ///< Rotation angle in radians
};

/**
 * @brief Quaternion class for representing 3D rotations
 *
 * Quaternions provide a robust way to represent rotations without gimbal lock.
 * They consist of a scalar part (w) and a vector part (x, y, z).
 * A unit quaternion (length = 1) represents a valid rotation.
 *
 * @note Quaternions q and -q represent the same rotation
 */
class Quaternion {
public:
    float w, x, y, z;  ///< Quaternion components (w is scalar, xyz is vector)

    /// Default constructor - creates identity quaternion (no rotation)
    Quaternion();

    /**
     * @brief Constructs a quaternion from components
     * @param w Scalar component
     * @param x X component of vector part
     * @param y Y component of vector part
     * @param z Z component of vector part
     */
    Quaternion(float w, float x, float y, float z);

    // Basic operations

    template<typename T>
    inline Quaternion operator*(const T scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    template<typename T>
    inline Quaternion operator/(const T scalar) const {
        assert(scalar != 0 && "Division by zero in Quaternion::operator/");
        return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
    }

    template<typename T>
    friend inline Quaternion operator*(const T scalar, const Quaternion& q) {
        return q * scalar;
    }

    template<typename T>
    friend inline Quaternion operator/(const T scalar, const Quaternion& q) {
        assert(scalar != 0 && "Division by zero in Quaternion::operator/");
        return q / scalar;
    }

    Quaternion operator+(const Quaternion q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }

    Quaternion operator-(const Quaternion& q) const {
        return Quaternion(
            w - q.w,
            x - q.x,
            y - q.y,
            z - q.z
        );
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

    /// Quaternion multiplication (concatenates rotations: q1 * q2 means "apply q2, then q1")
    Quaternion operator*(const Quaternion& q) const;

    // Utility functions
    /// Returns the length (magnitude) of the quaternion
    float length() const;

    /// Returns a normalized copy (unit quaternion for valid rotations)
    Quaternion normalised() const;

    /// Returns the conjugate (reverses rotation direction)
    inline Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /// Returns the inverse (opposite rotation)
    Quaternion inverse() const;

    // Conversion functions
    /// Converts quaternion to 4x4 rotation matrix
    Mat4 toRotationMatrix() const;

    /// Converts quaternion to euler angles (pitch, yaw, roll)
    Vec3 toEulerAngles() const;

    /**
     * @brief Creates a quaternion from euler angles
     * @param pitch Rotation around X axis (radians)
     * @param yaw Rotation around Y axis (radians)
     * @param roll Rotation around Z axis (radians)
     * @return Quaternion representing the rotation
     */
    static Quaternion fromEulerAngles(float pitch, float yaw, float roll);

    /// Creates a quaternion from a 3x3 rotation matrix
    static Quaternion fromRotationMatrix(Mat3 rotMat);

    /// Creates a quaternion from axis-angle struct
    static Quaternion fromAxisAngle(const AxisAngle& aa);

    /**
     * @brief Creates a quaternion from axis and angle
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     * @return Quaternion representing the rotation
     */
    static Quaternion fromAxisAngle(const Vec3& axis, float angle);

    /// Converts quaternion to axis-angle representation
    AxisAngle toAxisAngle() const;

    // Rotation functions
    /// Rotates a 3D vector by this quaternion
    Vec3 rotateVector(const Vec3& v) const;

    // Interpolation
    /**
     * @brief Spherical linear interpolation between two quaternions
     * @param a Start quaternion
     * @param b End quaternion
     * @param t Interpolation parameter (0 = a, 1 = b)
     * @return Interpolated quaternion with constant angular velocity
     */
    static Quaternion slerp(const Quaternion& a, Quaternion b, float t);
};