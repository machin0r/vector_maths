/**
 * @file Vector.hpp
 * @brief 2D, 3D, and 4D vector classes for linear algebra operations
 *
 * Provides Vec2, Vec3, and Vec4 classes with standard vector operations
 * including arithmetic, dot/cross products, and normalization.
 */

#pragma once
#include <cmath>
#include <iostream>
#include <cassert>

/**
 * @brief 2D vector class for 2D math and graphics
 *
 * Provides standard vector operations including arithmetic, dot product,
 * cross product (2D), length calculations, and interpolation.
 */
class Vec2 {
public:
	float x, y;  ///< Vector components

	/// Default constructor - initializes to zero vector
	Vec2();

	/**
	 * @brief Constructs a 2D vector with given components
	 * @param x X component
	 * @param y Y component
	 */
	Vec2(float x, float y);

	// Basic operations
	inline Vec2 operator+(const Vec2& other) const {
		return Vec2(x + other.x, y + other.y);
	}

	inline Vec2 operator-(const Vec2& other) const {
		return Vec2(x - other.x, y - other.y);
	}

	template<typename T>
	inline Vec2 operator*(const T scalar) const {
		return Vec2(x * scalar, y * scalar);
	}

	template<typename T>
	inline Vec2 operator/(const T scalar) const {
		assert(scalar != 0 && "Division by zero in Vec2::operator/");
		return Vec2(x / scalar, y / scalar);
	}

	inline bool operator==(const Vec2& other) const {
		float epsilon = 0.0001f;
		return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
	}

	inline bool operator!=(const Vec2& other) const {
		return !(*this == other);
	}

	template<typename T>
	friend inline Vec2 operator*(const T scalar, const Vec2& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec2 operator/(const T scalar, const Vec2& vec) {
		return vec / scalar;
	}

	friend inline std::ostream& operator<<(std::ostream& os, const Vec2& vec) {
		return os << "Vec2(" << vec.x << ", " << vec.y << ")";
	}

	// Utility functions
	/// Returns the length (magnitude) of the vector
	float length() const;

	/// Returns the squared length (avoids sqrt, faster for comparisons)
	float lengthSquared() const;

	/// Returns a normalized copy of this vector (length = 1)
	Vec2 normalised() const;

	/// Computes dot product with another vector
	float dot(const Vec2& other) const;

	/// Computes 2D cross product (returns scalar z-component)
	float cross(const Vec2& other) const;

	/// Linearly interpolates between two vectors (t=0 returns a, t=1 returns b)
	static Vec2 lerp(const Vec2& a, const Vec2& b, float t);

	/// Returns the distance between two vectors
	static float distance(const Vec2& a, const Vec2& b);
};

/**
 * @brief 3D vector class for 3D math and graphics
 *
 * Provides standard 3D vector operations including arithmetic, dot product,
 * cross product, length calculations, and interpolation.
 */
class Vec3 {
public:
	float x, y, z;  ///< Vector components

	/// Default constructor - initializes to zero vector
	Vec3();

	/**
	 * @brief Constructs a 3D vector with given components
	 * @param x X component
	 * @param y Y component
	 * @param z Z component
	 */
	Vec3(float x, float y, float z);

	// Basic operations
	inline Vec3 operator+(const Vec3& other) const {
		return Vec3(x + other.x, y + other.y, z + other.z);
	}

	inline Vec3 operator-(const Vec3& other) const {
		return Vec3(x -other.x, y - other.y, z - other.z);
	}

	template<typename T>
	inline Vec3 operator*(const T scalar) const {
		return Vec3(x * scalar, y * scalar, z * scalar);
	}

	template<typename T>
	inline Vec3 operator/(const T scalar) const {
		assert(scalar != 0 && "Division by zero in Vec3::operator/");
		return Vec3(x / scalar, y / scalar, z / scalar);
	}

	inline bool operator==(const Vec3& other) const {
		float epsilon = 0.0001f;
		return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon && std::abs(z - other.z) < epsilon;
	}

	inline bool operator!=(const Vec3& other) const {
		return !(*this == other);
	}

	template<typename T>
	friend inline Vec3 operator*(const T scalar, const Vec3& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec3 operator/(const T scalar, const Vec3& vec) {
		return vec / scalar;
	}

	friend inline std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
		return os << "Vec3(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
	}

	// Utility functions
	/// Returns the length (magnitude) of the vector
	float length() const;

	/// Returns the squared length (avoids sqrt, faster for comparisons)
	float lengthSquared() const;

	/// Returns a normalized copy of this vector (length = 1)
	Vec3 normalised() const;

	/// Computes dot product with another vector
	float dot(const Vec3& other) const;

	/// Computes 3D cross product (returns perpendicular vector)
	Vec3 cross(const Vec3& other) const;

	/// Linearly interpolates between two vectors (t=0 returns a, t=1 returns b)
	static Vec3 lerp(const Vec3& a, const Vec3& b, float t);

	/// Returns the distance between two vectors
	static float distance(const Vec3& a, const Vec3& b);
};

/**
 * @brief 4D vector class for homogeneous coordinates and quaternions
 *
 * Provides standard 4D vector operations. Commonly used for homogeneous
 * coordinates in graphics (x, y, z, w) where w is typically 1 for points
 * and 0 for directions.
 */
class Vec4 {
public:
	float x, y, z, w;  ///< Vector components

	/// Default constructor - initializes to zero vector
	Vec4();

	/**
	 * @brief Constructs a 4D vector with given components
	 * @param x X component
	 * @param y Y component
	 * @param z Z component
	 * @param w W component (homogeneous coordinate)
	 */
	Vec4(float x, float y, float z, float w);

	// Basic operations
	inline Vec4 operator+(const Vec4& other) const {
		return Vec4(x + other.x, y + other.y, z + other.z, w + other.w);
	}

	inline Vec4 operator-(const Vec4& other) const {
		return Vec4(x - other.x, y - other.y, z - other.z, w - other.w);
	}

	template<typename T>
	inline Vec4 operator*(const T scalar) const {
		return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
	}

	template<typename T>
	inline Vec4 operator/(const T scalar) const {
		assert(scalar != 0 && "Division by zero in Vec4::operator/");
		return Vec4(x / scalar, y / scalar, z / scalar, w / scalar);
	}

	inline bool operator==(const Vec4& other) const {
		float epsilon = 0.0001f;
		return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon && std::abs(z - other.z) < epsilon && std::abs(w - other.w) < epsilon;
	}

	inline bool operator!=(const Vec4& other) const {
		return !(*this == other);
	}

	template<typename T>
	friend inline Vec4 operator*(const T scalar, const Vec4& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec4 operator/(const T scalar, const Vec4& vec) {
		return vec / scalar;
	}

	friend inline std::ostream& operator<<(std::ostream& os, const Vec4& vec) {
		return os << "Vec4(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";
	}

	// Utility functions
	/// Returns the length (magnitude) of the vector
	float length() const;

	/// Returns the squared length (avoids sqrt, faster for comparisons)
	float lengthSquared() const;

	/// Returns a normalized copy of this vector (length = 1)
	Vec4 normalised() const;

	/// Computes dot product with another vector
	float dot(const Vec4& other) const;

	/// Linearly interpolates between two vectors (t=0 returns a, t=1 returns b)
	static Vec4 lerp(const Vec4& a, const Vec4& b, float t);

	/// Returns the distance between two vectors
	static float distance(const Vec4& a, const Vec4& b);
};