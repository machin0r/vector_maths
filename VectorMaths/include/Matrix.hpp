/**
 * @file Matrix.hpp
 * @brief 3x3 and 4x4 matrix classes for linear transformations
 *
 * Provides Mat3 and Mat4 classes with standard matrix operations including
 * arithmetic, multiplication, transpose, inverse, and transformation matrices.
 * Matrices are stored in column-major order for OpenGL compatibility.
 */

#pragma once
#include "Vector.hpp"
#include "Quaternion.hpp"

#include <cmath>
#include <cassert>

/**
 * @brief 3x3 matrix class for 2D transformations and rotations
 *
 * Stored in column-major order. Commonly used for 2D transformations
 * and as rotation matrices in 3D.
 */
class Mat3 {
public:
	float m[9];  ///< Matrix elements in column-major order

	/// Default constructor - initializes to identity matrix
	Mat3();

	/**
	 * @brief Constructs a matrix from an array of 9 floats
	 * @param elements Array of 9 floats in column-major order
	 */
	Mat3(float elements[9]);

	// Basic operations
	template<typename T>
	inline Mat3 operator*(const T& scalar) const {
		float result[9];

		for (int i = 0; i < 9; i++) {
			result[i] = m[i] * scalar;
		}

		return Mat3(result);
	}

	template<typename T>
	friend inline Mat3 operator*(const T& scalar, const Mat3& mat) {
		return mat * scalar;
	}

	template<typename T>
	inline Mat3 operator/(const T& scalar) const {
		assert(scalar != 0 && "Division by zero in Mat3::operator/");
		float result[9];

		for (int i = 0; i < 9; i++) {
			result[i] = m[i] / scalar;
		}

		return Mat3(result);
	}

	inline bool operator==(const Mat3& other) const {
		float epsilon = 0.0001f;
		for (int i = 0; i < 9; i++) {
			if (std::abs(m[i] - other.m[i]) > epsilon) {
				return false;
			}
		}
		return true;
	}

	inline bool operator!=(const Mat3& other) const {
		return !(*this == other);
	}

	friend inline std::ostream& operator<<(std::ostream& os, const Mat3& mat) {
		os << "Mat3:\n";
		for (int i = 0; i < 9; i += 3) {
			os << "[" << mat.m[i] << ", " << mat.m[i + 1] << ", " << mat.m[i + 2] << "]\n";
		}
		return os;
	}

	/// Transforms a 3D vector by this matrix
	Vec3 operator*(const Vec3& other) const;

	/// Matrix multiplication
	Mat3 operator*(const Mat3& other) const;

	// Utility functions
	/// Returns the transpose of this matrix
	Mat3 transpose() const;

	/**
	 * @brief Accesses matrix element at given row and column
	 * @param row Row index (0-2)
	 * @param col Column index (0-2)
	 * @return Element at (row, col)
	 */
	float at(int row, int col) const;

	/// Computes the determinant of the matrix
	static float determinant(const Mat3& matrix);
};

/**
 * @brief 4x4 matrix class for 3D transformations
 *
 * Stored in column-major order. Used for 3D transformations including
 * translation, rotation, scaling, and projection matrices.
 */
class Mat4 {
public:
	float m[16];  ///< Matrix elements in column-major order

	/// Default constructor - initializes to identity matrix
	Mat4();

	/**
	 * @brief Constructs a matrix from an array of 16 floats
	 * @param elements Array of 16 floats in column-major order
	 */
	Mat4(float elements[16]);

	// Basic operations
	template<typename T>
	inline Mat4 operator*(const T& scalar) const {
		float result[16];

		for (int i = 0; i < 16; i++) {
			result[i] = m[i] * scalar;
		}

		return Mat4(result);
	}

	template<typename T>
	friend inline Mat4 operator*(const T& scalar, const Mat4& mat) {
		return mat * scalar;
	}

	template<typename T>
	inline Mat4 operator/(const T& scalar) const {
		assert(scalar != 0 && "Division by zero in Mat4::operator/");
		float result[16];

		for (int i = 0; i < 16; i++) {
			result[i] = m[i] / scalar;
		}

		return Mat4(result);
	}

	inline bool operator==(const Mat4& other) const {
		float epsilon = 0.0001f;
		for (int i = 0; i < 16; i++) {
			if (std::abs(m[i] - other.m[i]) > epsilon) {
				return false;
			}
		}
		return true;
	}

	inline bool operator!=(const Mat4& other) const {
		return !(*this == other);
	}

	friend inline std::ostream& operator<<(std::ostream& os, const Mat4& mat) {
		os << "Mat4:\n";
		for (int i = 0; i < 16; i += 4) {
			os << "[" << mat.m[i] << ", " << mat.m[i + 1] << ", " << mat.m[i + 2] << ", " << mat.m[i + 3] << "]\n";
		}
		return os;
	}


	/// Matrix multiplication
	Mat4 operator*(const Mat4& other) const;

	/// Transforms a 4D vector by this matrix
	Vec4 operator*(const Vec4& other) const;

	// Transformation matrices
	/// Creates a translation matrix from a 3D offset
	Mat4 translation(const Vec3& translation);

	/// Creates a rotation matrix from quaternion (local/object space)
	Mat4 local_rotation(const Quaternion& rotation);

	/// Creates a rotation matrix from quaternion (world space)
	Mat4 world_rotation(const Quaternion& rotation);

	/// Creates a non-uniform scale matrix
	Mat4 scale(const Vec3& scale);

	/**
	 * @brief Creates a perspective projection matrix
	 * @param fov Field of view in radians
	 * @param aspect Aspect ratio (width/height)
	 * @param near Near clipping plane distance
	 * @param far Far clipping plane distance
	 * @return Perspective projection matrix
	 */
	Mat4 perspective(float fov, float aspect, float near, float far);

	/**
	 * @brief Creates an orthographic projection matrix
	 * @param left Left boundary
	 * @param right Right boundary
	 * @param bottom Bottom boundary
	 * @param top Top boundary
	 * @param near Near clipping plane
	 * @param far Far clipping plane
	 * @return Orthographic projection matrix
	 */
	Mat4 ortho(const float& left, const float& right, const float& bottom, const float& top, const float& near, const float& far);

	/**
	 * @brief Creates a view matrix (camera transformation)
	 * @param eye Camera position
	 * @param target Point to look at
	 * @param up Up direction vector
	 * @return View matrix
	 */
	Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up);

	/**
	 * @brief Accesses matrix element at given row and column
	 * @param row Row index (0-3)
	 * @param col Column index (0-3)
	 * @return Element at (row, col)
	 */
	float at(int row, int col) const;

	/// Returns the inverse of this matrix
	Mat4 inverse() const;

	/// Returns the transpose of this matrix
	Mat4 transpose() const;

	/// Computes the determinant of the matrix
	static float determinant(const Mat4& matrix);

	/// Helper function to calculate minor determinant for matrix inversion
	static float calculate_minor_determinant(const Mat4& matrix, int row, int column);
};