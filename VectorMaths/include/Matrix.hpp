#pragma once
#include "Vector.hpp"
#include "Quaternion.hpp"
#include <cmath>

class Mat3 {
public:
	float m[9];

	// Constructors
	Mat3();
	Mat3(float[9]);

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

	Vec3 operator*(const Vec3& other) const;
	Mat3 operator*(const Mat3& other) const;


	// Utility functions
	Mat3 transpose() const;

	float at(int row, int col) const;


	static float determinant(const Mat3& matrix);

};

class Mat4 {
public:
	float m[16];

	// Constructors
	Mat4();
	Mat4(float[16]);

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


	Mat4 operator*(const Mat4& other) const;
	Vec4 operator*(const Vec4& other) const;

	// Utility functions
	Mat4 translation(const Vec3& translation);
	Mat4 local_rotation(const Quaternion& rotation);
	Mat4 world_rotation(const Quaternion& rotation);

	float at(int row, int col) const;

	Mat4 scale(const Vec3& scale);
	Mat4 perspective(float fov, float aspect, float near, float far);
	Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up);

	Mat4 inverse() const;
	Mat4 transpose() const;

	static float determinant(const Mat4& matrix);
	static float calculate_minor_determinant(const Mat4& matrix, int row, int column);
};