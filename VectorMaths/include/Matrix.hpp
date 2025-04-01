#pragma once
#include "Vector.hpp"
#include "Quaternion.hpp"

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

	template<typename T>
	friend inline Mat3 operator/(const T& scalar, const Mat3& mat) {
		return mat / scalar;
	}


	Vec3 operator*(const Vec3& other) const;
	Mat3 operator*(const Mat3& other) const;


	// Utility functions
	Mat3 transpose() const;

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

	template<typename T>
	friend inline Mat4 operator/(const T& scalar, const Mat4& mat) {
		return mat / scalar;
	}


	Mat4 operator*(const Mat4& other) const;
	Vec4 operator*(const Vec4& other) const;

	// Utility functions
	Mat4 translation(const Vec3& translation);
	Mat4 rotation(const Quaternion& rotation);
	Mat4 scale(const Vec3& scale);
	//Mat4 perspective(float fov, float aspect, float near, float far);
	//Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec4& up);

	Mat4 inverse() const;
	Mat4 transpose() const;

	static float determinant(const Mat4& matrix);
	static float calculate_minor_determinant(const Mat4& matrix, int row, int column);
};