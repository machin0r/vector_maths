#pragma once
#include "Vector.hpp"

class Mat3 {
public:
	float m[9];

	Mat3();
	Mat3(float[9]);

	Mat3 operator*(const float& scalar) const;
	Mat3 operator*(const Mat3& other) const;
	Vec3 operator*(const Vec3& other) const;

	Mat3 transpose() const;

	static float determinant(const Mat3& matrix);

};

class Mat4 {
public:
	float m[16];

	Mat4();
	Mat4(float[16]);

	Mat4 operator*(const float& scalar) const;
	Mat4 operator*(const Mat4& other) const;
	Vec4 operator*(const Vec4& other) const;

	Mat4 translation(const Vec3& translation);
	Mat4 rotation(const Quaternion& rotation);
	Mat4 scale(const Vec3& scale);
	Mat4 perspective(float fov, float aspect, float near, float far);
	Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec4& up);

	Mat4 inverse() const;
	Mat4 transpose() const;

	static float determinant(const Mat4& matrix);
	static float calculate_minor_determinant(const Mat4& matrix, int row, int column);
};