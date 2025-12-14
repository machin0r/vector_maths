#include "../include/Matrix.hpp"
#include "../include/Quaternion.hpp"

#include <cmath>
#include <stdexcept>

// Mat3
Mat3::Mat3() {
	// Identity matrix
	m[0] = 1.0f; m[3] = 0.0f; m[6] = 0.0f;
	m[1] = 0.0f; m[4] = 1.0f; m[7] = 0.0f;
	m[2] = 0.0f; m[5] = 0.0f; m[8] = 1.0f;
}

Mat3::Mat3(float values[9]) {
	for (int i = 0; i < 9; i++) {
		m[i] = values[i];
	}
}

Vec3 Mat3::operator*(const Vec3& other) const {
	float x = Vec3(m[0], m[3], m[6]).dot(other);
	float y = Vec3(m[1], m[4], m[7]).dot(other);
	float z = Vec3(m[2], m[5], m[8]).dot(other);

	return Vec3(x, y, z);
}

Mat3 Mat3::operator*(const Mat3& other) const {
	float result[9];

	for (int j = 0; j < 3; j++) {  // Column index
		for (int i = 0; i < 3; i++) {  // Row index
			float dot_product = Vec3(m[i], m[i + 3], m[i + 6])
				.dot(Vec3(other.m[j * 3], other.m[j * 3 + 1], other.m[j * 3 + 2]));
			result[j * 3 + i] = dot_product;
		}
	}

	return Mat3(result);
}

float Mat3::at(int row, int col) const {
	if (row < 0 || row >= 3 || col < 0 || col >= 3) {
		throw std::out_of_range("Matrix index out of bounds");
	}
	return m[col * 3 + row];
}

Mat3 Mat3::transpose() const {
	float result[9] = {
		m[0], m[3], m[6],
		m[1], m[4], m[7],
		m[2], m[5], m[8]
	};
	return Mat3(result);
}

float Mat3::determinant(const Mat3& matrix) {
	float a = matrix.m[0] * ((matrix.m[4] * matrix.m[8]) - (matrix.m[5] * matrix.m[7]));
	float b = matrix.m[3] * ((matrix.m[1] * matrix.m[8]) - (matrix.m[2] * matrix.m[7]));
	float c = matrix.m[6] * ((matrix.m[1] * matrix.m[5]) - (matrix.m[2] * matrix.m[4]));

	return (a - b) + c;

}

// Mat4
Mat4::Mat4() {
	// Identity matrix
	m[0] = 1.0f; m[4] = 0.0f; m[8] = 0.0f; m[12] = 0.0f;
	m[1] = 0.0f; m[5] = 1.0f; m[9] = 0.0f; m[13] = 0.0f;
	m[2] = 0.0f; m[6] = 0.0f; m[10] = 1.0f; m[14] = 0.0f;
	m[3] = 0.0f; m[7] = 0.0f; m[11] = 0.0f; m[15] = 1.0f;

}

Mat4::Mat4(float values[16]) {
	for (int i = 0; i < 16; i++) {
		m[i] = values[i];
	}
}

Mat4 Mat4::operator*(const Mat4& other) const {
	float result[16];

	for (int j = 0; j < 4; j++) {  // Column index
		for (int i = 0; i < 4; i++) {  // Row index
			float dot_product = Vec4(m[i], m[i + 4], m[i + 8], m[i + 12])
				.dot(Vec4(other.m[j * 4], other.m[j * 4 + 1],
					other.m[j * 4 + 2], other.m[j * 4 + 3]));
			result[j * 4 + i] = dot_product;
		}
	}
	return Mat4(result);
}


Vec4 Mat4::operator*(const Vec4& other) const {
	float x = Vec4(m[0], m[4], m[8], m[12]).dot(other);
	float y = Vec4(m[1], m[5], m[9], m[13]).dot(other);
	float z = Vec4(m[2], m[6], m[10], m[14]).dot(other);
	float w = Vec4(m[3], m[7], m[11], m[15]).dot(other);

	return Vec4(x, y, z, w);
}

Mat4 Mat4::translation(const Vec3& translation) {
	Mat4 result = *this;

	result.m[12] += translation.x;  // X translation
	result.m[13] += translation.y;  // Y translation
	result.m[14] += translation.z; // Z translation

	return result;
}

float Mat4::at(int row, int col) const {
	if (row < 0 || row >= 4 || col < 0 || col >= 4) {
		throw std::out_of_range("Matrix index out of bounds");
	}
	return m[col * 4 + row];
}


Mat4 Mat4::local_rotation(const Quaternion& rotation) {
	Mat4 rotation_matrix = rotation.toRotationMatrix();

	return *this * rotation_matrix;
}

Mat4 Mat4::world_rotation(const Quaternion& rotation) {
	Mat4 rotation_matrix = rotation.toRotationMatrix();

	return rotation_matrix * *this;
}


Mat4 Mat4::scale(const Vec3& scale) {
	float scaling[16] = {
		scale.x, 0, 0, 0,
		0, scale.y, 0, 0,
		0, 0, scale.z, 0,
		0, 0, 0, 1
	};

	Mat4 scalingMatrix(scaling);

	return (*this) * scalingMatrix;
}


Mat4 Mat4::perspective(float fov, float aspect, float near, float far) {
	float tanHalfFov = std::tan(fov * 0.5f);

	float result[16];
	// Init to 0s
	for (int i = 0; i < 16; i++) {
		result[i] = 0.0f;
	}

	result[0] = 1.0f / (aspect * tanHalfFov);  // Scale X
	result[5] = 1.0f / tanHalfFov; // Scale Y
	result[10] = -(far + near) / (far - near); // Scale and translate Z
	result[11] = -1.0f;
	result[14] = -(2.0f * far * near) / (far - near);  // Translate Z

	Mat4 perspectiveMatrix(result);
	return perspectiveMatrix;
}

Mat4 Mat4::ortho(const float& left, const float& right, const float& bottom, const float& top, const float& near, const float& far) {

	float orthoTransformVals[16] = {
		(2 / (right - left)), 0.0f, 0.0f, 0.0f,
		0.0f, (2 / (top - bottom)), 0.0f, 0.0f,
		0.0f, 0.0f, (-2 / (far - near)), 0.0f,
		 -((right + left) / (right - left)), -((top + bottom) / (top - bottom)), -((far + near) / (far - near)),1.0f
	};

	Mat4 orthoMatrix(orthoTransformVals);

	return orthoMatrix;
}

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
	Vec3 forward = (eye - target).normalised();
	Vec3 right = up.cross(forward).normalised();
	Vec3 newUp = forward.cross(right);

	float viewMatVals[16] = {
		right.x, newUp.x, forward.x, 0.0f,
		right.y, newUp.y, forward.y, 0.0f,
		right.z, newUp.z, forward.z, 0.0f,
		0.0f,0.0f,0.0f,1.0f
	};

	Mat4 viewMatrix(viewMatVals);

	viewMatrix = viewMatrix.translation(Vec3(-eye.x, -eye.y, -eye.z));

	return viewMatrix;
}

float Mat4::calculate_minor_determinant(const Mat4& matrix, int row, int column)
{
	float submatrix_vals[9];
	int index = 0;
	for (int j = 0; j < 4; j++) {  // Column index
		if (j == column) continue;
		for (int i = 0; i < 4; i++) {  // Row index
			if (i == row) continue;
			submatrix_vals[index] = matrix.m[j * 4 + i];
			index++;
		}
	}
	Mat3 submatrix(submatrix_vals);
	return Mat3::determinant(submatrix);
}

float Mat4::determinant(const Mat4& matrix) {

	return matrix.m[0] * Mat4::calculate_minor_determinant(matrix, 0, 0) -
		matrix.m[4] * Mat4::calculate_minor_determinant(matrix, 0, 1) +
		matrix.m[8] * Mat4::calculate_minor_determinant(matrix, 0, 2) -
		matrix.m[12] * Mat4::calculate_minor_determinant(matrix, 0, 3);
}

Mat4 Mat4::inverse() const {

	float determinant = Mat4::determinant(*this);

	// Check for non-invertible matrix
	if (std::abs(determinant) < 1e-6f) {
		return Mat4();
	}

	float adjugate_matrix_values[16];

	// Iterate through the 4x4 matrix and calculate the detminant of all the submatrices
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			float sign = ((r + c) % 2 == 0) ? 1.0f : -1.0f;
			float val = sign * Mat4::calculate_minor_determinant(*this, r, c);
			adjugate_matrix_values[c * 4 + r] = val;
		}
	}

	Mat4 adjugate_matrix(adjugate_matrix_values);

	return adjugate_matrix * (1 / determinant);
}

Mat4 Mat4::transpose() const {
	float result[16] = {
		m[0], m[4], m[8], m[12],
		m[1], m[5], m[9], m[13],
		m[2], m[6], m[10], m[14],
		m[3], m[7], m[11], m[15]
	};
	return Mat4(result);
}