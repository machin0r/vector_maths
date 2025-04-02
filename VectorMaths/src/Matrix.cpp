#include "../include/Matrix.hpp"
#include "../include/Quaternion.hpp"

#include <cmath>

// Mat3

Vec3 Mat3::operator*(const Vec3& other) const {
	float x = Vec3(m[0], m[1], m[2]).dot(other);
	float y = Vec3(m[3], m[4], m[5]).dot(other);
	float z = Vec3(m[6], m[7], m[8]).dot(other);

	return Vec3(x, y, z);
}

Mat3 Mat3::operator*(const Mat3& other) const {
	float result[9];

	Vec3 otherCol0(other.m[0], other.m[3], other.m[6]);
	Vec3 otherCol1(other.m[1], other.m[4], other.m[7]);
	Vec3 otherCol2(other.m[2], other.m[5], other.m[8]);

	Vec3 resultCol0 = (*this) * otherCol0;
	Vec3 resultCol1 = (*this) * otherCol1;
	Vec3 resultCol2 = (*this) * otherCol2;

	result[0] = resultCol0.x; result[1] = resultCol1.x; result[2] = resultCol2.x;
	result[3] = resultCol0.y; result[4] = resultCol1.y; result[5] = resultCol2.y;
	result[6] = resultCol0.z; result[7] = resultCol1.z; result[8] = resultCol2.z;

	return Mat3(result);
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
	float a = matrix.m[0] * ((matrix.m[4] * matrix.m[8]) - (matrix.m[7] * matrix.m[5]));
	float b = matrix.m[1] * ((matrix.m[3] * matrix.m[8]) - (matrix.m[6] * matrix.m[5]));
	float c = matrix.m[2] * ((matrix.m[3] * matrix.m[7]) - (matrix.m[6] * matrix.m[4]));

	return (a - b) + c;

}

// Mat4
Mat4 Mat4::operator*(const Mat4& other) const {	
	float result[16];

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			float dot_product = Vec4(m[(i*4)], m[(i * 4) + 1], m[(i * 4) + 2], m[(i * 4) + 3])
								.dot(Vec4(other.m[j], other.m[j + 4], other.m[j + 8], other.m[j + 12]));
			result[((i * 4) + j)] = dot_product;
		}
	}
	return Mat4(result);
}


Vec4 Mat4::operator*(const Vec4& other) const {
	float x = Vec4(m[0], m[1], m[2], m[3]).dot(other);
	float y = Vec4(m[4], m[5], m[6], m[7]).dot(other);
	float z = Vec4(m[8], m[9], m[10], m[11]).dot(other);
	float w = Vec4(m[12], m[13], m[14], m[15]).dot(other);
	
	return Vec4(x, y, z, w);
}

Mat4 Mat4::translation(const Vec3& translation) {
	Mat4 result = *this;

	result.m[12] += translation.x;  // X translation
	result.m[13] += translation.y;  // Y translation
	result.m[14] += translation.z; // Z translation

	return result;
}


//Mat4 Mat4::rotation(const Quaternion& rotation) {
//	Mat4 rotation_matrix = rotation.toRotationMatrix();
//
//	return *this * rotation_matrix;
//}


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


// Mat4 Mat4::perspective(float fov, float aspect, float near, float far);
// Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& target, const Vec4& up);


float Mat4::calculate_minor_determinant(const Mat4& matrix, int row, int column)
{
	float submatrix_vals[9];
	int index = 0;
	for (int i = 0; i < 4; i++) {
		if (i == row) continue;
		for (int j = 0; j < 4; j++) {
			if (j == column) continue;
			submatrix_vals[index] = matrix.m[(i * 4) + j];
			index++;
		}
	}
	Mat3 submatrix(submatrix_vals);
	return Mat3::determinant(submatrix);
}

float Mat4::determinant(const Mat4& matrix) {

	return matrix.m[0] * Mat4::calculate_minor_determinant(matrix, 0, 0) -
		matrix.m[1] * Mat4::calculate_minor_determinant(matrix, 0, 1) +
		matrix.m[2] * Mat4::calculate_minor_determinant(matrix, 0, 2) -
		matrix.m[3] * Mat4::calculate_minor_determinant(matrix, 0, 3);
}

Mat4 Mat4::inverse() const {

	float determinant = Mat4::determinant(*this);

	// Check for non-invertible matrix
	if (std::abs(determinant) < 1e-6f) {
		return Mat4();
	}

	float adjugate_matrix_values[16];
	int index = 0;

	// Iterate through the 4x4 matrix and calculate the detminant of all the submatrices
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			float sign = ((r + c) % 2 == 0) ? 1.0f : -1.0f;
			float val = sign * Mat4::calculate_minor_determinant(*this, r, c);
			adjugate_matrix_values[index] = val;
			index++;
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