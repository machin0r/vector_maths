#include "../include/Quaternion.hpp"
#include "../include/Vector.hpp"
#include "../include/Matrix.hpp"

#include <cmath>

// Quaternion

float Quaternion::length() const {
	return std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

Quaternion Quaternion::normalise() const {
	float magnitude = length();
	return Quaternion((w / magnitude), (x / magnitude), (y / magnitude), (z / magnitude));
}

Quaternion Quaternion::inverse() const {
	float mag = length();
	float mag_sqrd = mag * mag;
	Quaternion conjugated_matrix = conjugate();
	return Quaternion((conjugated_matrix.w / mag_sqrd), (conjugated_matrix.x / mag_sqrd), (conjugated_matrix.y / mag_sqrd), (conjugated_matrix.z / mag_sqrd));
}

Mat4 Quaternion::toRotationMatrix() const {
	/*
	 q = q0 + q1i + q2j + q3k
	*/

	float result[16] = { ((2 * ((w * w) + (x * x))) - 1), (2 * ((x * y) - (w * z))), (2 * ((x * z) + (w * y))), 0.0f,
		(2 * ((x * y) + (w * z))), ((2 * ((w * w) + (y * y))) - 1), (2 * (y * z) - (w * x)), 0.0f,
		(2 * ((x * z) - (w * y))), (2 * ((y * z) + (w * x))), ((2 * ((w * w) + (z * z))) - 1), 1.0f,
		0.0f, 0.0f, 0.0f, 1.0f };

	return Mat4(result);
}

Vec3 Quaternion::toEulerAngles() const {
	float roll = std::atan(2 * ((w * x) + (y * z))) / (1 - (2 * ((x * x) + (y * y))));
	float pitch = std::asin(2 * ((w * y) - (z * x)));
	float yaw = std::atan(2 * ((w * z) + (x * y))) / (1 - (2 * ((y * y) + (z * z))));

	return Vec3(roll, pitch, yaw);
}

Quaternion Quaternion::fromEulerAngles(float pitch, float yaw, float roll) {
	float w = (std::cos(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2)) + (std::sin(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2));
	float x = (std::sin(roll / 2) * std::cos(pitch / 2) * std::cos(yaw / 2)) + (std::cos(roll / 2) * std::sin(pitch / 2) * std::sin(yaw / 2));
	float y = (std::cos(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2)) + (std::sin(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2));
	float z = (std::cos(roll / 2) * std::cos(pitch / 2) * std::sin(yaw / 2)) + (std::sin(roll / 2) * std::sin(pitch / 2) * std::cos(yaw / 2));

	return Quaternion(w, x, y, z);
}


