#include "../include/Quaternion.hpp"
#include "../include/Vector.hpp"
#include "../include/Matrix.hpp"

#include <cmath>

// Quaternion
Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator*(const Quaternion& q) const {
	/*
	* base_quat = w + xi + yj + zk
	* input_quat = q.w + q.xi + q.yj + q.zk
	* output_quat = to + t1i + t2j + t3k
	*/
	float t0 = (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z);
	float t1 = (w * q.x) + (x * q.w) - (y * q.z) + (z * q.y);
	float t2 = (w * q.y) + (x * q.z) + (y * q.w) - (z * q.x);
	float t3 = (w * q.z) - (x * q.y) + (y * q.x) + (z * q.w);

	return Quaternion(t0, t1, t2, t3);
}

float Quaternion::length() const {
	return std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

Quaternion Quaternion::normalised() const {
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
	float result[16] = {
			((2 * ((w * w) + (x * x))) - 1), (2 * ((x * y) + (w * z))), (2 * ((x * z) - (w * y))), 0.0f,
			(2 * ((x * y) - (w * z))), ((2 * ((w * w) + (y * y))) - 1), (2 * ((y * z) + (w * x))), 0.0f,
			(2 * ((x * z) + (w * y))), (2 * (y * z) - (w * x)), ((2 * ((w * w) + (z * z))) - 1), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

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

Quaternion Quaternion::fromRotationMatrix(Mat3 rotMat) {
	float w, x, y, z;

	float trace = rotMat.m[0] + rotMat.m[4] + rotMat.m[8];

	if (trace > 0) {
		w = std::sqrt((trace + 1) / 2);
		x = (rotMat.m[5] - rotMat.m[7]) / (4 * w);
		y = (rotMat.m[6] - rotMat.m[2]) / (4 * w);
		z = (rotMat.m[1] - rotMat.m[3]) / (4 * w);
	}
	else {
		if ((rotMat.m[0] > rotMat.m[4]) && (rotMat.m[0] > rotMat.m[8])) {
			x = std::sqrt((1 + (rotMat.m[0] - rotMat.m[4] - rotMat.m[8])) / 2);
			y = (rotMat.m[3] + rotMat.m[1]) / (4 * x);
			z = (rotMat.m[6] + rotMat.m[2]) / (4 * x);
			w = (rotMat.m[5] - rotMat.m[7]) / (4 * x);

		}
		else if (rotMat.m[4] > rotMat.m[8]) {
			y = std::sqrt((1 + (rotMat.m[4] - rotMat.m[0] - rotMat.m[8])) / 2);
			z = (rotMat.m[7] + rotMat.m[5]) / (4 * y);
			w = (rotMat.m[6] - rotMat.m[2]) / (4 * y);
			x = (rotMat.m[3] + rotMat.m[1]) / (4 * y);
		}
		else {
			z = std::sqrt((1 + (rotMat.m[8] - rotMat.m[0] - rotMat.m[4])) / 2);
			w = (rotMat.m[1] - rotMat.m[3]) / (4 * z);
			x = (rotMat.m[6] + rotMat.m[2]) / (4 * z);
			y = (rotMat.m[7] + rotMat.m[5]) / (4 * z);
		}
	}

	Quaternion rotationQuaternion(w, x, y, z);
	rotationQuaternion = rotationQuaternion.normalised();

	return rotationQuaternion;
}


Vec3 Quaternion::rotateVector(const Vec3& v) const {
	Quaternion unit_quat = normalised();

	Quaternion vectorQ(0, v.x, v.y, v.z);

	Quaternion result = unit_quat * vectorQ * unit_quat.conjugate();

	return Vec3(result.x, result.y, result.z);
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, float t) {
	float angle = std::acos(a.w * b.w) + (a.x * b.x) + (a.y * b.y) + (a.z * b.z);

	float first_factor = std::sin((1 - t) * angle) / std::sin(angle);
	float second_factor = std::sin(t * angle) / std::sin(angle);

	return Quaternion((first_factor * a) + (second_factor * b));
}


