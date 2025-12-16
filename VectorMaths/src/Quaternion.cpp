/**
 * @file Quaternion.cpp
 * @brief Implementation of Quaternion class
 */

#include "../include/Quaternion.hpp"
#include "../include/Vector.hpp"
#include "../include/Matrix.hpp"

#include <cmath>
#include <algorithm>

// Quaternion
Quaternion::Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator*(const Quaternion& q) const {
	/*
	* base_quat = w + xi + yj + zk
	* input_quat = q.w + q.xi + q.yj + q.zk
	* output_quat = t0o + t1i + t2j + t3k
	*/
	float t0 = (q.w * w) - (q.x * x) - (q.y * y) - (q.z * z);
	float t1 = (q.w * x) + (q.x * w) + (q.y * z) - (q.z * y);
	float t2 = (q.w * y) - (q.x * z) + (q.y * w) + (q.z * x);
	float t3 = (q.w * z) + (q.x * y) - (q.y * x) + (q.z * w);

	return Quaternion(t0, t1, t2, t3);
}

float Quaternion::length() const {
	return std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
}

Quaternion Quaternion::normalised() const {
	float magnitude = length();

	if (magnitude < 1e-6f) {
		return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
	}

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
	// Roll (x-axis rotation)
	float sinr_cosp = 2.0f * (w * x + y * z);
	float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	float roll = std::atan2(sinr_cosp, cosr_cosp);

	// Pitch (y-axis rotation)
	float sinp = 2.0f * (w * y - z * x);
	float pitch = std::asin(sinp);

	// Yaw (z-axis rotation)
	float siny_cosp = 2.0f * (w * z + x * y);
	float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	float yaw = std::atan2(siny_cosp, cosy_cosp);

	return Vec3(roll, pitch, yaw);
}

Quaternion Quaternion::fromEulerAngles(float pitch, float yaw, float roll) {
	// Standard ZYX (Yaw-Pitch-Roll) Tait-Bryan angles
	float cy = std::cos(yaw * 0.5f);
	float sy = std::sin(yaw * 0.5f);
	float cp = std::cos(pitch * 0.5f);
	float sp = std::sin(pitch * 0.5f);
	float cr = std::cos(roll * 0.5f);
	float sr = std::sin(roll * 0.5f);

	float w = cr * cp * cy + sr * sp * sy;
	float x = sr * cp * cy - cr * sp * sy;
	float y = cr * sp * cy + sr * cp * sy;
	float z = cr * cp * sy - sr * sp * cy;

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

Quaternion Quaternion::fromAxisAngle(const AxisAngle& aa) {
	Vec3 normalisedAxis = aa.axis.normalised();
	float angle = aa.angle;

	float x = normalisedAxis.x * std::sin(angle / 2);
	float y = normalisedAxis.y * std::sin(angle / 2);
	float z = normalisedAxis.z * std::sin(angle / 2);
	float w = std::cos(angle / 2);

	Quaternion quaternion(w, x, y, z);
	quaternion = quaternion.normalised();

	return quaternion;
}

Quaternion Quaternion::fromAxisAngle(const Vec3& axis, float angle) {
	return fromAxisAngle(AxisAngle{ axis, angle });
}

AxisAngle Quaternion::toAxisAngle() const {
	Quaternion q = normalised();

	AxisAngle result;
	result.angle = 2.0f * std::acos(q.w);

	float sinHalfAngle = std::sqrt(1.0f - q.w * q.w);

	if (sinHalfAngle < 0.0001f) {
		result.axis = Vec3(1.0f, 0.0f, 0.0f);
	}
	else {
		result.axis = Vec3(
			q.x / sinHalfAngle,
			q.y / sinHalfAngle,
			q.z / sinHalfAngle
		);
	}

	return result;

}



Vec3 Quaternion::rotateVector(const Vec3& v) const {
	Quaternion unit_quat = normalised();

	Quaternion vectorQ(0, v.x, v.y, v.z);

	// Note: Due to reversed multiplication convention (q1*q2 means apply q2 then q1),
	// we write conjugate * v * quat to get the mathematical result quat * v * conjugate
	Quaternion result = unit_quat.conjugate() * vectorQ * unit_quat;

	return Vec3(result.x, result.y, result.z);
}

Quaternion Quaternion::slerp(const Quaternion& a, Quaternion b, float t) {
	float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
	if (dot < 0.0f) {
		dot = -dot;
		b = -b;
	}
	
	dot = std::clamp(dot, -1.0f, 1.0f);

	// If very close, use lerp
	if (dot > 0.9995f) {
		Quaternion result = a + t * (b - a);
		return result;
	}

	float angle = std::acos(dot);
	float sin_angle = std::sin(angle);

	float w1 = std::sin((1.0f - t) * angle) / sin_angle;
	float w2 = std::sin(t * angle) / sin_angle;

	return (w1 * a) + (w2 * b);
}


