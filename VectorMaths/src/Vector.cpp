#include "../include/Vector.hpp"
#include <cmath>

// Vec2
Vec2::Vec2() : x(0.0f), y(0.0f) {}
Vec2::Vec2(float x, float y) : x(x), y(y) {}

float Vec2::length() const {
	return std::sqrt((x * x) + (y * y));
}

Vec2 Vec2::normalised() const {
	float magnitude = length();

	if (magnitude < 1e-6f) {
		return Vec2(0.0f, 0.0f);
	}

	return Vec2((x / magnitude), (y / magnitude));
}

float Vec2::dot(const Vec2& other) const {
	return ((x * other.x) + (y * other.y));
}

float Vec2::cross(const Vec2& other) const {
	return (x * other.y) - (y * other.x);
}

Vec2 Vec2::lerp(const Vec2& a, const Vec2& b, float t)
{
	// Clamp t between 0-1
	t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

	return Vec2((a.x + (b.x - a.x) * t),
				(a.y + (b.y - a.y) * t));
}
float Vec2::distance(const Vec2& a, const Vec2& b) {
	return (b - a).length();
}

// Vec3
Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
Vec3::Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

float Vec3::length() const {
	return std::sqrt((x * x) + (y * y) + (z * z));
}

Vec3 Vec3::normalised() const {
	float magnitude = length();

	if (magnitude < 1e-6f) {
		return Vec3(0.0f, 0.0f, 0.0f);
	}

	return Vec3((x / magnitude), (y / magnitude), (z/magnitude));
}

float Vec3::dot(const Vec3& other) const {
	return ((x * other.x) + (y * other.y) + (z * other.z));
}

Vec3 Vec3::cross(const Vec3& other) const {
	return Vec3(((y * other.z) - (z * other.y)),
				((z * other.x) - (x * other.z)),
				((x * other.y) - (y * other.x)));
}

Vec3 Vec3::lerp(const Vec3& a, const Vec3& b, float t)
{
	// Clamp t between 0-1
	t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

	return Vec3((a.x + (b.x - a.x) * t),
				(a.y + (b.y - a.y) * t),
				(a.z + (b.z - a.z) * t));
}
float Vec3::distance(const Vec3& a, const Vec3& b) {
	return (b - a).length();
}

// Vec4
Vec4::Vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
Vec4::Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

float Vec4::length() const {
	return std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
}

Vec4 Vec4::normalised() const {
	float magnitude = length();

	if (magnitude < 1e-6f) {
		return Vec4(0.0f, 0.0f, 0.0f, 0.0f);
	}

	return Vec4((x / magnitude), (y / magnitude), (z / magnitude), (w / magnitude));
}

float Vec4::dot(const Vec4& other) const {
	return ((x * other.x) + (y * other.y) + (z * other.z) + (w * other.w));
}

Vec4 Vec4::lerp(const Vec4& a, const Vec4& b, float t)
{
	// Clamp t between 0-1
	t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

	return Vec4((a.x + (b.x - a.x) * t),
		(a.y + (b.y - a.y) * t),
		(a.z + (b.z - a.z) * t),
		(a.w + (b.w - a.w) * t));
}

float Vec4::distance(const Vec4& a, const Vec4& b) {
	return (b - a).length();
}