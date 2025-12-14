#include "../include/Collision.hpp"
#include <cmath>


Ray::Ray() : origin(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f) {}

Ray::Ray(const Vec3& origin, const Vec3& direction)
	: origin(origin),
	direction(direction.normalised()) {}

Vec3 Ray::getPoint(float t) const {
	return origin + (direction * t);
}


AABB::AABB() : min(0.0f, 0.0f, 0.0f), max(0.0f, 0.0f, 0.0f) {}
AABB::AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}
AABB AABB::fromCenterAndExtents(const Vec3& center, const Vec3& halfExtents) {
	return AABB(center - halfExtents, center + halfExtents);
}

Vec3 AABB::getExtents() const {
	return (max - min) / 2;
}

Vec3 AABB::getCenter() const {
	return(min + max) / 2;
}


bool AABB::contains(const Vec3& point) const {
	bool inX = (max.x >= point.x) && (point.x >= min.x);
	bool inY = (max.y >= point.y) && (point.y >= min.y);
	bool inZ = (max.z >= point.z) && (point.z >= min.z);

	return inX && inY && inZ;
}

void AABB::expand(const Vec3& point) {
	max.x = std::fmax(max.x, point.x);
	max.y = std::fmax(max.y, point.y);
	max.z = std::fmax(max.z, point.z);
	min.x = std::fmin(min.x, point.x);
	min.y = std::fmin(min.y, point.y);
	min.z = std::fmin(min.z, point.z);
}

AABB AABB::merge(const AABB& other) {
	float maxX = std::fmax(max.x, other.max.x);
	float maxY = std::fmax(max.y, other.max.y);
	float maxZ = std::fmax(max.z, other.max.z);
	float minX = std::fmin(min.x, other.min.x);
	float minY = std::fmin(min.y, other.min.y);
	float minZ = std::fmin(min.z, other.min.z);

	Vec3 maxVec(maxX, maxY, maxZ);
	Vec3 minVec(minX, minY, minZ);

	return AABB(minVec, maxVec);
}


Sphere::Sphere() : center(0.0f, 0.0f, 0.0f), radius(1.0f) {}

Sphere::Sphere(const Vec3& center, const float radius)
	: center(center),
	radius(radius) {
}

bool Sphere::contains(const Vec3& point) const {
	return (point - center).length() <= radius;
}