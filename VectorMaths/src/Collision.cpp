#include "../include/Collision.hpp"
#include <cmath>


Ray::Ray() : origin(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f) {}

Ray::Ray(const Vec3& origin, const Vec3& direction)
	: origin(origin),
	direction(direction.normalised()) {}

Vec3 Ray::getPoint(float t) const {
	return origin + (direction * t);
}