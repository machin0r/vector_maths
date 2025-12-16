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

bool rayIntersectsSphere(const Ray& ray, const Sphere& sphere, float& distance) {
	float raySphereDot = (sphere.center - ray.origin).dot(ray.direction);
	if (raySphereDot < 0) {
		return false;
	}
	Vec3 p = ray.origin + (ray.direction * raySphereDot);
	float d = (p - sphere.center).length();
	if (d > sphere.radius) {
		return false;
	}
	float offset = std::sqrt(std::pow(sphere.radius, 2) - std::pow(d, 2));
	float t1 = raySphereDot - offset;
	
	if (t1 < 0) {
		float t2 = raySphereDot + offset;
		if (t2 < 0) {
			return false;
		}
		distance = t2;
	}
	else {
		distance = t1; 
	}
	return true;
}

bool rayIntersectsPlane(const Ray& ray, const Vec3& planeNormal, const Vec3& planePoint, float& distance) {
	float dotProduct = planeNormal.dot(ray.direction);
	if (std::abs(dotProduct) < 1e-6f) {
		return false;
	}

	Vec3 diff = planePoint - ray.origin;
	float t = diff.dot(planeNormal) / dotProduct;
	if (t < 0) {
		return false;
	}
	distance = t;
	return true;
}

bool rayIntersectsAABB(const Ray& ray, const AABB& box, float& distance) {
	float tMinX = (box.min.x - ray.origin.x) / ray.direction.x;
	float tMaxX = (box.max.x - ray.origin.x) / ray.direction.x;

	float tMinY = (box.min.y - ray.origin.x) / ray.direction.y;
	float tMaxY = (box.max.y - ray.origin.x) / ray.direction.y;

	float tMinZ = (box.min.z - ray.origin.x) / ray.direction.z;
	float tMaxZ = (box.max.z - ray.origin.x) / ray.direction.z;

	if (ray.direction.x < 0) {
		std::swap(tMinX, tMaxX);
	}
	if (ray.direction.y < 0) {
		std::swap(tMinY, tMaxY);
	}
	if (ray.direction.z < 0) {
		std::swap(tMinZ, tMaxZ);
	}

	float tMin = std::max(std::max(tMinX, tMinY), tMinZ);
	float tMax = std::min(std::min(tMaxX, tMaxY), tMaxZ);

	// Check for intersection
	if (tMin > tMax) return false;
	if (tMax < 0) return false;

	distance = (tMin >= 0) ? tMin : tMax;

	return true;
}

bool aabbIntersectsAABB(const AABB& a, const AABB& b) {
	return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
		(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
		(a.min.z <= b.max.z && a.max.z >= b.min.z);
}

bool pointInAABB(const Vec3& point, const AABB& box) {
	return (point.x <= box.max.x && point.x >= box.min.x) &&
		(point.y <= box.max.y && point.y >= box.min.y) &&
		(point.z <= box.max.z && point.z >= box.min.z);
}

bool sphereIntersectsSphere(const Sphere& a, const Sphere& b) {
	Vec3 diff = (a.center - b.center);
	float radiusSum = a.radius + b.radius;
	return diff.lengthSquared() <= (radiusSum * radiusSum);
}