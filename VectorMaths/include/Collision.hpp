#pragma once
#include "Vector.hpp"

#include <cmath>

class Ray {
public:
	Vec3 origin, direction;

	// Constructors
	Ray();
	Ray(const Vec3& origin, const Vec3& direction);

	// Operations
	Vec3 getPoint(float t) const;
};

class AABB {
public:
	Vec3 min, max;

	// Constructors
	AABB();
	AABB(const Vec3& min, const Vec3& max);
	static AABB fromCenterAndExtents(const Vec3& center, const Vec3& halfExtents);


	// Operations
	Vec3 getExtents() const;
	Vec3 getCenter() const;
	bool contains(const Vec3& point) const;
	void expand(const Vec3& point);
	AABB merge(const AABB& other);

};

class Sphere {
public:
	Vec3 center;
	float radius;

	// Constructors
	Sphere();
	Sphere(const Vec3& center, const float radius);

	// Operations
	bool contains(const Vec3& point) const;
};