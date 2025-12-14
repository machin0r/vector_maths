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
