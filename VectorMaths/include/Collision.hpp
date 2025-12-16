/**
 * @file Collision.hpp
 * @brief Geometric primitives and intersection tests for collision detection
 *
 * Provides ray, AABB, and sphere classes along with intersection
 * test functions for collision detection.
 */

#pragma once
#include "Vector.hpp"

#include <cmath>

/**
 * @brief Represents a ray in 3D space for collision detection and raycasting
 *
 * A ray consists of an origin point and a normalized direction vector,
 * extending infinitely in one direction. Commonly used for mouse picking,
 * line-of-sight tests, and physics queries.
 *
 * @note The direction vector is automatically normalized in the constructor
 */
class Ray {
public:
	Vec3 origin;     ///< Starting point of the ray
	Vec3 direction;  ///< Normalized direction vector

	/// Default constructor - ray at origin pointing along positive Z axis
	Ray();

	/**
	 * @brief Constructs a ray with given origin and direction
	 * @param origin Starting point of the ray
	 * @param direction Direction vector (will be normalized automatically)
	 */
	Ray(const Vec3& origin, const Vec3& direction);

	/**
	 * @brief Returns a point at distance t along the ray
	 * @param t Distance along the ray (t=0 is origin, t=1 is one unit away)
	 * @return Point at position origin + direction * t
	 */
	Vec3 getPoint(float t) const;
};

/**
 * @brief Axis-Aligned Bounding Box for collision detection
 *
 * An AABB is defined by minimum and maximum corners. All edges are aligned
 * with the world axes, making intersection tests very fast.
 */
class AABB {
public:
	Vec3 min;  ///< Minimum corner (most negative X, Y, Z)
	Vec3 max;  ///< Maximum corner (most positive X, Y, Z)

	/// Default constructor - creates a zero-sized box at origin
	AABB();

	/**
	 * @brief Constructs an AABB from min and max corners
	 * @param min Minimum corner
	 * @param max Maximum corner
	 */
	AABB(const Vec3& min, const Vec3& max);

	/**
	 * @brief Creates an AABB from center point and half-extents
	 * @param center Center point of the box
	 * @param halfExtents Half the size in each dimension
	 * @return AABB centered at given point
	 */
	static AABB fromCenterAndExtents(const Vec3& center, const Vec3& halfExtents);

	/// Returns the half-extents (half-size) of the box in each dimension
	Vec3 getExtents() const;

	/// Returns the center point of the box
	Vec3 getCenter() const;

	/// Returns true if the point is inside or on the surface of the box
	bool contains(const Vec3& point) const;

	/**
	 * @brief Expands the AABB to include the given point
	 * @param point Point to include in the bounding box
	 */
	void expand(const Vec3& point);

	/**
	 * @brief Creates a new AABB that encompasses both this box and another
	 * @param other The other bounding box to merge with
	 * @return New AABB containing both boxes
	 */
	AABB merge(const AABB& other);
};

/**
 * @brief Sphere primitive for collision detection
 *
 * A sphere is defined by a center point and radius. Sphere tests are
 * typically faster than AABB tests but less tight-fitting for non-spherical objects.
 */
class Sphere {
public:
	Vec3 center;   ///< Center point of the sphere
	float radius;  ///< Radius of the sphere

	/// Default constructor - unit sphere at origin
	Sphere();

	/**
	 * @brief Constructs a sphere with given center and radius
	 * @param center Center point
	 * @param radius Radius (must be positive)
	 */
	Sphere(const Vec3& center, const float radius);

	/// Returns true if the point is inside or on the surface of the sphere
	bool contains(const Vec3& point) const;
};

// ========== Intersection Functions ==========

/**
 * @brief Tests if a ray intersects a sphere
 * @param ray The ray to test
 * @param sphere The sphere to test against
 * @param[out] distance Set to distance along ray to intersection point if hit
 * @return true if intersection occurs, false otherwise
 */
bool rayIntersectsSphere(const Ray& ray, const Sphere& sphere, float& distance);

/**
 * @brief Tests if a ray intersects a plane
 * @param ray The ray to test
 * @param planeNormal Normal vector of the plane (should be normalized)
 * @param planePoint Any point on the plane
 * @param[out] distance Set to distance along ray to intersection point if hit
 * @return true if intersection occurs, false if ray is parallel or behind
 */
bool rayIntersectsPlane(const Ray& ray, const Vec3& planeNormal, const Vec3& planePoint, float& distance);

/**
 * @brief Tests if a ray intersects an AABB
 * @param ray The ray to test
 * @param box The AABB to test against
 * @param[out] distance Set to distance along ray to intersection point if hit
 * @return true if intersection occurs, false otherwise
 */
bool rayIntersectsAABB(const Ray& ray, const AABB& box, float& distance);

/**
 * @brief Tests if two AABBs overlap
 * @param a First AABB
 * @param b Second AABB
 * @return true if boxes overlap (including touching), false otherwise
 * @note Very fast - only 6 comparisons
 */
bool aabbIntersectsAABB(const AABB& a, const AABB& b);

/**
 * @brief Tests if a point is inside an AABB
 * @param point The point to test
 * @param box The AABB to test against
 * @return true if point is inside or on boundary, false otherwise
 */
bool pointInAABB(const Vec3& point, const AABB& box);

/**
 * @brief Tests if two spheres overlap
 * @param a First sphere
 * @param b Second sphere
 * @return true if spheres overlap (including touching), false otherwise
 * @note Uses squared distance to avoid sqrt
 */
bool sphereIntersectsSphere(const Sphere& a, const Sphere& b);