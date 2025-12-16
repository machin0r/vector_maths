/**
 * @file CollisionTests.cpp
 * @brief Unit tests for collision detection functions
 */

#include <gtest/gtest.h>
#include "Collision.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ========== Ray Tests ==========

TEST(RayTest, DefaultConstructor) {
    Ray r;
    EXPECT_FLOAT_EQ(r.origin.x, 0.0f);
    EXPECT_FLOAT_EQ(r.origin.y, 0.0f);
    EXPECT_FLOAT_EQ(r.origin.z, 0.0f);
    EXPECT_FLOAT_EQ(r.direction.x, 0.0f);
    EXPECT_FLOAT_EQ(r.direction.y, 0.0f);
    EXPECT_FLOAT_EQ(r.direction.z, 1.0f);
}

TEST(RayTest, ParameterizedConstructor) {
    Ray r(Vec3(1.0f, 2.0f, 3.0f), Vec3(1.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(r.origin.x, 1.0f);
    EXPECT_FLOAT_EQ(r.origin.y, 2.0f);
    EXPECT_FLOAT_EQ(r.origin.z, 3.0f);
    // Direction should be normalized
    EXPECT_NEAR(r.direction.length(), 1.0f, 1e-6f);
}

TEST(RayTest, DirectionNormalization) {
    Ray r(Vec3(0.0f, 0.0f, 0.0f), Vec3(3.0f, 4.0f, 0.0f));
    // Vec3(3, 4, 0) has length 5, so normalized is (0.6, 0.8, 0)
    EXPECT_NEAR(r.direction.x, 0.6f, 1e-5f);
    EXPECT_NEAR(r.direction.y, 0.8f, 1e-5f);
    EXPECT_FLOAT_EQ(r.direction.z, 0.0f);
    EXPECT_NEAR(r.direction.length(), 1.0f, 1e-6f);
}

TEST(RayTest, GetPoint) {
    Ray r(Vec3(1.0f, 2.0f, 3.0f), Vec3(1.0f, 0.0f, 0.0f));
    Vec3 p0 = r.getPoint(0.0f);
    Vec3 p5 = r.getPoint(5.0f);

    EXPECT_FLOAT_EQ(p0.x, 1.0f);
    EXPECT_FLOAT_EQ(p0.y, 2.0f);
    EXPECT_FLOAT_EQ(p0.z, 3.0f);

    EXPECT_FLOAT_EQ(p5.x, 6.0f);
    EXPECT_FLOAT_EQ(p5.y, 2.0f);
    EXPECT_FLOAT_EQ(p5.z, 3.0f);
}

// ========== AABB Tests ==========

TEST(AABBTest, DefaultConstructor) {
    AABB box;
    EXPECT_FLOAT_EQ(box.min.x, 0.0f);
    EXPECT_FLOAT_EQ(box.min.y, 0.0f);
    EXPECT_FLOAT_EQ(box.min.z, 0.0f);
    EXPECT_FLOAT_EQ(box.max.x, 0.0f);
    EXPECT_FLOAT_EQ(box.max.y, 0.0f);
    EXPECT_FLOAT_EQ(box.max.z, 0.0f);
}

TEST(AABBTest, ParameterizedConstructor) {
    AABB box(Vec3(-1.0f, -2.0f, -3.0f), Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
    EXPECT_FLOAT_EQ(box.min.y, -2.0f);
    EXPECT_FLOAT_EQ(box.min.z, -3.0f);
    EXPECT_FLOAT_EQ(box.max.x, 1.0f);
    EXPECT_FLOAT_EQ(box.max.y, 2.0f);
    EXPECT_FLOAT_EQ(box.max.z, 3.0f);
}

TEST(AABBTest, FromCenterAndExtents) {
    AABB box = AABB::fromCenterAndExtents(Vec3(5.0f, 10.0f, 15.0f), Vec3(1.0f, 2.0f, 3.0f));
    EXPECT_FLOAT_EQ(box.min.x, 4.0f);
    EXPECT_FLOAT_EQ(box.min.y, 8.0f);
    EXPECT_FLOAT_EQ(box.min.z, 12.0f);
    EXPECT_FLOAT_EQ(box.max.x, 6.0f);
    EXPECT_FLOAT_EQ(box.max.y, 12.0f);
    EXPECT_FLOAT_EQ(box.max.z, 18.0f);
}

TEST(AABBTest, GetCenter) {
    AABB box(Vec3(-2.0f, -4.0f, -6.0f), Vec3(2.0f, 4.0f, 6.0f));
    Vec3 center = box.getCenter();
    EXPECT_FLOAT_EQ(center.x, 0.0f);
    EXPECT_FLOAT_EQ(center.y, 0.0f);
    EXPECT_FLOAT_EQ(center.z, 0.0f);
}

TEST(AABBTest, GetExtents) {
    AABB box(Vec3(-2.0f, -4.0f, -6.0f), Vec3(2.0f, 4.0f, 6.0f));
    Vec3 extents = box.getExtents();
    EXPECT_FLOAT_EQ(extents.x, 2.0f);
    EXPECT_FLOAT_EQ(extents.y, 4.0f);
    EXPECT_FLOAT_EQ(extents.z, 6.0f);
}

TEST(AABBTest, ContainsPoint) {
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));

    // Points inside
    EXPECT_TRUE(box.contains(Vec3(0.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(box.contains(Vec3(0.5f, 0.5f, 0.5f)));

    // Points on boundary
    EXPECT_TRUE(box.contains(Vec3(1.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(box.contains(Vec3(-1.0f, 1.0f, -1.0f)));

    // Points outside
    EXPECT_FALSE(box.contains(Vec3(2.0f, 0.0f, 0.0f)));
    EXPECT_FALSE(box.contains(Vec3(0.0f, -2.0f, 0.0f)));
    EXPECT_FALSE(box.contains(Vec3(0.0f, 0.0f, 2.0f)));
}

TEST(AABBTest, Expand) {
    AABB box(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f));

    box.expand(Vec3(2.0f, 0.5f, 0.5f));
    EXPECT_FLOAT_EQ(box.max.x, 2.0f);

    box.expand(Vec3(-1.0f, 0.5f, 0.5f));
    EXPECT_FLOAT_EQ(box.min.x, -1.0f);
}

TEST(AABBTest, Merge) {
    AABB box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f));
    AABB box2(Vec3(2.0f, 2.0f, 2.0f), Vec3(3.0f, 3.0f, 3.0f));

    AABB merged = box1.merge(box2);
    EXPECT_FLOAT_EQ(merged.min.x, 0.0f);
    EXPECT_FLOAT_EQ(merged.min.y, 0.0f);
    EXPECT_FLOAT_EQ(merged.min.z, 0.0f);
    EXPECT_FLOAT_EQ(merged.max.x, 3.0f);
    EXPECT_FLOAT_EQ(merged.max.y, 3.0f);
    EXPECT_FLOAT_EQ(merged.max.z, 3.0f);
}

// ========== Sphere Tests ==========

TEST(SphereTest, DefaultConstructor) {
    Sphere s;
    EXPECT_FLOAT_EQ(s.center.x, 0.0f);
    EXPECT_FLOAT_EQ(s.center.y, 0.0f);
    EXPECT_FLOAT_EQ(s.center.z, 0.0f);
    EXPECT_FLOAT_EQ(s.radius, 1.0f);
}

TEST(SphereTest, ParameterizedConstructor) {
    Sphere s(Vec3(1.0f, 2.0f, 3.0f), 5.0f);
    EXPECT_FLOAT_EQ(s.center.x, 1.0f);
    EXPECT_FLOAT_EQ(s.center.y, 2.0f);
    EXPECT_FLOAT_EQ(s.center.z, 3.0f);
    EXPECT_FLOAT_EQ(s.radius, 5.0f);
}

TEST(SphereTest, ContainsPoint) {
    Sphere s(Vec3(0.0f, 0.0f, 0.0f), 5.0f);

    // Points inside
    EXPECT_TRUE(s.contains(Vec3(0.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(s.contains(Vec3(3.0f, 0.0f, 0.0f)));

    // Point on boundary
    EXPECT_TRUE(s.contains(Vec3(5.0f, 0.0f, 0.0f)));

    // Point outside
    EXPECT_FALSE(s.contains(Vec3(6.0f, 0.0f, 0.0f)));
    EXPECT_FALSE(s.contains(Vec3(4.0f, 4.0f, 0.0f)));
}

// ========== Intersection Function Tests ==========

TEST(IntersectionTest, RayIntersectsSphere_Hit) {
    Ray ray(Vec3(0.0f, 0.0f, -10.0f), Vec3(0.0f, 0.0f, 1.0f));
    Sphere sphere(Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    float distance;

    EXPECT_TRUE(rayIntersectsSphere(ray, sphere, distance));
    EXPECT_NEAR(distance, 8.0f, 1e-5f); // 10 - 2 = 8
}

TEST(IntersectionTest, RayIntersectsSphere_Miss) {
    Ray ray(Vec3(0.0f, 0.0f, -10.0f), Vec3(1.0f, 0.0f, 0.0f));
    Sphere sphere(Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    float distance;

    EXPECT_FALSE(rayIntersectsSphere(ray, sphere, distance));
}

TEST(IntersectionTest, RayIntersectsSphere_InsideOrigin) {
    Ray ray(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f));
    Sphere sphere(Vec3(0.0f, 0.0f, 0.0f), 5.0f);
    float distance;

    EXPECT_TRUE(rayIntersectsSphere(ray, sphere, distance));
}

TEST(IntersectionTest, RayIntersectsPlane_Hit) {
    Ray ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(0.0f, 0.0f, 1.0f));
    Vec3 planeNormal(0.0f, 0.0f, 1.0f);
    Vec3 planePoint(0.0f, 0.0f, 0.0f);
    float distance;

    EXPECT_TRUE(rayIntersectsPlane(ray, planeNormal, planePoint, distance));
    EXPECT_NEAR(distance, 5.0f, 1e-5f);
}

TEST(IntersectionTest, RayIntersectsPlane_Parallel) {
    Ray ray(Vec3(0.0f, 0.0f, -5.0f), Vec3(1.0f, 0.0f, 0.0f));
    Vec3 planeNormal(0.0f, 0.0f, 1.0f);
    Vec3 planePoint(0.0f, 0.0f, 0.0f);
    float distance;

    EXPECT_FALSE(rayIntersectsPlane(ray, planeNormal, planePoint, distance));
}

TEST(IntersectionTest, RayIntersectsPlane_Behind) {
    Ray ray(Vec3(0.0f, 0.0f, 5.0f), Vec3(0.0f, 0.0f, 1.0f));
    Vec3 planeNormal(0.0f, 0.0f, 1.0f);
    Vec3 planePoint(0.0f, 0.0f, 0.0f);
    float distance;

    EXPECT_FALSE(rayIntersectsPlane(ray, planeNormal, planePoint, distance));
}

TEST(IntersectionTest, RayIntersectsAABB_Hit) {
    Ray ray(Vec3(0.0f, 0.0f, -10.0f), Vec3(0.0f, 0.0f, 1.0f));
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));
    float distance;

    EXPECT_TRUE(rayIntersectsAABB(ray, box, distance));
    EXPECT_NEAR(distance, 9.0f, 1e-5f); // 10 - 1 = 9
}

TEST(IntersectionTest, RayIntersectsAABB_Miss) {
    Ray ray(Vec3(5.0f, 0.0f, -10.0f), Vec3(0.0f, 0.0f, 1.0f));
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));
    float distance;

    EXPECT_FALSE(rayIntersectsAABB(ray, box, distance));
}

TEST(IntersectionTest, AABBIntersectsAABB_Overlapping) {
    AABB box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f));
    AABB box2(Vec3(1.0f, 1.0f, 1.0f), Vec3(3.0f, 3.0f, 3.0f));

    EXPECT_TRUE(aabbIntersectsAABB(box1, box2));
    EXPECT_TRUE(aabbIntersectsAABB(box2, box1)); // Test symmetry
}

TEST(IntersectionTest, AABBIntersectsAABB_Touching) {
    AABB box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f));
    AABB box2(Vec3(1.0f, 0.0f, 0.0f), Vec3(2.0f, 1.0f, 1.0f));

    EXPECT_TRUE(aabbIntersectsAABB(box1, box2));
}

TEST(IntersectionTest, AABBIntersectsAABB_Separate) {
    AABB box1(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f));
    AABB box2(Vec3(2.0f, 0.0f, 0.0f), Vec3(3.0f, 1.0f, 1.0f));

    EXPECT_FALSE(aabbIntersectsAABB(box1, box2));
}

TEST(IntersectionTest, PointInAABB_Inside) {
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));

    EXPECT_TRUE(pointInAABB(Vec3(0.0f, 0.0f, 0.0f), box));
    EXPECT_TRUE(pointInAABB(Vec3(0.5f, 0.5f, 0.5f), box));
}

TEST(IntersectionTest, PointInAABB_OnBoundary) {
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));

    EXPECT_TRUE(pointInAABB(Vec3(1.0f, 0.0f, 0.0f), box));
    EXPECT_TRUE(pointInAABB(Vec3(-1.0f, 1.0f, -1.0f), box));
}

TEST(IntersectionTest, PointInAABB_Outside) {
    AABB box(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f));

    EXPECT_FALSE(pointInAABB(Vec3(2.0f, 0.0f, 0.0f), box));
    EXPECT_FALSE(pointInAABB(Vec3(0.0f, -2.0f, 0.0f), box));
}

TEST(IntersectionTest, SphereIntersectsSphere_Overlapping) {
    Sphere s1(Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    Sphere s2(Vec3(3.0f, 0.0f, 0.0f), 2.0f);

    EXPECT_TRUE(sphereIntersectsSphere(s1, s2));
    EXPECT_TRUE(sphereIntersectsSphere(s2, s1)); // Test symmetry
}

TEST(IntersectionTest, SphereIntersectsSphere_Touching) {
    Sphere s1(Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    Sphere s2(Vec3(4.0f, 0.0f, 0.0f), 2.0f);

    EXPECT_TRUE(sphereIntersectsSphere(s1, s2));
}

TEST(IntersectionTest, SphereIntersectsSphere_Separate) {
    Sphere s1(Vec3(0.0f, 0.0f, 0.0f), 2.0f);
    Sphere s2(Vec3(5.0f, 0.0f, 0.0f), 2.0f);

    EXPECT_FALSE(sphereIntersectsSphere(s1, s2));
}

TEST(IntersectionTest, SphereIntersectsSphere_Contained) {
    Sphere s1(Vec3(0.0f, 0.0f, 0.0f), 5.0f);
    Sphere s2(Vec3(1.0f, 0.0f, 0.0f), 2.0f);

    EXPECT_TRUE(sphereIntersectsSphere(s1, s2));
}
