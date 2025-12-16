/**
 * @file Transform.hpp
 * @brief Transform class for scene graph hierarchies
 *
 * Provides a transform component with position, rotation, and scale,
 * supporting parent-child hierarchies for scene graphs. Matrices are
 * cached and only recalculated when marked dirty.
 */

#pragma once

#include "Vector.hpp"
#include "Quaternion.hpp"
#include "Matrix.hpp"
#include <vector>

/**
 * @brief Transform component for 3D objects in a scene hierarchy
 *
 * Stores position, rotation (quaternion), and scale, with support for
 * parent-child relationships. Local and world matrices are cached and
 * only recalculated when the transform changes (lazy evaluation).
 *
 * @note Matrices are marked dirty when transform changes or parent hierarchy updates
 */
class Transform {
private:
	Vec3 position;              ///< Local position relative to parent
	Quaternion rotation;        ///< Local rotation relative to parent
	Vec3 scale;                 ///< Local scale relative to parent

	Transform* parent;                   ///< Parent transform (nullptr if root)
	std::vector<Transform*> children;    ///< Child transforms

	mutable Mat4 localMatrix;   ///< Cached local transformation matrix
	mutable Mat4 worldMatrix;   ///< Cached world transformation matrix
	mutable bool dirty;         ///< True if matrices need recalculation

public:
	/// Default constructor - creates identity transform at origin
	Transform();

	/**
	 * @brief Constructs a transform with given position, rotation, and scale
	 * @param position Local position
	 * @param rotation Local rotation
	 * @param scale Local scale
	 */
	Transform(const Vec3& position, const Quaternion& rotation, const Vec3& scale);

	// Accessors
	/// Returns the local transformation matrix (relative to parent)
	Mat4 GetLocalMatrix() const;

	/// Returns the world transformation matrix (in world space)
	Mat4 GetWorldMatrix() const;

	/// Returns the local position
	Vec3 GetPosition() const;

	/// Returns the local rotation
	Quaternion GetRotation() const;

	/// Returns the local scale
	Vec3 GetScale() const;

	/// Returns pointer to parent transform (nullptr if root)
	Transform* GetParent() const;

	/// Returns const reference to vector of child transforms
	const std::vector<Transform*>& GetChildren() const;

	// Mutators
	/// Sets the local position (marks dirty)
	void SetPosition(const Vec3& newPosition);

	/// Sets the local rotation (marks dirty)
	void SetRotation(const Quaternion& newRotation);

	/// Sets the local scale (marks dirty)
	void SetScale(const Vec3& newScale);

	/**
	 * @brief Sets the parent transform
	 * @param newParent New parent (nullptr to detach)
	 */
	void SetParent(Transform* newParent);

	/**
	 * @brief Adds a child transform
	 * @param child Child to add (automatically sets this as parent)
	 */
	void AddChild(Transform* child);

	/**
	 * @brief Removes a child transform
	 * @param child Child to remove
	 */
	void RemoveChild(Transform* child);

	// Utility Methods
	/// Marks matrices as needing recalculation (propagates to children)
	void MarkDirty();

	/// Adds to current position (marks dirty)
	void Translate(const Vec3& translation);

	/// Applies additional rotation (marks dirty)
	void Rotate(const Quaternion& extraRotation);

	/**
	 * @brief Rotates transform to look at target point
	 * @param target Point to look at in world space
	 * @param up Up direction vector
	 */
	void LookAt(const Vec3& target, const Vec3& up);

	// Transform directions
	/// Returns the forward direction vector in world space
	Vec3 Forward() const;

	/// Returns the right direction vector in world space
	Vec3 Right() const;

	/// Returns the up direction vector in world space
	Vec3 Up() const;
};