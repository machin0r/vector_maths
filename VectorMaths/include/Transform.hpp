#pragma once

#include "Vector.hpp"
#include "Quaternion.hpp"
#include "Matrix.hpp"
#include <vector>


class Transform {
private:
	Vec3 position;
	Quaternion rotation;
	Vec3 scale;

	Transform* parent;
	std::vector<Transform*> children;

	mutable Mat4 localMatrix;
	mutable Mat4 worldMatrix;
	mutable bool dirty; // Flag indicating matrices need recalculation

public:
	//Constructors
	Transform();
	Transform(const Vec3& position, const Quaternion& rotation, const Vec3& scale);


	// Accessors
	Mat4 GetLocalMatrix() const;
	Mat4 GetWorldMatrix() const;
	Vec3 GetPosition() const;
	Quaternion GetRotation() const;
	Vec3 GetScale() const;
	Transform* GetParent() const;
	const std::vector<Transform*>& GetChildren() const;

	// Mutators
	void SetPosition(const Vec3& newPosition);
	void SetRotation(const Quaternion& newRotation);
	void SetScale(const Vec3& newScale);
	void SetParent(Transform* newParent);
	void AddChild(Transform* child);
	void RemoveChild(Transform* child);

	// Utililty Methods
	void MarkDirty();
	void Translate(const Vec3& translation);
	void Rotate(const Quaternion& extraRotation);
	void LookAt(const Vec3& target, const Vec3& up);

	// Transform directions
	Vec3 Forward() const;
	Vec3 Right() const;
	Vec3 Up() const;
};