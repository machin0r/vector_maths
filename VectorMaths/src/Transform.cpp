#include "../include/Vector.hpp"
#include "../include/Matrix.hpp"
#include "../include/Quaternion.hpp"
#include "../include/Transform.hpp"
#include <algorithm>
#include <cmath>

// Constructors
Transform::Transform() 
	: position(Vec3()),
		rotation(Quaternion()),
		scale(Vec3(1.0f, 1.0f, 1.0f)),
		parent(nullptr),
		dirty(true)
	{}

Transform::Transform(const Vec3& position, const Quaternion& rotation, const Vec3& scale)
	: position(position),
		rotation(rotation),
		scale(scale),
		parent(nullptr),
		dirty(true)
{}

// Accessors
Mat4 Transform::GetLocalMatrix() const {
	if (dirty) {
		// Scale - Rotate - Translate

		Mat4 scaleMat;  // Identitiy Matrix
		scaleMat = scaleMat.scale(scale);

		Mat4 rotateMat = rotation.toRotationMatrix();

		localMatrix = rotateMat * scaleMat;
		localMatrix = localMatrix.translation(position);

		dirty = false;
	}
	return localMatrix;
}

Mat4 Transform::GetWorldMatrix() const
{
	if (parent) {
		worldMatrix = parent->GetWorldMatrix() * GetLocalMatrix();
	}
	else {
		worldMatrix = GetLocalMatrix();
	}
	return worldMatrix;
}

Vec3 Transform::GetPosition() const {
	return position;
}

Quaternion Transform::GetRotation() const {
	return rotation;
}

Vec3 Transform::GetScale()  const{
	return scale;
}

Transform* Transform::GetParent() const {
	return parent;
}

const std::vector<Transform*>& Transform::GetChildren() const {
	return children;
}

// Mutators
void Transform::SetPosition(const Vec3& newPosition) {
	position = newPosition;
	MarkDirty();
}

void Transform::SetRotation(const Quaternion& newRotation) {
	rotation = newRotation;
	MarkDirty();
}

void Transform::SetScale(const Vec3& newScale) {
	scale = newScale;
	MarkDirty();
}

void Transform::SetParent(Transform* newParent) {
	parent = newParent;
	MarkDirty();
}

void Transform::AddChild(Transform* child) {
	children.emplace_back(child);
	child->SetParent(this);
}

void Transform::RemoveChild(Transform* child) {
	auto it = std::remove(children.begin(), children.end(), child);

	if (it != children.end()) {
		children.erase(it, children.end());

		if (child && child->GetParent() == this) {
			child->SetParent(nullptr);
		}
	}
}

// Utililty Methods
void Transform::MarkDirty() {
	dirty = true;

	for (Transform* child : children) {
		child->MarkDirty();
	}
}

void Transform::Translate(const Vec3& translation) {
	position = position + translation;
	MarkDirty();
}

void Transform::Rotate(const Quaternion& extraRotation) {
	rotation = rotation * extraRotation;
	MarkDirty();
}

void Transform::LookAt(const Vec3& target, const Vec3& up) {
	Vec3 direction = (target - position).normalised();
	Vec3 right = up.cross(direction).normalised();
	Vec3 newUp = direction.cross(right);

	float rotMatVals[9] = {
		right.x, newUp.x, direction.x,
		right.y, newUp.y, direction.y,
		right.z, newUp.z, direction.z
	};

	Mat3 rotationMatrix(rotMatVals);

	rotation = Quaternion::fromRotationMatrix(rotationMatrix);

	MarkDirty();
}

// Transform directions
Vec3 Transform::Forward() const {
	Vec3 forward(0, 0, -1);  // OpenGL convention
	return rotation.rotateVector(forward);
}

Vec3 Transform::Right() const {
	Vec3 right(1, 0, 0);
	return rotation.rotateVector(right);
}

Vec3 Transform::Up() const {
	Vec3 up(0, 1, 0);
	return rotation.rotateVector(up);
}