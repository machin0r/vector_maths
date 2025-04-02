#pragma once

class Vec2 {
public:
	float x, y;

	// Constructors
	Vec2();
	Vec2(float x, float y);

	// Basic operations
	inline Vec2 operator+(const Vec2& other) const {
		return Vec2(x + other.x, y + other.y);
	}

	inline Vec2 operator-(const Vec2& other) const {
		return Vec2(x - other.x, y - other.y);
	}

	template<typename T>
	inline Vec2 operator*(const T scalar) const {
		return Vec2(x * scalar, y * scalar);
	}

	template<typename T>
	inline Vec2 operator/(const T scalar) const {
		return Vec2(x / scalar, y / scalar);
	}

	template<typename T>
	friend inline Vec2 operator*(const T scalar, const Vec2& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec2 operator/(const T scalar, const Vec2& vec) {
		return vec / scalar;
	}

	// Utility functions
	float length() const;
	Vec2 normalised() const;
	float dot(const Vec2& other) const;
	float cross(const Vec2& other) const;

	static Vec2 lerp(const Vec2& a, const Vec2& b, float t);
	static float distance(const Vec2& a, const Vec2& b);

};

class Vec3 {
public:
	float x, y, z;

	// Constructors
	Vec3();
	Vec3(float x, float y, float z);

	// Basic operations
	inline Vec3 operator+(const Vec3& other) const {
		return Vec3(x + other.x, y + other.y, z + other.z);
	}

	inline Vec3 operator-(const Vec3& other) const {
		return Vec3(x -other.x, y - other.y, z - other.z);
	}

	template<typename T>
	inline Vec3 operator*(const T scalar) const {
		return Vec3(x * scalar, y * scalar, z * scalar);
	}

	template<typename T>
	inline Vec3 operator/(const T scalar) const {
		return Vec3(x / scalar, y / scalar, z / scalar);
	}

	template<typename T>
	friend inline Vec3 operator*(const T scalar, const Vec3& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec3 operator/(const T scalar, const Vec3& vec) {
		return vec / scalar;
	}

	// Utility functions
	float length() const;
	Vec3 normalised() const;
	float dot(const Vec3& other) const;
	Vec3 cross(const Vec3& other) const;

	static Vec3 lerp(const Vec3& a, const Vec3& b, float t);
	static float distance(const Vec3& a, const Vec3& b);

};

class Vec4 {
public:
	float x, y, z, w;

	// Constructors
	Vec4();
	Vec4(float x, float y, float z, float w);

	// Basic operations
	inline Vec4 operator+(const Vec4& other) const {
		return Vec4(x + other.x, y + other.y, z + other.z, w + other.w);
	}

	inline Vec4 operator-(const Vec4& other) const {
		return Vec4(x - other.x, y - other.y, z - other.z, w - other.w);
	}

	template<typename T>
	inline Vec4 operator*(const T scalar) const {
		return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
	}

	template<typename T>
	inline Vec4 operator/(const T scalar) const {
		return Vec4(x / scalar, y / scalar, z / scalar, w / scalar);
	}

	template<typename T>
	friend inline Vec4 operator*(const T scalar, const Vec4& vec) {
		return vec * scalar;
	}

	template<typename T>
	friend inline Vec4 operator/(const T scalar, const Vec4& vec) {
		return vec / scalar;
	}

	// Utility functions
	float length() const;
	Vec4 normalised() const;
	float dot(const Vec4& other) const;

	static Vec4 lerp(const Vec4& a, const Vec4& b, float t);
	static float distance(const Vec4& a, const Vec4& b);

};