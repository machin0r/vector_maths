# VectorMaths

A C++ 3D maths library.

## Features

- **Vector Maths**: 2D, 3D, and 4D vectors with common operations (dot, cross, normalization, lerp)
- **Matrix Operations**: 3x3 and 4x4 matrices with multiplication, transformation utilities
- **Quaternions**: Rotation representation with slerp interpolation and euler/axis-angle conversions
- **Transforms**: Scene graph hierarchy with local/world space conversions
- **Collision Detection**: Ray, AABB, and sphere primitives with intersection tests

**Design Choices:**
- Column-major matrix storage
- Quaternion multiplication (q1 * q2 applies q2, then q1)

## Build Instructions

**Prerequisites:**
- CMake 3.14+
- C++17 compatible compiler (MSVC, GCC, Clang)

**Build Steps:**
```bash
git clone https://github.com/yourusername/VectorMaths.git
cd VectorMaths
cmake -B build -DBUILD_TESTS=ON
cmake --build build
```

**Run Tests:**
```bash
cd build
ctest --output-on-failure
```

## Quick Start

### Basic Vector Operations
```cpp
#include "Vector.hpp"

Vec3 a(1.0f, 2.0f, 3.0f);
Vec3 b(4.0f, 5.0f, 6.0f);

Vec3 sum = a + b;
float dot = a.dot(b);
Vec3 cross = a.cross(b);
Vec3 normalized = a.normalised();
```

### Matrix Transformations
```cpp
#include "Matrix.hpp"

Mat4 projection = Mat4::perspective(45.0f, 16.0f/9.0f, 0.1f, 100.0f);
Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
Mat4 model = Mat4::translation(1.0f, 0.0f, 0.0f);

Mat4 mvp = projection * view * model;
```

### Quaternion Rotations
```cpp
#include "Quaternion.hpp"

// 90 degree rotation around Y axis
Quaternion q = Quaternion::fromAxisAngle(Vec3(0, 1, 0), M_PI / 2);

Vec3 point(1, 0, 0);
Vec3 rotated = q.rotateVector(point);  // (0, 0, -1)

// Smooth interpolation between rotations
Quaternion q2 = Quaternion::fromEulerAngles(0, M_PI, 0);
Quaternion interpolated = Quaternion::slerp(q, q2, 0.5f);
```

### Transform Hierarchy
```cpp
#include "Transform.hpp"

Transform root;
Transform child;
child.setParent(&root);

root.setPosition(Vec3(10, 0, 0));
child.setLocalPosition(Vec3(5, 0, 0));

Vec3 worldPos = child.getWorldPosition();  // (15, 0, 0)
```

### Collision Detection
```cpp
#include "Collision.hpp"

Ray ray(Vec3(0, 0, 0), Vec3(1, 0, 0));
Sphere sphere(Vec3(5, 0, 0), 1.0f);

float distance;
if (rayIntersectsSphere(ray, sphere, distance)) {
    Vec3 hitPoint = ray.getPoint(distance);
}
```

## API Overview

| Class | Description |
|-------|-------------|
| `Vec2`, `Vec3`, `Vec4` | Vector types with arithmetic and geometric operations |
| `Mat3`, `Mat4` | Matrix types with multiplication and transformation builders |
| `Quaternion` | Rotation representation with interpolation and conversions |
| `Transform` | Scene graph node with parent-child relationships |
| `Ray`, `AABB`, `Sphere` | Collision primitives with intersection functions |

Full API documentation is available in the header files (Doxygen-style comments).

## Testing

The library uses Google Test for unit testing. Current test coverage includes:

- **Vector Tests**: 23 tests covering all operations for Vec2, Vec3, Vec4
- **Matrix Tests**: Identity, multiplication, transformations
- **Quaternion Tests**: Multiplication, conversions, interpolation
- **Collision Tests**: Ray-sphere, ray-AABB, AABB-AABB intersections

Run tests with:
```bash
cmake --build build --target VectorMathsTests
cd build/tests
./VectorMathsTests  # or VectorMathsTests.exe on Windows
```

Example output:
```
[==========] Running 50 tests from 5 test suites.
[----------] 23 tests from Vec2Test, Vec3Test, Vec4Test
[ RUN      ] Vec2Test.Addition
[       OK ] Vec2Test.Addition (0 ms)
...
[==========] 50 tests from 5 test suites ran.
[  PASSED  ] 50 tests.
```
