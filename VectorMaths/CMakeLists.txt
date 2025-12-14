cmake_minimum_required(VERSION 3.14)
project(VectorMaths VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

# Add source files
set(SOURCES
    src/Vector.cpp
    src/Matrix.cpp
    src/Quaternion.cpp
    src/Transform.cpp
    src/Collision.cpp
)

# Add header files
set(HEADERS
    include/Vector.hpp
    include/Matrix.hpp
    include/Quaternion.hpp
    include/Transform.hpp
    include/Collision.hpp
)

# Create library
add_library(VectorMaths ${SOURCES} ${HEADERS})

# Set include directories
target_include_directories(VectorMaths PUBLIC include)