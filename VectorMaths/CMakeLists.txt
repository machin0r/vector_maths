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

# ========== Testing ==========

# Option to enable/disable tests
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    # Enable testing
    enable_testing()

    # Fetch Google Test
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )

    # For Windows: Prevent overriding parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Make Google Test available
    FetchContent_MakeAvailable(googletest)

    # Add test subdirectory
    add_subdirectory(tests)
endif()