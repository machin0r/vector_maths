# Test executable
add_executable(VectorMathsTests
    "${CMAKE_CURRENT_SOURCE_DIR}/VectorTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MatrixTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/QuaternionTests.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/CollisionTests.cpp"
)

# Link against Google Test and our library
target_link_libraries(VectorMathsTests
    PRIVATE
    GTest::gtest_main
    VectorMaths
)

# Discover tests for CTest
include(GoogleTest)
gtest_discover_tests(VectorMathsTests)
