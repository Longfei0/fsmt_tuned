cmake_minimum_required(VERSION 3.14.0)
project(free-space VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/basic.c
    src/motion_primitive.c
    src/free_space.c

)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    m
)

# UNIT TESTS
enable_testing()
add_executable(sample_line_segment_test 
    test/sample_line_segment_test.cpp)
target_link_libraries(sample_line_segment_test
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME})

add_executable(sample_motion_primitive_test
    test/sample_motion_primitive_test.cpp)
target_link_libraries(sample_motion_primitive_test
    ${GTEST_LIBRARIES}
    ${PROJECT_NAME})

include(GoogleTest)
gtest_discover_tests(sample_line_segment_test)
gtest_discover_tests(sample_motion_primitive_test)

#add_executable(main src/main.c)
#target_link_libraries(main PRIVATE ${PROJECT_NAME})