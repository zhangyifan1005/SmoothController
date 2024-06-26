cmake_minimum_required(VERSION 2.8.4)
project(kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

#include(Eigen.cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



set(SOURCE_FILES source/kalman.cpp source/kalman_controller.cpp source/smoother.cpp source/smooth_controller.cpp test_smooth.cpp)
add_executable(test_smooth ${SOURCE_FILES})
target_include_directories(test_smooth PRIVATE inc)
target_link_libraries(test_smooth PUBLIC Eigen3::Eigen)