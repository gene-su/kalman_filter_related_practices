cmake_minimum_required(VERSION 3.2)

project(kalman_filter)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-Wall")

add_executable (main src/main.cpp src/kalman_filter.cpp)