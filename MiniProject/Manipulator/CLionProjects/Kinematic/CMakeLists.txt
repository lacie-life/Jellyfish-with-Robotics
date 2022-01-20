cmake_minimum_required(VERSION 3.14)
project(Kinematic)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_executable(Kinematic main.cpp)

target_link_libraries(Kinematic
        ${Eigen3_LIBRARIES})