cmake_minimum_required(VERSION 3.14)
project(MecanumControl)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS})

add_executable(MecanumControl main.cpp)

target_link_libraries(MecanumControl
        ${Eigen3_LIBRARIES})