cmake_minimum_required (VERSION 3.0)

project (transformcoordinate)

find_package (Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (transform main.cpp)
target_link_libraries (transform ${EIGEN3_LIBRARIES})



