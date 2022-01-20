cmake_minimum_required(VERSION 2.8)
project(Script)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED
             dynamixel_sdk)

include_directories(EIGEN3_INCLUDE_DIR
                    ~/Drivers/DynamixelSDK/c++/include/dynamixel_sdk)

add_executable(Script main.cpp)

target_link_libraries(Script ~/Drivers/DynamixelSDK/c++/build/linux64/libdxl_x64_cpp.so)



