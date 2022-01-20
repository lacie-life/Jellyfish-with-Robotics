cmake_minimum_required(VERSION 3.14)
project(Manipulator)

add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED
             dynamixel_sdk
        )

include_directories(EIGEN3_INCLUDE_DIR
                    ~/Drivers/DynamixelSDK/c++/include/dynamixel_sdk
                    Manipulator/include)

add_executable(Manipulator src/main.cpp
        include/robotis_manipulator_math.h
                           src/robotis_manipulator_math.cpp
        include/robotis_manipulator_log.h
                           src/robotis_manipulator_log.cpp
        include/robotis_manipulator_common.h
                           src/robotis_manipulator_common.cpp
        include/robotis_manipulator_manager.h
                           src/robotis_manipulator_manager.cpp
        include/robotis_manipulator_trajectory_generator.h
                           src/robotis_manipulator_trajectory_generator.cpp
        include/robotis_manipulator.h
                           src/robotis_manipulator.cpp include/manipulator.h src/manipulator.cpp include/kinematic.h include/custom_trajectory.h src/kinematic.cpp src/custom_trajectory.cpp)

target_link_libraries(Manipulator ~/Drivers/DynamixelSDK/c++/build/linux64/libdxl_x64_cpp.so)
