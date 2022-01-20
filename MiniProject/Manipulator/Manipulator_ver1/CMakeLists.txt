cmake_minimum_required(VERSION 2.8)
project(Manipulator_ver1)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        include/dynamixel_sdk
        include/manipulator_base
        include/my_manipulator)

add_executable(manipulator
        src/dynamixel_sdk/group_bulk_read.cpp
        src/dynamixel_sdk/group_bulk_write.cpp
        src/dynamixel_sdk/group_sync_write.cpp
        src/dynamixel_sdk/group_bulk_read.cpp
        src/dynamixel_sdk/packet_handler.cpp
        src/dynamixel_sdk/port_handler.cpp
        src/dynamixel_sdk/port_handler_arduino.cpp
        src/dynamixel_sdk/port_handler_linux.cpp
        src/dynamixel_sdk/port_handler_mac.cpp
        src/dynamixel_sdk/port_handler_windows.cpp
        src/dynamixel_sdk/protocol1_packet_handler.cpp
        src/dynamixel_sdk/protocol2_packet_handler.cpp
        src/dynamixel_sdk/dynamixel.h
        src/manipulator_base/manipulator_math.cpp
        src/manipulator_base/manipulator_log.cpp
        src/manipulator_base/manipulator_common.cpp
        src/manipulator_base/manipulator_manager.cpp
        src/manipulator_base/manipulator_trajectory_generator.cpp
        src/manipulator_base/manipulator.cpp
        src/my_manipulator/kinematics.cpp
        src/my_manipulator/custom_trajectory.cpp
        src/my_manipulator/dynamixel.cpp
        src/my_manipulator/my_manipulator.cpp
        src/main.cpp)

target_link_libraries(manipulator
        ${Eigen3_LIBRARIES})
