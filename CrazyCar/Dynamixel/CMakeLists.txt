################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel)

add_compile_options(-std=c++11)

################################################################################
# Build
################################################################################
include_directories(
  ~/Desktop/Dynamixel/include/dynamixel_sdk
  ~/Desktop/Dynamixel/include
)

add_library(dynamixel_lib
  include/dynamixel.h
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
  src/dynamixel.cpp
)

add_executable(dynamixel 
				src/main.cpp
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
  src/dynamixel.cpp
				)

#target_link_libraries(dynamixel dynamixel_lib)





