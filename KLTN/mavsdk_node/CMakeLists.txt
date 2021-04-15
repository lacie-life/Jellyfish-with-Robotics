cmake_minimum_required(VERSION 2.8.3)
project(mavsdk_node)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(MAVSDK REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mavsdk_node
    src/main.cpp
)

target_link_libraries(mavsdk_node
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk
)