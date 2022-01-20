cmake_minimum_required(VERSION 2.8.3)
project(simple_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  tf
  sensor_msgs
  image_transport
  nav_msgs
)

find_package(realsense2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_control src/main.cpp)

target_link_libraries(simple_control
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARY}
)
