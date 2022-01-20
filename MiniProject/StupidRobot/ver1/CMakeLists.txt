cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
  tf
  sensor_msgs
  cv_bridge
  image_transport
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

add_executable(serial_example_node src/serial_example_node.cpp)

target_link_libraries(serial_example_node
  ${catkin_LIBRARIES}
  ${realsense2_LIBRARY}
)

