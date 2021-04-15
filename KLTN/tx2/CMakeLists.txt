cmake_minimum_required(VERSION 2.8.3)
project(vision_to_mavros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  mavros_msgs
)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/vision_to_mavros.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(t265_fisheye_undistort_node src/t265_fisheye_undistort.cpp)
target_link_libraries(t265_fisheye_undistort_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${realsense2_LIBRARY}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
