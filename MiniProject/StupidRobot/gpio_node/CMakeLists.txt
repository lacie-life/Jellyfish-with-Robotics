cmake_minimum_required(VERSION 2.8.3)
project(gpio_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpio_node src/main.cpp)

target_link_libraries(gpio_node
  ${catkin_LIBRARIES}
  JetsonGPIO
)
