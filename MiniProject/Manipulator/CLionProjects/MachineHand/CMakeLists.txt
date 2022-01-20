cmake_minimum_required(VERSION 2.6)
project(MachineHand)

set(CMAKE_CXX_STANDARD 14)

find_package( Eigen3 REQUIRED
        DynamixelSDK)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ~/Project/Lib/Driver/DynamixelSDK/c++/include/dyanmixelSDK)

add_executable(MachineHand
        src/main.cpp
        include/kinematics.h
        src/kinematics.cpp
        include/dynamixel.h
        src/dynamixel.cpp include/machine_math.h src/machine_math.cpp include/common.h src/common.cpp)

target_link_libraries(MachineHand
        ${Eigen3_LIBRARIES}
        ~/Project/Lib/Driver/DynamixelSDK/c++/build/linux64)
