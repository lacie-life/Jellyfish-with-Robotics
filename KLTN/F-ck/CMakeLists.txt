cmake_minimum_required (VERSION 3.0)

project (ArUcoDetection)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package (Eigen3 3.3 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(realsense2 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (ArUcoDetection arucoDetection.cpp)
target_link_libraries (ArUcoDetection ${EIGEN3_LIBRARIES})
target_link_libraries(ArUcoDetection ${realsense2_LIBRARY} ${OpenCV_LIBS})

add_executable (MarkerDetection fakeDetection.cpp)
target_link_libraries (MarkerDetection ${EIGEN3_LIBRARIES})
target_link_libraries(MarkerDetection ${realsense2_LIBRARY} ${OpenCV_LIBS})