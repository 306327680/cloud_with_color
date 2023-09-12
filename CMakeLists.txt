cmake_minimum_required(VERSION 3.25)
project(cloudWithcolor)

set(CMAKE_CXX_STANDARD 17)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package( Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(rosbag REQUIRED)
include_directories(${rosbag_INCLUDE_DIRS})
find_package(sensor_msgs REQUIRED)
include_directories(${sensor_msgs_INCLUDE_DIRS})
find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

add_executable(cloudWithcolor main.cpp)
target_link_libraries(cloudWithcolor    ${cv_bridge_LIBRARIES}  ${sensor_msgs_LIBRARIES} ${rosbag_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})