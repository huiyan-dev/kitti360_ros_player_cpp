cmake_minimum_required(VERSION 3.5)
project(kitti360_ros_player_cpp)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g ") # -pthread

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport
  cv_bridge
  rosbag
  nav_msgs
  tf2_ros
)

find_package(Eigen3 3.4.0 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy sensor_msgs
  DEPENDS
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(kitti360_helper src/nodes/kitti360_helper.cc src/kitti360.cc)
target_link_libraries(kitti360_helper
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  glog::glog
  Eigen3::Eigen
)