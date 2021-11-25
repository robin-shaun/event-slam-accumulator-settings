cmake_minimum_required(VERSION 3.0.2)
project(dv_ros)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  celex5_msgs
  cv_bridge
  ddynamic_reconfigure
  dvs_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(OpenCV 3.3.1 REQUIRED)
find_package(dv REQUIRED)
find_package(fmt 7.0.3 REQUIRED)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES see_slam
  CATKIN_DEPENDS cv_bridge ddynamic_reconfigure dvs_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs tf celex5_msgs
  #  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)

file(GLOB_RECURSE DV_ROS_LIB_HDRS dv_ros/*.h)
file(GLOB_RECURSE DV_ROS_LIB_SRCS dv_ros/*.cc)
file(GLOB_RECURSE DV_ROS_NODE_SRC dv_ros/*_node.cc)
list(REMOVE_ITEM DV_ROS_LIB_SRCS ${DV_ROS_NODE_SRC})

add_library(dv_ros_lib STATIC ${DV_ROS_LIB_HDRS} ${DV_ROS_LIB_SRCS} dv_ros/dv_ros_node.cc)
target_link_libraries(dv_ros_lib
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  dv::dvsdk)

add_executable(dv_ros_node ${DV_ROS_NODE_SRC})
target_link_libraries(dv_ros_node
  dv_ros_lib)
