cmake_minimum_required(VERSION 3.0.2)
project(eskf_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(eskf_localization src/eskf_localization.cpp)
target_link_libraries(eskf_localization ${catkin_LIBRARIES})

