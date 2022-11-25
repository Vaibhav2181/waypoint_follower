cmake_minimum_required(VERSION 3.0.2)
project(waypoint_follower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  move_base
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_follower
#  CATKIN_DEPENDS actionlib actionlib_msgs dynamic_reconfigure geometry_msgs move_base move_base_msgs nav_msgs roscpp rospy std_msgs std_srvs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_follower
  src/waypoint_follower.cpp
)
add_dependencies(waypoint_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_follower
  ${catkin_LIBRARIES}
)


