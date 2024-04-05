cmake_minimum_required(VERSION 3.0.2)
project(map_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_tools
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

