cmake_minimum_required(VERSION 2.8.3)
project(graph_optimization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_optimization
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
