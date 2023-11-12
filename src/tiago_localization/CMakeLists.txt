cmake_minimum_required(VERSION 2.8.3)
project(tiago_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  amcl
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs amcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tiago_localization_node src/tiago_localization_node.cpp)

target_link_libraries(tiago_localization_node
  ${catkin_LIBRARIES}
)
