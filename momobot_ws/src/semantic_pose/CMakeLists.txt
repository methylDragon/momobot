cmake_minimum_required(VERSION 2.8.3)
project(semantic_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES semantic_pose
 CATKIN_DEPENDS roscpp tf std_msgs geometry_msgs
 DEPENDS system_lib
)

add_compile_options(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(places
  src/places.cpp
)

add_executable(semantic_pose_node src/semantic_pose_node.cpp)

target_link_libraries(semantic_pose_node
  places
  ${catkin_LIBRARIES}
)
