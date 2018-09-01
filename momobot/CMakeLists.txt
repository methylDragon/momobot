cmake_minimum_required(VERSION 2.8.3)
project(momobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf
    nav_msgs 
    lino_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES momobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs lino_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(momo_base_node src/momo_base_node.cpp)
target_link_libraries(momo_base_node ${catkin_LIBRARIES})
