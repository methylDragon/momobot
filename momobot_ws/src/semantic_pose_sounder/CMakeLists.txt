cmake_minimum_required(VERSION 2.8.3)
project(semantic_pose_sounder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
)

catkin_package()
catkin_python_setup()
