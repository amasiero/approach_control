cmake_minimum_required(VERSION 2.8.3)
project(approach_control_people)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  p2os_msgs
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)