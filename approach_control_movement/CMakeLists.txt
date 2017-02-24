cmake_minimum_required(VERSION 2.8.3)
project(approach_control_movement)

find_package(catkin REQUIRED COMPONENTS
  p2os_driver
  roscpp
  rospy
)

catkin_python_setup()

catkin_package()