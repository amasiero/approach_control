cmake_minimum_required(VERSION 2.8.3)
project(approach_control_sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package()