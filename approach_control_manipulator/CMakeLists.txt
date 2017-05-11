cmake_minimum_required(VERSION 2.8.3)
project(approach_control_manipulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
#  message_generation
)

catkin_python_setup()

#add_service_files(
#    FILES
#    Gesture.srv
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

catkin_package(
#    CATKIN_DEPENDS 
#    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)