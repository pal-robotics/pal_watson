cmake_minimum_required(VERSION 2.8.3)
project(pal_watson_visual_recognition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  actionlib
  actionlib_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
   FILES
   ClassifyImage.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

#############
## Install ##
#############
install(PROGRAMS
  scripts/watson_recognizer_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

