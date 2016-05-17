cmake_minimum_required(VERSION 2.8.3)
project(ros_hard_drive)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ros_computer_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES ros_hard_drive
 #  CATKIN_DEPENDS rospy
 #  DEPENDS system_lib
 CATKIN_DEPENDS ros_computer_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)