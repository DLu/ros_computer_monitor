cmake_minimum_required(VERSION 2.8.3)
project(ros_top)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rosnode rospy ros_computer_msgs) #python-psutil
catkin_python_setup()
catkin_package( CATKIN_DEPENDS rosnode rospy ros_computer_msgs)

install(PROGRAMS scripts/ros_top
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
