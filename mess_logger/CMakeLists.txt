cmake_minimum_required(VERSION 3.0.2)
project(mess_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mess_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs mess_msgs nav_msgs sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/mess_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)