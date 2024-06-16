cmake_minimum_required(VERSION 3.0.2)
project(mess_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  MESS2UGV.msg
  StateUAV.msg
  StateUGV.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_generation message_runtime sensor_msgs std_msgs
)
