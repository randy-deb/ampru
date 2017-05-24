cmake_minimum_required(VERSION 2.8.3)
project(ampru_msgs)

find_package(
  catkin
  REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WheelControl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
)
