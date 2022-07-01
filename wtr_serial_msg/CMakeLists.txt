cmake_minimum_required(VERSION 3.0.2)
project(wtr_serial_msg)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  em_ev.msg
  em_fb_raw.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)
