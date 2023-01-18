cmake_minimum_required(VERSION 3.0.2)
project(champy_action_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_action_files(
  FILES
  parking.action
  unparking.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)