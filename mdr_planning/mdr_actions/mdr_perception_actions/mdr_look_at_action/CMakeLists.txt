cmake_minimum_required(VERSION 2.8.3)
project(mdr_look_at_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  genmsg
  message_generation
  roslint
  rospy
  roslaunch
  mcr_perception_msgs
  mas_perception_libs
)

catkin_python_setup()

add_action_files(DIRECTORY ros/action
  FILES
    LookAt.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    mcr_perception_msgs
)

catkin_package(
 CATKIN_DEPENDS
   actionlib_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    ros/scripts/look_at_action
    ros/scripts/look_at_client_test
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

roslint_python()
roslaunch_add_file_check(ros/launch)
