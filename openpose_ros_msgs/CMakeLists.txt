cmake_minimum_required(VERSION 2.8.7)
project(openpose_ros_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  message_runtime
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

add_message_files(
  FILES
  BoundingBox.msg
  OpenPoseHuman.msg
  OpenPoseHumanList.msg
  PointWithProb.msg
)

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  OpenPoseHumanList.action
)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs 
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs  
)