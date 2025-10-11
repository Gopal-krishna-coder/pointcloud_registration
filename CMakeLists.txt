cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_registration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  message_generation
)

add_service_files(
  FILES
  RegisterPointCloud.srv
  SaveAccumulatedCloud.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)