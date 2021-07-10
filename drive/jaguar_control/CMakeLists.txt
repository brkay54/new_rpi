cmake_minimum_required(VERSION 3.0.2)
project(jaguar_control)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  roslib
  fly_msgs
  mavros
  mavros_msgs
  libmavconn
  nav_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jaguar_control 
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs tf roslib fly_msgs

)



include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


add_executable(drive_path_follower src/drive_path_follower.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_link_libraries(drive_path_follower ${catkin_LIBRARIES} )





