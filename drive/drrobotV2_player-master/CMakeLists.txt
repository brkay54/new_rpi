cmake_minimum_required(VERSION 2.8.3)
project(drrobot_jaguarV2_player)

find_package(catkin REQUIRED COMPONENTS DrRobotMotionSensorDriver geometry_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf)


 add_message_files(
   FILES
   CustomSensor.msg
   IMUInfo.msg
   MotorInfo.msg
   MotorInfoArray.msg
   PowerInfo.msg
   Range.msg
   RangeArray.msg
   StandardSensor.msg
 )


 generate_messages(
   DEPENDENCIES
   geometry_msgs   nav_msgs   sensor_msgs   std_msgs
 )


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS DrRobotMotionSensorDriver geometry_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(drrobot_jaguarv2_player_node src/drrobot_player.cpp)
#add_executable(drrobot_jaguarv2_msg_tester_node src/drrobotMsgTester.cpp)
#add_executable(drrobot_jaguarv2_keyboard_teleop_node src/drrobot_keyboard_teleop.cpp)
#add_executable(drrobot_jaguarv2_IMU_node src/drrobot_imu.cpp)


target_link_libraries(drrobot_jaguarv2_player_node  ${catkin_LIBRARIES} DrRobotMotionSensorDriver )
#target_link_libraries(drrobot_jaguarv2_msg_tester_node ${catkin_LIBRARIES})
#target_link_libraries(drrobot_jaguarv2_keyboard_teleop_node ${catkin_LIBRARIES})
#target_link_libraries(drrobot_jaguarv2_IMU_node ${catkin_LIBRARIES})


