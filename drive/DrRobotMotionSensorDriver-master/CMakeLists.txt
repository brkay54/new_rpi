cmake_minimum_required(VERSION 2.8.3)
project(DrRobotMotionSensorDriver)


find_package(catkin REQUIRED)




catkin_package(

)


add_library(DrRobotMotionSensorDriver
   src/DrRobotMotionSensorDriver.cpp
 )



