cmake_minimum_required(VERSION 3.0.2)
project(fly_control)

find_package(catkin REQUIRED COMPONENTS
  libmavconn
  mavros
  mavros_msgs
  roscpp
  std_msgs
  geometry_msgs
  fly_msgs
        nav_msgs
   sensor_msgs 
   cv_bridge 
   image_transport

)

find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
find_package(PythonLibs 2.7)
link_directories(${GAZEBO_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fixed_fly
  CATKIN_DEPENDS libmavconn mavros mavros_msgs roscpp std_msgs geometry_msgs fly_msgs 

)


include_directories( SYSTEM ${catkin_INCLUDE_DIRS} )


add_executable(fly_path_follower src/fly_path_follower.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_include_directories(fly_path_follower PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(fly_path_follower ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(sim_fly_path_follower src/sim_fly_path_follower.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_include_directories(sim_fly_path_follower PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(sim_fly_path_follower ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(camera_trig src/camera_trig.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_link_libraries(camera_trig ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(sim_camera_trig src/sim_camera_trig.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_link_libraries(sim_camera_trig ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(land src/land.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/path_segment.cpp include/path_segment.h)
target_include_directories(land PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(land ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(tester src/tester.cpp include/path_plotter.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/matplotlibcpp.h include/path_plotter.cpp include/path_plotter.h include/path_segment.cpp include/path_segment.h)
target_include_directories(tester PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(tester ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )


#add_executable(pose_plotter src/pose_plotter.cpp include/path_plotter.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/matplotlibcpp.h include/path_plotter.cpp include/path_plotter.h include/path_segment.cpp include/path_segment.h)
#target_include_directories(pose_plotter PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(pose_plotter ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )


#add_executable(error_plotter src/error_plotter.cpp include/path_plotter.cpp include/commons.h include/commons.cpp include/survey_path_generator.h include/survey_path_generator.cpp include/matplotlibcpp.h include/path_plotter.cpp include/path_plotter.h include/path_segment.cpp include/path_segment.h)
#target_include_directories(error_plotter PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(error_plotter ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} )


