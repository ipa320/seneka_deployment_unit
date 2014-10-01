cmake_minimum_required(VERSION 2.8.3)
project(seneka_sensornode_detection)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  moveit_core 
  moveit_ros_planning_interface
  pluginlib	
  cob_object_detection_msgs
  message_generation
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  setCameraPose.srv
  getCameraPose.srv
  calibrateCamera.srv
  getExtCalibration.srv
  compensateInaccuracy.srv
  setSensornodePose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES seneka_sensornode_detection
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs
    tf 
    moveit_core 
    moveit_ros_planning_interface 
    cob_object_detection_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)

## Declare a cpp executable
add_executable(sensornode_detection
  src/sensornode_detection.cpp
  src/SerializeIO.cpp
)

add_dependencies(sensornode_detection cob_object_detection_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sensornode_detection
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

