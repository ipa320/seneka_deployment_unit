cmake_minimum_required(VERSION 2.8.3)
project(seneka_pnp)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  tf
  moveit_core 
  moveit_ros_planning_interface
  pluginlib	
  message_generation
  genmsg
  actionlib
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)


add_action_files(
  DIRECTORY 
  action 
  FILES 
  QuanjoManipulation.action
)
add_service_files(
  FILES
  getState.srv
  setTransition.srv
  setStop.srv
)
add_message_files(
  FILES
  quanjo_manipulation.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  control_msgs
  actionlib_msgs
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
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    tf
    moveit_core
    moveit_ros_planning_interface
    pluginlib
    message_runtime
  DEPENDS
    OpenCV
    Eigen
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
# add_executable(seneka_pnp_node src/seneka_pnp_node.cpp)
add_executable(seneka_pick_and_place
  src/seneka_pnp.cpp
  src/SerializeIO.cpp
)

## Specify libraries to link a library or executable target against
add_library(seneka_pnp_tools src/seneka_pnp_tools.cpp)

add_dependencies(seneka_pnp_tools seneka_sensornode_detection_gencpp)

target_link_libraries(seneka_pick_and_place
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  seneka_pnp_tools
)
