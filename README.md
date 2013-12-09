SENEKA_DEPLOYMENT_UNIT
======================

This repository contains source code for the deployment units developed at Fraunhofer IPA in the SENEKA project.
The package is made to work with ros groovy and gazebo version 1.5

----------------------------------------------------------------------
PACKAGES:
=========

Functionality packages:\n
seneka_pnp_planer (not working) -> pick and place planner for the deployment of the sensor node \n
seneka_sensornode_detection (not working) -> starts the cob_fiducial node and manages the detection process \n

Description packages:\n
seneka_ugv_description -> Quanjo description files and gazebo launch \n
seneka_moveit_config -> moveit config for Quanjo \n
universal_robot -> configured universal robot stack for seneka project \n

Install: \n
git clone https://github.com/Equanox/cob_object_perception/tree/seneka_fiducials (rosbuild package) \n
git clone https://github.com/Equanox/seneka_deployment_unit (catkin package) \n

Getting started with Quanjo in gazebo: \n
roslaunch seneka_ugv_description ugv.launch \n
roslaunch seneka_moveit_config move_group.launch \n
roslaunch seneka_moveit_config moveit_rviz.launch \n



