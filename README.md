SENEKA_DEPLOYMENT_UNIT
======================

This repository contains source code for the deployment units developed at Fraunhofer IPA in the SENEKA project.
The package is made to work with ros groovy and gazebo version 1.5

----------------------------------------------------------------------
PACKAGES:
=========

Functionality packages:
-----------------------
* seneka_pnp_planer (not working) -> pick and place planner for the deployment of the sensor node
* seneka_sensornode_detection (not working) -> starts the cob_fiducial node and manages the detection process

Description packages:
-----------------------
* seneka_ugv_description -> Quanjo description files and gazebo launch
* seneka_moveit_config -> moveit config for Quanjo
* universal_robot -> configured universal robot stack for seneka project

Install:
--------
* git clone https://github.com/Equanox/cob_object_perception/tree/seneka_fiducials (rosbuild package)
* git clone https://github.com/Equanox/seneka_deployment_unit (catkin package)

Getting started with Quanjo in gazebo:
----------------------------------------
* roslaunch seneka_ugv_description ugv.launch
* roslaunch seneka_moveit_config move_group.launch
* roslaunch seneka_moveit_config moveit_rviz.launch



