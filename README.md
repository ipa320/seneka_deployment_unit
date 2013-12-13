SENEKA_DEPLOYMENT_UNIT
======================

This repository contains source code for the deployment units developed at Fraunhofer IPA in the SENEKA project.
The package is made to work with ros groovy and gazebo version 1.5

The seneka project uses the cob_fiducial node for marker detection. A documentation of cob_fiducials can be found here http://wiki.ros.org/cob_fiducials

----------------------------------------------------------------------

PACKAGES:
=========

Functionality packages:
-----------------------
* seneka_pnp_planer (not working) -> pick and place planner for the deployment of the sensor node
* seneka_sensornode_detection (not working) -> starts the cob_fiducial node and manages the detection process

Description packages:
-----------------------
* seneka_ugv_description -> Quanjo description files and gazebo launch + sensornode xacro blueprint
* seneka_moveit_config -> moveit config for Quanjo
* seneka_msgs -> msgs to send fiducial Array from cob_fiducials to seneka_sensornode_detection
* universal_robot -> configured universal robot stack for seneka project

----------------------------------------------------------------------

Install:
--------
* git clone https://github.com/Equanox/universal_robot/tree/seneka_quanjo (rosbuild package)
* git clone https://github.com/ipa320/cob_perception_common (rosbuild package)
* git clone https://github.com/Equanox/cob_object_perception/tree/seneka_fiducials (rosbuild package)
* git clone https://github.com/Equanox/seneka_deployment_unit (catkin package)

----------------------------------------------------------------------

Getting started with Quanjo in gazebo:
----------------------------------------
* roslaunch seneka_ugv_description ugv.launch
* roslaunch seneka_moveit_config move_group.launch
* roslaunch seneka_moveit_config moveit_rviz.launch
* rosrun seneka_sensornode_detection sensornode_detection (places the sensornode to a defined position)

----------------------------------------------------------------------

Getting started with detection of sensornode:
-----------------------------------------
* roslaunch seneka_sensornode_detection sensornode_detection.launch (starts freenect,cob_fiducial and sensornode_detection)


TODO: (for milestone february)
----------------------------------------------------------------------
* determine sensornode position and give advises(console output) for optimal grabbing position (directly in front of quanjo).
* define coordinate system of the sensorsonde with grabbing points
* choose a grabbing strategy
* choose a pickup strategy (start with very simple task)
* synchronize movement of both arms (ideas?)
* when sensornode is in position start the pickup process

----------------------------------------------------------------------
* Author: Matthias Nösner 
* Readme updated: 13.12.2013

