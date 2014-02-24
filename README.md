SENEKA_DEPLOYMENT_UNIT
======================

This repository contains source code for the deployment units developed at Fraunhofer IPA in the SENEKA project.
The package is made to work with ros groovy and gazebo version 1.5

The seneka project uses the cob_fiducial package for marker detection. A documentation of cob_fiducials can be found here http://wiki.ros.org/cob_fiducials

----------------------------------------------------------------------

PACKAGES:
=========

Functionality packages:
-----------------------
* seneka_pnp -> pick and place planner for picking up the sensorsonde
* seneka_sensornode_detection -> Brings up the cob_fiducial node and manages the detection process. It also publish all the necessary tf positions of the sensorsonde
				
Description packages:
-----------------------
* seneka_ugv_description -> Quanjo description files and gazebo_bringup + seneka_bringup
* seneka_moveit_config -> moveit config for Quanjo
* seneka_msgs -> msgs to send fiducial Array from cob_fiducials to seneka_sensornode_detection
* universal_robot -> configured universal robot stack for seneka project

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


Getting started:
=========

Required:
---------
* git clone https://github.com/ipa320/universal_robot/tree/groovy_dev (WRONG!!!! -> Talk to Matthias Gruler) 
* git clone https://github.com/ipa320/cob_perception_common (catkin) (make sure you are using the groovy_dev_catkin branch) 
* git clone https://github.com/ipa320/cob_object_perception (rosbuild)
* git clone https://github.com/Equanox/seneka_deployment_unit (catkin)

1. Configuration
---------------------------------------------------------------------
* Go to the launchfile of the cob_fiducial package **../cob_object_perception/cob_fiducials/ros/launch/fiducials.launch**.
Make sure that the rgb_topic argument points to the right image publisher.
E.g. for the Kinect use ** /camera/rgb/ **. 

* Additionaly assure that the Fast PiTag markers are used (the different marker types are loaded using a .yaml file).
See http://wiki.ros.org/cob_fiducials for details.

2. Connect the Pipeline
---------------------------------------------------------------------
* roslaunch seneka_sensornode_detection sensornode_detection.launch (starts freenect,cob_fiducial and sensornode_detection nodes)
* rostopic hz /fiducials/detect_fiducials (start marker detection)
* roslaunch seneka_ugv_description ugv_bringup.launch robot_ip_l:=192.168.1.11 robot_ip_r:=192.168.1.12 (connection for two ur10's with static ip's. The pc needs a static ip in the same subnet)
* roslaunch seneka_moveit_config move_group.launch (load the move group and moveit config)
* roslaunch seneka_moveit_config moveit_rviz.launch (visualization of planed trajectories)

3. Pick up
---------------------------------------------------------------------
* rosrun seneka_pnp seneka_pick_and_place (starts the pick up process for a sensosonde, watch to rviz and HANDS ON RED BUTTON)


---------------------------------------------------------------------
* Author: Matthias Nösner 
* Readme updated: 13.12.2013

