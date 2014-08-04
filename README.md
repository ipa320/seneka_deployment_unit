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
* seneka_msgs -> not used
* universal_robot -> configured universal robot stack for seneka project


Getting started:
=========

##Required:
---------
* git clone https://github.com/ipa-mig/universal_robot  [rosbuild] (use the seneka_groovy_dev branch) 
* git clone https://github.com/ipa320/cob_perception_common  [catkin] (make sure you are using the groovy_dev_catkin branch) 
* git clone https://github.com/ipa320/cob_object_perception  [rosbuild]
* git clone https://github.com/ipa320/seneka_deployment_unit  [catkin]

##1. Configuration
---------------------------------------------------------------------
* Go to the launchfile of the cob_fiducial package **../cob_object_perception/cob_fiducials/ros/launch/fiducials.launch**.
Make sure that the rgb_topic argument points to the right image publisher.
E.g. for the Kinect use **/camera/rgb/**.

* Additionaly assure that the Fast PiTag markers are used (the different marker types are loaded using a .yaml file). See http://wiki.ros.org/cob_fiducials for details.

---------------------------------------------------------------------
* All the coordinate systems of the sensorsonde (marker,handle, grabpoints) can be adjusted in **../seneka_deployment_unit/seneka_sensornode_detection/common/sensorsonde_coordinates.def**. Right now only fiducial1 has valid coordinates.

* gripper length and camera position is hardcoded in **../seneka_deployment_unit/seneka_sensornode_detection/src/sensornode_detection.cpp**

* for plan execution the boolean **planexecution** in **seneka_pnp.cpp** need to be **true**

##2. Connect the Pipeline (stable)
---------------------------------------------------------------------
* roslaunch seneka_sensornode_detection sensornode_detection.launch (starts freenect,cob_fiducial and sensornode_detection nodes)
* rostopic hz /fiducials/detect_fiducials (start marker detection)
* roslaunch seneka_ugv_description ugv_bringup.launch robot_ip_r:=192.168.1.11 robot_ip_l:=192.168.1.12 (connection for two ur10's with static ip's. The pc needs a static ip in the same subnet)
* roslaunch seneka_moveit_config move_group.launch (load the move group and moveit config)
* roslaunch seneka_moveit_config moveit_rviz.launch (visualization of planed trajectories)
* rosrun image_view image_view image:=/fiducials/image

##3. Pick up (development)
---------------------------------------------------------------------
* rosrun seneka_pnp seneka_pick_and_place

This node manages the pick up process for a sensorsonde (REMEMBER: Watch at rviz and **HANDS ON THE RED BUTTONS**).
There are multiple sleep commands in the code to enable visual trajectory checking with rviz before executing the trajectory.

At first the node is planning a path to pickup the sensorsonde. 

Second the arms move through the waypoints saved in **..seneka_deployment_unit/seneka_pnp/common/teached_dual_arm_movement.def**.

##4. Teach (development)
---------------------------------------------------------------------
* rosrun seneka_pnp seneka_teach

For teaching it is necessary to be connected with both arms. Each time you press **t** the position and orientation of the endeffectors are saved.

**- Note1:** You can't move the arms when a connection between arms and pc is established. So each time you want to move the arms you have to kill the 
ugv_bringup node and reconnect before you press **t**.

**- Note2:** There is no file override protection. You can easily change the storage file in the code and recompile.

##5. TODO
---------------------------------------------------------------------
* Right now only the marker with id 1 is used for the detection!

---------------------------------------------------------------------

* Author: Matthias Nösner 
* Readme updated: 24.02.2014

