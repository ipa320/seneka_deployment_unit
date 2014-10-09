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
* seneka_pnp -> pick and place state machine for picking up the sensorsonde
* seneka_sensornode_detection -> Brings up the cob_fiducial node and manages the detection process. It also publish all the necessary tf positions of the sensorsonde.


Getting started:
=========

##Required:
---------
**.. for running the real robot**

* git clone https://github.com/equanox/universal_robot  (use the **ros-industrial-hydro-devel-backported-groovy** branch) 
* git clone https://github.com/ipa320/cob_perception_common (make sure you are using the **groovy_dev_catkin** branch) 
* git clone https://github.com/equanox/cob_object_perception (**seneka_fiducials** branch) [rosbuild]
* git clone https://github.com/ipa320/seneka_deployment_unit (use the **groovy-dev* branch)


##Additional Installation
* sudo apt-get install ros-groovy-pr2
* sudo apt-get install ros-groovy-pr2-simulator
* sudo apt-get install ros-groovy-pr2-controllers
* gazebo_msgs
* sudo apt-get install ros-groovy-freenect-stack
* on ubuntu 12.04 ( sudo modprobe -r gspca_kinect )
* on ubuntu 12.04 ( echo 'blacklist gspca_kinect' | sudo tee -a /etc/modprobe.d/blacklist.conf )
* git clone https://github.com/Equanox/moveit_ros **groovy-devel** branch, for lma_kinematics 

##1. Configuration
---------------------------------------------------------------------
* Go to the launchfile of the cob_fiducial package **../cob_object_perception/cob_fiducials/ros/launch/fiducials.launch**.
Make sure that the rgb_topic argument points to the right image publisher.
E.g. for the Kinect use **/camera/rgb/**.

* Additionaly assure that the Fast PiTag markers are used (the different marker types are loaded using a .yaml file). See http://wiki.ros.org/cob_fiducials for details.

---------------------------------------------------------------------
**These steps are only necessary if you use the ipa320 stack of cob_object_perception**

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

To start pickup/deploy front you can use a action (pickup => val=1, deploy => val=2)

* rostopic pub /seneka_pick_and_place/goal seneka_pnp/QuanjoManipulationActionGoal  

This node manages the pick up process for a sensorsonde (REMEMBER: Watch at rviz and **HANDS ON THE RED BUTTONS**).

##5. TODO
---------------------------------------------------------------------
* Right now only the marker with id 1 is used for the detection!
* Switch to camera with shutter
* Documentation

---------------------------------------------------------------------

* Author: Matthias Nösner 
* Readme updated: 09.10.2014

