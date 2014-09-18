/*
 * seneka_pnp_tools.h
 *
 *  Created on: 18.06.2014
 *      Author: Matthias Nösner
 */

#ifndef SENEKA_PNP_TOOLS_H_
#define SENEKA_PNP_TOOLS_H_

#include <ros/ros.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <moveit/move_group_interface/move_group.h>

#include <sensor_msgs/JointState.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "seneka_sensornode_detection/compensateInaccuracy.h"

#include <tf/transform_listener.h>

struct handhold{
	geometry_msgs::Pose handle;
	geometry_msgs::Pose entry;
	geometry_msgs::Pose up;
	geometry_msgs::Pose down;
};

struct sensornode{	
	bool success;
	geometry_msgs::Pose pose;
	std::vector<handhold> handholds;
};

struct node_pose {
	std::string name;
	geometry_msgs::Pose pose;
	geometry_msgs::Pose handle_r;
	geometry_msgs::Pose handle_l;
	std::vector<std::string> joint_names_r;
	std::vector<std::string> joint_names_l;
	std::vector<double> joint_states_r;
	std::vector<double> joint_states_l;
};

struct dual_arm_joints{
	std::vector<double> both;
	std::vector<double> right;
	std::vector<double> left;
};

struct dualArmJointState{
	std::string name;
	sensor_msgs::JointState right;
	sensor_msgs::JointState left;
	sensor_msgs::JointState both;	
};


namespace seneka_pnp_tools{

//reads the sensornode pose from tf poses
sensornode getSensornodePose();

//forward kinematics solver
void fk_solver(ros::NodeHandle*, std::vector<double>&, std::vector<double>&, geometry_msgs::Pose*, geometry_msgs::Pose*);

//enables multiple planning
bool multiplan(move_group_interface::MoveGroup*, moveit::planning_interface::MoveGroup::Plan*);

//code from prace project, IPA: Benjamin Maidel
move_group_interface::MoveGroup::Plan mergePlan(move_group_interface::MoveGroup::Plan, move_group_interface::MoveGroup::Plan);

//scaleing the speed and acceleration components of a trajectory
moveit::planning_interface::MoveGroup::Plan scaleTrajSpeed(moveit::planning_interface::MoveGroup::Plan, double);

dual_arm_joints generateIkSolutions(ros::NodeHandle, std::vector<double>&, std::vector<double>&, geometry_msgs::Pose, geometry_msgs::Pose, moveit_msgs::Constraints, moveit_msgs::Constraints, 
									bool equaljointstates  = true, bool freezeikleft = false, bool freezeikright = false);

//returns the state distance between two joint states
double getStateDistance(std::vector<double>, std::vector<double>);

//copied from seneka_interactive_tools to avoid dependency
//generateIKConstraints helps with simplified Constraint generation
//Commands: copy -> copy joint positions of group to the constraint 
//			all -> apply to all joints in group
//			0,1,2... -> apply to given joint number (multiple joints possible)
// command, joint_names, joint_positions, tolerance, position
moveit_msgs::Constraints generateIKConstraints(const char*, std::vector<std::string>, std::vector<double>, double, double position = 0);

//This function initializes a vector wit all states
//all states are hardcoded
std::vector<dualArmJointState> createArmStates();

//creates a single state with name, and 2*6 joints
//name, 6*right, 6*left
dualArmJointState createArmState(const char*,
								 double, double, double, double, double, double,
								 double, double, double, double, double, double);

//crawls a vector for a state with specific name
bool getArmState(std::vector<dualArmJointState>&, const char*, dualArmJointState*);

//computes the correct yaw angle for grabbing the sonde
bool sensornodeYawRotation(geometry_msgs::Pose);

//invokes a service call to compensate the inaccuracy whe grabbing for pickup rear pose
bool compensateInaccuracyDO(ros::NodeHandle);
bool compensateInaccuracyUNDO(ros::NodeHandle);
}

#endif