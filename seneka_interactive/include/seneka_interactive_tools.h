/*
 * seneka_interactive_tools.h
 *
 *  Created on: 23.06.2014
 *      Author: Matthias Nösner      
 */

#ifndef SENEKA_INTERACTIVE_TOOLS_H_
#define SENEKA_INTERACTIVE_TOOLS_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <boost/algorithm/string.hpp>

const unsigned int SIMULATE_CARTESIAN_PATH = 0; 
const unsigned int SIMULATE_JOINT_TARGET = 1; 
const unsigned int SIMULATE_POSE_TARGET = 2; 
const unsigned int SIMULATE_FROM_REALWORLD = 3; 
const unsigned int SIMULATE_PICKUP_PROCESS = 4; 

struct simulation_order{
	bool simulate;
	uint simulated_arm;
	uint iterations;
	uint option;	
};

namespace seneka_interactive_tools{

//	moveit_msgs::Constraints generateIKConstraints(const char* command, std::vector<std::string> joint_names, std::vector<double> joint_positions, moveit_msgs::Constraints constraints, double tolerance, double position = 0){
//		
//		moveit_msgs::Constraints new_constraints generateIKConstraints(command,joint_names,joint_positions,tolerance,position);
//		
//	}

	//generateIKConstraints helps with simplified Constraint generation
	//Commands: copy -> copy joint positions of group to the constraint 
	//			all -> apply to all joints in group
	//			0,1,2... -> apply to given joint number (multiple joints possible)
	moveit_msgs::Constraints generateIKConstraints(const char* command, std::vector<std::string> joint_names, std::vector<double> joint_positions, double tolerance, double position = 0) {
		
		//options
		bool all = false;
		bool copy = false;
		std::vector < moveit_msgs::JointConstraint > joint_constraints;

		//exploit the commands
		std::string cmd(command);
		std::vector < std::string > fields;
		boost::split(fields, cmd, boost::is_any_of(" "));
		for (uint i = 0; i < fields.size(); i++) {
			ROS_INFO("command: %s \n", fields[i].c_str());
			if(!fields[i].compare("all")) all = true;
			if(!fields[i].compare("copy")) copy = true;
		}
		ROS_INFO("command value all: %d \n", all);
		ROS_INFO("command value copy: %d \n", copy);

		//extract the affected joints		
		//move_group_interface::MoveGroup movegroup(group);
		//std::vector < std::string > joint_names = movegroup.getJoints();
		//std::vector<double> joint_values = movegroup.getCurrentJointValues();
		
		if (all) { //extract all
			for (uint i = 0; i < joint_names.size(); i++) {
				moveit_msgs::JointConstraint jconstraint;
				jconstraint.joint_name = joint_names[i];
				jconstraint.weight = 0.5;
				jconstraint.position = joint_positions[i];
				joint_constraints.push_back(jconstraint);
			}
		} else { //extract only specific joints
			for (uint i = 0; i < fields.size(); i++) {//iterate through all commands
				for (uint j = 0; j < 12; j++) {	//iterate through possible joint numbers
					char joint_number[20];
					sprintf(joint_number, "%u", j);
					if (!fields[i].compare(joint_number)) {
						if (j < joint_names.size() && j < joint_positions.size()) {
							moveit_msgs::JointConstraint jconstraint;
							jconstraint.joint_name = joint_names[j];//name
							jconstraint.weight = 0.5;
							jconstraint.position = joint_positions[j];//value
							joint_constraints.push_back(jconstraint);
						} else {
							ROS_INFO("Ups.. joint %u is not available. There are only %u joints known", j, (uint) joint_names.size());
						}
					}
				}
			}
		}

		if (!copy) {// use the value given in position			
			for (uint i = 0; i < joint_constraints.size(); i++){
				joint_constraints[i].position = position;		
			}	
		}
		
		//copy the tolerance
		for (uint i = 0; i < joint_constraints.size(); i++){
			joint_constraints[i].tolerance_above = tolerance;
			joint_constraints[i].tolerance_below = tolerance;
		}

		moveit_msgs::Constraints constraints;
		constraints.joint_constraints = joint_constraints;
		
		return constraints;
	}
}

#endif