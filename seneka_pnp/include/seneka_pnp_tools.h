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

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

namespace seneka_pnp_tools{

	//wrapper for calling the fk service due to some failures when using it with move_group.h
	void fk_solver(ros::NodeHandle *node_handle, std::vector<double> &joint_positions_r, std::vector<double> &joint_positions_l, geometry_msgs::Pose *pose_l, geometry_msgs::Pose *pose_r){
	
		ros::ServiceClient service_computefk;
		service_computefk = node_handle->serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");		
		
		robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
		robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
		robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
		robot_state::JointStateGroup* joint_state_group_l = kinematic_state_l->getJointStateGroup("left_arm_group");  
		robot_state::JointStateGroup* joint_state_group_r = kinematic_state_l->getJointStateGroup("right_arm_group"); 
	
		moveit_msgs::GetPositionFK::Request service_request;
		moveit_msgs::GetPositionFK::Response service_response; 
		sensor_msgs::JointState js;
	
		//------left-----------------------
		js.name = joint_state_group_l->getJointNames();
		js.position = joint_positions_l;
	
		service_request.header.frame_id = "world_dummy_link";
		service_request.fk_link_names.push_back("left_arm_ee_link");
		service_request.robot_state.joint_state = js;
	
		service_computefk.call(service_request, service_response);
		if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
			for(uint i=0;i<service_response.pose_stamped.size();i++){
				std::cout << service_response.pose_stamped[i].pose.position << std::endl;
				std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
				*pose_l = service_response.pose_stamped[i].pose;
			}
		}   
	
		//-----right-----------------
		js.name = joint_state_group_r->getJointNames();
		js.position = joint_positions_r;
	
		service_request.header.frame_id = "world_dummy_link";
		service_request.fk_link_names.push_back("right_arm_ee_link");
		service_request.robot_state.joint_state = js;
	
		service_computefk.call(service_request, service_response);
		if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
			for(uint i=0;i<service_response.pose_stamped.size();i++){
				std::cout << service_response.pose_stamped[i].pose.position << std::endl;
				std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
				*pose_r = service_response.pose_stamped[i].pose;
			}
		}  
	}


}

#endif