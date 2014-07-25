#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <iomanip> 

#include <interactive_markers/interactive_marker_server.h>
#include <seneka_interactive_tools.h>

// Moveit msgs
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

//services
#include "seneka_interactive/setStartState.h"
#include "seneka_interactive/setGoalState.h"
#include "seneka_interactive/simulate.h"
#include "seneka_interactive/createNewState.h"
#include "seneka_interactive/getStates.h"
#include "seneka_interactive/printStatesToFile.h"
#include "seneka_interactive/toggleArmFreeze.h"
#include "seneka_interactive/setJointState.h"
#include "seneka_interactive/getJointStates.h"
#include "seneka_interactive/setConstraints.h"
#include "seneka_interactive/generateIK.h"



#include <boost/bind.hpp>

const double PI = 3.14159265359;

class SenekaInteractive {

private:
	ros::NodeHandle node_handle_;

	ros::ServiceServer service_setStartState_, service_setGoalState_,
			service_simulate_, service_createNewState_, service_getStates_,
			service_printStatesToFile_, service_toggleArmFreeze_;
	ros::ServiceServer service_setJointState_, service_getJointStates_, service_setConstraints_,
			service_generateIK_;

	ros::Publisher robot_state_publisher_l_, robot_state_publisher_r_;
	interactive_markers::InteractiveMarkerServer* server_;
	geometry_msgs::Pose marker_pose_;
	geometry_msgs::Pose handle_l_, handle_r_;
	moveit_msgs::Constraints constraints_l, constraints_r;
	bool marker_changed_;
	double gripper_length;
	std::vector<node_pose> stored_poses;

	//bool simulate_;
	bool generateIK_;
	bool equaljointstates_;
	simulation_order simulate_;

	bool freeze_ik_left_, freeze_ik_right_;

	node_pose start_pose_, tmp_pose_, goal_pose_;

public:
	//Constructor
	SenekaInteractive(ros::NodeHandle& nh) {
		node_handle_ = nh;
		init();
	}

	//Destructor
	~SenekaInteractive() {
	}

	void init() {

		create_stored_poses();

		robot_state_publisher_l_ = node_handle_.advertise
				< moveit_msgs::DisplayRobotState > ("robot_state_l", 1);
		robot_state_publisher_r_ = node_handle_.advertise
				< moveit_msgs::DisplayRobotState > ("robot_state_r", 1);

		service_setStartState_ = node_handle_.advertiseService(
				"seneka_interactive/setStartState",
				&SenekaInteractive::setStartState, this);
		service_setGoalState_ = node_handle_.advertiseService(
				"seneka_interactive/setGoalState",
				&SenekaInteractive::setGoalState, this);
		service_simulate_ = node_handle_.advertiseService(
				"seneka_interactive/simulate", &SenekaInteractive::simulate,
				this);
		service_createNewState_ = node_handle_.advertiseService(
				"seneka_interactive/createNewState",
				&SenekaInteractive::createNewState, this);
		service_getStates_ = node_handle_.advertiseService(
				"seneka_interactive/getStates", &SenekaInteractive::getStates,
				this);
		service_printStatesToFile_ = node_handle_.advertiseService(
				"seneka_interactive/printStatesToFile",
				&SenekaInteractive::printStatesToFile, this);
		service_toggleArmFreeze_ = node_handle_.advertiseService(
				"seneka_interactive/toggleArmFreeze",
				&SenekaInteractive::toggleArmFreeze, this);
		service_setJointState_ = node_handle_.advertiseService(
				"seneka_interactive/setJointState",
				&SenekaInteractive::setJointState, this);
		service_getJointStates_ = node_handle_.advertiseService(
				"seneka_interactive/getJointStates",
				&SenekaInteractive::getJointStates, this);
		service_setConstraints_ = node_handle_.advertiseService(
				"seneka_interactive/setConstraints",
				&SenekaInteractive::setConstraints, this);
		service_generateIK_ = node_handle_.advertiseService(
				"seneka_interactive/generateIK",
				&SenekaInteractive::generateIK, this);

		tmp_pose_.name = "tmp_pose";

		freeze_ik_right_ = false;
		freeze_ik_left_ = false;

		marker_changed_ = false;
		simulate_.simulate = false;
		simulate_.option = 0;
		generateIK_ = false;
		equaljointstates_ = false;

		gripper_length = 0.26;

		interactiveMarker();
		mainLoop();
	}

	/*
	 
	///transform from left arm to right arm and add a jiggle factor
	moveit_msgs::Constraints transformJointStates(std::vector<double> joints) {

		double jiggle = PI;

		moveit_msgs::Constraints constraint;

		moveit_msgs::JointConstraint jconstraint;

		jconstraint.joint_name = "right_arm_shoulder_pan_joint";
		jconstraint.position = 1.04915;
		jconstraint.tolerance_above = PI / 4;
		jconstraint.tolerance_below = PI / 4;
		jconstraint.weight = 1;
		constraint.joint_constraints.push_back(jconstraint);

		jconstraint.joint_name = "right_arm_shoulder_lift_joint";
		jconstraint.position = 0;
		jconstraint.tolerance_above = 0.6;
		jconstraint.tolerance_below = 0.6;
		jconstraint.weight = 1;
		//constraint.joint_constraints.push_back(jconstraint);

		return constraint;
	}

	moveit_msgs::Constraints leftArmConstraints() {

		double jiggle = PI;

		moveit_msgs::Constraints constraint;

		moveit_msgs::JointConstraint jconstraint;
		jconstraint.joint_name = "left_arm_shoulder_pan_joint";
		jconstraint.position = -1.04728;
		jconstraint.tolerance_above = PI / 4;
		jconstraint.tolerance_below = PI / 4;
		jconstraint.weight = 1;
		constraint.joint_constraints.push_back(jconstraint);

		jconstraint.joint_name = "left_arm_shoulder_lift_joint";
		jconstraint.position = 1.09;
		jconstraint.tolerance_above = 0.6;
		jconstraint.tolerance_below = 0.6;
		jconstraint.weight = 1;
		//constraint.joint_constraints.push_back(jconstraint);

		return constraint;
	}*/
	
	node_pose smartJointValues(node_pose np) {

		for (uint i = 0; i < np.joint_states_r.size(); i++) {
			np.joint_states_r[i] = createSmartJointValue(np.joint_states_r[i]);
		}

		for (uint i = 0; i < np.joint_states_l.size(); i++) {
			np.joint_states_l[i] = createSmartJointValue(np.joint_states_l[i]);
		}

		return np;
	}
	
	std::vector<double> smartJointValues(std::vector<double> joint_positions) {

		for (uint i = 0; i < joint_positions.size(); i++) {
			joint_positions[i] = createSmartJointValue(joint_positions[i]);
		}

		return joint_positions;
	}

	double createSmartJointValue(double jointvalue) {

		if (jointvalue > PI) {
			jointvalue -= 2 * PI;
		} else if (jointvalue < -PI) {
			jointvalue += 2 * PI;
		} else {
			//joint value already in shape
		}

		return jointvalue;
	}

	

	//returns distance between two states based on cumulated difference of joints
	double getStateDistance(std::vector<double> a, std::vector<double> b) {

		double distance = 0;

		for (uint i = 0; i < a.size() && i < b.size(); i++) {
			
			double res = a[i] - b[i];
			distance += std::sqrt(res * res);
			//ROS_INFO("[%u] RJ:[%f]   CJ:[%f]  RES:[%f]  DIST:[%f]",i,a[i],b[i],res,distance);
		}

		return distance;
	}

	void generateIkSolutions() {

		std::vector<double> joint_values_l;
		std::vector<double> joint_values_r;

		std::vector < std::string > joint_names_l;
		std::vector < std::string > joint_names_r;

		if (generateIK_) {

			ROS_INFO("Generate IK Solution");
			ros::ServiceClient service_client;
			service_client = node_handle_.serviceClient < moveit_msgs::GetPositionIK > ("compute_ik");

			geometry_msgs::Pose target_pose_l = handle_l_;
			geometry_msgs::Pose target_pose_r = handle_r_;

			bool result = false;
			moveit_msgs::GetPositionIK::Request service_request;
			moveit_msgs::GetPositionIK::Response service_response;

			service_request.ik_request.attempts = 20;
			service_request.ik_request.pose_stamped.header.frame_id = "world_dummy_link";
			service_request.ik_request.avoid_collisions = false;
			
			tmp_pose_ = smartJointValues(start_pose_);

			//freezes ik generation when necessary
			if (!freeze_ik_left_) {
				//left arm
				service_request.ik_request.group_name = "left_arm_group";
				service_request.ik_request.pose_stamped.pose = target_pose_l;
				service_request.ik_request.constraints = constraints_l;
				
				
				uint samples;
				equaljointstates_ ? samples = 100 : samples = 1;

				std::vector<double> referencejoints;
				referencejoints = tmp_pose_.joint_states_l;
				double statedistance = 6 * 2 * PI;

				for (uint i = 0; (i < samples); i++) {
					joint_values_l.clear();
					service_client.call(service_request, service_response);

					if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {

						robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
						robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
						robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
						robot_state::JointStateGroup* joint_state_group_l =	kinematic_state_l->getJointStateGroup("left_arm_group");

						for (uint i = 0; i < 6; i++) {
							//std::cout << service_response.solution.joint_state.name[i] << ": ";
							//std::cout << service_response.solution.joint_state.position[i] << std::endl;

							joint_values_l.push_back(service_response.solution.joint_state.position[i]);
							joint_names_l.push_back(service_response.solution.joint_state.name[i]);
						}

						joint_state_group_l->setVariableValues(joint_values_l);
						joint_values_l = smartJointValues(joint_values_l);

						double tmp = getStateDistance(referencejoints,	joint_values_l);
						ROS_INFO("STATE DISTANCE: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
						if (tmp < statedistance) {

							moveit_msgs::DisplayRobotState msg_l;
							robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
							robot_state_publisher_l_.publish(msg_l);

							ROS_INFO("IK Solution L: TRUE");
							result = true;

							tmp_pose_.joint_names_l = joint_names_l;
							tmp_pose_.joint_states_l = joint_values_l;
							statedistance = tmp;
						}

						//joint_state_group_l;

					} else {
						ROS_INFO("IK Solution L: FALSE");
					}
				}
			}

			//freezes ik generation when necessary
			if (!freeze_ik_right_) {
				service_request.ik_request.group_name = "right_arm_group";
				service_request.ik_request.pose_stamped.pose = target_pose_r;
				//if(result)
				service_request.ik_request.constraints = constraints_r;
				
				uint samples;
				equaljointstates_ ? samples = 100 : samples = 1;

				std::vector<double> referencejoints;
				referencejoints = tmp_pose_.joint_states_r;
				double statedistance = 6 * 2 * PI;

				for (uint i = 0; (i < samples); i++) {
					joint_values_r.clear();
					service_client.call(service_request, service_response);

					if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {

						/* Load the robot model */
						robot_model_loader::RobotModelLoader robot_model_loader_r("robot_description");
						robot_model::RobotModelPtr kinematic_model_r = robot_model_loader_r.getModel();
						robot_state::RobotStatePtr kinematic_state_r(new robot_state::RobotState(kinematic_model_r));
						robot_state::JointStateGroup* joint_state_group_r = kinematic_state_r->getJointStateGroup("right_arm_group");

						for (uint i = 6; i < 12; i++) {
							std::cout << service_response.solution.joint_state.name[i] << ": ";
							std::cout << service_response.solution.joint_state.position[i] << std::endl;

							joint_values_r.push_back(
									service_response.solution.joint_state.position[i]);
							joint_names_r.push_back(
									service_response.solution.joint_state.name[i]);
						}

						joint_state_group_r->setVariableValues(joint_values_r);
						joint_values_r = smartJointValues(joint_values_r);
						
						double tmp = getStateDistance(referencejoints,	joint_values_r);
						ROS_INFO("STATE DISTANCE: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
						if (tmp < statedistance) {

							moveit_msgs::DisplayRobotState msg_r;
							robot_state::robotStateToRobotStateMsg(*kinematic_state_r,	msg_r.state);
							robot_state_publisher_r_.publish(msg_r);

							ROS_INFO("IK Solution R: TRUE");

							tmp_pose_.joint_names_r = joint_names_r;
							tmp_pose_.joint_states_r = joint_values_r;
							statedistance = tmp;
						}
						
					} else {
						ROS_INFO("IK Solution R: FALSE");
					}
				}
			}

			tmp_pose_ = smartJointValues(tmp_pose_);
			marker_changed_ = false;
		}
		generateIK_ = false;
	}

	//HERE
	/* simulates from start to goal state using different planning techniques
	 * option = 0: linear path planning using CartesianPath
	 * option = 1:
	 * option = 2:
	 */
	void simulation(uint option, uint iterations){
		
		if(iterations == 0) iterations = 1;
		uint success_counter = 0;	
		
		for(uint i=0; i < iterations; i++){
			
			bool feedback = false;
					
			if(option == 0)	feedback = simulateCartesianPath();
			else if(option == 1) feedback = simulateJointTarget();
			else if(option == 2) feedback = simulatePoseTarget();
			else ROS_INFO("This option is not available");//Nothing to do			
			
			if(feedback) success_counter++;
		}
		
		//evaluation
		float percentage = (float)success_counter/iterations * 100;
		ROS_INFO("The success rate was %f %% doing %u iterations", percentage, iterations);
	}
	
	bool simulatePoseTarget(){
		
		double visualizationtime = 0;
		
		move_group_interface::MoveGroup group_both("both_arms");
		moveit::planning_interface::MoveGroup::Plan two_arm_plan;
		
		//set start state to start_pose
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		robot_state::JointStateGroup* joint_state_group_r =	kinematic_state->getJointStateGroup("right_arm_group");
		robot_state::JointStateGroup* joint_state_group_l =	kinematic_state->getJointStateGroup("left_arm_group");

		joint_state_group_r->setVariableValues(start_pose_.joint_states_r);
		joint_state_group_l->setVariableValues(start_pose_.joint_states_l);
		
		group_both.setWorkspace (0, 0, 0, 5, 5, 5);
		group_both.setStartState(*kinematic_state);
		group_both.setGoalOrientationTolerance(0.01);
		group_both.setGoalPositionTolerance(0.01);
		group_both.setPlanningTime(15.0);
		
		group_both.setPoseTarget(goal_pose_.handle_r, "right_arm_ee_link");
		group_both.setPoseTarget(goal_pose_.handle_l, "left_arm_ee_link");
		
		bool ret = false;
		if (group_both.plan(two_arm_plan)){
			
			ret = true;
			//visualize target state and store in tmp_pose_
			uint trajectory_size = two_arm_plan.trajectory_.joint_trajectory.points.size();
			std::vector<double> left, right;
			for (uint i = 0;i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions.size() - 6; i++) {
				left.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions[i]);
			}
			for (uint i = 6; i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions.size(); i++) {
				right.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions[i]);
			}

			tmp_pose_.handle_r = goal_pose_.handle_r;
			tmp_pose_.handle_l = goal_pose_.handle_l;
			tmp_pose_.joint_states_r = right;
			tmp_pose_.joint_states_l = left;

			robot_state_publisher_r_.publish(DisplayRobotStateFromJointStates("right_arm_group", tmp_pose_.joint_states_r));
			robot_state_publisher_l_.publish(DisplayRobotStateFromJointStates("left_arm_group", tmp_pose_.joint_states_l));

			sleep(visualizationtime);			
		}	
		
		return ret;
	}
	
	bool simulateJointTarget() {

		double visualizationtime = 0;

		move_group_interface::MoveGroup group_r("right_arm_group");
		move_group_interface::MoveGroup group_l("left_arm_group");
		move_group_interface::MoveGroup group_both("both_arms");

		moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
		moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, two_arm_plan, mergedPlan;

		//set start state to start_pose
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		robot_state::JointStateGroup* joint_state_group_r =	kinematic_state->getJointStateGroup("right_arm_group");
		robot_state::JointStateGroup* joint_state_group_l =	kinematic_state->getJointStateGroup("left_arm_group");

		joint_state_group_r->setVariableValues(start_pose_.joint_states_r);
		joint_state_group_l->setVariableValues(start_pose_.joint_states_l);

//	  group_r.setWorkspace (0, 0, 0, 5, 5, 5);
//	  group_r.setStartState(*kinematic_state);
//	  group_r.setGoalOrientationTolerance(0.01);
//	  group_r.setPlanningTime(10.0);
//
//	  group_l.setWorkspace (0, 0, 0, 5, 5, 5);
//	  group_l.setStartState(*kinematic_state);
//	  group_l.setGoalOrientationTolerance(0.01);
//	  group_l.setPlanningTime(10.0);

		group_both.setWorkspace (0, 0, 0, 5, 5, 5);
		group_both.setStartState(*kinematic_state);
		group_both.setGoalOrientationTolerance(0.01);
		group_both.setGoalPositionTolerance(0.01);
		group_both.setPlanningTime(15.0);

		std::vector<double> joints_combined;
		for (uint i = 0; i < goal_pose_.joint_states_l.size(); i++) {
			joints_combined.push_back(goal_pose_.joint_states_l[i]);
		}
		for (uint i = 0; i < goal_pose_.joint_states_r.size(); i++) {
			joints_combined.push_back(goal_pose_.joint_states_r[i]);
		}
		group_both.setJointValueTarget(joints_combined);

		bool ret = false;
		if (group_both.plan(two_arm_plan)) {
			
			ret = true;
			//visualize target state and store in tmp_pose_
			uint trajectory_size = two_arm_plan.trajectory_.joint_trajectory.points.size();
			std::vector<double> left, right;
			for (uint i = 0;i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions.size() - 6; i++) {
				left.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions[i]);
			}
			for (uint i = 6; i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions.size(); i++) {
				right.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size - 1].positions[i]);
			}

			tmp_pose_.handle_r = goal_pose_.handle_r;
			tmp_pose_.handle_l = goal_pose_.handle_l;
			tmp_pose_.joint_states_r = right;
			tmp_pose_.joint_states_l = left;

			robot_state_publisher_r_.publish(DisplayRobotStateFromJointStates("right_arm_group", tmp_pose_.joint_states_r));
			robot_state_publisher_l_.publish(DisplayRobotStateFromJointStates("left_arm_group", tmp_pose_.joint_states_l));

			sleep(visualizationtime);
		}
		
		return ret;
	}

	//Simulates the planning with cartesian path between a start state and the goal position
	bool simulateCartesianPath() {

		double visualizationtime = 0;

		move_group_interface::MoveGroup group_r("right_arm_group");
		move_group_interface::MoveGroup group_l("left_arm_group");

		moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
		moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, mergedPlan;

		//set start state to start_pose
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
		robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
		robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
		robot_state::JointStateGroup* joint_state_group_r = kinematic_state->getJointStateGroup("right_arm_group");
		robot_state::JointStateGroup* joint_state_group_l = kinematic_state->getJointStateGroup("left_arm_group");

		joint_state_group_r->setVariableValues(start_pose_.joint_states_r);
		joint_state_group_l->setVariableValues(start_pose_.joint_states_l);

		group_r.setWorkspace(0, 0, 0, 5, 5, 5);
		group_r.setStartState(*kinematic_state);
		group_r.setGoalOrientationTolerance(0.01);
		group_r.setPlanningTime(10.0);

		group_l.setWorkspace(0, 0, 0, 5, 5, 5);
		group_l.setStartState(*kinematic_state);
		group_l.setGoalOrientationTolerance(0.01);
		group_l.setPlanningTime(10.0);

		std::vector<geometry_msgs::Pose> waypoints_r, waypoints_l;
		waypoints_r.clear();
		waypoints_l.clear();
		waypoints_r.push_back(goal_pose_.handle_r);
		waypoints_l.push_back(goal_pose_.handle_l);

		double fraction_r = 0;
		double fraction_l = 0;
		uint attempts = 50;
		//-------RIGHT-------------------------
		for (uint i = 0; fraction_r < 1.0 && i < attempts; i++) {
			fraction_r = group_r.computeCartesianPath(waypoints_r, 0.01, // eef_step
					1000.0,   // jump_threshold
					trajectory_r);
		}
		linear_plan_r.trajectory_ = trajectory_r;
		sleep(visualizationtime);

		//-------LEFT-------------------------
		for (uint i = 0; fraction_l < 1.0 && i < attempts; i++) {
			fraction_l = group_l.computeCartesianPath(waypoints_l, 0.01, // eef_step
					1000.0,   // jump_threshold
					trajectory_l);
		}
		linear_plan_l.trajectory_ = trajectory_l;
		sleep(visualizationtime);

		bool ret = false;
		if(fraction_r >= 1 && fraction_l >= 1){
			
			ret = true;
			//set joint_state to the last state in computed trajectory
			uint trajectory_size = linear_plan_r.trajectory_.joint_trajectory.points.size();
			tmp_pose_.joint_names_r = linear_plan_r.trajectory_.joint_trajectory.joint_names;
			tmp_pose_.joint_states_r = linear_plan_r.trajectory_.joint_trajectory.points[trajectory_size - 1].positions;
			joint_state_group_r->setVariableValues(tmp_pose_.joint_states_r);

			trajectory_size = linear_plan_l.trajectory_.joint_trajectory.points.size();
			tmp_pose_.joint_names_l = linear_plan_l.trajectory_.joint_trajectory.joint_names;
			tmp_pose_.joint_states_l = linear_plan_l.trajectory_.joint_trajectory.points[trajectory_size - 1].positions;
			joint_state_group_l->setVariableValues(tmp_pose_.joint_states_l);
			
			tmp_pose_.handle_r = goal_pose_.handle_r;
			tmp_pose_.handle_l = goal_pose_.handle_l;

			moveit_msgs::DisplayRobotState msg_r;
			robot_state::robotStateToRobotStateMsg(*kinematic_state, msg_r.state);
			robot_state_publisher_r_.publish(msg_r);
		}		
		
		return ret;
	}

	void processFeedback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
		/*ROS_INFO_STREAM( feedback->marker_name << " is now at "
		 << feedback->pose.position.x << ", " << feedback->pose.position.y
		 << ", " << feedback->pose.position.z );*/

		//std::cout << feedback->pose.position << std::endl;
		//std::cout << feedback->pose.orientation << std::endl;
		marker_pose_ = feedback->pose;
		createGrabPoses(marker_pose_);
		marker_changed_ = true;
	}

	void interactiveMarker() {

		// create an interactive marker server on the topic namespace simple_marker
		server_ = new interactive_markers::InteractiveMarkerServer(
				"SenekaInteractive");

		// create an interactive marker for our server
		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.frame_id = "/world_dummy_link";
		int_marker.name = "my_marker";
		int_marker.description = "Simple 1-DOF Control";
		int_marker.pose.position.x = 2;
		int_marker.scale = 1;

		// create a grey box marker
		visualization_msgs::Marker box_marker;
		box_marker.type = visualization_msgs::Marker::CUBE;
		box_marker.pose.position.x = 0;
		box_marker.pose.position.y = 0;
		box_marker.pose.position.z = 0.35;
		box_marker.scale.x = 0.34;
		box_marker.scale.y = 0.34;
		box_marker.scale.z = 0.70;
		box_marker.color.r = 0.5;
		box_marker.color.g = 0.5;
		box_marker.color.b = 0.5;
		box_marker.color.a = 1.0;

		// create a non-interactive control which contains the box
		visualization_msgs::InteractiveMarkerControl box_control;
		box_control.always_visible = false;
		box_control.markers.push_back(box_marker);

		// add the control to the interactive marker
		int_marker.controls.push_back(box_control);

		//add fixed 6-Dof
		visualization_msgs::InteractiveMarkerControl control;
		control.orientation_mode =
				visualization_msgs::InteractiveMarkerControl::FIXED;

		control.orientation.w = 1;
		control.orientation.x = 1;
		control.orientation.y = 0;
		control.orientation.z = 0;
		control.name = "rotate_x";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_x";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		control.orientation.w = 1;
		control.orientation.x = 0;
		control.orientation.y = 1;
		control.orientation.z = 0;
		control.name = "rotate_z";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_z";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		control.orientation.w = 1;
		control.orientation.x = 0;
		control.orientation.y = 0;
		control.orientation.z = 1;
		control.name = "rotate_y";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker.controls.push_back(control);
		control.name = "move_y";
		control.interaction_mode =
				visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker.controls.push_back(control);

		// add the interactive marker to our collection &
		// tell the server to call processFeedback() when feedback arrives for it
		server_->insert(int_marker,
				boost::bind(&SenekaInteractive::processFeedback, this, _1));

		// 'commit' changes and send to all clients
		server_->applyChanges();
	}

	moveit_msgs::DisplayRobotState DisplayRobotStateFromJointStates(
			const char *group, std::vector<double> joint_values) {

		robot_model_loader::RobotModelLoader robot_model_loader_l(
				"robot_description");
		robot_model::RobotModelPtr kinematic_model_l =
				robot_model_loader_l.getModel();
		robot_state::RobotStatePtr kinematic_state_l(
				new robot_state::RobotState(kinematic_model_l));
		robot_state::JointStateGroup* joint_state_group_l =
				kinematic_state_l->getJointStateGroup(group);

		joint_state_group_l->setVariableValues(joint_values);

		moveit_msgs::DisplayRobotState msg;
		robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg.state);
		//robot_state_publisher_l_.publish( msg ); 
		return msg;
	}

	//------------ Services -----------------------------------
	bool setStartState(seneka_interactive::setStartState::Request &req,
			seneka_interactive::setStartState::Response &res) 
	{
		std::string requested_name = req.name;

		for (uint i = 0; i < stored_poses.size(); i++) {
			if (!stored_poses[i].name.compare(requested_name)) {

				visualization_msgs::InteractiveMarker marker;
				node_pose pose = stored_poses[i];
				
				res.name = stored_poses[i].name;
				res.joint_names_r = stored_poses[i].joint_names_r;
				res.joint_names_l = stored_poses[i].joint_names_l;
				res.joint_states_r = stored_poses[i].joint_states_r;
				res.joint_states_l = stored_poses[i].joint_states_l;

				server_->get("my_marker", marker);
				marker.pose = stored_poses[i].pose;
				server_->insert(marker);
				server_->applyChanges();
				
				pose.handle_r = handle_r_;
				pose.handle_l = handle_l_;

				robot_state_publisher_r_.publish(
						DisplayRobotStateFromJointStates("right_arm_group",
								stored_poses[i].joint_states_r));
				robot_state_publisher_l_.publish(
						DisplayRobotStateFromJointStates("left_arm_group",
								stored_poses[i].joint_states_l));

				tmp_pose_ = pose;
				start_pose_ = pose;
				return true;
			}
		}
		return false;
	}

	bool setGoalState(seneka_interactive::setGoalState::Request &req,
			seneka_interactive::setGoalState::Response &res) 
	{
		std::string requested_name = req.name;

		for (uint i = 0; i < stored_poses.size(); i++) {
			if (!stored_poses[i].name.compare(requested_name)) {

				visualization_msgs::InteractiveMarker marker;
				node_pose pose = stored_poses[i];
				pose = stored_poses[i];
				

				res.name = stored_poses[i].name;
				res.joint_names_r = stored_poses[i].joint_names_r;
				res.joint_names_l = stored_poses[i].joint_names_l;
				res.joint_states_r = stored_poses[i].joint_states_r;
				res.joint_states_l = stored_poses[i].joint_states_l;

				server_->get("my_marker", marker);
				marker.pose = stored_poses[i].pose;
				server_->insert(marker);
				server_->applyChanges();
				
				pose.handle_r = handle_r_;
				pose.handle_l = handle_l_;

				robot_state_publisher_r_.publish(
						DisplayRobotStateFromJointStates("right_arm_group",
								stored_poses[i].joint_states_r));
				robot_state_publisher_l_.publish(
						DisplayRobotStateFromJointStates("left_arm_group",
								stored_poses[i].joint_states_l));

				tmp_pose_ = pose;
				goal_pose_ = pose;
				return true;
			}
		}

		return false;
	}

	bool setJointState(seneka_interactive::setJointState::Request &req,
			seneka_interactive::setJointState::Response &res) 
	{
		node_pose new_pose = tmp_pose_;
		new_pose.name = "tmp_pose";

		std::string group = req.group;
		uint joint = req.joint;
		double value = req.value;

		res.success = false;

		if (!group.compare("r")) {

			if (joint < new_pose.joint_states_r.size()) {
				new_pose.joint_states_r[joint] = value;
				res.jointstates = new_pose.joint_states_r;
			} else {
				return false;
			}
		} else if (!group.compare("l")) {

			if (joint < new_pose.joint_states_l.size()) {
				new_pose.joint_states_l[joint] = value;
				res.jointstates = new_pose.joint_states_l;
			} else {
				return false;
			}
		} else {
			return false;
		}

		robot_state_publisher_r_.publish(
				DisplayRobotStateFromJointStates("right_arm_group",
						new_pose.joint_states_r));
		robot_state_publisher_l_.publish(
				DisplayRobotStateFromJointStates("left_arm_group",
						new_pose.joint_states_l));

		tmp_pose_ = new_pose;
		tmp_pose_ = smartJointValues(tmp_pose_);

		res.success = true;
		return res.success;
	}

	bool getJointStates(seneka_interactive::getJointStates::Request &req,
			seneka_interactive::getJointStates::Response &res) 
	{
		node_pose new_pose = tmp_pose_;
		std::string group = req.group;
		res.success = false;

		if (!group.compare("r")) {
			res.name = "right_arm_joints";
			res.jointstates = new_pose.joint_states_r;
			res.success = true;
		} else if (!group.compare("l")) {
			res.name = "left_arm_joints";
			res.jointstates = new_pose.joint_states_l;
			res.success = true;
		} else {
			return false;
		}

		return res.success;
	}

	bool createNewState(seneka_interactive::createNewState::Request &req,
			seneka_interactive::createNewState::Response &res) {
		node_pose new_pose = tmp_pose_;
		new_pose.name = req.name;
		new_pose.pose = marker_pose_;
		new_pose.handle_r = handle_r_;
		new_pose.handle_l = handle_l_;

		res.name = new_pose.name;
		res.joint_names_r = new_pose.joint_names_r;
		res.joint_names_l = new_pose.joint_names_l;
		res.joint_states_r = new_pose.joint_states_r;
		res.joint_states_l = new_pose.joint_states_l;

		stored_poses.push_back(new_pose);

		return true;
	}

	bool getStates(seneka_interactive::getStates::Request &req,
			seneka_interactive::getStates::Response &res) {
		for (uint i = 0; i < stored_poses.size(); i++) {
			res.states.push_back(stored_poses[i].name);
		}

		return true;
	}

	bool simulate(seneka_interactive::simulate::Request &req,
			seneka_interactive::simulate::Response &res) 
	{
		simulate_.simulate = true;
		simulate_.iterations = req.iterations;
		simulate_.simulated_arm = SIMULATED_ARM_RIGHT;
		
		res.msg = "Simulating one iteration!";
		if(!req.option.compare("cartesian")){
			simulate_.option = 0;
		}
		else if(!req.option.compare("jointtarget")){
			simulate_.option = 1;
		}
		else if(!req.option.compare("posetarget")){
			simulate_.option = 2;
		}
		else{
			res.msg = "Sry this is not known. The possible options are \n [cartesian] \n [jointtarget] \n [posetarget]";
			simulate_.simulate = false;
		}
		return true;
	}

	bool toggleArmFreeze(seneka_interactive::toggleArmFreeze::Request &req,
			seneka_interactive::toggleArmFreeze::Response &res) 
	{
		if (!req.toggle.compare("right")) {
			freeze_ik_right_ = !freeze_ik_right_;
		}

		if (!req.toggle.compare("left")) {
			freeze_ik_left_ = !freeze_ik_left_;
		}

		res.right = freeze_ik_right_;
		res.left = freeze_ik_left_;

		return true;
	}
	

	bool generateIK(seneka_interactive::generateIK::Request &req,
			seneka_interactive::generateIK::Response &res) 
	{
		generateIK_ = true;
		equaljointstates_ = req.equal_joint_states;
		
		res.success = true;
		return res.success;
	}
		
		
	bool setConstraints(seneka_interactive::setConstraints::Request &req,
			seneka_interactive::setConstraints::Response &res) 
	{
		
		node_pose new_pose = tmp_pose_;
		std::string cmd = req.cmd;
		std::string group = req.group;
		double tolerance = req.tolerance;
		double position = req.position;
		res.success = false;
		
		if(!group.compare("r")){
			constraints_r = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_r, new_pose.joint_states_r, tolerance, position);
			res.success = true;
		}
		
		if(!group.compare("l")){
			constraints_l = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_l, new_pose.joint_states_l, tolerance, position);
			res.success = true;
		}	
		
		return true;
	}

	bool printStatesToFile(seneka_interactive::printStatesToFile::Request &req,
			seneka_interactive::printStatesToFile::Response &res) 
	{
		std::ofstream outf;

		std::string cmd(
				"/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_interactive/common/");
		std::string name(req.filename.c_str());
		std::string ext(".txt");
		cmd = cmd + name + ext;

		outf.open(cmd.c_str());
		//req.filename

		for (uint i = 0; i < stored_poses.size(); i++) {

			outf << "Name: " << stored_poses[i].name << "\n";
			outf << "---right---" << "\n";

			for (uint j = 0; j < stored_poses[i].joint_names_r.size(); j++)
				outf << stored_poses[i].joint_names_r[j] << ": ["
						<< stored_poses[i].joint_states_r[j] << "]" << "\n";
			outf << "---left---" << "\n";
			for (uint j = 0; j < stored_poses[i].joint_names_l.size(); j++)
				outf << stored_poses[i].joint_names_l[j] << ": ["
						<< stored_poses[i].joint_states_l[j] << "]" << "\n";

			outf << " -----Pose-----\n";
			outf << stored_poses[i].pose << "\n";

			outf << " -----handle_r-----\n";
			outf << stored_poses[i].handle_r << "\n";

			outf << " -----handle_l-----\n";
			outf << stored_poses[i].handle_l << "\n";

			outf << " -----programmatical-----\n";
			for (uint j = 0; j < stored_poses[i].joint_names_r.size(); j++)
				outf << "pose.joint_states_r.push_back" << "("
						<< stored_poses[i].joint_states_r[j] << ");" << "\n";
			for (uint j = 0; j < stored_poses[i].joint_names_l.size(); j++)
				outf << "pose.joint_states_l.push_back" << "("
						<< stored_poses[i].joint_states_l[j] << ");" << "\n";
			outf << "\n";
			for (uint j = 0; j < stored_poses[i].joint_names_r.size(); j++)
				outf << "joint_positions_r.push_back" << "("
						<< stored_poses[i].joint_states_r[j] << ");" << "\n";
			for (uint j = 0; j < stored_poses[i].joint_names_l.size(); j++)
				outf << "joint_positions_l.push_back" << "("
						<< stored_poses[i].joint_states_l[j] << ");" << "\n";

			outf << "-------------------------------------------" << "\n";
			outf << "\n\n\n\n\n\n";
		}
		outf.close();

		res.msg = "All states written to file";

		return true;
	}
	//------------ Services --------END---------------------------

	//computes the poses of the handles in reference to the marker position
	void createGrabPoses(geometry_msgs::Pose &marker_pose) {

		handle_l_ = marker_pose_;
		handle_r_ = marker_pose_;

		handle_l_.position.x += 0;
		handle_l_.position.y += 0.2125;
		handle_l_.position.z += 0.70 + gripper_length;
		handle_l_.orientation.x = 0.0095722;
		handle_l_.orientation.y = -0.0108288;
		handle_l_.orientation.z = -0.706344;
		handle_l_.orientation.w = 0.707722;

		handle_r_.position.x += 0;
		handle_r_.position.y -= 0.2125;
		handle_r_.position.z += 0.70 + gripper_length;
		handle_r_.orientation.x = 0;
		handle_r_.orientation.y = 0;
		handle_r_.orientation.z = 0.706678;
		handle_r_.orientation.w = 0.707535;
	}

	void create_stored_poses() {

		node_pose pose;

		pose.joint_names_r.push_back("right_arm_shoulder_pan_joint");
		pose.joint_names_r.push_back("right_arm_shoulder_lift_joint");
		pose.joint_names_r.push_back("right_arm_elbow_joint");
		pose.joint_names_r.push_back("right_arm_wrist_1_joint");
		pose.joint_names_r.push_back("right_arm_wrist_2_joint");
		pose.joint_names_r.push_back("right_arm_wrist_3_joint");

		pose.joint_names_l.push_back("left_arm_shoulder_pan_joint");
		pose.joint_names_l.push_back("left_arm_shoulder_lift_joint");
		pose.joint_names_l.push_back("left_arm_elbow_joint");
		pose.joint_names_l.push_back("left_arm_wrist_1_joint");
		pose.joint_names_l.push_back("left_arm_wrist_2_joint");
		pose.joint_names_l.push_back("left_arm_wrist_3_joint");

		//prepack front
		pose.name = "prepack";
		/*pose.joint_states_r.push_back(-1.10186);
		 pose.joint_states_r.push_back(-1.72479);
		 pose.joint_states_r.push_back(4.82816);
		 pose.joint_states_r.push_back(-3.10249);
		 pose.joint_states_r.push_back(-2.67068);
		 pose.joint_states_r.push_back(-3.34081);
		 
		 pose.joint_states_l.push_back(1.10002);
		 pose.joint_states_l.push_back(-1.41744);
		 pose.joint_states_l.push_back(-4.82702);
		 pose.joint_states_l.push_back(-0.0431126);
		 pose.joint_states_l.push_back(-3.61312);
		 pose.joint_states_l.push_back(3.36654);*/

		//
		pose.joint_states_r.push_back(1.55605);
		pose.joint_states_r.push_back(-1.43362);
		pose.joint_states_r.push_back(-5.15157);
		pose.joint_states_r.push_back(0.333291);
		pose.joint_states_r.push_back(-0.0127843);
		pose.joint_states_r.push_back(2.9103);

		pose.joint_states_l.push_back(-1.55801);
		pose.joint_states_l.push_back(-1.73358);
		pose.joint_states_l.push_back(-1.1049);
		pose.joint_states_l.push_back(2.67448);
		pose.joint_states_l.push_back(0.012207);
		pose.joint_states_l.push_back(3.53443);

		pose.pose.position.x = 1.38785;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.549912;
		stored_poses.push_back(pose);

		//pregrasp-rear-close
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-close";

		pose.joint_states_r.push_back(1.1);
		pose.joint_states_r.push_back(-1);
		pose.joint_states_r.push_back(2);
		pose.joint_states_r.push_back(-1.1);
		pose.joint_states_r.push_back(-0.0127962);
		pose.joint_states_r.push_back(3.05);
		pose.joint_states_l.push_back(-1.1);
		pose.joint_states_l.push_back(-2);
		pose.joint_states_l.push_back(-2);
		pose.joint_states_l.push_back(-2.2);
		pose.joint_states_l.push_back(0.012214);
		pose.joint_states_l.push_back(3.141);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559795;
		stored_poses.push_back(pose);

		//pregrasp-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear";

		pose.joint_states_r.push_back(1.3);
		pose.joint_states_r.push_back(-0.6);
		pose.joint_states_r.push_back(0.5);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-0.0127962);
		pose.joint_states_r.push_back(3.05);
		pose.joint_states_l.push_back(-1.3);
		pose.joint_states_l.push_back(-2.5);
		pose.joint_states_l.push_back(-0.5);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(0.012214);
		pose.joint_states_l.push_back(3.141);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559795;
		stored_poses.push_back(pose);

		//prepack-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "prepack-rear";

		pose.joint_states_r.push_back(1.55604);
		pose.joint_states_r.push_back(-0.348553);
		pose.joint_states_r.push_back(-1.12702);
		pose.joint_states_r.push_back(1.50556);
		pose.joint_states_r.push_back(-0.0127962);
		pose.joint_states_r.push_back(2.91043);
		pose.joint_states_l.push_back(-1.558);
		pose.joint_states_l.push_back(-2.79);
		pose.joint_states_l.push_back(1.12702);
		pose.joint_states_l.push_back(1.52883);
		pose.joint_states_l.push_back(0.012214);
		pose.joint_states_l.push_back(-2.74885);

		pose.pose.position.x = 1.38785;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559795;
		stored_poses.push_back(pose);

		//packed-rear-h1
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-h1";

		pose.joint_states_r.push_back(1.60587);
		pose.joint_states_r.push_back(-1.01243);
		pose.joint_states_r.push_back(-1.98466);
		pose.joint_states_r.push_back(2.98633);
		pose.joint_states_r.push_back(0.0370389);
		pose.joint_states_r.push_back(2.95235);
		pose.joint_states_l.push_back(-1.60649);
		pose.joint_states_l.push_back(-2.11752);
		pose.joint_states_l.push_back(1.97315);
		pose.joint_states_l.push_back(0.199017);
		pose.joint_states_l.push_back(-0.0364981);
		pose.joint_states_l.push_back(-2.96735);

		pose.pose.position.x = 0.495247;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.563951;
		stored_poses.push_back(pose);

		//packed-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear";

		pose.joint_states_r.push_back(1.58806);
		pose.joint_states_r.push_back(-1.43113);
		pose.joint_states_r.push_back(-1.80313);
		pose.joint_states_r.push_back(3.21233);
		pose.joint_states_r.push_back(0.019241);
		pose.joint_states_r.push_back(2.96233);
		pose.joint_states_l.push_back(-1.58925);
		pose.joint_states_l.push_back(-1.69424);
		pose.joint_states_l.push_back(1.79061);
		pose.joint_states_l.push_back(0.00715636);
		pose.joint_states_l.push_back(-0.0193046);
		pose.joint_states_l.push_back(-3.01626);

		pose.pose.position.x = 0.251843;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		stored_poses.push_back(pose);

		start_pose_ = pose;
	}

	//main loop
	//check Sensornode position and start planning
	void mainLoop() {

		ros::AsyncSpinner spinner(1); // Use 4 threads
		spinner.start();

		ros::Rate loop_rate(1);
		while (ros::ok()) {
			
			if(constraints_r.joint_constraints.size() > 0)
				ROS_INFO("Joint Constraints right active");
			if(constraints_l.joint_constraints.size() > 0)
				ROS_INFO("Joint Constraints left active");
				
			generateIkSolutions();
			if (simulate_.simulate) {
				simulation(simulate_.option,simulate_.iterations);
				simulate_.simulate = false;
			}
			loop_rate.sleep();
		}
	}
};

int main(int argc, char** argv) {
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "SenekaInteractive");
	ros::NodeHandle nh;

	/// Create SenekaPickAndPlace instance with mainLoop inside
	SenekaInteractive* seneka_pnp = new SenekaInteractive(nh);

	delete seneka_pnp;
	return 0;
}

/*
 //Only Here that its not lost!!! Bur i don't think this will be used again
 //  std::ofstream outf_;
 //  outf_.open("/home/matthias/data.txt"); 
 
 bool workspace_sim(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
 
 tje_lock_.lock();
 ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
 moveit_msgs::DisplayTrajectory display_trajectory;

 moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
 moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, mergedPlan;

 unsigned int used_handle_r = 2;//use the real handle id 1-6
 unsigned int used_handle_l = 5;//use the real handle id 1-6
 used_handle_r--;
 used_handle_l--;

 std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
 
 for(double x = 1.5 ; x <= 1.8 ; x = x + 0.5){
 for(double y = -0.5 ; y <= 0.5 ; y = y + 0.5){	  

 group_r->setStartStateToCurrentState();      
 group_l->setStartStateToCurrentState();
 group_both->setStartStateToCurrentState();

 waypoints_r.clear();
 waypoints_l.clear();

 setSensornodeState(x,y);

 if(!getSensornodePose()){
 return false;
 }

 geometry_msgs::Pose target_pose2_r = group_r->getCurrentPose().pose;
 geometry_msgs::Pose target_pose2_l = group_l->getCurrentPose().pose;

 target_pose2_r.position.x = handholds_[used_handle_r].entry.translation.x;
 target_pose2_r.position.y = handholds_[used_handle_r].entry.translation.y;
 target_pose2_r.position.z = handholds_[used_handle_r].entry.translation.z;
 waypoints_r.push_back(target_pose2_r);
 target_pose2_r.orientation.w = handholds_[used_handle_r].entry.rotation.w;
 target_pose2_r.orientation.x = handholds_[used_handle_r].entry.rotation.x;
 target_pose2_r.orientation.y = handholds_[used_handle_r].entry.rotation.y;
 target_pose2_r.orientation.z = handholds_[used_handle_r].entry.rotation.z;   
 waypoints_r.push_back(target_pose2_r);
 target_pose2_r.position.x = handholds_[used_handle_r].down.translation.x;
 target_pose2_r.position.y = handholds_[used_handle_r].down.translation.y;
 target_pose2_r.position.z = handholds_[used_handle_r].down.translation.z;
 waypoints_r.push_back(target_pose2_r);
 target_pose2_r.position.x = handholds_[used_handle_r].up.translation.x;
 target_pose2_r.position.y = handholds_[used_handle_r].up.translation.y;
 target_pose2_r.position.z = handholds_[used_handle_r].up.translation.z;
 waypoints_r.push_back(target_pose2_r);

 target_pose2_l.position.x = handholds_[used_handle_l].entry.translation.x;
 target_pose2_l.position.y = handholds_[used_handle_l].entry.translation.y;
 target_pose2_l.position.z = handholds_[used_handle_l].entry.translation.z;
 waypoints_l.push_back(target_pose2_l);
 target_pose2_l.orientation.w = handholds_[used_handle_l].entry.rotation.w;
 target_pose2_l.orientation.x = handholds_[used_handle_l].entry.rotation.x;
 target_pose2_l.orientation.y = handholds_[used_handle_l].entry.rotation.y;
 target_pose2_l.orientation.z = handholds_[used_handle_l].entry.rotation.z;
 waypoints_l.push_back(target_pose2_l);
 target_pose2_l.position.x = handholds_[used_handle_l].down.translation.x;
 target_pose2_l.position.y = handholds_[used_handle_l].down.translation.y;
 target_pose2_l.position.z = handholds_[used_handle_l].down.translation.z;
 waypoints_l.push_back(target_pose2_l);
 target_pose2_l.position.x = handholds_[used_handle_l].up.translation.x;
 target_pose2_l.position.y = handholds_[used_handle_l].up.translation.y;
 target_pose2_l.position.z = handholds_[used_handle_l].up.translation.z;
 waypoints_l.push_back(target_pose2_l);


 //-----
 outf_ << x << " " << y << " ";

 double fraction_r = group_r->computeCartesianPath(waypoints_r,
 0.01,  // eef_step
 1000.0,   // jump_threshold
 trajectory_r);

 double fraction_l = group_l->computeCartesianPath(waypoints_l,
 0.01,  // eef_step
 1000.0,   // jump_threshold
 trajectory_l);
 
 //group_r->setPoseTarget(target_pose2_r);
 //bool plan = group_r->plan(linear_plan_r);

 ROS_INFO("FRACTION: %f", fraction_r);
 //if(!plan){
 if(fraction_r < 1.0 || fraction_l < 1.0){
 outf_ << "x";
 } else {
 outf_ << 1;
 }
 
 //-----
 //-----
 /*bool result = false;
 moveit_msgs::GetPositionIK::Request service_request;
 moveit_msgs::GetPositionIK::Response service_response; 

 service_request.ik_request.attempts = 10;
 service_request.ik_request.pose_stamped.header.frame_id = "world_dummy_link"; 
 service_request.ik_request.avoid_collisions = true;

 //left arm
 service_request.ik_request.group_name = "left_arm_group";
 service_request.ik_request.pose_stamped.pose = target_pose2_l;    
 service_client.call(service_request, service_response);
 
 if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
 result = true;
 } 

 //right arm
 if(result){
 service_request.ik_request.group_name = "right_arm_group"; 
 service_request.ik_request.pose_stamped.pose = target_pose2_r;    
 service_client.call(service_request, service_response);

 if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
 result = true;
 } else {
 result = false;
 }
 }

 outf_ << x << " " << y << " ";

 //result
 if(result){
 outf_ << 1;
 } else {
 outf_ << "x";
 }*//*
 
 outf_  << '\n';	
 //-----

 }
 outf_ << '\n';
 }
 outf_.close();
 tje_lock_.unlock(); 
 return true;
 }
 
 bool setSensornodeState(double x, double y)
 {
 gazebo_msgs::ModelState state;
 geometry_msgs::Pose pose;
 gazebo_msgs::SetModelState::Request req;
 gazebo_msgs::SetModelState::Response resp;

 gazebo_msgs::GetModelState::Request get_req;
 gazebo_msgs::GetModelState::Response get_resp;
 

 //set pose in world_dummy_link
 pose.position.x = x;
 pose.position.y = y;
 pose.position.z = 0.01;

 state.reference_frame = "world_dummy_link";
 state.model_name = "sensornode";
 state.pose = pose;    

 req.model_state = state;
 service_gazebo.call(req,resp);   

 //get pose in world frame
 get_req.model_name = "sensornode";
 get_req.relative_entity_name = "world";
 service_gazebo_get.call(get_req,get_resp);

 pose = get_resp.pose;


 //set pose in world frame
 //use orientation in world frame (don't use "world_dummy_link")
 pose.orientation.w =  0.438058031738; 
 pose.orientation.x =  0.438036624618;
 pose.orientation.y = -0.555087759704;
 pose.orientation.z = -0.555073558504;

 state.reference_frame = "world";
 state.model_name = "sensornode";
 state.pose = pose;    

 req.model_state = state;
 service_gazebo.call(req,resp);    

 ROS_INFO("SetModelState : %u", resp.success);
 }
 
 
 
 */

