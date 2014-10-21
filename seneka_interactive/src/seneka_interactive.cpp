#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <iomanip> 

#include <interactive_markers/interactive_marker_server.h>
#include <seneka_interactive_tools.h>
#include <seneka_pnp_tools.h>

// Moveit msgs
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit/move_group_interface/move_group.h>
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


#include <tf/transform_datatypes.h>

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
#include "seneka_interactive/evaluateIK.h"

#include <seneka_sensornode_detection/setSensornodePose.h>

#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

const double PI = 3.14159265359;

class SenekaInteractive {

private:
	ros::NodeHandle node_handle_;

	ros::ServiceServer service_setStartState_, service_setGoalState_,
			service_simulate_, service_createNewState_, service_getStates_,
			service_printStatesToFile_, service_toggleArmFreeze_;
	ros::ServiceServer service_setJointState_, service_getJointStates_, service_setConstraints_,
			service_generateIK_, service_evaluateIK_;
	
	ros::Publisher vis_pub_;

	ros::Publisher robot_state_publisher_l_, robot_state_publisher_r_;
	interactive_markers::InteractiveMarkerServer* server_;
	geometry_msgs::Pose marker_pose_;
	geometry_msgs::Pose handle_l_, handle_r_;
	moveit_msgs::Constraints constraints_l_, constraints_r_;
	bool marker_changed_;
	double gripper_length, gripper_depth;
	std::vector<node_pose> stored_poses;

	//bool simulate_;
	bool generateIK_;
	bool equaljointstates_;
	simulation_order simulate_;
	
	std::vector< std::vector<double> > solutions_from_ik_iteration_r_;
	std::vector< std::vector<double> > solutions_from_ik_iteration_l_;
	uint ik_solutions_iterator_r_;
	uint ik_solutions_iterator_l_;
	
	bool freeze_ik_left_, freeze_ik_right_;

	node_pose start_pose_, tmp_pose_, goal_pose_;
	boost::mutex lock;

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
		service_evaluateIK_ = node_handle_.advertiseService(
				"seneka_interactive/evaluateIK",
				&SenekaInteractive::evaluateIK, this);
		
		vis_pub_ = node_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

		tmp_pose_.name = "tmp_pose";

		freeze_ik_right_ = false;
		freeze_ik_left_ = false;

		marker_changed_ = false;
		simulate_.simulate = false;
		simulate_.option = 0;
		generateIK_ = false;
		equaljointstates_ = false;

		//in m
		gripper_length = 0.26;
		gripper_depth = 0.00;// for rear position use 0.01
		
		ik_solutions_iterator_r_ = 0;
		ik_solutions_iterator_l_ = 0;
		
		interactiveMarker();
		mainLoop();
	}

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

	node_pose generateIkSolutions(node_pose start_pose, node_pose goal_pose, bool equaljointstates, bool freezeikleft, bool freezeikright ) {

		solutions_from_ik_iteration_r_.clear();
		solutions_from_ik_iteration_l_.clear();
		
		std::vector<double> joint_values_l;
		std::vector<double> joint_values_r;

		std::vector < std::string > joint_names_l;
		std::vector < std::string > joint_names_r;

		ROS_INFO("Generate IK Solution");
		ros::ServiceClient service_client;
		service_client = node_handle_.serviceClient < moveit_msgs::GetPositionIK > ("compute_ik");

		geometry_msgs::Pose target_pose_l = goal_pose.handle_l;
		geometry_msgs::Pose target_pose_r = goal_pose.handle_r;

		bool result = false;
		moveit_msgs::GetPositionIK::Request service_request;
		moveit_msgs::GetPositionIK::Response service_response;

		service_request.ik_request.timeout = ros::Duration(0.01);
		service_request.ik_request.attempts = 10;
		service_request.ik_request.pose_stamped.header.frame_id = "world_dummy_link";
		service_request.ik_request.avoid_collisions = false;
		
		node_pose node;
		//node = smartJointValues(start_pose);
		node = start_pose;
		
		//freezes ik generation when necessary
		if (!freezeikleft) {
			//left arm
			service_request.ik_request.group_name = "left_arm_group";
			service_request.ik_request.pose_stamped.pose = target_pose_l;
			service_request.ik_request.constraints = constraints_l_;
			service_request.ik_request.robot_state.joint_state.name = node.joint_names_l;
			service_request.ik_request.robot_state.joint_state.position = node.joint_states_l;


			uint samples;
			equaljointstates ? samples = 1500 : samples = 1;

			std::vector<double> referencejoints;
			referencejoints = node.joint_states_l;
			double statedistance = 6 * 2 * PI;

			robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
			robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
			robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
			robot_state::JointStateGroup* joint_state_group_l =	kinematic_state_l->getJointStateGroup("left_arm_group");
			
			for (uint i = 0; (i < samples); i++) {
				joint_values_l.clear();
				joint_names_l.clear();
				service_client.call(service_request, service_response);

				if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {

					for (uint i = 0; i < 6; i++) {
						//std::cout << service_response.solution.joint_state.name[i] << ": ";
						//std::cout << service_response.solution.joint_state.position[i] << std::endl;

						joint_values_l.push_back(service_response.solution.joint_state.position[i]);
						joint_names_l.push_back(service_response.solution.joint_state.name[i]);
					}

					joint_state_group_l->setVariableValues(joint_values_l);
					//joint_values_l = smartJointValues(joint_values_l);

					solutions_from_ik_iteration_l_.push_back(joint_values_l);
					double tmp = getStateDistance(referencejoints,	joint_values_l);
					ROS_INFO("STATE DISTANCE L: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
					if (tmp < statedistance) {

						moveit_msgs::DisplayRobotState msg_l;
						robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
						robot_state_publisher_l_.publish(msg_l);

						ROS_INFO("IK Solution L: TRUE");
						result = true;

						node.joint_names_l = joint_names_l;
						node.joint_states_l = joint_values_l;
						statedistance = tmp;
					}

					//joint_state_group_l;

				} else {
					ROS_INFO("IK Solution L: FALSE");
				}
			}
		}

		//freezes ik generation when necessary
		if (!freezeikright) {
			service_request.ik_request.group_name = "right_arm_group";
			service_request.ik_request.pose_stamped.pose = target_pose_r;
			service_request.ik_request.robot_state.joint_state.name = node.joint_names_r;
			service_request.ik_request.robot_state.joint_state.position = node.joint_states_r;
			//if(result)
			service_request.ik_request.constraints = constraints_r_;

			uint samples;
			equaljointstates ? samples = 1500 : samples = 1;

			std::vector<double> referencejoints;
			referencejoints = node.joint_states_r;
			double statedistance = 6 * 2 * PI;

			/* Load the robot model */
			robot_model_loader::RobotModelLoader robot_model_loader_r("robot_description");
			robot_model::RobotModelPtr kinematic_model_r = robot_model_loader_r.getModel();
			robot_state::RobotStatePtr kinematic_state_r(new robot_state::RobotState(kinematic_model_r));
			robot_state::JointStateGroup* joint_state_group_r = kinematic_state_r->getJointStateGroup("right_arm_group");
			
			for (uint i = 0; (i < samples); i++) {
				joint_names_r.clear();
				joint_values_r.clear();
				service_client.call(service_request, service_response);

				if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {

					for (uint i = 6; i < 12; i++) {
						
						joint_values_r.push_back(
								service_response.solution.joint_state.position[i]);
						joint_names_r.push_back(
								service_response.solution.joint_state.name[i]);
					}

					joint_state_group_r->setVariableValues(joint_values_r);
					//joint_values_r = smartJointValues(joint_values_r);

					solutions_from_ik_iteration_r_.push_back(joint_values_r);
					double tmp = getStateDistance(referencejoints,	joint_values_r);
					ROS_INFO("STATE DISTANCE R: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
					if (tmp < statedistance) {

						moveit_msgs::DisplayRobotState msg_r;
						robot_state::robotStateToRobotStateMsg(*kinematic_state_r,	msg_r.state);
						robot_state_publisher_r_.publish(msg_r);

						ROS_INFO("IK Solution R: TRUE");

						node.joint_names_r = joint_names_r;
						node.joint_states_r = joint_values_r;
						statedistance = tmp;
					}

				} else {
					ROS_INFO("IK Solution R: FALSE");
				}
			}
		}

		tmp_pose_ = node;
		//tmp_pose_ = smartJointValues(tmp_pose_);
	
		return node;
	}
		
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
					
			if(option == SIMULATE_CARTESIAN_PATH)	feedback = simulateCartesianPath(start_pose_, goal_pose_);
			else if(option == SIMULATE_JOINT_TARGET) feedback = simulateJointTarget(start_pose_, goal_pose_);
			else if(option == SIMULATE_POSE_TARGET) feedback = simulatePoseTarget();
			else if(option == SIMULATE_FROM_REALWORLD) feedback = simulateFromRealWorld(start_pose_);
			else if(option == SIMULATE_PICKUP_PROCESS) feedback = simulatePickupProcess(start_pose_);
			else if(option == BENCHMARK) feedback = mybenchmark();
			else ROS_INFO("This option is not available");//Nothing to do
			
			if(feedback) success_counter++;
		}
		
		//evaluation
		float percentage = (float)success_counter/iterations * 100;
		ROS_INFO("The success rate was %f %% doing %u iterations", percentage, iterations);
	}
	
	bool setSensornodePose(double x, double y, double z, double roll, double pitch, double yaw){
		
		tf::Quaternion qt;
		qt = tf::createQuaternionFromRPY(0+roll,PI/6+pitch,0+yaw);
		
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		
		tf::quaternionTFToMsg(qt, pose.orientation);
//		pose.orientation.x = 0;
//		pose.orientation.y = 0;
//		pose.orientation.z = 0;
//		pose.orientation.w = 1;
		
		ros::ServiceClient service_client;
		service_client = node_handle_.serviceClient < seneka_sensornode_detection::setSensornodePose > ("/sensornode_detection/setSensornodePose");
		
		seneka_sensornode_detection::setSensornodePose::Request service_request;
		seneka_sensornode_detection::setSensornodePose::Response service_response;
		
		service_request.pose = pose;
		
		service_client.call(service_request, service_response);
		
		if(!service_response.success)
			return false;
		
		return true;
	}
	
	bool mybenchmark(){
		
		ROS_INFO("BENCHMARK");
		
		unsigned int trys = 0;
		unsigned int success = 0;
		
		for(int i=145; i <= 160; i = i + 5){
			for(int j=-20; j <= 20; j = j + 10){
				
				double ii = (double) i /100;
				double jj = (double) j /100;
				
				ROS_INFO("ii: %f    jj: %f \n", ii, jj);
				trys++;
				lock.lock();
				if(!setSensornodePose(ii, jj, 0.0, 0.0, 0.0, 0.0)){
					lock.unlock();									
					continue;
				}
				lock.unlock();
				
				setStartState("pregrasp-rear");
				if(simulatePickupProcess(start_pose_))
					success++;
			}
		}
		
		ROS_INFO("Trys: %d \n", trys);
		ROS_INFO("Success: %d \n", success);
		
		return true;
	}
	
	//HERE
	//simulates from pregrasp to pickedup
	bool simulatePickupProcess(node_pose start_pose){
		
		ROS_INFO("PICKUP");
		node_pose tmp_pose = start_pose;
		
		unsigned int used_handle_r = 2;//use the real handle id 1-6
		unsigned int used_handle_l = 5;//use the real handle id 1-6
		used_handle_r--;
		used_handle_l--;
		
//		//set Sensornode pose
//		lock.lock();//---
//		if(!setSensornodePose(1.6, 0.0, 0.0, 0.0, 0.0, 0.0))
//			return false;
//		lock.unlock();//---
				
		lock.lock();
		sensornode rwnode = seneka_pnp_tools::getSensornodePose();
		lock.unlock();
		
		node_pose dual_arm_pose;
		
		if(rwnode.success){

			dual_arm_pose.pose = rwnode.pose;
			dual_arm_pose.joint_names_r = start_pose.joint_names_r;
			dual_arm_pose.joint_names_l = start_pose.joint_names_l;
			dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].entry;
			dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].entry;			
			
			if(simulateCartesianPath(start_pose, dual_arm_pose)){
				
				tmp_pose = tmp_pose_;
				dual_arm_pose.joint_names_r = start_pose.joint_names_r;
				dual_arm_pose.joint_names_l = start_pose.joint_names_l;
				dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].down;
				dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].down;	

				if(simulateCartesianPath(tmp_pose, dual_arm_pose)){

					tmp_pose = tmp_pose_;
					dual_arm_pose.joint_states_r = tmp_pose.joint_states_r;
					dual_arm_pose.joint_states_l = tmp_pose.joint_states_l;		
					dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].up;
					dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].up;
					
					if(simulateCartesianPath(tmp_pose, dual_arm_pose)){
						tmp_pose = tmp_pose_;
						setGoalState("prepack-rear");					
						if(simulateJointTarget(tmp_pose, goal_pose_)){

							return true;					
						}
					}				
				}
			}
		}
		
		return false;
	}
	
//	bool createSafeTrajectory(node_pose start_pose,  node_pose goal_pose){
//		
//		
//	}
	
	//simulates from pregrasp to pickedup
	bool simulateFromRealWorld(node_pose start_pose){
		
		unsigned int used_handle_r = 2;//use the real handle id 1-6
		unsigned int used_handle_l = 5;//use the real handle id 1-6
		used_handle_r--;
		used_handle_l--;
		
		lock.lock();
		if(!setSensornodePose(1.6, 0.1, 0.0, 0.0, 0.0, 0.0))
			return false;
		lock.unlock();
		
		lock.lock();
		sensornode rwnode = seneka_pnp_tools::getSensornodePose();
		lock.unlock();
		
		if(rwnode.success){
			
			node_pose dual_arm_pose;
			dual_arm_pose.pose = rwnode.pose;
			dual_arm_pose.joint_names_r = start_pose.joint_names_r;
			dual_arm_pose.joint_names_r = start_pose.joint_names_l;
			dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].entry;
			dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].entry;
			
			//generate ik from handhold position
			lock.lock();
			node_pose tmp_pose = generateIkSolutions(start_pose, dual_arm_pose, true, freeze_ik_left_, freeze_ik_right_);
			lock.unlock();
			dual_arm_pose.joint_states_r = tmp_pose.joint_states_r;
			dual_arm_pose.joint_states_l = tmp_pose.joint_states_l;
			
			//move to down pose with jointtarget
			if(simulateJointTarget(start_pose, dual_arm_pose)){
		
				start_pose = tmp_pose_;
				dual_arm_pose.joint_names_r = start_pose.joint_names_r;
				dual_arm_pose.joint_names_r = start_pose.joint_names_l;
				dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].down;
				dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].down;
				
				lock.lock();
				tmp_pose = generateIkSolutions(tmp_pose, dual_arm_pose, true, freeze_ik_left_, freeze_ik_right_);
				lock.unlock();		
				
				dual_arm_pose.joint_states_r = tmp_pose.joint_states_r;
				dual_arm_pose.joint_states_l = tmp_pose.joint_states_l;
			
				if(simulateJointTarget(start_pose, dual_arm_pose)){

					start_pose = tmp_pose_;
					dual_arm_pose.joint_names_r = start_pose.joint_names_r;
					dual_arm_pose.joint_names_r = start_pose.joint_names_l;
					dual_arm_pose.handle_r = rwnode.handholds[used_handle_r].up;
					dual_arm_pose.handle_l = rwnode.handholds[used_handle_l].up;

					lock.lock();
					tmp_pose = generateIkSolutions(tmp_pose, dual_arm_pose, true, freeze_ik_left_, freeze_ik_right_);
					lock.unlock();			
					
					dual_arm_pose.joint_states_r = tmp_pose.joint_states_r;
					dual_arm_pose.joint_states_l = tmp_pose.joint_states_l;

					if(simulateJointTarget(start_pose, dual_arm_pose)){
						
						start_pose = tmp_pose_;
						setGoalState("prepack-rear");
						if(simulateJointTarget(start_pose, goal_pose_)){
							return true;
						}
					}
				}
			}		
		}		
		return false;
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
		group_both.setPlannerId("RRTConnectkConfigDefault");
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
	
	bool simulateJointTarget(node_pose start_pose, node_pose goal_pose) {

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

		joint_state_group_r->setVariableValues(start_pose.joint_states_r);
		joint_state_group_l->setVariableValues(start_pose.joint_states_l);

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
		group_both.setPlannerId("RRTConnectkConfigDefault");
		group_both.setPlanningTime(15.0);

		std::vector<double> joints_combined;
		for (uint i = 0; i < goal_pose.joint_states_l.size(); i++) {
			joints_combined.push_back(goal_pose.joint_states_l[i]);
		}
		for (uint i = 0; i < goal_pose.joint_states_r.size(); i++) {
			joints_combined.push_back(goal_pose.joint_states_r[i]);
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
	bool simulateCartesianPath(node_pose start_pose, node_pose goal_pose) {

		double visualizationtime = 5;

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

		joint_state_group_r->setVariableValues(start_pose.joint_states_r);
		joint_state_group_l->setVariableValues(start_pose.joint_states_l);

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
		waypoints_r.push_back(goal_pose.handle_r);
		waypoints_l.push_back(goal_pose.handle_l);

		double fraction_r = 0;
		double fraction_l = 0;
		uint attempts = 50;
		//-------RIGHT-------------------------
		for (uint i = 0; fraction_r < 1.0 && i < attempts; i++) {
			fraction_r = group_r.computeCartesianPath(waypoints_r, 0.01, // eef_step
					10.0,   // jump_threshold
					trajectory_r);
		}
		linear_plan_r.trajectory_ = trajectory_r;
		sleep(visualizationtime);

		//-------LEFT-------------------------
		for (uint i = 0; fraction_l < 1.0 && i < attempts; i++) {
			fraction_l = group_l.computeCartesianPath(waypoints_l, 0.01, // eef_step
					10.0,   // jump_threshold
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

	void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
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
		
		
		server_->applyChanges();
			
		handle_marker_r();
		handle_marker_l();
	}
	
	void handle_marker_r(){
		
		// create an interactive marker for our server_
		  visualization_msgs::InteractiveMarker int_marker;
		  int_marker.header.frame_id = "/world_dummy_link";
		  int_marker.name = "right_handle";
		  int_marker.description = "the right handle";

		  // create a grey box marker
		  visualization_msgs::Marker box_marker;
		  box_marker.type = visualization_msgs::Marker::CUBE;
		  box_marker.scale.x = 0.05;
		  box_marker.scale.y = 0.4;
		  box_marker.scale.z = 0.05;
		  box_marker.color.r = 0.0;
		  box_marker.color.g = 1;
		  box_marker.color.b = 0.0;
		  box_marker.color.a = 1.0;

		  // create a non-interactive control which contains the box
		  visualization_msgs::InteractiveMarkerControl box_control;
		  box_control.always_visible = false;
		  box_control.markers.push_back( box_marker );

		  // add the control to the interactive marker
		  int_marker.controls.push_back( box_control );

		  // create a control which will move the box
		  // this control does not contain any markers,
		  // which will cause RViz to insert two arrows
		  visualization_msgs::InteractiveMarkerControl rotate_control;
		  rotate_control.name = "move_x";
		  rotate_control.interaction_mode =
		      visualization_msgs::InteractiveMarkerControl::NONE;


		  // add the control to the interactive marker
		  int_marker.controls.push_back(rotate_control);

		  // add the interactive marker to our collection &
		  // tell the server_ to call processFeedback() when feedback arrives for it
		  server_->insert(int_marker);

		  // 'commit' changes and send to all clients
		  server_->applyChanges();
	}
	
	void handle_marker_l(){
		
		// create an interactive marker for our server_
		  visualization_msgs::InteractiveMarker int_marker;
		  int_marker.header.frame_id = "/world_dummy_link";
		  int_marker.name = "left_handle";
		  int_marker.description = "the left handle";

		  // create a grey box marker
		  visualization_msgs::Marker box_marker;
		  box_marker.type = visualization_msgs::Marker::CUBE;
		  box_marker.scale.x = 0.05;
		  box_marker.scale.y = 0.4;
		  box_marker.scale.z = 0.05;
		  box_marker.color.r = 0.0;
		  box_marker.color.g = 1;
		  box_marker.color.b = 0.0;
		  box_marker.color.a = 1.0;

		  // create a non-interactive control which contains the box
		  visualization_msgs::InteractiveMarkerControl box_control;
		  box_control.always_visible = false;
		  box_control.markers.push_back( box_marker );

		  // add the control to the interactive marker
		  int_marker.controls.push_back( box_control );

		  // create a control which will move the box
		  // this control does not contain any markers,
		  // which will cause RViz to insert two arrows
		  visualization_msgs::InteractiveMarkerControl rotate_control;
		  rotate_control.name = "move_x";
		  rotate_control.interaction_mode =
		      visualization_msgs::InteractiveMarkerControl::NONE;


		  // add the control to the interactive marker
		  int_marker.controls.push_back(rotate_control);

		  // add the interactive marker to our collection &
		  // tell the server_ to call processFeedback() when feedback arrives for it
		  server_->insert(int_marker);

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
	
	bool setStartState(std::string name){
		
		std::string requested_name = name;
				
		for (uint i = 0; i < stored_poses.size(); i++) {
			if (!stored_poses[i].name.compare(requested_name)) {

				visualization_msgs::InteractiveMarker marker;
				node_pose pose = stored_poses[i];
				
				server_->get("my_marker", marker);
				marker.pose = pose.pose;
				server_->insert(marker);
				server_->applyChanges();
				marker_pose_ = marker.pose;
				lock.lock();
				createGrabPoses(marker.pose);
				lock.unlock();
				
				pose.handle_r = handle_r_;
				pose.handle_l = handle_l_;
				robot_state_publisher_r_.publish(
						DisplayRobotStateFromJointStates("right_arm_group",
								pose.joint_states_r));
				robot_state_publisher_l_.publish(
						DisplayRobotStateFromJointStates("left_arm_group",
								pose.joint_states_l));

				tmp_pose_ = pose;
				start_pose_ = pose;
				return true;
			}
		}
		return false;
	}
	
	bool setGoalState(std::string name){
		
		std::string requested_name = name;

		for (uint i = 0; i < stored_poses.size(); i++) {
			if (!stored_poses[i].name.compare(requested_name)) {

				visualization_msgs::InteractiveMarker marker;
				node_pose pose = stored_poses[i];
				pose = stored_poses[i];	

				server_->get("my_marker", marker);
				marker.pose = pose.pose;
				server_->insert(marker);
				server_->applyChanges();
				marker_pose_ = marker.pose;
				lock.lock();
				createGrabPoses(marker.pose);
				lock.unlock();
				
				pose.handle_r = handle_r_;
				pose.handle_l = handle_l_;
				robot_state_publisher_r_.publish(
						DisplayRobotStateFromJointStates("right_arm_group",
								pose.joint_states_r));
				robot_state_publisher_l_.publish(
						DisplayRobotStateFromJointStates("left_arm_group",
								pose.joint_states_l));

				tmp_pose_ = pose;
				goal_pose_ = pose;
				return true;
			}
		}
	}

	//------------ Services -----------------------------------
	bool setStartState(seneka_interactive::setStartState::Request &req,
			seneka_interactive::setStartState::Response &res) 
	{		
		if(setStartState(req.name)){
			res.name = start_pose_.name;
			res.joint_names_r = start_pose_.joint_names_r;
			res.joint_names_l = start_pose_.joint_names_l;
			res.joint_states_r = start_pose_.joint_states_r;
			res.joint_states_l = start_pose_.joint_states_l;
			return true;
		}
		
		return false;
	}
	
	bool setGoalState(seneka_interactive::setGoalState::Request &req,
			seneka_interactive::setGoalState::Response &res) 
	{
		if(setGoalState(req.name)){
			res.name = start_pose_.name;
			res.joint_names_r = goal_pose_.joint_names_r;
			res.joint_names_l = goal_pose_.joint_names_l;
			res.joint_states_r = goal_pose_.joint_states_r;
			res.joint_states_l = goal_pose_.joint_states_l;
			return true;
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
		//tmp_pose_ = smartJointValues(tmp_pose_);

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
		//simulate_.simulated_arm = SIMULATED_ARM_RIGHT;
		
		res.msg = "Simulating one iteration!";
		if(!req.option.compare("cartesian")){
			simulate_.option = SIMULATE_CARTESIAN_PATH;
		}
		else if(!req.option.compare("jointtarget")){
			simulate_.option = SIMULATE_JOINT_TARGET;
		}
		else if(!req.option.compare("posetarget")){
			simulate_.option = SIMULATE_POSE_TARGET;
		}
		else if(!req.option.compare("realworld")){
			simulate_.option = SIMULATE_FROM_REALWORLD;
		}
		else if(!req.option.compare("pickup")){
			simulate_.option = SIMULATE_PICKUP_PROCESS;
		}
		else if(!req.option.compare("benchmark")){
			simulate_.option = BENCHMARK;
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
	
	//HEREEVA
	bool evaluateIK(seneka_interactive::evaluateIK::Request &req,
			seneka_interactive::evaluateIK::Response &res) 
	{
		res.distance = -1;
		
		robot_model_loader::RobotModelLoader robot_model_loader_r("robot_description");
		robot_model::RobotModelPtr kinematic_model_r = robot_model_loader_r.getModel();
		robot_state::RobotStatePtr kinematic_state_r(new robot_state::RobotState(kinematic_model_r));
		robot_state::JointStateGroup* joint_state_group_r =	kinematic_state_r->getJointStateGroup("right_arm_group");
		
		robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
		robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
		robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
		robot_state::JointStateGroup* joint_state_group_l =	kinematic_state_l->getJointStateGroup("left_arm_group");
		
		if(!req.cmd.compare("next")){
			
			if(!req.arm.compare("r")){
				
				ik_solutions_iterator_r_++;

				if(solutions_from_ik_iteration_r_.size() > 0){
					if( !(ik_solutions_iterator_r_ < solutions_from_ik_iteration_r_.size()) )
						ik_solutions_iterator_r_ = 0;

					joint_state_group_r->setVariableValues(solutions_from_ik_iteration_r_[ik_solutions_iterator_r_]);

					moveit_msgs::DisplayRobotState msg_r;
					robot_state::robotStateToRobotStateMsg(*kinematic_state_r, msg_r.state);
					robot_state_publisher_r_.publish(msg_r);
					
					res.distance = getStateDistance(start_pose_.joint_states_r, solutions_from_ik_iteration_r_[ik_solutions_iterator_r_]);
				}


			}
			
			if(!req.arm.compare("l")){
				
				ik_solutions_iterator_l_++;

				if(solutions_from_ik_iteration_l_.size() > 0){
					if( !(ik_solutions_iterator_l_ < solutions_from_ik_iteration_l_.size()) )
						ik_solutions_iterator_l_ = 0;

					joint_state_group_l->setVariableValues(solutions_from_ik_iteration_l_[ik_solutions_iterator_l_]);

					moveit_msgs::DisplayRobotState msg_l;
					robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
					robot_state_publisher_l_.publish(msg_l);
					
					res.distance = getStateDistance(start_pose_.joint_states_l, solutions_from_ik_iteration_r_[ik_solutions_iterator_r_]);
				}


			}
		}
		
		if(!req.cmd.compare("store")){
			
			node_pose pose;
			pose.pose = marker_pose_;
			pose.handle_r = handle_r_;
			pose.handle_l = handle_l_;
			pose.joint_states_r = solutions_from_ik_iteration_r_[ik_solutions_iterator_r_];
			pose.joint_states_l = solutions_from_ik_iteration_l_[ik_solutions_iterator_l_];
			pose.joint_names_r = tmp_pose_.joint_names_r;
			pose.joint_names_l = tmp_pose_.joint_names_l;
			tmp_pose_  = pose;
			
			//visualize
			joint_state_group_r->setVariableValues(pose.joint_states_r);
			moveit_msgs::DisplayRobotState msg_r;
			robot_state::robotStateToRobotStateMsg(*kinematic_state_r, msg_r.state);
			robot_state_publisher_r_.publish(msg_r);
			
			joint_state_group_l->setVariableValues(pose.joint_states_l);
			moveit_msgs::DisplayRobotState msg_l;
			robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
			robot_state_publisher_l_.publish(msg_l);
		}
		
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
			constraints_r_ = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_r, new_pose.joint_states_r, tolerance, position);
			res.success = true;
		}
		
		if(!group.compare("l")){
			constraints_l_ = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_l, new_pose.joint_states_l, tolerance, position);
			res.success = true;
		}	
		
		if(!group.compare("b")){
			constraints_r_ = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_r, new_pose.joint_states_r, tolerance, position);
			constraints_l_ = seneka_interactive_tools::generateIKConstraints(cmd.c_str(), new_pose.joint_names_l, new_pose.joint_states_l, tolerance, position);
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
			outf << "\n";
			outf << "states.push_back(createArmState(\"" << stored_poses[i].name << "\", ";
			for (uint j = 0; j < stored_poses[i].joint_names_r.size(); j++)
							outf << stored_poses[i].joint_states_r[j] << ", ";
			for (uint j = 0; j < stored_poses[i].joint_names_l.size()-1; j++)
							outf << stored_poses[i].joint_states_l[j] << ", ";
			outf << stored_poses[i].joint_states_l.back() << "));\n";
			

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
		
		//create rotation matrix from quaternion
		tf::Quaternion qt(marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z, marker_pose.orientation.w);
		tf::Matrix3x3 m(qt);
						
		//create handle pose in marker coordinate system
		tf::Vector3 handle_offset_r(0, - (0.215 + gripper_depth), 0.70 + gripper_length);
		tf::Vector3 handle_offset_l(0, 0.215 + gripper_depth, 0.70 + gripper_length);
				
		//multiply with rotation matrix
		std::cout << handle_offset_r.y() << std::endl;
		handle_offset_r = m * handle_offset_r;
		handle_offset_l = m * handle_offset_l;
		std::cout << handle_offset_r.y() << std::endl;
		
		//type conversion tf::Vector3 -> geometry_msgs::Vector3
		geometry_msgs::Vector3 vec3_r, vec3_l;
 		vector3TFToMsg(handle_offset_r, vec3_r);
 		vector3TFToMsg(handle_offset_l, vec3_l);
		
 		//add handle pose in reference system of marker
 		handle_r_ = marker_pose;	
		handle_l_ = marker_pose;
	
		handle_r_.position.x += vec3_r.x;
 		handle_r_.position.y += vec3_r.y;
 		handle_r_.position.z += vec3_r.z;
 		
 		handle_l_.position.x += vec3_l.x;
 		handle_l_.position.y += vec3_l.y;
 		handle_l_.position.z += vec3_l.z;
 
 		
 		
 		
 		
 		
 		
 		//------- DEV --------
 		tf::Quaternion qt_boxmarker; 
 		tf::Quaternion qt_l90, qt_r90; 	
 		
 		//---right handle---
 		//fixed rotation	
 		qt_r90.setRPY(0,0,PI/2);
 		//box_marker to tf::Quaternion
 		tf::quaternionMsgToTF(marker_pose.orientation, qt_boxmarker); 		
 		//multiply
 		qt_boxmarker *= qt_r90; 		
 		//back trnsform
 		tf::quaternionTFToMsg(qt_boxmarker, handle_r_.orientation);
 		
 		
 		//---left handle---
 		//fixed rotation	
 		qt_l90.setRPY(0,0,-PI/2); 		
 		//box_marker to tf::Quaternion	
 		tf::quaternionMsgToTF(marker_pose.orientation, qt_boxmarker); 		
 		//multiply
 		qt_boxmarker *= qt_l90; 		
 		//back trnsform
 		tf::quaternionTFToMsg(qt_boxmarker, handle_l_.orientation);
 		//------- DEV --------

 		//working stuff
// 		handle_l_.position.x += 0;
// 		handle_l_.position.y += 0.215 + gripper_depth;
// 		handle_l_.position.z += 0.70 + gripper_length;
// 		handle_l_.orientation.x = 0.0095722;
// 		handle_l_.orientation.y = -0.0108288;
// 		handle_l_.orientation.z = -0.706344;
// 		handle_l_.orientation.w = 0.707722;
// 		handle_l_.orientation = marker_pose.orientation;

//		handle_r_.position.x += 0;
//		handle_r_.position.y -= 0.215 + gripper_depth;
//		handle_r_.position.z += 0.70 + gripper_length;
//		handle_r_.orientation.x = 0;
//		handle_r_.orientation.y = 0;
//		handle_r_.orientation.z = 0.706678;
//		handle_r_.orientation.w = 0.707535;
//		handle_r_.orientation = marker_pose.orientation;
		
		visualization_msgs::InteractiveMarker marker;
		server_->get("right_handle", marker);
		marker.pose = handle_r_;
		server_->insert(marker);
		server_->applyChanges();
	
		server_->get("left_handle", marker);
		marker.pose = handle_l_;
		server_->insert(marker);
		server_->applyChanges();
		
		tmp_pose_.pose = marker_pose;
		tmp_pose_.handle_r = handle_r_;
		tmp_pose_.handle_l = handle_l_;
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
		pose.joint_states_l.push_back(-3.141);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559795;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);

		//pregrasp-rear-new
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-new";

		pose.joint_states_r.push_back(1.3);
		pose.joint_states_r.push_back(-1.2);
		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-0.0127962);
		pose.joint_states_r.push_back(0);
		pose.joint_states_l.push_back(-1.3);
		pose.joint_states_l.push_back(-2);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0.012214);
		pose.joint_states_l.push_back(0);

		pose.pose.position.x = 4;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);

		//prepack-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "prepack-rear";

		pose.joint_states_r.push_back(1.56475);
		pose.joint_states_r.push_back(-0.37355);
		pose.joint_states_r.push_back(-0.934844);
		pose.joint_states_r.push_back(1.33843);
		pose.joint_states_r.push_back(-0.00526746);
		pose.joint_states_r.push_back(2.91156);
		pose.joint_states_l.push_back(-1.56666);
		pose.joint_states_l.push_back(-2.77179);
		pose.joint_states_l.push_back(0.952993);
		pose.joint_states_l.push_back(1.85097);
		pose.joint_states_l.push_back(0.00491897);
		pose.joint_states_l.push_back(-2.97376);


		pose.pose.position.x = 1.38785;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559795;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);

		//packed-rear-h1
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-h1";

		pose.joint_states_r.push_back(1.59366);
		pose.joint_states_r.push_back(-0.861927);
		pose.joint_states_r.push_back(-2.03566);
		pose.joint_states_r.push_back(2.89088);
		pose.joint_states_r.push_back(0.0236453);
		pose.joint_states_r.push_back(2.94829);
		pose.joint_states_l.push_back(-1.59398);
		pose.joint_states_l.push_back(-2.28216);
		pose.joint_states_l.push_back(2.03645);
		pose.joint_states_l.push_back(0.238648);
		pose.joint_states_l.push_back(-0.0224017);
		pose.joint_states_l.push_back(-2.93453);

		pose.pose.position.x = 0.495247;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.563951;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		

		//packed-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear";

		pose.joint_states_r.push_back(1.58117);
		pose.joint_states_r.push_back(-1.17341);
		pose.joint_states_r.push_back(-1.97047);
		pose.joint_states_r.push_back(3.1297);
		pose.joint_states_r.push_back(0.0111507);
		pose.joint_states_r.push_back(2.95578);
		pose.joint_states_l.push_back(-1.58218);
		pose.joint_states_l.push_back(-1.97401);
		pose.joint_states_l.push_back(1.97396);
		pose.joint_states_l.push_back(-0.0148622);
		pose.joint_states_l.push_back(-0.0106062);
		pose.joint_states_l.push_back(-2.92669);

		pose.pose.position.x = 0.315229;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-rear-drop
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-drop";
		
		pose.joint_states_r.push_back(1.58224);
		pose.joint_states_r.push_back(-1.15317);
		pose.joint_states_r.push_back(-2.25563);
		pose.joint_states_r.push_back(3.39586);
		pose.joint_states_r.push_back(0.0122261);
		pose.joint_states_r.push_back(2.95453);
		pose.joint_states_l.push_back(-1.58336);
		pose.joint_states_l.push_back(-1.99447);
		pose.joint_states_l.push_back(2.25903);
		pose.joint_states_l.push_back(-0.27799);
		pose.joint_states_l.push_back(-0.0117867);
		pose.joint_states_l.push_back(-2.92817);

		pose.pose.position.x = 0.315229;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.401951;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);		
		
		
		
		//home
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "home";

		pose.joint_states_r.push_back(-1.5705);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-2.5);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5705);
		pose.joint_states_l.push_back(-3.141);
		pose.joint_states_l.push_back(2.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//pregrasp-rear-h1
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-h1";

		pose.joint_states_r.push_back(-1.5705);
		pose.joint_states_r.push_back(-1.6);
		pose.joint_states_r.push_back(-2.5);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5705);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(2.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//pregrasp-rear-h2
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-h2";

		pose.joint_states_r.push_back(-1.5705);
		pose.joint_states_r.push_back(-1.6);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5705);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//pregrasp-rear-h3
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-h3";

		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(-1.6);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		
		//pregrasp-rear-h4
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-h4";

		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(-1.6);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//pregrasp-rear-h5
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-rear-h5";

		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(-1.0);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(-1.5);
		pose.joint_states_l.push_back(-2.1);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.559808;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//pregrasp
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp";

		pose.joint_states_r.push_back(-1.9705);
		pose.joint_states_r.push_back(-2.441);
		pose.joint_states_r.push_back(-0.8);
		pose.joint_states_r.push_back(3.2);
		pose.joint_states_r.push_back(3.241);//3.241
		pose.joint_states_r.push_back(-3.3);
		pose.joint_states_l.push_back(1.9705);
		pose.joint_states_l.push_back(-0.7);
		pose.joint_states_l.push_back(0.8);
		pose.joint_states_l.push_back(0.0);
		pose.joint_states_l.push_back(3.041);
		pose.joint_states_l.push_back(3.3);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.663502;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//pregrasp
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-h1";

		pose.joint_states_r.push_back(-1.5705);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-2.3);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5705);
		pose.joint_states_l.push_back(-3.141);
		pose.joint_states_l.push_back(2.3);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);
		

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.663502;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//pregrasp-jointflip
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pregrasp-jointflip";

		pose.joint_states_r.push_back(-1.9705);
		pose.joint_states_r.push_back(-2.441);
		pose.joint_states_r.push_back(-0.8);
		pose.joint_states_r.push_back(3.2);
		pose.joint_states_r.push_back(-3.041);//3.241
		pose.joint_states_r.push_back(-3.3);
		pose.joint_states_l.push_back(1.9705);
		pose.joint_states_l.push_back(-0.7);
		pose.joint_states_l.push_back(0.8);
		pose.joint_states_l.push_back(0.0);
		pose.joint_states_l.push_back(3.041);
		pose.joint_states_l.push_back(3.3);

		pose.pose.position.x = 3;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.663502;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//prepack front
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "prepack";
		
		pose.joint_states_r.push_back(-1.12281);
		pose.joint_states_r.push_back(-1.86896);
		pose.joint_states_r.push_back(-0.884882);
		pose.joint_states_r.push_back(2.75421);
		pose.joint_states_r.push_back(-2.69283);
		pose.joint_states_r.push_back(-3.34126);
		pose.joint_states_l.push_back(1.12093);
		pose.joint_states_l.push_back(-1.27239);
		pose.joint_states_l.push_back(0.884651);
		pose.joint_states_l.push_back(0.388099);
		pose.joint_states_l.push_back(2.6925);
		pose.joint_states_l.push_back(3.34191);

		pose.pose.position.x = 1.38785;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.585605;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-front
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-front";
		
		pose.joint_states_r.push_back(-0.312103);
		pose.joint_states_r.push_back(-1.06522);
		pose.joint_states_r.push_back(-1.6799);
		pose.joint_states_r.push_back(2.74525);
		pose.joint_states_r.push_back(-1.8825);
		pose.joint_states_r.push_back(-3.34157);
		pose.joint_states_l.push_back(0.31022);
		pose.joint_states_l.push_back(-2.07608);
		pose.joint_states_l.push_back(1.67987);
		pose.joint_states_l.push_back(0.396405);
		pose.joint_states_l.push_back(1.88218);
		pose.joint_states_l.push_back(3.34167);

		pose.pose.position.x = 0.905;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.6376;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-front-drop
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-front-drop";
		
		pose.joint_states_r.push_back(-0.485031);
		pose.joint_states_r.push_back(-1.0706);
		pose.joint_states_r.push_back(-2.09996);
		pose.joint_states_r.push_back(3.17073);
		pose.joint_states_r.push_back(-2.055);
		pose.joint_states_r.push_back(-3.34151);
		pose.joint_states_l.push_back(0.483189);
		pose.joint_states_l.push_back(-2.07072);
		pose.joint_states_l.push_back(2.09992);
		pose.joint_states_l.push_back(-0.0290206);
		pose.joint_states_l.push_back(2.05472);
		pose.joint_states_l.push_back(3.34168);

	
		pose.pose.position.x = 0.905;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.396825;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-front-tidy-1
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-front-tidy-1";
		
		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.754735);
		pose.joint_states_r.push_back(-2.22018);
		pose.joint_states_r.push_back(2.97532);
		pose.joint_states_r.push_back(-1.41303);
		pose.joint_states_r.push_back(-2.9114);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.387);
		pose.joint_states_l.push_back(2.22061);
		pose.joint_states_l.push_back(0.164381);
		pose.joint_states_l.push_back(1.41261);
		pose.joint_states_l.push_back(2.911);
		
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-front-tidy-2
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-front-tidy-2";
		
		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.754735);
		pose.joint_states_r.push_back(-1.8);
		pose.joint_states_r.push_back(2.97532);
		pose.joint_states_r.push_back(-1.41303);
		pose.joint_states_r.push_back(-2.9114);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.387);
		pose.joint_states_l.push_back(1.8);
		pose.joint_states_l.push_back(0.164381);
		pose.joint_states_l.push_back(1.41261);
		pose.joint_states_l.push_back(2.911);
		
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-front-tidy-3
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-front-tidy-3";		

		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.754735);
		pose.joint_states_r.push_back(-1.8);
		pose.joint_states_r.push_back(2.97532);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-2.9114);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.387);
		pose.joint_states_l.push_back(1.8);
		pose.joint_states_l.push_back(0.164381);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(2.911);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//pre-deploy-front
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "pre-deploy-front";	
		
		pose.joint_states_r.push_back(-1.19165);
		pose.joint_states_r.push_back(-2.03979);
		pose.joint_states_r.push_back(-0.95167);
		pose.joint_states_r.push_back(2.99189);
		pose.joint_states_r.push_back(-2.76167);
		pose.joint_states_r.push_back(-3.34119);
		pose.joint_states_l.push_back(1.18987);
		pose.joint_states_l.push_back(-1.10155);
		pose.joint_states_l.push_back(0.951376);
		pose.joint_states_l.push_back(0.150599);
		pose.joint_states_l.push_back(2.76145);
		pose.joint_states_l.push_back(3.34199);

		pose.pose.position.x = 1.4842;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.392748;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//deploy-front-legs-down
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "deploy-front-legs-down";	
		
		pose.joint_states_r.push_back(-1.20739);
		pose.joint_states_r.push_back(-2.20594);
		pose.joint_states_r.push_back(-1.24661);
		pose.joint_states_r.push_back(3.45341);
		pose.joint_states_r.push_back(-2.7774);
		pose.joint_states_r.push_back(-3.34077);
		pose.joint_states_l.push_back(1.20572);
		pose.joint_states_l.push_back(-0.935421);
		pose.joint_states_l.push_back(1.24632);
		pose.joint_states_l.push_back(-0.310875);
		pose.joint_states_l.push_back(2.7773);
		pose.joint_states_l.push_back(3.3416);

		pose.pose.position.x = 1.4842;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.0724;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);

		//deploy-front
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "deploy-front";	
		
		pose.joint_states_r.push_back(-1.21741);
		pose.joint_states_r.push_back(-2.3318);
		pose.joint_states_r.push_back(-1.27237);
		pose.joint_states_r.push_back(3.60509);
		pose.joint_states_r.push_back(-2.78742);
		pose.joint_states_r.push_back(-3.34071);
		pose.joint_states_l.push_back(1.21578);
		pose.joint_states_l.push_back(-0.809573);
		pose.joint_states_l.push_back(1.27207);
		pose.joint_states_l.push_back(-0.462503);
		pose.joint_states_l.push_back(2.78736);
		pose.joint_states_l.push_back(3.34157);

		pose.pose.position.x = 1.4842;
		pose.pose.position.y = 0;
		pose.pose.position.z = -0.06;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-rear-tidy1
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-1";	

		pose.joint_states_r.push_back(1.75);
		pose.joint_states_r.push_back(-1.15317);
		pose.joint_states_r.push_back(-2.25563);
		pose.joint_states_r.push_back(3.39267);
		pose.joint_states_r.push_back(0.0122261);
		pose.joint_states_r.push_back(2.95453);
		pose.joint_states_l.push_back(-1.75);
		pose.joint_states_l.push_back(-1.99447);
		pose.joint_states_l.push_back(2.25903);
		pose.joint_states_l.push_back(-0.27799);
		pose.joint_states_l.push_back(-0.0117867);
		pose.joint_states_l.push_back(-2.92817);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//packed-rear-tidy2
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-2";	

		pose.joint_states_r.push_back(1.75);
		pose.joint_states_r.push_back(-1.15317);
		pose.joint_states_r.push_back(-2.25563);
		pose.joint_states_r.push_back(3.39267);
		pose.joint_states_r.push_back(-1.25);
		pose.joint_states_r.push_back(2.95453);
		pose.joint_states_l.push_back(-1.75);
		pose.joint_states_l.push_back(-1.99447);
		pose.joint_states_l.push_back(2.25903);
		pose.joint_states_l.push_back(-0.27799);
		pose.joint_states_l.push_back(1.2);
		pose.joint_states_l.push_back(-2.92817);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//packed-rear-tidy3
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-3";	

		pose.joint_states_r.push_back(1.75);
		pose.joint_states_r.push_back(-0.47);
		pose.joint_states_r.push_back(-2.25563);
		pose.joint_states_r.push_back(3.39267);
		pose.joint_states_r.push_back(-1.25);
		pose.joint_states_r.push_back(4);
		pose.joint_states_l.push_back(-1.75);
		pose.joint_states_l.push_back(-2.7);
		pose.joint_states_l.push_back(2.25903);
		pose.joint_states_l.push_back(-0.27799);
		pose.joint_states_l.push_back(1.2);
		pose.joint_states_l.push_back(-4);


		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//packed-rear-tidy4
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-4";	
		
		pose.joint_states_r.push_back(1.75);
		pose.joint_states_r.push_back(-1.45);
		pose.joint_states_r.push_back(-0.5);
		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(-1.25);
		pose.joint_states_r.push_back(4);
		pose.joint_states_l.push_back(-1.75);
		pose.joint_states_l.push_back(-1.7);
		pose.joint_states_l.push_back(0.5);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(1.2);
		pose.joint_states_l.push_back(-4);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//packed-rear-tidy5
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-5";	

		pose.joint_states_r.push_back(-1.75);
		pose.joint_states_r.push_back(-1.45);
		pose.joint_states_r.push_back(-0.5);
		pose.joint_states_r.push_back(1.5);
		pose.joint_states_r.push_back(-1.25);
		pose.joint_states_r.push_back(2);
		pose.joint_states_l.push_back(1.75);
		pose.joint_states_l.push_back(-1.7);
		pose.joint_states_l.push_back(0.5);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(1.2);
		pose.joint_states_l.push_back(-2);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		
		//packed-rear-tidy6
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-6";

		
		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.3);
		pose.joint_states_r.push_back(-2);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(2);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.8);
		pose.joint_states_l.push_back(2);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(-2);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);		
		
		
		//packed-rear-tidy7
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-7";

		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.3);
		pose.joint_states_r.push_back(-2);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(0);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.8);
		pose.joint_states_l.push_back(2);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);
		
		//packed-rear-tidy7
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "packed-rear-tidy-8";

		pose.joint_states_r.push_back(-1.5);
		pose.joint_states_r.push_back(-0.3);
		pose.joint_states_r.push_back(-2);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(3.141);
		pose.joint_states_r.push_back(-1.7);
		pose.joint_states_l.push_back(1.5);
		pose.joint_states_l.push_back(-2.8);
		pose.joint_states_l.push_back(2);
		pose.joint_states_l.push_back(0);
		pose.joint_states_l.push_back(3.141);
		pose.joint_states_l.push_back(1.7);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);		
		
		//deploy-rear
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "deploy-rear";

		pose.joint_states_r.push_back(1.56588);
		pose.joint_states_r.push_back(-0.99264);
		pose.joint_states_r.push_back(1.1422);
		pose.joint_states_r.push_back(-0.111279);
		pose.joint_states_r.push_back(-0.00414274);
		pose.joint_states_r.push_back(2.90332);
		pose.joint_states_l.push_back(-1.56759);
		pose.joint_states_l.push_back(-2.1331);
		pose.joint_states_l.push_back(-1.16742);
		pose.joint_states_l.push_back(3.3402);
		pose.joint_states_l.push_back(0.00398801);
		pose.joint_states_l.push_back(-2.98127);

		pose.pose.position.x = 1.49593;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0.18204;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);	
		
		
		//deploy-rear-drop
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "deploy-rear-drop";

		pose.joint_states_r.push_back(1.56623);
		pose.joint_states_r.push_back(-0.673796);
		pose.joint_states_r.push_back(1.21241);
		pose.joint_states_r.push_back(-0.496834);
		pose.joint_states_r.push_back(-0.00379266);
		pose.joint_states_r.push_back(2.89981);
		pose.joint_states_l.push_back(-1.56782);
		pose.joint_states_l.push_back(-2.45347);
		pose.joint_states_l.push_back(-1.24077);
		pose.joint_states_l.push_back(3.73626);
		pose.joint_states_l.push_back(0.00375942);
		pose.joint_states_l.push_back(-2.98361);

		pose.pose.position.x = 1.49593;
		pose.pose.position.y = 0;
		pose.pose.position.z = -0.1638;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);	
		
		
		//deploy-rear-drop-free
		pose.joint_states_r.clear();
		pose.joint_states_l.clear();
		pose.name = "deploy-rear-drop-free";

		pose.joint_states_r.push_back(1.4);
		pose.joint_states_r.push_back(-0.673796);
		pose.joint_states_r.push_back(1.21241);
		pose.joint_states_r.push_back(-0.496834);
		pose.joint_states_r.push_back(-0.00379266);
		pose.joint_states_r.push_back(2.89981);
		pose.joint_states_l.push_back(-1.4);
		pose.joint_states_l.push_back(-2.45347);
		pose.joint_states_l.push_back(-1.24077);
		pose.joint_states_l.push_back(3.73626);
		pose.joint_states_l.push_back(0.00375942);
		pose.joint_states_l.push_back(-2.98361);

		pose.pose.position.x = 1.49593;
		pose.pose.position.y = 0;
		pose.pose.position.z = -0.1638;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		stored_poses.push_back(pose);	

		start_pose_ = pose;
	}

	//main loop
	//check Sensornode position and start planning
	void mainLoop() {

		ros::AsyncSpinner spinner(2); // Use 4 threads
		spinner.start();

		ros::Rate loop_rate(1);
		while (ros::ok()) {
						
			if(constraints_r_.joint_constraints.size() > 0)
				ROS_INFO("Joint Constraints right active");
			if(constraints_l_.joint_constraints.size() > 0)
				ROS_INFO("Joint Constraints left active");
			
			if(generateIK_){
				node_pose target_pose = tmp_pose_;//goal_pose_
				tmp_pose_ = generateIkSolutions(start_pose_, target_pose, equaljointstates_,freeze_ik_left_,freeze_ik_right_);
				generateIK_ = false;
			}
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

