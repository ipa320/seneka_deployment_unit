#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <cstdio>
#include <iomanip> 

#include <interactive_markers/interactive_marker_server.h>

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

#include <boost/bind.hpp>


const double PI = 3.14159265359;

struct node_pose{
	std::string name;
	geometry_msgs::Pose pose;
	geometry_msgs::Pose handle_r;
	geometry_msgs::Pose handle_l;
	std::vector<std::string> joint_names_r;
	std::vector<std::string> joint_names_l;
	std::vector<double> joint_states_r;
	std::vector<double> joint_states_l;		
};

class SenekaInteractive
{
	
private:
  ros::NodeHandle node_handle_;
  
  ros::ServiceServer service_setStartState_, service_setGoalState_, service_simulate_, service_createNewState_, service_getStates_, service_printStatesToFile_, service_toggleArmFreeze_;
  ros::ServiceServer service_setJointState_, service_getJointStates_;
  
  ros::Publisher robot_state_publisher_l_, robot_state_publisher_r_;
  interactive_markers::InteractiveMarkerServer* server_;
  geometry_msgs::Pose marker_pose_;
  geometry_msgs::Pose handle_l_, handle_r_;  
  bool marker_changed_;
  double gripper_length;
  std::vector<node_pose> stored_poses; 
  
  bool simulate_;
  
  bool freeze_ik_left_, freeze_ik_right_;

  node_pose start_pose_, tmp_pose_, goal_pose_;
  
public:
  //Constructor
  SenekaInteractive(ros::NodeHandle& nh){
    node_handle_ = nh;
    init();
  }

  //Destructor
  ~SenekaInteractive(){
  }

  void init(){
	
	create_stored_poses();	
	robot_state_publisher_l_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>( "robot_state_l", 1 );
	robot_state_publisher_r_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>( "robot_state_r", 1 );
	
	service_setStartState_ = node_handle_.advertiseService("seneka_interactive/setStartState", &SenekaInteractive::setStartState, this);
	service_setGoalState_ = node_handle_.advertiseService("seneka_interactive/setGoalState", &SenekaInteractive::setGoalState, this);
	service_simulate_ = node_handle_.advertiseService("seneka_interactive/simulate", &SenekaInteractive::simulate, this);
	service_createNewState_ = node_handle_.advertiseService("seneka_interactive/createNewState", &SenekaInteractive::createNewState, this);
	service_getStates_ = node_handle_.advertiseService("seneka_interactive/getStates", &SenekaInteractive::getStates, this);
	service_printStatesToFile_ = node_handle_.advertiseService("seneka_interactive/printStatesToFile", &SenekaInteractive::printStatesToFile, this);
	service_toggleArmFreeze_ = node_handle_.advertiseService("seneka_interactive/toggleArmFreeze", &SenekaInteractive::toggleArmFreeze, this);
	service_setJointState_ = node_handle_.advertiseService("seneka_interactive/setJointState", &SenekaInteractive::setJointState, this);
	service_getJointStates_ = node_handle_.advertiseService("seneka_interactive/getJointStates", &SenekaInteractive::getJointStates, this);
	
	tmp_pose_.name = "tmp_pose";
	
	freeze_ik_right_ = false;
	freeze_ik_left_ = false;
	
    marker_changed_ = false;
    simulate_ = false;
    
    gripper_length = 0.26;
    
	interactiveMarker();
    mainLoop();
  }
  
  //transform from left arm to right arm and add a jiggle factor
  moveit_msgs::Constraints transformJointStates(std::vector<double> joints){
	  
	  /*
	  left_arm_shoulder_pan_joint
	  left_arm_shoulder_lift_joint
	  left_arm_elbow_joint
	  left_arm_wrist_1_joint
	  left_arm_wrist_2_joint
	  left_arm_wrist_3_joint
	  */
	    
	  double jiggle = PI;
	  
	  moveit_msgs::Constraints constraint;
	  
	  moveit_msgs::JointConstraint jconstraint;
	  
	  jconstraint.joint_name = "right_arm_shoulder_pan_joint";
	  jconstraint.position = 1.04915;
	  jconstraint.tolerance_above = PI/4;
	  jconstraint.tolerance_below = PI/4;
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
  
  moveit_msgs::Constraints leftArmConstraints(){
	  
	  double jiggle = PI;
	  
	  moveit_msgs::Constraints constraint;
	  
	  moveit_msgs::JointConstraint jconstraint;	  
	  jconstraint.joint_name = "left_arm_shoulder_pan_joint";
	  jconstraint.position = -1.04728;
	  jconstraint.tolerance_above = PI/4;
	  jconstraint.tolerance_below = PI/4;
	  jconstraint.weight = 1;
	  constraint.joint_constraints.push_back(jconstraint);
	  
	  jconstraint.joint_name = "left_arm_shoulder_lift_joint";
	  jconstraint.position = 1.09;
	  jconstraint.tolerance_above = 0.6;
	  jconstraint.tolerance_below = 0.6;
	  jconstraint.weight = 1;
	  //constraint.joint_constraints.push_back(jconstraint);
	  
	  return constraint;
  }
  
  void generateIkSolutions(){  
	  
	  std::vector<double> joint_values_l;
	  std::vector<double> joint_values_r;
	  
	  std::vector<std::string> joint_names_l;
	  std::vector<std::string> joint_names_r;
	  
	  if(marker_changed_){
		  
		  ROS_INFO("Generate IK Solution");  		  
		  ros::ServiceClient service_client;
		  service_client = node_handle_.serviceClient<moveit_msgs::GetPositionIK> ("compute_ik");

		  geometry_msgs::Pose target_pose_l = handle_l_;
		  geometry_msgs::Pose target_pose_r = handle_r_;

		  bool result = false;
		  moveit_msgs::GetPositionIK::Request service_request;
		  moveit_msgs::GetPositionIK::Response service_response; 

		  service_request.ik_request.attempts = 10;
		  service_request.ik_request.pose_stamped.header.frame_id = "world_dummy_link"; 
		  service_request.ik_request.avoid_collisions = false;

		  //freezes ik generation when necessary
		  if(!freeze_ik_left_){
			  //left arm
			  service_request.ik_request.group_name = "left_arm_group";
			  service_request.ik_request.pose_stamped.pose = target_pose_l;    
			  service_request.ik_request.constraints = leftArmConstraints();
			  service_client.call(service_request, service_response);

			  if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){

				  robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
				  robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
				  robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
				  robot_state::JointStateGroup* joint_state_group_l = kinematic_state_l->getJointStateGroup("left_arm_group");

				  for(uint i=0; i<6;i++){
					  std::cout << service_response.solution.joint_state.name[i] << ": ";
					  std::cout << service_response.solution.joint_state.position[i] << std::endl;			

					  joint_values_l.push_back(service_response.solution.joint_state.position[i]);  
					  joint_names_l.push_back(service_response.solution.joint_state.name[i]);
				  }

				  joint_state_group_l->setVariableValues(joint_values_l);

				  moveit_msgs::DisplayRobotState msg_l;
				  robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
				  robot_state_publisher_l_.publish( msg_l );  

				  ROS_INFO("IK Solution L: TRUE");
				  result = true;

				  tmp_pose_.joint_names_l = joint_names_l;
				  tmp_pose_.joint_states_l = joint_values_l;

			  } else {
				  ROS_INFO("IK Solution L: FALSE");		  
			  }
		  }

		  //freezes ik generation when necessary
		  if(!freeze_ik_right_){
			  service_request.ik_request.group_name = "right_arm_group"; 
			  service_request.ik_request.pose_stamped.pose = target_pose_r;    
			  //if(result)
			  service_request.ik_request.constraints = transformJointStates(joint_values_l);
			  service_client.call(service_request, service_response);		  

			  if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){

				  /* Load the robot model */
				  robot_model_loader::RobotModelLoader robot_model_loader_r("robot_description");
				  robot_model::RobotModelPtr kinematic_model_r = robot_model_loader_r.getModel();
				  robot_state::RobotStatePtr kinematic_state_r(new robot_state::RobotState(kinematic_model_r));
				  robot_state::JointStateGroup* joint_state_group_r = kinematic_state_r->getJointStateGroup("right_arm_group");

				  for(uint i=6; i<12;i++){
					  std::cout << service_response.solution.joint_state.name[i] << ": ";
					  std::cout << service_response.solution.joint_state.position[i] << std::endl;				

					  joint_values_r.push_back(service_response.solution.joint_state.position[i]);  
					  joint_names_r.push_back(service_response.solution.joint_state.name[i]);
				  }

				  joint_state_group_r->setVariableValues(joint_values_r); 

				  moveit_msgs::DisplayRobotState msg_r;
				  robot_state::robotStateToRobotStateMsg(*kinematic_state_r, msg_r.state);
				  robot_state_publisher_r_.publish( msg_r );  

				  ROS_INFO("IK Solution R: TRUE");

				  tmp_pose_.joint_names_r = joint_names_r;
				  tmp_pose_.joint_states_r = joint_values_r;

			  } else {
				  ROS_INFO("IK Solution R: FALSE");
			  }
		  }

		  smartJointValues();
		  marker_changed_ = false;		  
	  }
  }
  
  void smartJointValues(){
	  
	  for(uint i=0; i < tmp_pose_.joint_states_r.size(); i++){
		  tmp_pose_.joint_states_r[i] = createSmartJointValue(tmp_pose_.joint_states_r[i]);
	  }
	  
	  for(uint i=0; i < tmp_pose_.joint_states_l.size(); i++){
		  tmp_pose_.joint_states_l[i] = createSmartJointValue(tmp_pose_.joint_states_l[i]);
	  }
	  
  }
  
  double createSmartJointValue(double jointvalue){
	  
	  if(jointvalue > PI){
		  jointvalue -= 2*PI;		  
	  }
	  else if(jointvalue < -PI){
		  jointvalue += 2*PI;
	  }
	  else{
		  //joint value already in shape
	  }
	  
	  return jointvalue;
  }
  
  void simpolateCartesianPath(){

	  double visualizationtime = 5;

	  move_group_interface::MoveGroup group_r("right_arm_group");
	  move_group_interface::MoveGroup group_l("left_arm_group");	
	  move_group_interface::MoveGroup group_both("both_arms");	    
	  
	  moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
	  moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, two_arm_plan, mergedPlan;
 
	  //set start state to start_pose
	  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	  robot_state::JointStateGroup* joint_state_group_r = kinematic_state->getJointStateGroup("right_arm_group");
	  robot_state::JointStateGroup* joint_state_group_l = kinematic_state->getJointStateGroup("left_arm_group");

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
	  
	  //group_both.setWorkspace (0, 0, 0, 5, 5, 5);
	  group_both.setStartState(*kinematic_state);
	  //group_both.setGoalOrientationTolerance(0.01);
	  //group_both.setGoalPositionTolerance(5);
	  group_both.setPlanningTime(15.0);

//	  std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
//	  waypoints_r.clear();
//	  waypoints_l.clear();
//
//	  waypoints_r.push_back(handle_r_);
//	  waypoints_l.push_back(handle_l_);
//
//	  double fraction_r = 0;
//	  double fraction_l = 0;
//	  uint attempts = 10;    
//	  //-------RIGHT-------------------------
//	  for(uint i=0; fraction_r < 1.0 && i < attempts; i++){
//		  fraction_r = group_r.computeCartesianPath(waypoints_r,
//				  0.01,  // eef_step
//				  1000.0,   // jump_threshold
//				  trajectory_r);
//	  }
//	  linear_plan_r.trajectory_ = trajectory_r;
//	  sleep(visualizationtime);	  
	  
	  std::vector<double> joints_combined;
	  for(uint i=0; i < goal_pose_.joint_states_l.size(); i++){
		  joints_combined.push_back(goal_pose_.joint_states_l[i]);
	  }
	  for(uint i=0; i < goal_pose_.joint_states_r.size(); i++){
	  		  joints_combined.push_back(goal_pose_.joint_states_r[i]);
	  }
	  group_both.setJointValueTarget(joints_combined);
	  //group_both.setPoseTarget(handle_r_, "right_arm_ee_link");
	  //group_both.setPoseTarget(handle_l_, "left_arm_ee_link");
	  	  
	  if(group_both.plan(two_arm_plan)){
		  
		  //visualize target state and store in tmp_pose_
		  uint trajectory_size = two_arm_plan.trajectory_.joint_trajectory.points.size();	  
		  std::vector<double> left, right;
		  for(uint i = 0; i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size-1].positions.size()-6; i++){	
			  left.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size-1].positions[i]);
		  }
		  for(uint i = 6; i < two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size-1].positions.size(); i++){	
			  right.push_back(two_arm_plan.trajectory_.joint_trajectory.points[trajectory_size-1].positions[i]);
		  }		  
		  
		  tmp_pose_.joint_states_r = right;
		  tmp_pose_.joint_states_l = left;
		  
		  robot_state_publisher_r_.publish( DisplayRobotStateFromJointStates("right_arm_group", tmp_pose_.joint_states_r) );
		  robot_state_publisher_l_.publish( DisplayRobotStateFromJointStates("left_arm_group", tmp_pose_.joint_states_l) );
	  	  
		  sleep(visualizationtime);
	  }	    
  }
  
  //Simulates the planning with cartesian path between a start state and the actual handle positions of the sensorsonde
  void simulateCartesianPath(){	  

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
	    
	    joint_state_group_r->setVariableValues(start_pose_.joint_states_r); 
	    joint_state_group_l->setVariableValues(start_pose_.joint_states_l);	    

	    group_r.setWorkspace (0, 0, 0, 5, 5, 5);
	    group_r.setStartState(*kinematic_state);
	    group_r.setGoalOrientationTolerance(0.01);
	    group_r.setPlanningTime(10.0);

	    group_l.setWorkspace (0, 0, 0, 5, 5, 5);
	    group_l.setStartState(*kinematic_state);
	    group_l.setGoalOrientationTolerance(0.01);
	    group_l.setPlanningTime(10.0);
	    
	    std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
	    waypoints_r.clear();
	    waypoints_l.clear();
	    
	    waypoints_r.push_back(handle_r_);
	    waypoints_l.push_back(handle_l_);
	    
	    
	    double fraction_r = 0;
	    double fraction_l = 0;
	    uint attempts = 10;    
	    //-------RIGHT-------------------------
	    for(uint i=0; fraction_r < 1.0 && i < attempts; i++){
	      fraction_r = group_r.computeCartesianPath(waypoints_r,
							0.01,  // eef_step
							1000.0,   // jump_threshold
							trajectory_r);
	    }
	    linear_plan_r.trajectory_ = trajectory_r;
	    sleep(visualizationtime);

	    //-------LEFT-------------------------
	    for(uint i=0; fraction_l < 1.0 && i < attempts; i++){
	      fraction_l = group_l.computeCartesianPath(waypoints_l,
					   0.01,  // eef_step
					   1000.0,   // jump_threshold
					   trajectory_l);  
	    }      
	    linear_plan_l.trajectory_ = trajectory_l;
	    sleep(visualizationtime);	       
	  
	    //set joint_state to the last state in computed trajectory
	    uint trajectory_size = linear_plan_r.trajectory_.joint_trajectory.points.size();	
		tmp_pose_.joint_names_r = linear_plan_r.trajectory_.joint_trajectory.joint_names;
		tmp_pose_.joint_states_r = linear_plan_r.trajectory_.joint_trajectory.points[trajectory_size-1].positions;
		joint_state_group_r->setVariableValues(tmp_pose_.joint_states_r);   
	    
	    trajectory_size = linear_plan_l.trajectory_.joint_trajectory.points.size();	
		tmp_pose_.joint_names_l = linear_plan_l.trajectory_.joint_trajectory.joint_names;
		tmp_pose_.joint_states_l = linear_plan_l.trajectory_.joint_trajectory.points[trajectory_size-1].positions;
		joint_state_group_l->setVariableValues(tmp_pose_.joint_states_l); 
	    
	    moveit_msgs::DisplayRobotState msg_r;
	    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg_r.state);
	    robot_state_publisher_r_.publish( msg_r ); 
  }
  
  void processFeedback(
		  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    /*ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );*/
    
   //std::cout << feedback->pose.position << std::endl;
   //std::cout << feedback->pose.orientation << std::endl;
    
    marker_pose_ = feedback->pose;
    createGrabPoses(marker_pose_);
    marker_changed_ = true;
  }


  void interactiveMarker(){

	  // create an interactive marker server on the topic namespace simple_marker
	  server_ = new interactive_markers::InteractiveMarkerServer("SenekaInteractive");

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
	  box_control.markers.push_back( box_marker );	  

	  // add the control to the interactive marker
	  int_marker.controls.push_back( box_control );
	  
	  //add fixed 6-Dof
	  visualization_msgs::InteractiveMarkerControl control;
	  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
	  
	  control.orientation.w = 1;
	  control.orientation.x = 1;
	  control.orientation.y = 0;
	  control.orientation.z = 0;
	  control.name = "rotate_x";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	  int_marker.controls.push_back(control);
	  control.name = "move_x";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);

	  control.orientation.w = 1;
	  control.orientation.x = 0;
	  control.orientation.y = 1;
	  control.orientation.z = 0;
	  control.name = "rotate_z";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	  int_marker.controls.push_back(control);
	  control.name = "move_z";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);

	  control.orientation.w = 1;
	  control.orientation.x = 0;
	  control.orientation.y = 0;
	  control.orientation.z = 1;
	  control.name = "rotate_y";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	  int_marker.controls.push_back(control);
	  control.name = "move_y";
	  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	  int_marker.controls.push_back(control);	  
	 
	  // add the interactive marker to our collection &
	  // tell the server to call processFeedback() when feedback arrives for it
	  server_->insert(int_marker, boost::bind(&SenekaInteractive::processFeedback, this, _1));

	  // 'commit' changes and send to all clients
	  server_->applyChanges();
  }
  
  moveit_msgs::DisplayRobotState DisplayRobotStateFromJointStates(const char *group, std::vector<double> joint_values){
	  
	  robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
	  robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
	  robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
	  robot_state::JointStateGroup* joint_state_group_l = kinematic_state_l->getJointStateGroup(group);
	  
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
	  
	  for(uint i=0; i < stored_poses.size(); i++){
		  if(!stored_poses[i].name.compare(requested_name)){
			  
			  visualization_msgs::InteractiveMarker marker;
			  start_pose_ = stored_poses[i];
			  tmp_pose_ = stored_poses[i];
			  
			  res.name = stored_poses[i].name;
			  res.joint_names_r = stored_poses[i].joint_names_r;
			  res.joint_names_l = stored_poses[i].joint_names_l;
			  res.joint_states_r = stored_poses[i].joint_states_r;
			  res.joint_states_l = stored_poses[i].joint_states_l;			  
			  
			  server_->get("my_marker", marker);
			  marker.pose = stored_poses[i].pose;
			  server_->insert(marker);
			  server_->applyChanges();
			  
			  robot_state_publisher_r_.publish( DisplayRobotStateFromJointStates("right_arm_group", stored_poses[i].joint_states_r) );
			  robot_state_publisher_l_.publish( DisplayRobotStateFromJointStates("left_arm_group", stored_poses[i].joint_states_l) );			  
			  
			  return true;		
		  }
	  }
	  return false;
  }  
  
  bool setGoalState(seneka_interactive::setGoalState::Request &req,
		  seneka_interactive::setGoalState::Response &res)
  {
	  std::string requested_name = req.name;	  
	  
	  for(uint i=0; i < stored_poses.size(); i++){
		  if(!stored_poses[i].name.compare(requested_name)){
			  
			  visualization_msgs::InteractiveMarker marker;
			  tmp_pose_ = stored_poses[i];
			  goal_pose_ = stored_poses[i];
			  
			  res.name = stored_poses[i].name;
			  res.joint_names_r = stored_poses[i].joint_names_r;
			  res.joint_names_l = stored_poses[i].joint_names_l;
			  res.joint_states_r = stored_poses[i].joint_states_r;
			  res.joint_states_l = stored_poses[i].joint_states_l;
			  
			  server_->get("my_marker", marker);
			  marker.pose = stored_poses[i].pose;
			  server_->insert(marker);
			  server_->applyChanges();
			  
			  robot_state_publisher_r_.publish( DisplayRobotStateFromJointStates("right_arm_group", stored_poses[i].joint_states_r) );
			  robot_state_publisher_l_.publish( DisplayRobotStateFromJointStates("left_arm_group", stored_poses[i].joint_states_l) );
			  			  
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
	  
	  if(!group.compare("r")){
		  
		  if(joint < new_pose.joint_states_r.size()){
			  new_pose.joint_states_r[joint] = value;
			  res.jointstates = new_pose.joint_states_r;
		  } else {
			  return false;
		  }
	  } 
	  else if(!group.compare("l")){
		  
		  if(joint < new_pose.joint_states_l.size()){
			  new_pose.joint_states_l[joint] = value;
			  res.jointstates = new_pose.joint_states_l;
		  } else {
			  return false;
		  }
	  }
	  else {
		  return false;
	  }
	  
	  robot_state_publisher_r_.publish( DisplayRobotStateFromJointStates("right_arm_group", new_pose.joint_states_r) );
	  robot_state_publisher_l_.publish( DisplayRobotStateFromJointStates("left_arm_group", new_pose.joint_states_l) );

	  tmp_pose_ = new_pose; 
	  smartJointValues();
	  
	  res.success = true;
	  return res.success;
  }
  
  bool getJointStates(seneka_interactive::getJointStates::Request &req,
  		  seneka_interactive::getJointStates::Response &res)
  {
	  node_pose new_pose = tmp_pose_;
	  std::string group = req.group;
	  res.success = false;
	  
	  if(!group.compare("r")){
		  res.name = "right_arm_joints";
		  res.jointstates = new_pose.joint_states_r;
		  res.success = true;
	  } 
	  else if(!group.compare("l")){
		  res.name = "left_arm_joints";
		  res.jointstates = new_pose.joint_states_l;
		  res.success = true;
	  } 
	  else {
		  return false;
	  }
	  
	  return res.success;	  
  }
  
  bool createNewState(seneka_interactive::createNewState::Request &req,
		  seneka_interactive::createNewState::Response &res)
  {	  
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
		  seneka_interactive::getStates::Response &res)
  {	  
	  for(uint i = 0; i < stored_poses.size(); i++){
		  res.states.push_back(stored_poses[i].name);		  
	  }
	   
	  return true;
  }
  
  
  bool simulate(seneka_interactive::simulate::Request &req,
		  seneka_interactive::simulate::Response &res)
  {	  
	  res.msg = "Simulating one iteration!";
	  simulate_ = true;
	  return true;
  }
  
  bool toggleArmFreeze(seneka_interactive::toggleArmFreeze::Request &req,
		  seneka_interactive::toggleArmFreeze::Response &res)
  {
	  if(!req.toggle.compare("right")){
		  freeze_ik_right_ = !freeze_ik_right_;
	  }
	  
	  if(!req.toggle.compare("left")){
		  freeze_ik_left_ = !freeze_ik_left_;
	  }
	  
	  res.right = freeze_ik_right_;
	  res.left = freeze_ik_left_;
	  
	  return true;
  }
  
  bool printStatesToFile(seneka_interactive::printStatesToFile::Request &req,
		  seneka_interactive::printStatesToFile::Response &res)
  {	  
	  std::ofstream outf;
	  
	  std::string cmd("/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_interactive/common/");
	  std::string name(req.filename.c_str());
	  std::string ext(".txt");
	  cmd = cmd + name + ext; 
	  
	  outf.open(cmd.c_str()); 
	  //req.filename
	  
	  for(uint i=0; i < stored_poses.size();i++){
		  
		  outf << "Name: " << stored_poses[i].name << "\n";
		  outf << "---right---" << "\n";
		  
		  for(uint j=0; j<stored_poses[i].joint_names_r.size(); j++)
			  outf <<  stored_poses[i].joint_names_r[j] << ": [" << stored_poses[i].joint_states_r[j] << "]" << "\n";
		  outf << "---left---" << "\n";
		  for(uint j=0; j<stored_poses[i].joint_names_l.size(); j++)
			  outf <<  stored_poses[i].joint_names_l[j] << ": [" << stored_poses[i].joint_states_l[j] << "]" << "\n";
		  
		  outf << " -----Pose-----\n";	  
		  outf <<  stored_poses[i].pose << "\n";
		  
		  outf << " -----handle_r-----\n";	  
		  outf <<  stored_poses[i].handle_r << "\n";
		  
		  outf << " -----handle_l-----\n";	  
		  outf <<  stored_poses[i].handle_l << "\n";
		  
		  outf << " -----programmatical-----\n";
		  for(uint j=0; j<stored_poses[i].joint_names_r.size(); j++)
			  outf <<  "pose.joint_states_r.push_back"<< "(" << stored_poses[i].joint_states_r[j] << ");" << "\n";
		  for(uint j=0; j<stored_poses[i].joint_names_l.size(); j++)
			  outf <<  "pose.joint_states_l.push_back"<< "(" << stored_poses[i].joint_states_l[j] << ");" << "\n";
		  outf << "\n";
		  for(uint j=0; j<stored_poses[i].joint_names_r.size(); j++)
			  outf <<  "joint_positions_r.push_back"<< "(" << stored_poses[i].joint_states_r[j] << ");" << "\n";
		  for(uint j=0; j<stored_poses[i].joint_names_l.size(); j++)
			  outf <<  "joint_positions_l.push_back"<< "(" << stored_poses[i].joint_states_l[j] << ");" << "\n";
		  
		  outf << "-------------------------------------------" << "\n";
		  outf << "\n\n\n\n\n\n";			  	  
	  }	  
	  outf.close();
	  
	  res.msg = "All states written to file";
	  
	  return true;
  }
  //------------ Services --------END---------------------------

  //computes the poses of the handles in reference to the marker position
  void createGrabPoses(geometry_msgs::Pose &marker_pose){
	  
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
  
 void create_stored_poses(){
	  
	  node_pose pose;
	  pose.name = "prepack";

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
	  /*pose.joint_states_r.push_back(1.55605);
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
	  pose.pose.position.z = 0.549912;*/
	  
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
  void mainLoop(){

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start(); 

    ros::Rate loop_rate(1);
    while(ros::ok()){
    	
      generateIkSolutions();
      if(simulate_){
    	  simpolateCartesianPath();
    	  simulate_  = false;
      }
      //ROS_INFO("ALIVE");
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "SenekaInteractive");
    ros::NodeHandle nh;

    /// Create SenekaPickAndPlace instance with mainLoop inside
    SenekaInteractive* seneka_pnp = new SenekaInteractive(nh);
    
    delete seneka_pnp;
    return 0;
}




