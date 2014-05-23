#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

// Moveit msgs
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <boost/bind.hpp>

const double PI = 3.14159265359;

class SenekaInteractive
{
	
private:
  ros::NodeHandle node_handle_;
  
  ros::Publisher robot_state_publisher_l_, robot_state_publisher_r_;
  interactive_markers::InteractiveMarkerServer* server_;
  geometry_msgs::Pose marker_pose_;
  geometry_msgs::Pose handle_l_, handle_r_;  
  bool marker_changed_;
  double gripper_length;

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
	  
	robot_state_publisher_l_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>( "robot_state_l", 1 );
	robot_state_publisher_r_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>( "robot_state_r", 1 );
    marker_changed_ = false;
    
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
	    
	  double jiggle = PI/4;
	  
	  moveit_msgs::Constraints constraint;
	  
	  moveit_msgs::JointConstraint jconstraint;
	  
	  jconstraint.joint_name = "right_arm_shoulder_pan_joint";
	  jconstraint.position = -PI/2;
	  jconstraint.tolerance_above = jiggle;
	  jconstraint.tolerance_below = jiggle;
	  jconstraint.weight = 1;
	  constraint.joint_constraints.push_back(jconstraint);
	  
	  jconstraint.joint_name = "right_arm_shoulder_lift_joint";
	  jconstraint.position = -PI/2;
	  jconstraint.tolerance_above = jiggle;
	  jconstraint.tolerance_below = jiggle;
	  jconstraint.weight = 1;
	  constraint.joint_constraints.push_back(jconstraint);
	  
	  return constraint;
  }
  
  moveit_msgs::Constraints leftArmConstraints(){
	  
	  double jiggle = PI/4;
	  
	  moveit_msgs::Constraints constraint;
	  
	  moveit_msgs::JointConstraint jconstraint;	  
	  jconstraint.joint_name = "left_arm_shoulder_pan_joint";
	  jconstraint.position = PI/2;
	  jconstraint.tolerance_above = jiggle;
	  jconstraint.tolerance_below = jiggle;
	  jconstraint.weight = 1;
	  constraint.joint_constraints.push_back(jconstraint);
	  
	  jconstraint.joint_name = "left_arm_shoulder_lift_joint";
	  jconstraint.position = -PI/2;
	  jconstraint.tolerance_above = jiggle;
	  jconstraint.tolerance_below = jiggle;
	  jconstraint.weight = 1;
	  constraint.joint_constraints.push_back(jconstraint);
	  
	  return constraint;
  }
  
  void createGrabPoses(geometry_msgs::Pose &marker_pose){
	  
	  handle_l_ = marker_pose_;
	  handle_r_ = marker_pose_;	
	  
	  handle_l_.position.x += 0;
	  handle_l_.position.y += 0.2125;
	  handle_l_.position.z += 0.51 + gripper_length;
	  handle_l_.orientation.x = 0.0095722;
	  handle_l_.orientation.y = -0.0108288;
	  handle_l_.orientation.z = -0.706344;
	  handle_l_.orientation.w = 0.707722;		
	  
	  handle_r_.position.x += 0;
	  handle_r_.position.y -= 0.2125;
	  handle_r_.position.z += 0.51 + gripper_length;
	  handle_r_.orientation.x = 0;
	  handle_r_.orientation.y = 0;
	  handle_r_.orientation.z = 0.706678;
	  handle_r_.orientation.w = 0.707535;	
 }
  
  void generateIkSolutions(){  
	  
	  std::vector<double> joint_values_l;
	  std::vector<double> joint_values_r;
	  
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
			  }
			  
			  for(uint i=0; i<6;i++){
					  joint_values_l.push_back(service_response.solution.joint_state.position[i]);  
			  }
			  
			  joint_state_group_l->setVariableValues(joint_values_l);
			  
			  moveit_msgs::DisplayRobotState msg_l;
			  robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
			  robot_state_publisher_l_.publish( msg_l );  
			  
			  ROS_INFO("IK Solution L: TRUE");
			  result = true;

		  } else {
			  ROS_INFO("IK Solution L: FALSE");		  
		  }


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
			  }
			  
			  for(uint i=6; i<12;i++){
				  joint_values_r.push_back(service_response.solution.joint_state.position[i]);  
			  }

			  joint_state_group_r->setVariableValues(joint_values_r);

			  moveit_msgs::DisplayRobotState msg_r;
			  robot_state::robotStateToRobotStateMsg(*kinematic_state_r, msg_r.state);
			  robot_state_publisher_r_.publish( msg_r );  

			  ROS_INFO("IK Solution R: TRUE");

		  } else {
			  ROS_INFO("IK Solution R: FALSE");
		  }

		  marker_changed_ = false;		  
	  }
  }
  
  void processFeedback(
		  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    /*ROS_INFO_STREAM( feedback->marker_name << " is now at "
        << feedback->pose.position.x << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z );
    
    std::cout << feedback->pose.orientation << std::endl;*/
    
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
	  int_marker.scale = 0.5;

	  // create a grey box marker
	  visualization_msgs::Marker box_marker;
	  box_marker.type = visualization_msgs::Marker::CUBE;
	  box_marker.scale.x = 0.2;
	  box_marker.scale.y = 0.2;
	  box_marker.scale.z = 0.2;
	  box_marker.color.r = 0.5;
	  box_marker.color.g = 0.5;
	  box_marker.color.b = 0.5;
	  box_marker.color.a = 1.0;
	  
	  
	  // create a non-interactive control which contains the box
	  visualization_msgs::InteractiveMarkerControl box_control;
	  box_control.always_visible = true;
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


  //main loop
  //check Sensornode position and start planning
  void mainLoop(){

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start(); 

    ros::Rate loop_rate(1);
    while(ros::ok()){
    	
      generateIkSolutions();	
      ROS_INFO("ALIVE");
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




