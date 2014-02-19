/*
  seneka_pnp (pick and place)
  Author: Matthias Nösner  
*/

#include <ros/ros.h>

#include <opencv/cv.h>

#include <seneka_msgs/FiducialArray.h>
#include <seneka_msgs/Fiducial.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_state/robot_state.h>

class SenekaPickAndPlace
{

struct pose{
  double x;
  double y;
  double z;
};

struct quaternion{
  double w;
  double x;
  double y;
  double z;
};

struct pose3d{
  pose translation;
  quaternion rotation;
};

struct handhold{
  pose3d handle;
  pose3d entry;
  pose3d up;
  pose3d down;
};

private:
  ros::NodeHandle node_handle_;
  pose3d sensornode_;
  std::vector<handhold> handholds_;
  
  pose trigger_up;
  pose trigger_down;
  pose inital_grab_pose;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  tf::TransformListener listener_entry;
  tf::StampedTransform transform_entry;

  tf::TransformListener listener_up;
  tf::StampedTransform transform_up;
    
  tf::TransformListener listener_down;
  tf::StampedTransform transform_down;
  

public:
  //Constructor
  SenekaPickAndPlace(ros::NodeHandle& nh){
    node_handle_ = nh;
    init();
  }

  //Destructor
  ~SenekaPickAndPlace(){
  }

  void init(){
    
    trigger_up.x = 0;
    trigger_up.y = 0.1 ;
    trigger_up.z = 0;

    trigger_down.x = 0;
    trigger_down.y = -0.1 ;
    trigger_down.z = 0;

    inital_grab_pose.x = trigger_down.x - 0.2;
    inital_grab_pose.y = trigger_down.y;
    inital_grab_pose.z = trigger_down.z;
    
    
    mainLoop();
  }  

  bool getSensornodePose(){
    
    handholds_.clear();
    
    int ret = true;  
     
     for(unsigned int i = 1; i < 7;i++){
      
      //get all tf transformations
      char name[50];

      sprintf(name,"handle%u",i);
      try{
	listener.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
	listener.lookupTransform("/quanjo_body", name, ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ret = false;
      }

      sprintf(name,"grab_entry%u",i);
      try{
	listener_entry.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
	listener_entry.lookupTransform("/quanjo_body", name, ros::Time(0), transform_entry);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ret = false;
      }

      sprintf(name,"trigger_%u_up",i);
      try{
	listener_up.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
	listener_up.lookupTransform("/quanjo_body", name, ros::Time(0), transform_up);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ret = false;
      }

      sprintf(name,"trigger_%u_down",i);
      try{
	listener_down.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
	listener_down.lookupTransform("/quanjo_body", name, ros::Time(0), transform_down);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	ret = false;
      }

      handhold handh;
      //handle
      handh.handle.translation.x = transform.getOrigin().x();
      handh.handle.translation.y = transform.getOrigin().y();
      handh.handle.translation.z = transform.getOrigin().z();

      handh.handle.rotation.w = transform.getRotation().getW();
      handh.handle.rotation.x = transform.getRotation().getX();
      handh.handle.rotation.y = transform.getRotation().getY();
      handh.handle.rotation.z = transform.getRotation().getZ();

      //entry
      handh.entry.translation.x = transform_entry.getOrigin().x();
      handh.entry.translation.y = transform_entry.getOrigin().y();
      handh.entry.translation.z = transform_entry.getOrigin().z();

      handh.entry.rotation.w = transform_entry.getRotation().getW();
      handh.entry.rotation.x = transform_entry.getRotation().getX();
      handh.entry.rotation.y = transform_entry.getRotation().getY();
      handh.entry.rotation.z = transform_entry.getRotation().getZ();

      //up
      handh.up.translation.x = transform_up.getOrigin().x();
      handh.up.translation.y = transform_up.getOrigin().y();
      handh.up.translation.z = transform_up.getOrigin().z();

      handh.up.rotation.w = transform_up.getRotation().getW();
      handh.up.rotation.x = transform_up.getRotation().getX();
      handh.up.rotation.y = transform_up.getRotation().getY();
      handh.up.rotation.z = transform_up.getRotation().getZ();

      //down
      handh.down.translation.x = transform_down.getOrigin().x();
      handh.down.translation.y = transform_down.getOrigin().y();
      handh.down.translation.z = transform_down.getOrigin().z();

      handh.down.rotation.w = transform_down.getRotation().getW();
      handh.down.rotation.x = transform_down.getRotation().getX();
      handh.down.rotation.y = transform_down.getRotation().getY();
      handh.down.rotation.z = transform_down.getRotation().getZ();      

      handholds_.push_back(handh);
  
      if(i == 1){
	sensornode_.translation.x = transform.getOrigin().x();
	sensornode_.translation.y = transform.getOrigin().y();
	sensornode_.translation.z = transform.getOrigin().z();
  
	sensornode_.rotation.w = transform.getRotation().getW();
	sensornode_.rotation.x = transform.getRotation().getX();
	sensornode_.rotation.y = transform.getRotation().getY();
	sensornode_.rotation.z = transform.getRotation().getZ();
      }
    }
    return ret;
  }

  bool inGrabPosition(){
  
    //Check if the Sonde is in a valid grabbing position!!
    //Keep it simple and hardcoded for milestone februrary

    return true;
  }
    
  //pick and place planner
  void pnpPlanner(){
  
    std::cout << "---------------<planning>-----------------------------" << std::endl;
    unsigned int used_handle_r = 2;//real handle id 1-6
    unsigned int used_handle_l = 6;//real handle id 1-6
    used_handle_r--;
    used_handle_l--;
    
    move_group_interface::MoveGroup group_r("right_arm_group");
    move_group_interface::MoveGroup group_l("left_arm_group");
    move_group_interface::MoveGroup group("both_arms");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;;
                       
    //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    group_r.setWorkspace (0, 0, 0, 5, 5, 5);
    group_r.setStartStateToCurrentState();
    group_l.setWorkspace (0, 0, 0, 5, 5, 5);
    group_l.setStartStateToCurrentState();
    
    geometry_msgs::PoseStamped target_pose_r;
    target_pose_r.header.frame_id = "/quanjo_body";

    target_pose_r.pose.position.x = handholds_[used_handle_r].entry.translation.x;
    target_pose_r.pose.position.y = handholds_[used_handle_r].entry.translation.y;
    target_pose_r.pose.position.z = handholds_[used_handle_r].entry.translation.z;

    target_pose_r.pose.orientation.w = handholds_[used_handle_r].entry.rotation.w;
    target_pose_r.pose.orientation.x = handholds_[used_handle_r].entry.rotation.x;
    target_pose_r.pose.orientation.y = handholds_[used_handle_r].entry.rotation.y;
    target_pose_r.pose.orientation.z = handholds_[used_handle_r].entry.rotation.z;

    group_r.setPoseTarget(target_pose_r); 


    geometry_msgs::PoseStamped target_pose_l;
    target_pose_l.header.frame_id = "/quanjo_body";

    target_pose_l.pose.position.x = handholds_[used_handle_l].entry.translation.x;
    target_pose_l.pose.position.y = handholds_[used_handle_l].entry.translation.y;
    target_pose_l.pose.position.z = handholds_[used_handle_l].entry.translation.z;

    target_pose_l.pose.orientation.w = handholds_[used_handle_l].entry.rotation.w;
    target_pose_l.pose.orientation.x = handholds_[used_handle_l].entry.rotation.x;
    target_pose_l.pose.orientation.y = handholds_[used_handle_l].entry.rotation.y;
    target_pose_l.pose.orientation.z = handholds_[used_handle_l].entry.rotation.z;

    group_l.setPoseTarget(target_pose_l); 
    

    //------------------constraints------------------------
    /*moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "base_link";
    ocm.header.frame_id = "/quanjo_body";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 1;
    ocm.absolute_y_axis_tolerance = 1;
    ocm.absolute_z_axis_tolerance = 3.14;
    ocm.weight = 1.0;
  
    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(test_constraints);*/
    //----------------constraints------------------------

    group_r.setGoalOrientationTolerance(0.01);
    group_r.setPlanningTime(10.0);
    group_l.setGoalOrientationTolerance(0.01);
    group_l.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroup::Plan my_plan_r, my_plan_l;
    //my_plan.trajectory_ = trajectory;
    bool success_r = true;//group_r.plan(my_plan_r);
    bool success_l = true;//group_l.plan(my_plan_l);

    bool success = (success_r && success_l);
 
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(!success){
      std::cout << "WARNING IM SENDING THE TRAJECTORY IN 10sec" << std::endl;
      sleep(10.0);
      group_r.execute(my_plan_r);
      sleep(10.0);
      //group_l.execute(my_plan_l);
      //sleep(20.0);
    }


    //-------------------Linear Movement to through all points ------------------------
    if(success){

      group_r.setStartStateToCurrentState();      
      group_l.setStartStateToCurrentState();

      std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
      geometry_msgs::Pose target_pose2_r = group_r.getCurrentPose().pose;
      geometry_msgs::Pose target_pose2_l = group_l.getCurrentPose().pose;

      
    
      //right
      //waypoints_r.push_back(target_pose2_r);

      target_pose2_r.position.x = handholds_[used_handle_r].entry.translation.x;
      target_pose2_r.position.y = handholds_[used_handle_r].entry.translation.y;
      target_pose2_r.position.z = handholds_[used_handle_r].entry.translation.z;
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


      //left
      //waypoints_l.push_back(target_pose2_l);

      target_pose2_l.position.x = handholds_[used_handle_l].entry.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].entry.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].entry.translation.z;
      target_pose2_l.orientation.w = handholds_[used_handle_l].entry.rotation.w;
      target_pose2_l.orientation.x = handholds_[used_handle_l].entry.rotation.x;
      target_pose2_l.orientation.y = handholds_[used_handle_l].entry.rotation.y;
      target_pose2_l.orientation.z = handholds_[used_handle_l].entry.rotation.z;
      waypoints_l.push_back(target_pose2_l);

      std::cout << target_pose2_l << std::endl;

      target_pose2_l.position.x = handholds_[used_handle_l].down.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].down.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].down.translation.z;
      waypoints_l.push_back(target_pose2_l);

      std::cout << target_pose2_l << std::endl;

      target_pose2_l.position.x = handholds_[used_handle_l].up.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].up.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].up.translation.z;
      waypoints_l.push_back(target_pose2_l);

      std::cout << target_pose2_l << std::endl;


      moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
      moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l;
      double fraction_r = group_r.computeCartesianPath(waypoints_r,
						   0.01,  // eef_step
    						   0.0,   // jump_threshold
						   trajectory_r);  
      linear_plan_r.trajectory_ = trajectory_r;
      std::cout << "WARNING IM SENDING THE TRAJECTORY IN RRRRRRRRRRRRRRRR 10sec" << std::endl;
      sleep(10.0);
      //group_r.asyncExecute(linear_plan_r);
      sleep(10.0);


     double fraction_l = group_l.computeCartesianPath(waypoints_l,
						   0.01,  // eef_step
    						   0.0,   // jump_threshold
						   trajectory_l);  
      linear_plan_l.trajectory_ = trajectory_l;
      std::cout << "WARNING IM SENDING THE TRAJECTORY IN LLLLLLLLLLLLLLLL 10sec" << std::endl;
      sleep(10.0);
      // group_l.asyncExecute(linear_plan_l);
      sleep(10.0);
      

    }
    
    std::cout << "---------------</planning>----------------------------" << std::endl;
  }  

  //main loop
  //check Sensornode position and start planning
  void mainLoop(){
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Rate loop_rate(10);
    while(ros::ok()){
    
      bool valid_detection =  getSensornodePose();
	
      if(inGrabPosition() && valid_detection){
	ROS_INFO("IN POSITION");
	pnpPlanner();
      } else {
	ROS_INFO("WRONG POSITION");
      }
      //ros::spinOnce();
    }
  }
};


int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "SenekaPickAndPlace");
    ros::NodeHandle nh;

    /// Create SenekaPickAndPlace instance with mainLoop inside
    SenekaPickAndPlace* seneka_pnp = new SenekaPickAndPlace(nh);
    
    delete seneka_pnp;
    return 0;
}
