/*
  seneka_pnp (pick and place)
  Author: Matthias Nösner  
*/
#include <ros/ros.h>

#include <opencv/cv.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <SerializeIO.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_state/robot_state.h>

#include "seneka_pnp/getState.h"
#include "seneka_pnp/setTransition.h"
#include "seneka_pnp/setStop.h"

#include "actionlib_msgs/GoalStatusArray.h"
#include "control_msgs/FollowJointTrajectoryActionResult.h"

#include <boost/thread/mutex.hpp>


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

struct trajectory_execution_validation{
  bool finished;
  bool success;
};
  
private:
  ros::NodeHandle node_handle_;
  std::vector<handhold> handholds_;

  ros::Subscriber subscr_;
  
  std::vector<std::vector<double> > teached_wayp_r, teached_wayp_l;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  tf::TransformListener listener_entry;
  tf::StampedTransform transform_entry;

  tf::TransformListener listener_up;
  tf::StampedTransform transform_up;
    
  tf::TransformListener listener_down;
  tf::StampedTransform transform_down;

  std::string currentState_;
  std::string transition_;
  ros::ServiceServer service_getstate_,service_settransition_,service_setstop_;

  bool trajectoryexecution_;
  bool asynchandle_;

  move_group_interface::MoveGroup *group_r_;
  move_group_interface::MoveGroup *group_l_;
  move_group_interface::MoveGroup *group_both_;

  //tje = trajectory execution
  trajectory_execution_validation tje_validation_;  
  boost::mutex tje_lock_;

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
    
    //params
    currentState_ = "gazebo_home";
    transition_ = "";
    asynchandle_ = false;
    tje_lock_.lock();
    tje_validation_.finished = false;
    tje_validation_.success = false;
    tje_lock_.unlock();
    //trajectoryexecution = false;

    loadTeachedPoints(&teached_wayp_r,&teached_wayp_l);
    loadMoveGroups();
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
    }
    return ret;
  }

  bool inGrabPosition(){
  
    //Check if the Sonde is in a valid grabbing position!!

    return true;
  }
    
  //--------------------------------------------------------- Transitions------------------------------------------------------------------

  //TRANSITION: toHomeState
  bool toHomeState(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    bool ret = false;

    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit::planning_interface::MoveGroup::Plan mergedPlan;
    moveit::planning_interface::MoveGroup::Plan myPlan;

    group_l->setNamedTarget("lhome");
    group_r->setNamedTarget("rhome");
    
    bool success = false;
    //l
    success = group_l->plan(myPlan);
    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
    sleep(5.0);
    group_l->asyncExecute(myPlan);    

    if(success){
      tje_lock_.lock();
      tje_validation_.finished = false;
      tje_validation_.success = false;
      tje_lock_.unlock();
 
      asynchandle_ = group_l->asyncExecute(myPlan);    
      subscr_ = node_handle_.subscribe("/left_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);     
      while(!tje_validation_.finished){
	ROS_INFO("WAITING FOR TRAJECTORY EXECUTION TO FINISH");
      }
      subscr_ = ros::Subscriber();//ugly way to unsubscribe
      
      ret = tje_validation_.success;
    }

    
    //r
    if(success && ret){
      success = group_r->plan(myPlan);
      ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
      sleep(5.0);
      group_r->asyncExecute(myPlan);

      if(success){
	tje_lock_.lock();
	tje_validation_.finished = false;
	tje_validation_.success = false;
	tje_lock_.unlock();
 
	asynchandle_ = group_l->asyncExecute(myPlan);    
	subscr_ = node_handle_.subscribe("/right_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);     
	while(!tje_validation_.finished){
	  ROS_INFO("WAITING FOR TRAJECTORY EXECUTION TO FINISH");
	}
	subscr_ = ros::Subscriber();//ugly way to unsubscribe
      
	ret = tje_validation_.success;
      }
    }


    return ret;
  }

  //TRANSITION:avoidCollisionState
  //Moves the arm to a collision free state
  bool avoidCollisionState(move_group_interface::MoveGroup* group_l, move_group_interface::MoveGroup* group_r, move_group_interface::MoveGroup* group_both){
    
    bool ret = false;

    moveit::planning_interface::MoveGroup::Plan myPlan;

    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    std::vector<double> group_variable_values;
    std::vector<std::string> group_variable_names;
    group_variable_values = group_l->getCurrentJointValues();
    group_variable_names = group_l->getJoints();

    if(group_variable_values.size() != 6 || group_variable_names.size() != 6){
      ROS_ERROR("Number of joints must be 6");
    }
    
    //Gazebo Simulation -> Drive left arm out of initial collision pose
    group_variable_values[1] = -1.0;
    group_l->setJointValueTarget(group_variable_values);
    ret = group_l->plan(myPlan);
    sleep(5.0);

    if(ret){
      tje_lock_.lock();
      tje_validation_.finished = false;
      tje_validation_.success = false;
      tje_lock_.unlock();
 
      asynchandle_ = group_l->asyncExecute(myPlan);    
      subscr_ = node_handle_.subscribe("/left_arm_controller/follow_joint_trajectory/result", 1, &SenekaPickAndPlace::trajectoryStatus, this);     
      while(!tje_validation_.finished){
	ROS_INFO("WAITING FOR TRAJECTORY EXECUTION TO FINISH");
      }
      subscr_ = ros::Subscriber();//ugly way to unsubscribe
      
      ret = tje_validation_.success;
    }

    return ret;
  }
  
  //workaround to check asynch trajectory execution
  //for simulation with gazebo use this topic /left_arm_controller/follow_joint_trajectory/result
  void trajectoryStatus(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg){

    ROS_INFO("RESULT %i",msg->status.status);
    ROS_INFO("RESULT %i",msg->result.error_code);
      
    tje_lock_.lock();
    tje_validation_.finished = true;
    tje_validation_.success = true;//TODO: add error handling (http://mirror.umd.edu/roswiki/doc/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html)
    tje_lock_.unlock();
  }

  //pick and place planner
  void pnpPlanner(){

    std::cout << "---------------<planning>-----------------------------" << std::endl;
 
    unsigned int used_handle_r = 2;//use the real handle id 1-6
    unsigned int used_handle_l = 5;//use the real handle id 1-6
    used_handle_r--;
    used_handle_l--;
    
    move_group_interface::MoveGroup group_r("right_arm_group");
    move_group_interface::MoveGroup group_l("left_arm_group");
    move_group_interface::MoveGroup group_both("both_arms");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;;

    moveit::planning_interface::MoveGroup::Plan mergedPlan;

    group_r.setWorkspace (0, 0, 0, 5, 5, 5);
    group_r.setStartStateToCurrentState();
    group_l.setWorkspace (0, 0, 0, 5, 5, 5);
    group_l.setStartStateToCurrentState();
    group_both.setWorkspace (0, 0, 0, 5, 5, 5);
    group_both.setStartStateToCurrentState();

    group_r.setGoalOrientationTolerance(0.01);
    group_r.setPlanningTime(10.0);
    group_l.setGoalOrientationTolerance(0.01);
    group_l.setPlanningTime(10.0);
    group_both.setGoalOrientationTolerance(0.01);
    group_both.setPlanningTime(10.0);

    //The nonlinearplanner is not used right now

    /*geometry_msgs::PoseStamped target_pose_r;
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

    /*moveit::planning_interface::MoveGroup::Plan my_plan_r, my_plan_l;
    //my_plan.trajectory_ = trajectory;
    bool success_r = true;//group_r.plan(my_plan_r);
    bool success_l = true;//group_l.plan(my_plan_l);

    bool success = (success_r && success_l);
 
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if(!success){
      std::cout << "WARNING IM SENDING THE TRAJECTORY IN 10sec" << std::endl;
      //sleep(10.0);
      //group_r.execute(my_plan_r);
      //eep(10.0);
      //group_l.execute(my_plan_l);
      //sleep(20.0);
    }*/


    //-------------------Cartesian Planner - Linear Movement through all points (grabbing the sensornode) ------------------------
    if(true){

      bool planexecution = false;

      group_r.setStartStateToCurrentState();      
      group_l.setStartStateToCurrentState();
      group_both.setStartStateToCurrentState();

      std::vector<geometry_msgs::Pose> waypoints_r,waypoints_l;
      geometry_msgs::Pose target_pose2_r = group_r.getCurrentPose().pose;
      geometry_msgs::Pose target_pose2_l = group_l.getCurrentPose().pose;
      
      //------------------------Drive to entry point of pickup process-------------------------------------------------
      waypoints_r.clear();
      waypoints_l.clear();

      target_pose2_r.position.x = handholds_[used_handle_r].entry.translation.x;
      target_pose2_r.position.y = handholds_[used_handle_r].entry.translation.y;
      target_pose2_r.position.z = handholds_[used_handle_r].entry.translation.z;
      target_pose2_r.orientation.w = handholds_[used_handle_r].entry.rotation.w;
      target_pose2_r.orientation.x = handholds_[used_handle_r].entry.rotation.x;
      target_pose2_r.orientation.y = handholds_[used_handle_r].entry.rotation.y;
      target_pose2_r.orientation.z = handholds_[used_handle_r].entry.rotation.z;
      waypoints_r.push_back(target_pose2_r);

      target_pose2_l.position.x = handholds_[used_handle_l].entry.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].entry.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].entry.translation.z;
      target_pose2_l.orientation.w = handholds_[used_handle_l].entry.rotation.w;
      target_pose2_l.orientation.x = handholds_[used_handle_l].entry.rotation.x;
      target_pose2_l.orientation.y = handholds_[used_handle_l].entry.rotation.y;
      target_pose2_l.orientation.z = handholds_[used_handle_l].entry.rotation.z;
      waypoints_l.push_back(target_pose2_l);

      mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l);
      if(planexecution){
	group_both.execute(mergedPlan);
	sleep(10.0);
      }      

      //------------------------PICK UP-----------------------------------------------------------------------------
      waypoints_r.clear();
      waypoints_l.clear();

      target_pose2_r.position.x = handholds_[used_handle_r].down.translation.x;
      target_pose2_r.position.y = handholds_[used_handle_r].down.translation.y;
      target_pose2_r.position.z = handholds_[used_handle_r].down.translation.z;
      waypoints_r.push_back(target_pose2_r);
      target_pose2_r.position.x = handholds_[used_handle_r].up.translation.x;
      target_pose2_r.position.y = handholds_[used_handle_r].up.translation.y;
      target_pose2_r.position.z = handholds_[used_handle_r].up.translation.z;
      waypoints_r.push_back(target_pose2_r);

      target_pose2_l.position.x = handholds_[used_handle_l].down.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].down.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].down.translation.z;
      waypoints_l.push_back(target_pose2_l);
      target_pose2_l.position.x = handholds_[used_handle_l].up.translation.x;
      target_pose2_l.position.y = handholds_[used_handle_l].up.translation.y;
      target_pose2_l.position.z = handholds_[used_handle_l].up.translation.z;
      waypoints_l.push_back(target_pose2_l);
      
      mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l);
      if(planexecution){
	group_both.execute(mergedPlan);
	sleep(10.0);
      }  

      //------------------------TEACHED MOVEMENT--------------------------------------------------------------------
      /*      ROS_INFO("----------PLANNING WITH TEACHED WAYPOINTS------------");
      waypoints_r.clear();
      waypoints_l.clear();      

      for(unsigned int i = 0; i < teached_wayp_r.size();i++){
	if(teached_wayp_r[i].size() == 7){
	  target_pose2_r.position.x = teached_wayp_r[i][0];
	  target_pose2_r.position.y = teached_wayp_r[i][1];
	  target_pose2_r.position.z = teached_wayp_r[i][2];
	  target_pose2_r.orientation.w = teached_wayp_r[i][3];
	  target_pose2_r.orientation.x = teached_wayp_r[i][4];
	  target_pose2_r.orientation.y = teached_wayp_r[i][5];
	  target_pose2_r.orientation.z = teached_wayp_r[i][6];
	  waypoints_r.push_back(target_pose2_r);
	} else {
	  ROS_WARN("Something is wrong with the teached waypoints! Right arm  waypoint %u denied",i+1);
	}	
      }
      
      for(unsigned int i = 0; i < teached_wayp_l.size();i++){
	if(teached_wayp_l[i].size() == 7){
	  target_pose2_l.position.x = teached_wayp_l[i][0];
	  target_pose2_l.position.y = teached_wayp_l[i][1];
	  target_pose2_l.position.z = teached_wayp_l[i][2];
	  target_pose2_l.orientation.w = teached_wayp_l[i][3];
	  target_pose2_l.orientation.x = teached_wayp_l[i][4];
	  target_pose2_l.orientation.y = teached_wayp_l[i][5];
	  target_pose2_l.orientation.z = teached_wayp_l[i][6];
	  waypoints_l.push_back(target_pose2_l);
	} else {
	  ROS_WARN("Something is wrong with the teached waypoints! Right arm  waypoint %u denied",i+1);
	}	
      }

      mergedPlan = mergedPlanFromWaypoints(waypoints_r,waypoints_l);
      if(planexecution){
	//group_both.execute(mergedPlan);
	sleep(20.0);
      }
      */
    }
  } 

  move_group_interface::MoveGroup::Plan mergedPlanFromWaypoints(std::vector<geometry_msgs::Pose> &waypoints_r, std::vector<geometry_msgs::Pose> &waypoints_l){
    
    double visualizationtime = 15.0;

    move_group_interface::MoveGroup group_r("right_arm_group");
    move_group_interface::MoveGroup group_l("left_arm_group");

    moveit_msgs::RobotTrajectory trajectory_r, trajectory_l;
    moveit::planning_interface::MoveGroup::Plan linear_plan_r, linear_plan_l, mergedPlan;

    group_r.setWorkspace (0, 0, 0, 5, 5, 5);
    group_r.setStartStateToCurrentState();
    group_r.setGoalOrientationTolerance(0.01);
    group_r.setPlanningTime(10.0);

    group_l.setWorkspace (0, 0, 0, 5, 5, 5);
    group_l.setStartStateToCurrentState();
    group_l.setGoalOrientationTolerance(0.01);
    group_l.setPlanningTime(10.0);

    //-------RIGHT-------------------------
    group_r.computeCartesianPath(waypoints_r,
				 0.01,  // eef_step
				 0.0,   // jump_threshold
				 trajectory_r);  


    linear_plan_r.trajectory_ = trajectory_r;
    sleep(visualizationtime);

    //-------LEFT-------------------------
    group_l.computeCartesianPath(waypoints_l,
				 0.01,  // eef_step
				 0.0,   // jump_threshold
				 trajectory_l);  
      
    linear_plan_l.trajectory_ = trajectory_l;
    sleep(visualizationtime);

    mergedPlan = mergePlan(linear_plan_r,linear_plan_l);

    return mergedPlan;
  }

  //code from prace project, IPA: Benjamin Maidel
  move_group_interface::MoveGroup::Plan mergePlan(move_group_interface::MoveGroup::Plan plan1, move_group_interface::MoveGroup::Plan plan2)
  {
    move_group_interface::MoveGroup::Plan mergedPlan;
    mergedPlan = plan1;
    mergedPlan.trajectory_.joint_trajectory.joint_names.insert(mergedPlan.trajectory_.joint_trajectory.joint_names.end(),
							       plan2.trajectory_.joint_trajectory.joint_names.begin(),
							       plan2.trajectory_.joint_trajectory.joint_names.end());

    size_t i;
    for(i = 0; (i < mergedPlan.trajectory_.joint_trajectory.points.size())
	  && (i < plan2.trajectory_.joint_trajectory.points.size()); i++){

      mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.insert(
									     mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.end(),
									     plan2.trajectory_.joint_trajectory.points[i].accelerations.begin(),
									     plan2.trajectory_.joint_trajectory.points[i].accelerations.end());

      mergedPlan.trajectory_.joint_trajectory.points[i].positions.insert(
									 mergedPlan.trajectory_.joint_trajectory.points[i].positions.end(),
									 plan2.trajectory_.joint_trajectory.points[i].positions.begin(),
									 plan2.trajectory_.joint_trajectory.points[i].positions.end());

      mergedPlan.trajectory_.joint_trajectory.points[i].velocities.insert(
									  mergedPlan.trajectory_.joint_trajectory.points[i].velocities.end(),
									  plan2.trajectory_.joint_trajectory.points[i].velocities.begin(),
									  plan2.trajectory_.joint_trajectory.points[i].velocities.end());
    }

    if(plan1.trajectory_.joint_trajectory.points.size() > plan2.trajectory_.joint_trajectory.points.size()){

      for(size_t j = i; j < plan1.trajectory_.joint_trajectory.points.size(); j++){

	mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.insert(
									       mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.end(),
									       plan2.trajectory_.joint_trajectory.points.back().accelerations.begin(),
									       plan2.trajectory_.joint_trajectory.points.back().accelerations.end());

	mergedPlan.trajectory_.joint_trajectory.points[j].positions.insert(
									   mergedPlan.trajectory_.joint_trajectory.points[j].positions.end(),
									   plan2.trajectory_.joint_trajectory.points.back().positions.begin(),
									   plan2.trajectory_.joint_trajectory.points.back().positions.end());

	mergedPlan.trajectory_.joint_trajectory.points[j].velocities.insert(
									    mergedPlan.trajectory_.joint_trajectory.points[j].velocities.end(),
									    plan2.trajectory_.joint_trajectory.points.back().velocities.begin(),
									    plan2.trajectory_.joint_trajectory.points.back().velocities.end());
      }
    }

    if(plan1.trajectory_.joint_trajectory.points.size() < plan2.trajectory_.joint_trajectory.points.size()){

      trajectory_msgs::JointTrajectoryPoint point;
      for(size_t j = i; j < plan2.trajectory_.joint_trajectory.points.size(); j++){

	point  = mergedPlan.trajectory_.joint_trajectory.points.back();

	point.accelerations.insert(
				   point.accelerations.end(),
				   plan2.trajectory_.joint_trajectory.points[j].accelerations.begin(),
				   plan2.trajectory_.joint_trajectory.points[j].accelerations.end());

	point.positions.insert(
			       point.positions.end(),
			       plan2.trajectory_.joint_trajectory.points[j].positions.begin(),
			       plan2.trajectory_.joint_trajectory.points[j].positions.end());

	point.velocities.insert(
				point.velocities.end(),
				plan2.trajectory_.joint_trajectory.points[j].velocities.begin(),
				plan2.trajectory_.joint_trajectory.points[j].velocities.end());
	    
	point.time_from_start = plan2.trajectory_.joint_trajectory.points[j].time_from_start;

	mergedPlan.trajectory_.joint_trajectory.points.push_back(point);
      }
    }

    return mergedPlan;
  }

  void loadTeachedPoints(std::vector<std::vector<double> >* vec_r,std::vector<std::vector<double> >* vec_l){
    
    SerializeIO *ser = new SerializeIO("/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_pnp/common/teached_dual_arm_movement.def",'i');
    std::vector<std::vector<double> > tmp;

    ser->openArray("dual_arm_movement");
    ser->readArray("dual_arm_movement",&tmp);
    ser->closeArray();
    ser->close();

    for(unsigned int i = 0; i < tmp.size(); i++){
      
      if((i%2)==0){
	vec_r->push_back(tmp[i]);
      } else {
	vec_l->push_back(tmp[i]);
      }
    }
 
    delete ser;
    
  }	
  
  void loadMoveGroups()
  {
    
    group_r_ =  new move_group_interface::MoveGroup("right_arm_group");
    group_l_ =  new move_group_interface::MoveGroup("left_arm_group");
    group_both_ = new move_group_interface::MoveGroup("both_arms");

    //planning settings
    group_r_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_r_->setStartStateToCurrentState();
    group_l_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_l_->setStartStateToCurrentState();
    group_both_->setWorkspace (0, 0, 0, 5, 5, 5);
    group_both_->setStartStateToCurrentState();

    group_r_->setGoalOrientationTolerance(0.01);
    group_r_->setPlanningTime(10.0);
    group_l_->setGoalOrientationTolerance(0.01);
    group_l_->setPlanningTime(10.0);
    group_both_->setGoalOrientationTolerance(0.01);
    group_both_->setPlanningTime(10.0);

  }

  //STATES: home, gazebo_home, collision_free
  //TRANSITSIONS: avoidCollisionState
  std::string stateMachine(std::string currentState)
  {
    std::string transition = getTransition();

    //---------GAZEBO_HOME----------------------------------
    if(currentState.compare("gazebo_home") == 0){

      //Transitions
      if(transition.compare("avoidCollisionState") == 0){
	if(avoidCollisionState(group_l_,group_r_,group_both_)){
	  return "collision_free";
	} else {
	  return "unknown_state";
	}
      }      
      return "gazebo_home" ;
    }

    //------COLLISION_FREE----------------------------------
    else if(currentState.compare("collision_free") == 0){
      
      //Transitions
      if(transition.compare("toHomeState") == 0){
	if(toHomeState(group_l_,group_r_,group_both_))
	  return "home";
      }
      
      return "collision_free";
    }

    //------HOME-------------------------------------------
    else if(currentState.compare("home") == 0){
      return "home";
    }
    
    //------UNKNOWN_STATE-------------------------------------------
    else{
      return "unknown_state";
    }

    return "unknown_state";
  }

  //------------ Services -----------------------------------
  bool getState(seneka_pnp::getState::Request &req,
		seneka_pnp::getState::Response &res)
  {
    res.state = currentState_;
    return true;
  }
 
  bool setTransition(seneka_pnp::setTransition::Request &req,
		     seneka_pnp::setTransition::Response &res)
  {
    transition_ = req.transition;
    res.transition = transition_;
    return true;
  }

 bool setStop(seneka_pnp::setStop::Request &req,
		     seneka_pnp::setStop::Response &res)
  {
    group_l_->stop();
    group_r_->stop();
    group_both_->stop();
    
    transition_ = "";

    res.success = true;

    return true;
  }

  //------------ Services -----------------------------------
  std::string getTransition(){
    return transition_;
  }

  //main loop
  //check Sensornode position and start planning
  void mainLoop(){

    service_getstate_ = node_handle_.advertiseService("seneka_pnp/getState", &SenekaPickAndPlace::getState, this);
    service_settransition_ = node_handle_.advertiseService("seneka_pnp/setTransition", &SenekaPickAndPlace::setTransition, this);
    service_setstop_ = node_handle_.advertiseService("seneka_pnp/setStop", &SenekaPickAndPlace::setStop, this);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Rate loop_rate(1);
    while(ros::ok()){
      
      currentState_ = stateMachine(currentState_);
      ROS_INFO("Current state: %s",currentState_.c_str());
      //ROS_INFO("asynchandle: %d",asynchandle_);
      loop_rate.sleep();

      /*bool valid_detection =  getSensornodePose();
      if(inGrabPosition() && valid_detection){
	ROS_INFO("IN POSITION");
	//pnpPlanner();
	planToHomeState();
      } else {
	ROS_INFO("No valid position");
      }*/
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
