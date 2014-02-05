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
  float x;
  float y;
  float z;
};

struct quaternion{
  float w;
  float x;
  float y;
  float z;
};

struct pose3d{
  pose translation;
  quaternion rotation;
};

struct handhold{
  pose3d handle;
  pose3d up;
  pose3d down;
};

private:
  ros::NodeHandle node_handle_;
  pose3d sensornode_;
  std::vector<handhold> handholds_;

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
    mainLoop();
  }  

  bool getSensornodePose(){
    
    int ret = true;  
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    try{
      listener.waitForTransform("/quanjo_body", "/handle2", ros::Time::now(), ros::Duration(2.0));
      listener.lookupTransform("/quanjo_body","/handle2", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ret = false;
    }
  
    sensornode_.translation.x = transform.getOrigin().x();
    sensornode_.translation.y = transform.getOrigin().y();
    sensornode_.translation.z = transform.getOrigin().z();
  
    sensornode_.rotation.w = transform.getRotation().getW();
    sensornode_.rotation.x = transform.getRotation().getAxis().x();
    sensornode_.rotation.y = transform.getRotation().getAxis().y();
    sensornode_.rotation.z = transform.getRotation().getAxis().z();

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
    
    move_group_interface::MoveGroup group("right_arm_group");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ros::Publisher display_publisher = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    group.setWorkspace (0, 0, 0, 2, 2, 2);

    geometry_msgs::PoseStamped rnd_pose;
   
    rnd_pose.pose.position.x = sensornode_.translation.x;
    rnd_pose.pose.position.y = sensornode_.translation.y;
    rnd_pose.pose.position.z = sensornode_.translation.z;

    rnd_pose.header.frame_id = "/quanjo_body";

    rnd_pose.pose.orientation.w = sensornode_.rotation.w;
    rnd_pose.pose.orientation.x = sensornode_.rotation.x;
    rnd_pose.pose.orientation.y = sensornode_.rotation.y;
    rnd_pose.pose.orientation.z = sensornode_.rotation.z;
    /* rnd_pose.pose.orientation.w = 1;
    rnd_pose.pose.orientation.x = 0;
    rnd_pose.pose.orientation.y = 0;
    rnd_pose.pose.orientation.z = 0;*/

    std::cout << "X: " << sensornode_.translation.x  << std::endl;
    std::cout << "Y: " << sensornode_.translation.y  << std::endl;
    std::cout << "Z: " << sensornode_.translation.z  << std::endl;

    //-------------------------------
    /*  moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "right_arm_rviz_link";
    ocm.header.frame_id = "quanjo_body";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    group.setPathConstraints(test_constraints);

    
    /*robot_state::RobotState start_state(*group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    const robot_state::JointModelGroup *joint_model_group =
      start_state.getJointModelGroup(group.getName());
    start_state.setFromIK(joint_model_group, start_pose2);
    group.setStartState(start_state);*/
    //------------------------------

    group.setPoseTarget(rnd_pose); 

    //geometry_msgs::PoseStamped targetpose =  group.getPoseTarget("right_arm_rviz_link");

    group.setStartStateToCurrentState();

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setPlanningTime(10.0);
    group.move();
    /*bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    if (success)
    {  
	ROS_INFO("Visualizing plan 1 (again)");
	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
	// Sleep to give Rviz time to visualize the plan. 
	sleep(20.0);
	group.execute(my_plan);
    }*/
    sleep(20.0);
    
    std::cout << "---------------</planning>----------------------------" << std::endl;
  }  

  //main loop
  //check Sensornode position and start planning
  void mainLoop(){
    
         ros::AsyncSpinner spinner(4); // Use 4 threads
      spinner.start();

    ros::Rate loop_rate(10);
    while(ros::ok()){
    
      bool valid_detection = getSensornodePose();
    
      if(inGrabPosition() && valid_detection){
	ROS_INFO("Yeah... O_o please,please let me grab it !!!");
	pnpPlanner();
      } else {
	ROS_INFO("Are you crazy?... X_x I'm not a monkey with infinite arm length X_x!!!");
      }
      std::cout << "running" << std::endl;
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
