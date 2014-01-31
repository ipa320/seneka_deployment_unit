
#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <seneka_msgs/FiducialArray.h>
#include <seneka_msgs/Fiducial.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <SerializeIO.h>

#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>

#include <pluginlib/class_loader.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

//MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/move_group_interface/move_group.h>

// PI
#include <boost/math/constants/constants.hpp>


int main( int argc, char** argv )
{  
  ros::init (argc, argv, "state_display_tutorial");
  // Needed for ROS_INFO commands to work 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 10 );                                                                                        
  ros::Rate loop_rate(1);

 
  /* robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm_group");

  //Planning
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!nh.getParam("/move_group/planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");


  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(kinematic_model, nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

  for(int cnt=0; ros::ok(); cnt++){ 
    
    kinematic_state->setToRandomValues();
    std::vector<double> joint_state_val;
    kinematic_state->getStateValues(joint_state_val);
    
    std::cout << "-------------------------------------------------------" << std::endl;
    for(unsigned int i=0; i < joint_state_val.size(); i++){
      std::cout << joint_state_val[i] << std::endl;
    }


    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

 
    req.group_name = "right_arm_group";
    req.allowed_planning_time = 5.0;
  
    const robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("right_arm_group");

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(joint_state_group,0.1,0.1);
    req.goal_constraints.push_back(pose_goal);


    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);   
 
    context->solve(res);*/
  

    move_group_interface::MoveGroup group("right_arm_group",boost::shared_ptr<tf::Transformer>(),ros::Duration(5,5 ));
    geometry_msgs::PoseStamped rnd_pose = group.getRandomPose();
    group.setPoseTarget(rnd_pose);

 

    group.move();
 
    /*moveit_msgs::DisplayRobotState msg; 
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    
    robot_state_publisher.publish( msg );*/
     
    ros::spinOnce();
    loop_rate.sleep();
    //}

  return 0;
}



 /*int main( int argc, char** argv )
{  
  ros::init (argc, argv, "state_display_tutorial");

  ros::NodeHandle nh;                                                                                      
  ros::Rate loop_rate(1);

  moveit::planning_interface::MoveGroup group("right_arm_group",boost::shared_ptr<tf::Transformer>(),ros::Duration(5,5 ));  
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  return 0;
}*/
