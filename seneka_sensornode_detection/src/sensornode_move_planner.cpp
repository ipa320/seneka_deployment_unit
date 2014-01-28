
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
  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  /* Get the configuration for the joints in the right arm of the PR2*/
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm_group");

  ros::NodeHandle nh;
  ros::Publisher robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>( "tutorial_robot_state", 10 );                                                                                        
  ros::Rate loop_rate(1);

  //Planning
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

    if (!nh.getParam("planning_plugin", planner_plugin_name))
      ROS_FATAL_STREAM("Could not find planner plugin name");

    /* Make sure to catch all exceptions */
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

    /* CREATE A MOTION PLAN REQUEST FOR THE RIGHT ARM OF THE PR2 */
    /* We will ask the end-effector of the PR2 to go to a desired location */
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    /* Create the request */
    req.group_name = "right_arm_group";
    req.allowed_planning_time = 5.0;
  
    const robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("right_arm_group");

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(joint_state_group,0.1,0.1);
    req.goal_constraints.push_back(pose_goal);

    /* Construct the planning context */
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  
    
    /* CALL THE PLANNER */
    context->solve(res);


    move_group_interface::MoveGroup group("right_arm_group");
    group.setRandomTarget();
    group.plan();
    group.move();
    
    moveit_msgs::DisplayRobotState msg; 
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
    
    robot_state_publisher.publish( msg );
     
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
