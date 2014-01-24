#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <cob_object_detection_msgs/DetectObjects.h>

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

/* // MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>*/

const double PI = 3.14159265359;

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

void setGazeboSensornodePose(pose3d isensornode);
void getSensornodePose(pose3d *isensornode);
bool inGrabPosition(pose3d *isensornode);
void pnpPlanner(pose3d lhandle);

bool allow_gazebo_update = false;
pose3d sensornode;

void setGazeboSensornodePose(pose3d isensornode)
{
  geometry_msgs::Pose start_pose;
  start_pose.position.x = isensornode.translation.x;
  start_pose.position.y = isensornode.translation.y;
  start_pose.position.z = isensornode.translation.z;
  start_pose.orientation.w = isensornode.rotation.w;
  start_pose.orientation.x = isensornode.rotation.x;
  start_pose.orientation.y = isensornode.rotation.y;
  start_pose.orientation.z = isensornode.rotation.z;

  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string) "sensornode";
  modelstate.reference_frame = (std::string) "quanjo_body";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

	
  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;

  if (client.call(setmodelstate))
    {
      ROS_INFO("BRILLIANT!!!");
    }
  else
    {
      ROS_ERROR("Failed to call service ");
    }
}

void getSensornodePose(pose3d *isensornode){
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  pose3d sensornode_tmp;

  //standard value to avoid collision in gazebo with quanjo on startup
  sensornode_tmp.translation.x = 10;
  sensornode_tmp.translation.y = 10;
  sensornode_tmp.translation.z = 10;  
  sensornode_tmp.rotation.w = 1;
  sensornode_tmp.rotation.x = 0;
  sensornode_tmp.rotation.y = 0;
  sensornode_tmp.rotation.z = 0;   
    
  try{
    listener.waitForTransform("/quanjo_body", "/sensornode", ros::Time::now(), ros::Duration(2.0));
    listener.lookupTransform("/quanjo_body","/sensornode", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  sensornode_tmp.translation.x = transform.getOrigin().x();
  sensornode_tmp.translation.y = transform.getOrigin().y();
  sensornode_tmp.translation.z = transform.getOrigin().z();
  
  sensornode_tmp.rotation.w = transform.getRotation().getW();
  sensornode_tmp.rotation.x = transform.getRotation().getAxis().x();
  sensornode_tmp.rotation.y = transform.getRotation().getAxis().y();
  sensornode_tmp.rotation.z = transform.getRotation().getAxis().z();
  
  if(allow_gazebo_update)
    setGazeboSensornodePose(sensornode_tmp);

  //setting global variable
  isensornode = &sensornode_tmp;
}

bool inGrabPosition(pose3d *isensornode){
  
  //Check if the Sonde is in a valid grabbing position!!
  //Keep it simple and hardcoded for milestone februrary

  return true;
}

//pick and place planner
void pnpPlanner(pose3d lhandle){
  
  std::cout << "---------------<planning>-----------------------------" << std::endl;
  
  std::cout << "---------------</planning>----------------------------" << std::endl;
}

//-----------------------------------main----------------------------------------------------------------------------
int main( int argc, char** argv )
{  
  ros::init(argc, argv, "sensornode_operator");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node;
  
  ros::Rate loop_rate(10);
  
  getSensornodePose(&sensornode);
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Construct a planning scene - NOTE: this is for illustration purposes only.
  // The recommended way to construct a planning scene is to use the planning_scene_monitor
  //  to construct it for you.
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  /* SETUP THE PLANNER*/
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if (!node.getParam("planning_plugin", planner_plugin_name))
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
    if (!planner_instance->initialize(robot_model, node.getNamespace()))
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

  /* CREATE A MOTION PLAN REQUEST FOR THE RIGHT ARM OF THE PR2 */
  /* We will ask the end-effector of the PR2 to go to a desired location */
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  /* A desired pose */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "right_arm_base_link";
  pose.pose.position.x = 1;
  pose.pose.position.y = 1;
  pose.pose.position.z = 1;
  pose.pose.orientation.w = 0.5;

  /* A desired tolerance */
  std::vector<double> tolerance_pose(3, 100 );
  std::vector<double> tolerance_angle(3, 100 );
  /*Create the request */
  req.group_name = "right_arm_group";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("right_arm_ee_link",  pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  /* Construct the planning context */
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

  /* CALL THE PLANNER */
  context->solve(res);


  /* robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
  //robot_model_loader.loadKinematicsSolvers();
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
 
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();  
  robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup("right_arm_group");

  // Compute FK for a set of random joint values
  joint_state_group->setToRandomValues();
  const Eigen::Affine3d &end_effector_state = joint_state_group->getRobotState()->getLinkState("right_arm_ee_link")->getGlobalLinkTransform();  

  // Get the joint values
  std::vector<double> joint_values;
  joint_state_group->getVariableValues(joint_values);
  
  bool found_ik = joint_state_group->setFromIK(end_effector_state, 5, 0.1);
  
  /*  if(found_ik){

    joint_state_group->getVariableValues(joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
      {
	ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
  }
  else{
    
    ROS_INFO("Did not find IK solution");
    }

  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;  
  joint_state_group->getJacobian(joint_state_group->getJointModelGroup()->getLinkModelNames().back(),
				 reference_point_position,
				 jacobian);
				 ROS_INFO_STREAM("Jacobian: " << jacobian); */



  /* ROS_INFO("1");
  move_group_interface::MoveGroup group("right_arm_group");
  ROS_INFO("2");
  group.setRandomTarget();
  // plan the motion and then move the group to the sampled target 
  ROS_INFO("3");
  group.move();
  ROS_INFO("4");*/


  



  /* while(ros::ok()){
    
    getSensornodePose(&sensornode);
    
    if(inGrabPosition(&sensornode)){
      ROS_INFO("Yeah... O_o please,please let me grab it !!!");
      pnpPlanner(sensornode);//TODO pass handle position
    } else {
      ROS_INFO("Are you crazy?... X_x I'm not a monkey with infinite arm length X_x!!!");
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    }  */
  
  return 0;
}

