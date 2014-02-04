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
      listener.waitForTransform("/quanjo_body", "/sensornode", ros::Time::now(), ros::Duration(2.0));
      listener.lookupTransform("/quanjo_body","/sensornode", ros::Time(0), transform);
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
    
    move_group_interface::MoveGroup group("right_arm_group",boost::shared_ptr<tf::Transformer>(),ros::Duration(5,5));
    geometry_msgs::PoseStamped rnd_pose = group.getRandomPose();
    group.setPoseTarget(rnd_pose); 
    group.move();
    
    std::cout << "---------------</planning>----------------------------" << std::endl;
  }  

  //main loop
  //check Sensornode position and start planning
  void mainLoop(){

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
      ros::spinOnce();
      //loop_rate.sleep();
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
