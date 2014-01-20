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

void setGazeboPose(ros::NodeHandle node);

void setGazeboPose(pose origin, quaternion rotation)
{
        geometry_msgs::Pose start_pose;
        start_pose.position.x = origin.x;
        start_pose.position.y = origin.y;
        start_pose.position.z = origin.z;
        start_pose.orientation.w = rotation.w;
        start_pose.orientation.x = rotation.x;
        start_pose.orientation.y = rotation.y;
        start_pose.orientation.z = rotation.z;

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

//-----------------------------------main----------------------------------------------------------------------------
int main( int argc, char** argv )
{  
  ros::init(argc, argv, "sensornode_operator");
  ros::NodeHandle node;

  ros::Rate loop_rate(10);
  
  while(ros::ok()){
   
    tf::TransformListener listener;
    tf::StampedTransform transform;
    pose origin2;
    quaternion rotation2;

    origin2.x = 5;
    origin2.y = 5;
    origin2.z = 5;
  
    rotation2.w = 1;
    rotation2.x = 0;
    rotation2.y = 0;
    rotation2.z = 0;
   
    
    try{
      listener.waitForTransform("/quanjo_body", "/seneka_marker", ros::Time::now(), ros::Duration(2.0));
      listener.lookupTransform("/quanjo_body","/seneka_marker",
			       ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }


    std::cout << "X " << transform.getOrigin().x() << std::endl;
    std::cout << "Y " << transform.getOrigin().y() << std::endl;
    std::cout << "Z " << transform.getOrigin().z() << std::endl;
  
    origin2.x = transform.getOrigin().x();
    origin2.y = transform.getOrigin().y();
    origin2.z = transform.getOrigin().z();
  
    rotation2.w = 1;
    rotation2.x = 0;
    rotation2.y = 0;
    rotation2.z = 0;

    //TODO send the correct trafo... from quanjo body (or gazebo_world?)!!!
    setGazeboPose(origin2,rotation2);



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

