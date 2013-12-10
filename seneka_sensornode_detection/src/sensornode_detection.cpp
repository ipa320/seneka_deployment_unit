/**********************
* Author: Matthias Nösner
*
*
* 
********************** */

#include <ros/ros.h>

#include <seneka_msgs/FiducialArray.h>
#include <seneka_msgs/Fiducial.h>

void chatterCallback(const seneka_msgs::FiducialArray::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]",msg->container.size());
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "sensornode_detection");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/fiducials/fiducial_custom_array", 1000, chatterCallback);
  
  ros::spin();
  return 0;
}
//ros::ServiceClient client = n.ServiceClient<tf::TransformListener>("/fiducials/fiducials/tf_frames");
