/**********************
* Author: Matthias Nösner
*
*
* 
********************** */

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "sensornode_detection");
  ros::NodeHandle node;
  
  tf::TransformListener listener;
  
  ros::Rate rate(10.0);
 w cobhile (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/fiducials/fiducials/tf_frames","/fiducials/fiducials/tf_framesGGG",ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
    printf( "\n Y: %d", transform.getOrigin().y());
    printf( " X: %d \n\n", transform.getOrigin().x());
    rate.sleep();
  }

  return 0;
}
//ros::ServiceClient client = n.ServiceClient<tf::TransformListener>("/fiducials/fiducials/tf_frames");
