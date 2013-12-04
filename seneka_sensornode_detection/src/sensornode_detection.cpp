#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

tf::TransformListener *tran;

int main( int argc, char** argv )
{
  ros::init(argc,argv,"sensornode_detection");
  ros::NodeHandle n;
 
  //ros::ServiceClient client = n.ServiceClient<tf::TransformListener>("/fiducials/fiducials/tf_frames");

  if(ros::service::call("/fiducials/fiducials/tf_frames", true)){
    printf( "\nHello World\n\n" );
  }


}
