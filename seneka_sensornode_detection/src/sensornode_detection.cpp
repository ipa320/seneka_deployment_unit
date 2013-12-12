/**********************
* Author: Matthias Nösner
*
*
* Aspired functionallity:
* This stack should manage the sensornode detection. 
*          - It must provide the pose of the sensornode via tf.
*          - compute the final pose of quanjo in reference to the sensornode          
*                - check if the sensornode is tangible from this position
*                - check if there is a plan for gathering the sensornode    
*                - Note: !!!!Start with a fixed standard grabbing position (position constraint) 
*                            between quanjo and the sensorsonde (as reference take 
*                            camera positioning from Voxelmap)!!!!
*          - Initiate/Manage the different grabbing stages
********************** */

#include <ros/ros.h>
#include "gazebo_msgs/SetModelState.h"

//#include <cob_object_detection_msgs/DetectObjects.h>

#include <seneka_msgs/FiducialArray.h>
#include <seneka_msgs/Fiducial.h>

void setGazeboPose(ros::NodeHandle node);

void setGazeboPose(ros::NodeHandle node)
{
        geometry_msgs::Pose start_pose;
        start_pose.position.x = 2.0;
        start_pose.position.y = 2.0;
        start_pose.position.z = 0.6;
        start_pose.orientation.x = 0.0;
        start_pose.orientation.y = 0.0;
        start_pose.orientation.z = 0.0;
        start_pose.orientation.w = 0.0;

        geometry_msgs::Twist start_twist;
        start_twist.linear.x = 0.0;
        start_twist.linear.y = 0.0;
        start_twist.linear.z = 0.0;
        start_twist.angular.x = 0.0;
        start_twist.angular.y = 0.0;
        start_twist.angular.z = 0.0;

        gazebo_msgs::ModelState modelstate;
        modelstate.model_name = (std::string) "sensornode";
        modelstate.reference_frame = (std::string) "world";
        modelstate.pose = start_pose;
        modelstate.twist = start_twist;

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

void chatterCallback(const seneka_msgs::FiducialArray::ConstPtr& msg)
{
  //TODO: Give the callback another name
  //Functionallity: The callback should update the actual sensornode position and orientation.
  //                It should compute the mean pose (or maybe another strategy) from the detected markers
  //Note: I can image this functionallity could also be useful for the cob_fiducial node,
  //      then tf can be used to derive the sensornode pose in reference to quanjo.
  //      But the first approach is to  implement it  here!
  ROS_INFO("[sensornode_detection] I heard from  [%u] fiducial detections", msg->container.size());
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "sensornode_detection");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/fiducials/fiducial_custom_array", 1000, chatterCallback);
  
  //It is not possible to use this subscriber/callback right now cause cob_object_detection_msgs is still a rosbuild package
  //Maybe ask Richard/Jan if it is possible to make it a catkin package in the offical repo.
  //I'm too lazy right now to write a workaround or a self created cob_object_detection_msg catkin branch.
  //The marker detection can be started in a shell when using the  command in the next line.
  //CMD: rostopic hz /fiducials/detect_fiducials
  //ros::Subscriber detect_fid = node.subscribe("/fiducials/detect_fiducials", 1, dummyCallback);
  
  setGazeboPose(node);

  ros::spin();
  return 0;
}

