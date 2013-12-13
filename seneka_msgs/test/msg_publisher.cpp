#include <ros/ros.h>
#include <seneka_msgs/FiducialArray.h>
#include <seneka_msgs/Fiducial.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fiducial_publisher");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<seneka_msgs::FiducialArray>("fiducialarray", 1000);
  ros::Rate loop_rate(10);

  unsigned int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     seneka_msgs::FiducialArray container_msg;
     seneka_msgs::Fiducial fiducial_msg;

     fiducial_msg.fiducial_id = 0;
     container_msg.container.push_back(fiducial_msg);

    // ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(container_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
