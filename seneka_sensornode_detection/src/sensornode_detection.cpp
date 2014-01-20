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

struct point4d{
	double x;
	double y;
	double z;
	double e;
	bool detected;
};

struct fiducialmarker{
	unsigned int id;
	cv::Point3d trans;
	cv::Point3d rot;
};

bool publish_to_gazebo = false;

void setGazeboPose(pose origin, quaternion rotation);
bool loadParameters(std::vector<fiducialmarker>*);

//---------------------------<coordinate transformations>---------------------------------------
cv::Mat eulerToMatrix(double rot1, double rot2, double rot3);
void multiplyQuaternion(std::vector<double> q1,std::vector<double> q2,std::vector<double>* q);
std::vector<double> FrameToVec7(const cv::Mat frame);
//---------------------------</coordinate transformations>---------------------------------------

std::vector<fiducialmarker> fiducialmarkers;

boost::mutex tf_lock_;
ros::Timer tf_pub_timer_;
tf::StampedTransform marker_tf_;

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

//------------------------------<Callbacks>----------------------------------------------------------
void timerCallback(const ros::TimerEvent& event)
{
  /*tf_lock_.lock();
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(1.1, 0.0, 0.6) );
  transform.setRotation( tf::createQuaternionFromRPY(-PI/2,0,-PI/2) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "quanjo_body", "camera"));
  tf_lock_.unlock();*/
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


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(1.1, 0.0, 0.6) );
  transform.setRotation( tf::createQuaternionFromRPY(-PI/2,0,-PI/2) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/quanjo_body", "/camera"));



  pose origin;
  quaternion rotation;

  //Right now only the first detected marker is used
  //TODO: Make use of all detected markers
  if(msg->container.size() > 0){
    
    origin.x = msg->container[0].origin[0];
    origin.y = msg->container[0].origin[1];
    origin.z = msg->container[0].origin[2];
    
    rotation.w = msg->container[0].rotation[0];
    rotation.x = msg->container[0].rotation[1];
    rotation.y = msg->container[0].rotation[2];
    rotation.z = msg->container[0].rotation[3];

    std::cout << "X " << origin.x << std::endl;
    std::cout << "Y " << origin.y << std::endl;
    std::cout << "Z " << origin.z << std::endl;

    static tf::TransformBroadcaster br2;
    tf::Transform transform2;
    transform2.setOrigin( tf::Vector3(origin.x,origin.y,origin.z) );
    transform2.setRotation( tf::Quaternion(rotation.x,rotation.y,rotation.z,rotation.w));
    br2.sendTransform(tf::StampedTransform(transform2,  ros::Time::now(), "camera", "seneka_marker"));

    //-> Transform each marker pose to object pose
    for(unsigned int i = 0; i < fiducialmarkers.size(); i++){
      //for loop j to iterate through container
      if(msg->container[0].fiducial_id == fiducialmarkers[i].id){

	//all transformation matrices are named after the format tm_object_basesystem or q_object_basesystem
        //tm = transformation matrix, q = quaternion7
	//m = marker, sn = sensornode
	cv::Mat tm_m_sn = eulerToMatrix(fiducialmarkers[i].rot.x,fiducialmarkers[i].rot.y,fiducialmarkers[i].rot.z);
        cv::Mat tmp = cv::Mat(tm_m_sn.t());//transposed
	tm_m_sn = cv::Mat(3,4, CV_64FC1);
	for (int k=0; k<3; k++)
                    for (int l=0; l<3; l++)
                        tm_m_sn.at<double>(k,l) = tmp.at<double>(k,l);
	tm_m_sn.at<double>(0,3) = fiducialmarkers[i].trans.x;
	tm_m_sn.at<double>(1,3) = fiducialmarkers[i].trans.y;
	tm_m_sn.at<double>(2,3) = fiducialmarkers[i].trans.z;

	std::vector<double> q_sn_m = FrameToVec7(tm_m_sn);	
	  
	static tf::TransformBroadcaster br3;
	tf::Transform transform3;
	transform3.setOrigin( tf::Vector3(q_sn_m[0], q_sn_m[1], q_sn_m[2]));
	transform3.setRotation( tf::Quaternion(q_sn_m[4],q_sn_m[5],q_sn_m[6],q_sn_m[3]));
	br3.sendTransform(tf::StampedTransform(transform3,  ros::Time::now(), "seneka_marker", "sensornode"));


	if(publish_to_gazebo){

	  tf::TransformListener listener;
	  tf::StampedTransform transform2;

	  listener.waitForTransform("/sensornode", "/quanjo_body", ros::Time::now(), ros::Duration(2.0));
	  if(true){
	    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!! HERE !!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
	    
	    try{
	      listener.lookupTransform("/sensornode","/quanjo_body",
				       ros::Time(0), transform2);
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what());
	    }
	    

	    pose origin2;
	    quaternion rotation2;
	  
	    origin2.x = transform2.getOrigin().x();
	    origin2.y = transform2.getOrigin().y();
	    origin2.z = transform2.getOrigin().z()+2;
	   
	    rotation2.w = q_sn_m[3];
	    rotation2.x = q_sn_m[4];
	    rotation2.y = q_sn_m[5];
	    rotation2.z = q_sn_m[6];

	    //TODO send the correct trafo... from quanjo body (or gazebo_world?)!!!
	    setGazeboPose(origin2,rotation2);
	  }
	}      
      }
    }
  }
  
  //-> determine mean of object pose
  //-> postponed ;)   
  
  //-> update object(sensornode) pose
}
//------------------------------</Callbacks>----------------------------------------------------------

//Load Parameters positions of markers in reference to the sensorsonde using my SerializeIO class + scaling the values.
bool loadParameters(std::vector<fiducialmarker>* afiducialmarkers ){

	SerializeIO *ser = new SerializeIO("/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_sensornode_detection/launch/fiducial_poses.def",'i');

	fiducialmarker fiducial1,fiducial2,fiducial3,fiducial4,fiducial5, fiducial6;

	double scale = 0.001;//from mm to m

	if(!ser->readVariable("fiducial1_ID",&(fiducial1.id))) return false;
	if(!ser->readVariable("fiducial1_POSX",&(fiducial1.trans.x))) return false;
	if(!ser->readVariable("fiducial1_POSY",&(fiducial1.trans.y))) return false;
	if(!ser->readVariable("fiducial1_POSZ",&(fiducial1.trans.z))) return false;
	if(!ser->readVariable("fiducial1_ROT1",&(fiducial1.rot.x))) return false;
	if(!ser->readVariable("fiducial1_ROT2",&(fiducial1.rot.y))) return false;
	if(!ser->readVariable("fiducial1_ROT3",&(fiducial1.rot.z))) return false;

	if(!ser->readVariable("fiducial2_ID",&(fiducial2.id))) return false;
	if(!ser->readVariable("fiducial2_POSX",&(fiducial2.trans.x))) return false;
	if(!ser->readVariable("fiducial2_POSY",&(fiducial2.trans.y))) return false;
	if(!ser->readVariable("fiducial2_POSZ",&(fiducial2.trans.z))) return false;
	if(!ser->readVariable("fiducial2_ROT1",&(fiducial2.rot.x))) return false;
	if(!ser->readVariable("fiducial2_ROT2",&(fiducial2.rot.y))) return false;
	if(!ser->readVariable("fiducial2_ROT3",&(fiducial2.rot.z))) return false;

	if(!ser->readVariable("fiducial3_ID",&(fiducial3.id))) return false;
	if(!ser->readVariable("fiducial3_POSX",&(fiducial3.trans.x))) return false;
	if(!ser->readVariable("fiducial3_POSY",&(fiducial3.trans.y))) return false;
	if(!ser->readVariable("fiducial3_POSZ",&(fiducial3.trans.z))) return false;
	if(!ser->readVariable("fiducial3_ROT1",&(fiducial3.rot.x))) return false;
	if(!ser->readVariable("fiducial3_ROT2",&(fiducial3.rot.y))) return false;
	if(!ser->readVariable("fiducial3_ROT3",&(fiducial3.rot.z))) return false;

	if(!ser->readVariable("fiducial4_ID",&(fiducial4.id))) return false;
	if(!ser->readVariable("fiducial4_POSX",&(fiducial4.trans.x))) return false;
	if(!ser->readVariable("fiducial4_POSY",&(fiducial4.trans.y))) return false;
	if(!ser->readVariable("fiducial4_POSZ",&(fiducial4.trans.z))) return false;
	if(!ser->readVariable("fiducial4_ROT1",&(fiducial4.rot.x))) return false;
	if(!ser->readVariable("fiducial4_ROT2",&(fiducial4.rot.y))) return false;
	if(!ser->readVariable("fiducial4_ROT3",&(fiducial4.rot.z))) return false;

	if(!ser->readVariable("fiducial5_ID",&(fiducial5.id))) return false;
	if(!ser->readVariable("fiducial5_POSX",&(fiducial5.trans.x))) return false;
	if(!ser->readVariable("fiducial5_POSY",&(fiducial5.trans.y))) return false;
	if(!ser->readVariable("fiducial5_POSZ",&(fiducial5.trans.z))) return false;
	if(!ser->readVariable("fiducial5_ROT1",&(fiducial5.rot.x))) return false;
	if(!ser->readVariable("fiducial5_ROT2",&(fiducial5.rot.y))) return false;
	if(!ser->readVariable("fiducial5_ROT3",&(fiducial5.rot.z))) return false;

	if(!ser->readVariable("fiducial6_ID",&(fiducial6.id))) return false;
	if(!ser->readVariable("fiducial6_POSX",&(fiducial6.trans.x))) return false;
	if(!ser->readVariable("fiducial6_POSY",&(fiducial6.trans.y))) return false;
	if(!ser->readVariable("fiducial6_POSZ",&(fiducial6.trans.z))) return false;
	if(!ser->readVariable("fiducial6_ROT1",&(fiducial6.rot.x))) return false;
	if(!ser->readVariable("fiducial6_ROT2",&(fiducial6.rot.y))) return false;
	if(!ser->readVariable("fiducial6_ROT3",&(fiducial6.rot.z))) return false;

	afiducialmarkers->push_back(fiducial1);
	afiducialmarkers->push_back(fiducial2);
	afiducialmarkers->push_back(fiducial3);
	afiducialmarkers->push_back(fiducial4);
	afiducialmarkers->push_back(fiducial5);
	afiducialmarkers->push_back(fiducial6);

	//Scale to m
	for(unsigned int i = 0; i < afiducialmarkers->size(); i++){
	  
	  (*afiducialmarkers)[i].trans.x =  (*afiducialmarkers)[i].trans.x * scale;
	  (*afiducialmarkers)[i].trans.y =  (*afiducialmarkers)[i].trans.y * scale;
	  (*afiducialmarkers)[i].trans.z =  (*afiducialmarkers)[i].trans.z * scale;
	}

	ser->close();
	delete ser;
	return true;
}


//------------------------Coordinate Transformations----------------------------------------------------------------------------------


//XYZ Euler System to Matrix representation
cv::Mat eulerToMatrix(double rot1, double rot2, double rot3) {
  
	cv::Mat m(3,3,CV_64FC1);
	
	cv::Mat X_rot(3,3,CV_64F);
	cv::Mat Y_rot(3,3,CV_64F);
	cv::Mat Z_rot(3,3,CV_64F);
	
	double alpha =  rot1;
	double beta  =  rot2;
	double gamma =  rot3;
	
	X_rot.at<double>(0,0) =  1.0;
	X_rot.at<double>(0,1) =  0.0;
	X_rot.at<double>(0,2) =  0.0;
	X_rot.at<double>(1,0) =  0.0;
	X_rot.at<double>(1,1) =  cos(alpha);
	X_rot.at<double>(1,2) =  -sin(alpha);
	X_rot.at<double>(2,0) =  0.0;
	X_rot.at<double>(2,1) =  sin(alpha);
	X_rot.at<double>(2,2) =  cos(alpha);
	
	Y_rot.at<double>(0,0) =  cos(beta);
	Y_rot.at<double>(0,1) =  0.0;
	Y_rot.at<double>(0,2) =  sin(beta);
	Y_rot.at<double>(1,0) =  0.0;
	Y_rot.at<double>(1,1) =  1.0;
	Y_rot.at<double>(1,2) =  0.0;
	Y_rot.at<double>(2,0) =  -sin(beta);
	Y_rot.at<double>(2,1) =  0.0;
	Y_rot.at<double>(2,2) =  cos(beta);
	
	Z_rot.at<double>(0,0) =  cos(gamma);
	Z_rot.at<double>(0,1) =  -sin(gamma);
	Z_rot.at<double>(0,2) =  0.0;
	Z_rot.at<double>(1,0) =  sin(gamma);
	Z_rot.at<double>(1,1) =  cos(gamma);
	Z_rot.at<double>(1,2) =  0.0;
	Z_rot.at<double>(2,0) =  0.0;
	Z_rot.at<double>(2,1) =  0.0;
	Z_rot.at<double>(2,2) =  1.0;
	
	m = Z_rot*Y_rot*X_rot;
	
	return m;
}

// Function copied from cob_vision_ipa_utils/MathUtils.h to avoid dependency
inline float SIGN(float x)
{
  return (x >= 0.0f) ? +1.0f : -1.0f;
}

// Function copied from cob_vision_ipa_utils/MathUtils.h to avoid dependency
std::vector<double> FrameToVec7(const cv::Mat frame)
{
        // [0]-[2]: translation xyz
        // [3]-[6]: quaternion wxyz
        std::vector<double> pose(7, 0.0);

        double r11 = frame.at<double>(0,0);
        double r12 = frame.at<double>(0,1);
        double r13 = frame.at<double>(0,2);
        double r21 = frame.at<double>(1,0);
        double r22 = frame.at<double>(1,1);
        double r23 = frame.at<double>(1,2);
        double r31 = frame.at<double>(2,0);
        double r32 = frame.at<double>(2,1);
        double r33 = frame.at<double>(2,2);

        double qw = ( r11 + r22 + r33 + 1.0) / 4.0;
        double qx = ( r11 - r22 - r33 + 1.0) / 4.0;
        double qy = (-r11 + r22 - r33 + 1.0) / 4.0;
        double qz = (-r11 - r22 + r33 + 1.0) / 4.0;
        if(qw < 0.0f) qw = 0.0;
        if(qx < 0.0f) qx = 0.0;
        if(qy < 0.0f) qy = 0.0;
        if(qz < 0.0f) qz = 0.0;
        qw = std::sqrt(qw);
        qx = std::sqrt(qx);
        qy = std::sqrt(qy);
        qz = std::sqrt(qz);
        if(qw >= qx && qw >= qy && qw >= qz)
        {
            qw *= +1.0;
            qx *= SIGN(r32 - r23);
            qy *= SIGN(r13 - r31);
            qz *= SIGN(r21 - r12);
        }
        else if(qx >= qw && qx >= qy && qx >= qz)
        {
            qw *= SIGN(r32 - r23);
            qx *= +1.0;
            qy *= SIGN(r21 + r12);
            qz *= SIGN(r13 + r31);
        }
        else if(qy >= qw && qy >= qx && qy >= qz)
        {
            qw *= SIGN(r13 - r31);
            qx *= SIGN(r21 + r12);
            qy *= +1.0;
            qz *= SIGN(r32 + r23);
        }
        else if(qz >= qw && qz >= qx && qz >= qy)
        {
            qw *= SIGN(r21 - r12);
            qx *= SIGN(r31 + r13);
            qy *= SIGN(r32 + r23);
            qz *= +1.0;
        }
        else
        {
            printf("coding error\n");
        }
        double r = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        qw /= r;
        qx /= r;
        qy /= r;
        qz /= r;

        pose[3] = qw;
        pose[4] = qx;
        pose[5] = qy;
        pose[6] = qz;

        // Translation
        pose[0] = frame.at<double>(0,3);
        pose[1] = frame.at<double>(1,3);
        pose[2] = frame.at<double>(2,3);
        return pose;
}


/** Quaternion multiplication
 *
 */
void multiplyQuaternion(std::vector<double> q1,std::vector<double> q2, std::vector<double>* q)
{
    // First quaternion q1 (x1 y1 z1 r1)
    double x1=q1[0];
    double y1=q1[1];
    double z1=q1[2];
    double r1=q1[3];

    // Second quaternion q2 (x2 y2 z2 r2)
    double x2=q2[0];
    double y2=q2[1];
    double z2=q2[2];
    double r2=q2[3];


    q->push_back(x1*r2 + r1*x2 + y1*z2 - z1*y2);   // x component
    q->push_back(r1*y2 - x1*z2 + y1*r2 + z1*x2);   // y component
    q->push_back(r1*z2 + x1*y2 - y1*x2 + z1*r2);   // z component
    q->push_back(r1*r2 - x1*x2 - y1*y2 - z1*z2);   // r component
}

//-----------------------------------main----------------------------------------------------------------------------
int main( int argc, char** argv )
{  
  ros::init(argc, argv, "sensornode_detection");
  ros::NodeHandle node;

  //ros::Timer timer = node.createTimer(ros::Duration(0.1), timerCallback);
 
  ros::Subscriber sub = node.subscribe("/fiducials/fiducial_custom_array", 1000, chatterCallback);
 
  if(!loadParameters(&fiducialmarkers)){
    ROS_ERROR("Failed to load Fiducial parameters");
    return 0;
  }
  
  //It is not possible to use this subscriber/callback right now cause cob_object_detection_msgs is still a rosbuild package
  //Maybe ask Richard/Jan if it is possible to make it a catkin package in the offical repo.
  //I'm too lazy right now to write a workaround or a self created cob_object_detection_msg catkin branch.
  //The marker detection can be started in a shell when using the  command in the next line.
  //CMD: rostopic hz /fiducials/detect_fiducials
  

  ros::spin();
  return 0;
}

