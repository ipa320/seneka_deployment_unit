/*
  seneka_teach 

  teaching a duual arm trajectory
  Author: Matthias Nösner  
*/

#include <ros/ros.h>

#include <opencv/cv.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <SerializeIO.h>
#include <termios.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_state/robot_state.h>

class SenekaTeach
{
private:
  ros::NodeHandle node_handle_;
  SerializeIO *ser;
  int c;


public:

  SenekaTeach(ros::NodeHandle& nh){
    node_handle_ = nh;
    init();
  }

  //Destructor
  ~SenekaTeach(){
  }

  void init(){    
    ser = new SerializeIO("/home/matthias/groovy_workspace/catkin_ws/src/seneka_deployment_unit/seneka_pnp/common/teached_dual_arm_movement.def",'o');
    ser->openArray("dual_arm_movement");
    mainLoop();
  } 

 //std stuff to handle keycommands
  int getch()
  {
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering      
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
  }
  

  void teach(){

    if(c == 't'){
      move_group_interface::MoveGroup group_r("right_arm_group");
      move_group_interface::MoveGroup group_l("left_arm_group");

      group_r.setStartStateToCurrentState();      
      group_l.setStartStateToCurrentState();

      geometry_msgs::Pose pose_r = group_r.getCurrentPose().pose;
      geometry_msgs::Pose pose_l = group_l.getCurrentPose().pose;

      std::vector<double> vec_r,vec_l;
      vec_r.push_back(pose_r.position.x);
      vec_r.push_back(pose_r.position.y);
      vec_r.push_back(pose_r.position.z);
      vec_r.push_back(pose_r.orientation.w);
      vec_r.push_back(pose_r.orientation.x);
      vec_r.push_back(pose_r.orientation.y);
      vec_r.push_back(pose_r.orientation.z);

      vec_l.push_back(pose_l.position.x);
      vec_l.push_back(pose_l.position.y);
      vec_l.push_back(pose_l.position.z);
      vec_l.push_back(pose_l.orientation.w);
      vec_l.push_back(pose_l.orientation.x);
      vec_l.push_back(pose_l.orientation.y);
      vec_l.push_back(pose_l.orientation.z);

      ser->writeVector(vec_r);
      ser->writeVector(vec_l);
      
      ROS_INFO("Successfully teached poses for both arms");      
    }
  }

  void close(){
    ser->closeArray();
    ser->close();
    delete ser;
    ROS_INFO("Successfully closed array and file"); 
  }


  void mainLoop(){
    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Rate loop_rate(10);
    while(ros::ok() && c != 'c'){ 
      c = getch();  // call your non-blocking input function
      teach();
    }
    close();
  }
};




int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "SenekaTeach");
    ros::NodeHandle nh;

    /// Create SenekaTeach instance with mainLoop inside
    SenekaTeach* seneka_teach = new SenekaTeach(nh);
    
    delete seneka_teach;
    return 0;
}
