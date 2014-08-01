#include <seneka_pnp_tools.h>

sensornode seneka_pnp_tools::getSensornodePose(){

	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::TransformListener listener_entry;
	tf::StampedTransform transform_entry;
	tf::TransformListener listener_up;
	tf::StampedTransform transform_up;  
	tf::TransformListener listener_down;
	tf::StampedTransform transform_down;

	sensornode node;
	node.success = true;

	//sensornode pose
	try{
		listener.waitForTransform("/quanjo_body", "/sensornode" , ros::Time::now(), ros::Duration(0.2));
		listener.lookupTransform("/quanjo_body", "/sensornode", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		node.success = false;
	}

	node.pose.position.x = transform.getOrigin().x();
	node.pose.position.y = transform.getOrigin().y();
	node.pose.position.z = transform.getOrigin().z();

	node.pose.orientation.w = transform.getRotation().getW();
	node.pose.orientation.x = transform.getRotation().getX();
	node.pose.orientation.y = transform.getRotation().getY();
	node.pose.orientation.z = transform.getRotation().getZ();


	//handholds and helper points
	for(unsigned int i = 1; i < 7;i++){

		//get all tf transformations
		char name[50];

		sprintf(name,"handle%u",i);
		try{
			listener.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
			listener.lookupTransform("/quanjo_body", name, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			node.success = false;
		}

		sprintf(name,"grab_entry%u",i);
		try{
			listener_entry.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
			listener_entry.lookupTransform("/quanjo_body", name, ros::Time(0), transform_entry);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			node.success = false;
		}

		sprintf(name,"trigger_%u_up",i);
		try{
			listener_up.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
			listener_up.lookupTransform("/quanjo_body", name, ros::Time(0), transform_up);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			node.success = false;
		}

		sprintf(name,"trigger_%u_down",i);
		try{
			listener_down.waitForTransform("/quanjo_body", name , ros::Time::now(), ros::Duration(0.2));
			listener_down.lookupTransform("/quanjo_body", name, ros::Time(0), transform_down);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			node.success = false;
		}

		handhold handh;
		//handle
		handh.handle.position.x = transform.getOrigin().x();
		handh.handle.position.y = transform.getOrigin().y();
		handh.handle.position.z = transform.getOrigin().z();

		handh.handle.orientation.w = transform.getRotation().getW();
		handh.handle.orientation.x = transform.getRotation().getX();
		handh.handle.orientation.y = transform.getRotation().getY();
		handh.handle.orientation.z = transform.getRotation().getZ();

		//entry
		handh.entry.position.x = transform_entry.getOrigin().x();
		handh.entry.position.y = transform_entry.getOrigin().y();
		handh.entry.position.z = transform_entry.getOrigin().z();

		handh.entry.orientation.w = transform_entry.getRotation().getW();
		handh.entry.orientation.x = transform_entry.getRotation().getX();
		handh.entry.orientation.y = transform_entry.getRotation().getY();
		handh.entry.orientation.z = transform_entry.getRotation().getZ();

		//up
		handh.up.position.x = transform_up.getOrigin().x();
		handh.up.position.y = transform_up.getOrigin().y();
		handh.up.position.z = transform_up.getOrigin().z();

		handh.up.orientation.w = transform_up.getRotation().getW();
		handh.up.orientation.x = transform_up.getRotation().getX();
		handh.up.orientation.y = transform_up.getRotation().getY();
		handh.up.orientation.z = transform_up.getRotation().getZ();

		//down
		handh.down.position.x = transform_down.getOrigin().x();
		handh.down.position.y = transform_down.getOrigin().y();
		handh.down.position.z = transform_down.getOrigin().z();

		handh.down.orientation.w = transform_down.getRotation().getW();
		handh.down.orientation.x = transform_down.getRotation().getX();
		handh.down.orientation.y = transform_down.getRotation().getY();
		handh.down.orientation.z = transform_down.getRotation().getZ();      

		node.handholds.push_back(handh);
	}
	return node;
}
bool seneka_pnp_tools::multiplan(move_group_interface::MoveGroup* group, moveit::planning_interface::MoveGroup::Plan* plan){

	uint maxattempts = 20;
	uint attempt = 0;

	bool validplan = false;

	while(!validplan && attempt < maxattempts){

		validplan = group->plan(*plan);
		attempt++;
	}

	return validplan;	  
}


move_group_interface::MoveGroup::Plan seneka_pnp_tools::mergePlan(move_group_interface::MoveGroup::Plan plan1, move_group_interface::MoveGroup::Plan plan2)
{
	move_group_interface::MoveGroup::Plan mergedPlan;
	mergedPlan = plan1;
	mergedPlan.trajectory_.joint_trajectory.joint_names.insert(mergedPlan.trajectory_.joint_trajectory.joint_names.end(),
			plan2.trajectory_.joint_trajectory.joint_names.begin(),
			plan2.trajectory_.joint_trajectory.joint_names.end());

	size_t i;
	for(i = 0; (i < mergedPlan.trajectory_.joint_trajectory.points.size())
	&& (i < plan2.trajectory_.joint_trajectory.points.size()); i++){

		mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.insert(
				mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.end(),
				plan2.trajectory_.joint_trajectory.points[i].accelerations.begin(),
				plan2.trajectory_.joint_trajectory.points[i].accelerations.end());

		mergedPlan.trajectory_.joint_trajectory.points[i].positions.insert(
				mergedPlan.trajectory_.joint_trajectory.points[i].positions.end(),
				plan2.trajectory_.joint_trajectory.points[i].positions.begin(),
				plan2.trajectory_.joint_trajectory.points[i].positions.end());

		mergedPlan.trajectory_.joint_trajectory.points[i].velocities.insert(
				mergedPlan.trajectory_.joint_trajectory.points[i].velocities.end(),
				plan2.trajectory_.joint_trajectory.points[i].velocities.begin(),
				plan2.trajectory_.joint_trajectory.points[i].velocities.end());
	}

	if(plan1.trajectory_.joint_trajectory.points.size() > plan2.trajectory_.joint_trajectory.points.size()){

		for(size_t j = i; j < plan1.trajectory_.joint_trajectory.points.size(); j++){

			mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.insert(
					mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.end(),
					plan2.trajectory_.joint_trajectory.points.back().accelerations.begin(),
					plan2.trajectory_.joint_trajectory.points.back().accelerations.end());

			mergedPlan.trajectory_.joint_trajectory.points[j].positions.insert(
					mergedPlan.trajectory_.joint_trajectory.points[j].positions.end(),
					plan2.trajectory_.joint_trajectory.points.back().positions.begin(),
					plan2.trajectory_.joint_trajectory.points.back().positions.end());

			mergedPlan.trajectory_.joint_trajectory.points[j].velocities.insert(
					mergedPlan.trajectory_.joint_trajectory.points[j].velocities.end(),
					plan2.trajectory_.joint_trajectory.points.back().velocities.begin(),
					plan2.trajectory_.joint_trajectory.points.back().velocities.end());
		}
	}

	if(plan1.trajectory_.joint_trajectory.points.size() < plan2.trajectory_.joint_trajectory.points.size()){

		trajectory_msgs::JointTrajectoryPoint point;
		for(size_t j = i; j < plan2.trajectory_.joint_trajectory.points.size(); j++){

			point  = mergedPlan.trajectory_.joint_trajectory.points.back();

			point.accelerations.insert(
					point.accelerations.end(),
					plan2.trajectory_.joint_trajectory.points[j].accelerations.begin(),
					plan2.trajectory_.joint_trajectory.points[j].accelerations.end());

			point.positions.insert(
					point.positions.end(),
					plan2.trajectory_.joint_trajectory.points[j].positions.begin(),
					plan2.trajectory_.joint_trajectory.points[j].positions.end());

			point.velocities.insert(
					point.velocities.end(),
					plan2.trajectory_.joint_trajectory.points[j].velocities.begin(),
					plan2.trajectory_.joint_trajectory.points[j].velocities.end());

			point.time_from_start = plan2.trajectory_.joint_trajectory.points[j].time_from_start;

			mergedPlan.trajectory_.joint_trajectory.points.push_back(point);
		}
	}

	return mergedPlan;
}


moveit::planning_interface::MoveGroup::Plan seneka_pnp_tools::scaleTrajSpeed(moveit::planning_interface::MoveGroup::Plan plan, double factor){

	std::vector<trajectory_msgs::JointTrajectoryPoint> points = plan.trajectory_.joint_trajectory.points;

	for(uint i = 0; i < points.size();i++){

		for(uint j = 0; j < points[i].velocities.size();j++)
			points[i].velocities[j] = points[i].velocities[j] * factor;
		for(uint j = 0; j < points[i].accelerations.size();j++)
			points[i].velocities[j] = points[i].accelerations[j] * factor * factor;

		points[i].time_from_start = points[i].time_from_start * (1/factor);
	}

	plan.trajectory_.joint_trajectory.points = points;

	return plan;
}

//wrapper for calling the fk service due to some failures when using it with move_group.h
void seneka_pnp_tools::fk_solver(ros::NodeHandle *node_handle, std::vector<double> &joint_positions_r, std::vector<double> &joint_positions_l, geometry_msgs::Pose *pose_l, geometry_msgs::Pose *pose_r){

	ros::ServiceClient service_computefk;
	service_computefk = node_handle->serviceClient<moveit_msgs::GetPositionFK> ("compute_fk");		

	robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
	robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
	robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
	robot_state::JointStateGroup* joint_state_group_l = kinematic_state_l->getJointStateGroup("left_arm_group");  
	robot_state::JointStateGroup* joint_state_group_r = kinematic_state_l->getJointStateGroup("right_arm_group"); 

	moveit_msgs::GetPositionFK::Request service_request;
	moveit_msgs::GetPositionFK::Response service_response; 
	sensor_msgs::JointState js;

	//------left-----------------------
	js.name = joint_state_group_l->getJointNames();
	js.position = joint_positions_l;

	service_request.header.frame_id = "world_dummy_link";
	service_request.fk_link_names.push_back("left_arm_ee_link");
	service_request.robot_state.joint_state = js;

	service_computefk.call(service_request, service_response);
	if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
		for(uint i=0;i<service_response.pose_stamped.size();i++){
			std::cout << service_response.pose_stamped[i].pose.position << std::endl;
			std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
			*pose_l = service_response.pose_stamped[i].pose;
		}
	}   

	//-----right-----------------
	js.name = joint_state_group_r->getJointNames();
	js.position = joint_positions_r;

	service_request.header.frame_id = "world_dummy_link";
	service_request.fk_link_names.push_back("right_arm_ee_link");
	service_request.robot_state.joint_state = js;

	service_computefk.call(service_request, service_response);
	if(service_response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
		for(uint i=0;i<service_response.pose_stamped.size();i++){
			std::cout << service_response.pose_stamped[i].pose.position << std::endl;
			std::cout << service_response.pose_stamped[i].pose.orientation << std::endl;
			*pose_r = service_response.pose_stamped[i].pose;
		}
	}  
}


dual_arm_joints seneka_pnp_tools::generateIkSolutions(ros::NodeHandle nh,
												std::vector<double>& referencejoints_r, std::vector<double>& referencejoints_l,
												geometry_msgs::Pose handle_r, geometry_msgs::Pose handle_l, 
												moveit_msgs::Constraints constraints_r, moveit_msgs::Constraints constraints_l,
												bool equaljointstates, bool freezeikleft, bool freezeikright) 
{

	dual_arm_joints ret;
	
	ros::Publisher robot_state_publisher_l, robot_state_publisher_r;
	robot_state_publisher_l = nh.advertise
			< moveit_msgs::DisplayRobotState > ("robot_state_l", 1);
	robot_state_publisher_r = nh.advertise
			< moveit_msgs::DisplayRobotState > ("robot_state_r", 1);
		
	uint attempts = 100;//300
	
	std::vector<double> joint_values_l;
	std::vector<double> joint_values_r;

	std::vector < std::string > joint_names_l;
	std::vector < std::string > joint_names_r;

	ROS_INFO("Generate IK Solution");
	ros::ServiceClient service_client;
	service_client = nh.serviceClient < moveit_msgs::GetPositionIK > ("compute_ik");

	geometry_msgs::Pose target_pose_l = handle_l;
	geometry_msgs::Pose target_pose_r = handle_r;

	moveit_msgs::GetPositionIK::Request service_request;
	moveit_msgs::GetPositionIK::Response service_response;

	//service_request.ik_request.timeout = ros::duration(0.01);
	service_request.ik_request.attempts = 1;
	service_request.ik_request.pose_stamped.header.frame_id = "world_dummy_link";
	service_request.ik_request.avoid_collisions = false;
	
	//node_pose node;
	//node = smartJointValues(start_pose);
	//node = start_pose;
	
	std::deque<bool> successstorage;
	for(uint i=0; i< 10;i++){
		successstorage.push_back(true);
	}
	
		
	
	//freezes ik generation when necessary
	if (!freezeikleft) {
		//left arm
		service_request.ik_request.group_name = "left_arm_group";
		service_request.ik_request.pose_stamped.pose = target_pose_l;
		service_request.ik_request.constraints = constraints_l;

		uint samples;
		equaljointstates ? samples = attempts : samples = 1;
		
		double statedistance = 6 * 2 * M_PI;
		
		robot_model_loader::RobotModelLoader robot_model_loader_l("robot_description");
		robot_model::RobotModelPtr kinematic_model_l = robot_model_loader_l.getModel();
		robot_state::RobotStatePtr kinematic_state_l(new robot_state::RobotState(kinematic_model_l));
		robot_state::JointStateGroup* joint_state_group_l =	kinematic_state_l->getJointStateGroup("left_arm_group");

		double successrate = 1;
		bool success;
		for (uint i = 0; (i < samples); i++) {
			
			if(successrate < 0.5){
				service_request.ik_request.timeout = ros::Duration(0.05);
				service_request.ik_request.attempts = 20;
			} else {
				service_request.ik_request.timeout = ros::Duration(0.01);
				service_request.ik_request.attempts = 1;
			}
			
			joint_values_l.clear();
			service_client.call(service_request, service_response);

			if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {
				success = true;
				
				for (uint i = 0; i < 6; i++) {
					//std::cout << service_response.solution.joint_state.name[i] << ": ";
					//std::cout << service_response.solution.joint_state.position[i] << std::endl;

					joint_values_l.push_back(service_response.solution.joint_state.position[i]);
					//joint_names_l.push_back(service_response.solution.joint_state.name[i]);
				}

				//joint_state_group_l->setVariableValues(service_response.solution.joint_state.position);
				joint_state_group_l->setVariableValues(joint_values_l);
				//joint_values_l = smartJointValues(joint_values_l);

				double tmp = seneka_pnp_tools::getStateDistance(referencejoints_l,	joint_values_l);
				ROS_INFO("STATE DISTANCE L: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
				if (tmp < statedistance) {

					moveit_msgs::DisplayRobotState msg_l;
					robot_state::robotStateToRobotStateMsg(*kinematic_state_l, msg_l.state);
					robot_state_publisher_l.publish(msg_l);

					ROS_INFO("IK Solution L: TRUE");
					ret.left = joint_values_l;
					statedistance = tmp;
				}

				//joint_state_group_l;

			} else {
				success = false;
				ROS_INFO("IK Solution L: FALSE");
			}
			successstorage.pop_back();
			successstorage.push_front(success);
			uint successcounter = 0;
			for(uint j=0; j < successstorage.size(); j++){
				if(successstorage[j]) successcounter++;
			}
			successrate = (double)successcounter/successstorage.size();
			
		}
	}

	successstorage.clear();
	for(uint i=0; i< 10;i++){
		successstorage.push_back(true);
	}
	//freezes ik generation when necessary
	if (!freezeikright) {
		service_request.ik_request.group_name = "right_arm_group";
		service_request.ik_request.pose_stamped.pose = target_pose_r;
		service_request.ik_request.constraints = constraints_r;

		uint samples;
		equaljointstates ? samples = attempts : samples = 1;

		double statedistance = 6 * 2 * M_PI;

		/* Load the robot model */
		robot_model_loader::RobotModelLoader robot_model_loader_r("robot_description");
		robot_model::RobotModelPtr kinematic_model_r = robot_model_loader_r.getModel();
		robot_state::RobotStatePtr kinematic_state_r(new robot_state::RobotState(kinematic_model_r));
		robot_state::JointStateGroup* joint_state_group_r = kinematic_state_r->getJointStateGroup("right_arm_group");

		double successrate = 1;
		bool success;
		for (uint i = 0; (i < samples); i++) {
			
			if(successrate < 0.5){
				service_request.ik_request.timeout = ros::Duration(0.05);
				service_request.ik_request.attempts = 20;
			} else {
				service_request.ik_request.timeout = ros::Duration(0.01);
				service_request.ik_request.attempts = 1;
			}
			
			joint_values_r.clear();
			service_client.call(service_request, service_response);

			if (service_response.error_code.val	== moveit_msgs::MoveItErrorCodes::SUCCESS) {
				
				success = true;
				for (uint i = 6; i < 12; i++) {
					//std::cout << service_response.solution.joint_state.name[i] << ": ";
					//std::cout << service_response.solution.joint_state.position[i] << std::endl;

					joint_values_r.push_back(service_response.solution.joint_state.position[i]);
					//joint_names_r.push_back(service_response.solution.joint_state.name[i]);
				}

				joint_state_group_r->setVariableValues(joint_values_r);
				//joint_values_r = smartJointValues(joint_values_r);

				double tmp = seneka_pnp_tools::getStateDistance(referencejoints_r,	joint_values_r);
				ROS_INFO("STATE DISTANCE R: %f  \n   TMP DISTANCE: %f", statedistance, tmp);
				if (tmp < statedistance) {

					moveit_msgs::DisplayRobotState msg_r;
					robot_state::robotStateToRobotStateMsg(*kinematic_state_r,	msg_r.state);
					robot_state_publisher_r.publish(msg_r);

					ROS_INFO("IK Solution R: TRUE");

					ret.right = joint_values_r;
					statedistance = tmp;
				}

			} else {
				success = false;
				ROS_INFO("IK Solution R: FALSE");
			}
			
			successstorage.pop_back();
			successstorage.push_front(success);
			uint successcounter = 0;
			for(uint j=0; j < successstorage.size(); j++){
				if(successstorage[j]) successcounter++;
			}
			successrate = (double)successcounter/successstorage.size();
		}
	}

	for(uint i=0; i<ret.left.size();i++){
		ret.both.push_back(ret.left[i]);
	}
	for(uint i=0; i<ret.right.size();i++){
		ret.both.push_back(ret.right[i]);
	}
	//tmp_pose_ = smartJointValues(tmp_pose_);

	return ret;
}

//returns distance between two states based on cumulated difference of joints
double seneka_pnp_tools::getStateDistance(std::vector<double> a, std::vector<double> b) {

	double distance = 0;

	for (uint i = 0; i < a.size() && i < b.size(); i++) {
		
		double res = a[i] - b[i];
		distance += std::sqrt(res * res);
		//ROS_INFO("[%u] RJ:[%f]   CJ:[%f]  RES:[%f]  DIST:[%f]",i,a[i],b[i],res,distance);
	}

	return distance;
}

moveit_msgs::Constraints seneka_pnp_tools::generateIKConstraints(const char* command, std::vector<std::string> joint_names, std::vector<double> joint_positions, double tolerance, double position) {
	
	//options
	bool all = false;
	bool copy = false;
	std::vector < moveit_msgs::JointConstraint > joint_constraints;
	
	//exploit the commands
	std::string cmd(command);
	std::vector < std::string > fields;
	boost::split(fields, cmd, boost::is_any_of(" "));
	for (uint i = 0; i < fields.size(); i++) {
		ROS_INFO("command: %s \n", fields[i].c_str());
		if(!fields[i].compare("all")) all = true;
		if(!fields[i].compare("copy")) copy = true;
	}
	ROS_INFO("command value all: %d \n", all);
	ROS_INFO("command value copy: %d \n", copy);

	//extract the affected joints		
	//move_group_interface::MoveGroup movegroup(group);
	//std::vector < std::string > joint_names = movegroup.getJoints();
	//std::vector<double> joint_values = movegroup.getCurrentJointValues();
	
	if (all) { //extract all
		for (uint i = 0; i < joint_names.size(); i++) {
			moveit_msgs::JointConstraint jconstraint;
			jconstraint.joint_name = joint_names[i];
			jconstraint.position = joint_positions[i];
			jconstraint.weight = 0.5;
			joint_constraints.push_back(jconstraint);
		}
	} else { //extract only specific joints
		for (uint i = 0; i < fields.size(); i++) {//iterate through all commands
			for (uint j = 0; j < 12; j++) {	//iterate through possible joint numbers
				char joint_number[20];
				sprintf(joint_number, "%u", j);
				if (!fields[i].compare(joint_number)) {
					if (j < joint_names.size() && j < joint_positions.size()) {
						moveit_msgs::JointConstraint jconstraint;
						jconstraint.joint_name = joint_names[j];//name
						jconstraint.position = joint_positions[j];//value
						jconstraint.weight = 0.5;
						joint_constraints.push_back(jconstraint);
					} else {
						ROS_INFO("Ups.. joint %u is not available. There are only %u joints known", j, (uint) joint_names.size());
					}
				}
			}
		}
	}

	if (!copy) {// use the value given in position			
		for (uint i = 0; i < joint_constraints.size(); i++){
			joint_constraints[i].position = position;		
		}	
	}
	
	//copy the tolerance
	for (uint i = 0; i < joint_constraints.size(); i++){
		joint_constraints[i].tolerance_above = tolerance;
		joint_constraints[i].tolerance_below = tolerance;
	}

	moveit_msgs::Constraints constraints;
	constraints.joint_constraints = joint_constraints;
	
	return constraints;
}

dualArmJointState seneka_pnp_tools::createArmState(const char* state_name,
								 double r0, double r1, double r2, double r3, double r4, double r5,
								 double l0, double l1, double l2, double l3, double l4, double l5)
{
	dualArmJointState state;
	state.name = std::string(state_name);
	
	state.right.position.push_back(r0);	
	state.right.position.push_back(r1);	
	state.right.position.push_back(r2);	
	state.right.position.push_back(r3);	
	state.right.position.push_back(r4);	
	state.right.position.push_back(r5);	
	
	state.left.position.push_back(l0);	
	state.left.position.push_back(l1);	
	state.left.position.push_back(l2);	
	state.left.position.push_back(l3);	
	state.left.position.push_back(l4);	
	state.left.position.push_back(l5);	
	
	for(uint i=0; i<state.left.position.size();i++ ){
		state.both.position.push_back(state.left.position[i]);
	}
	for(uint i=0; i<state.right.position.size();i++ ){
		state.both.position.push_back(state.right.position[i]);
	}	
	
	return state;
}

std::vector<dualArmJointState> seneka_pnp_tools::createArmStates(){
	
	std::vector<dualArmJointState> states;
	//the home state
	states.push_back( createArmState("home", 
									-1.5705, 0, -2.5, 3.141, 3.141, -1.7,
									1.5705, -3.141, 2.5, 0, 3.141, 1.7));
	states.push_back( createArmState("prepack-rear", 
									1.55604, -0.348553, -1.12702, 1.50556, -0.0127962, 2.91043,
									-1.558, -2.79, 1.12702, 1.52883, 0.012214, -2.74885) );
	states.push_back( createArmState("packed-rear", 
									1.58806, -1.43113, -1.80313, 3.21233, 0.019241, 2.96233,
									-1.58925, -1.69424, 1.79061, 0.00715636, -0.0193046, -3.01626) );
	states.push_back( createArmState("packed-rear-h1",
									1.60587, -1.01243, -1.98466, 2.98633, 0.0370389, 2.95235,
									-1.60649, -2.11752, 1.97315, 0.199017, -0.0364981, -2.96735) );
	states.push_back( createArmState("pregrasp", 
									-1.9705, -2.441, -0.8, 3.2, 3.241, -3.3,
									1.9705, -0.7, 0.8, 0, 3.041, 3.3));
	states.push_back( createArmState("pregrasp-rear", 
									1.3, -0.6, 0.5, 0, -0.0127962, 3.05,
									-1.3, -2.5, -0.5, 3.141, 0.012214, -3.141));
	states.push_back( createArmState("pregrasp-rear-h1",
									-1.5705, -1.6, -2.5, 3.141, 3.141, -1.7,
									1.5705, -1.5, 2.5, 0, 3.141, 1.7) );
	states.push_back( createArmState("pregrasp-rear-h2", 
									-1.5705, -1.6, 0, 3.141, 3.141, -1.7,
									1.5705, -1.5, 0, 0, 3.141, 1.7) );
	states.push_back( createArmState("pregrasp-rear-h3",
									1.5, -1.6, 0, 3.141, 3.141, -1.7,
									-1.5, -1.5, 0, 0, 3.141, 1.7) );
	states.push_back( createArmState("pregrasp-rear-h4",
									1.5, -1.0, 0, 3.141, 3.141, -1.7,
									-1.5, -2.1, 0, 0, 3.141, 1.7) );	
	states.push_back( createArmState("pregrasp-rear-h5",
									1.5, -1.0, 0, 3.141, 0, -1.7,
									-1.5, -2.1, 0, 0, 0, 1.7) );	
	states.push_back( createArmState("prepack",
									-1.06935, -1.70806, -1.08, 2.788, 3.64382, -3.339,
									1.0674, -1.43427, 1.08065, 0.348302, 2.63745, 3.36682));
	states.push_back( createArmState("packed-front",
									1.04328, -0.780763, -1.7279, 2.50947, 5.75764, -3.34229,
									-1.03806, -2.35984, 1.72832, 0.627586, 0.531993, 3.37383) );
	states.push_back( createArmState("packed-front-drop",
									0.398362, -0.642881, -2.33532, 2.97863, 5.11272, -3.34176,
									-0.399573, -2.49877, 2.33571, 0.160897, 1.17048, 3.37129) );
		
	
	return states;
}

bool seneka_pnp_tools::getArmState(std::vector<dualArmJointState>& states, const char* state_name, dualArmJointState* result){
	
	for(uint i=0; i<states.size(); i++){
		if(!states[i].name.compare(state_name)){
			*result = states[i];
			return true;
		}
	}	
	return false;
}




































