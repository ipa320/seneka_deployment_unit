/*
 * QuanjoArmSupervisor.h
 *
 *  Created on: 12.08.2014
 *      Author: Matthias Nösner
 */

#ifndef QUANJO_ARM_SUPERVISOR_H_
#define QUANJO_ARM_SUPERVISOR_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <seneka_pnp/QuanjoManipulationAction.h>

protected:
	ros::NodeHandle node_handle_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<seneka_pnp::QuanjoManipulationAction> as_; 
	std::string action_name_;
	// create messages that are used to published feedback/result
	seneka_pnp::QuanjoManipulationFeedback feedback_;
	seneka_pnp::QuanjoManipulationResult result_;
public:
	QuanjoArmSupervisorAction(std::string name, SenekaPickAndPlace* p);

	  ~QuanjoArmSupervisorAction(void)
	  {
	  }

	  void executeCB(const seneka_pnp::QuanjoManipulationGoalConstPtr &goal)
	  {

	  }
	
#endif