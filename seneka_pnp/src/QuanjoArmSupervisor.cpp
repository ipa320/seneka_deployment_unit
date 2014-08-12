#include <QuanjoArmSupervisor.h>

class QuanjoArmSupervisorAction{
	
	QuanjoArmSupervisorAction(std::string name, SenekaPickAndPlace* p) :
	    as_(node_handle_, name, boost::bind(&QuanjoArmSupervisorAction::executeCB, this, _1), false),
	    action_name_(name)
	  {
	    as_.start();
	  }

	  ~QuanjoArmSupervisorAction(void)
	  {
	  }

	  void executeCB(const seneka_pnp::QuanjoManipulationGoalConstPtr &goal)
	  {

	  }
};