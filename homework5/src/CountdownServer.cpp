#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <homework5/CountdownAction.h>
using namespace std;

class CountdownAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<homework5::CountdownAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  homework5::CountdownFeedback feedback_;
  homework5::CountdownResult result_;

public:

  CountdownAction(std::string name) :
    as_(nh_, name, boost::bind(&CountdownAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~CountdownAction(void)
  {
  }

  void executeCB(const homework5::CountdownGoalConstPtr &goal)
  {
    // helper variables
    
    bool success = true;

	
    

    // start executing the action
		for(int i = goal->n; i >= 0; i--) {
			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				success = false;
				break;
			}
			// sleep for one second
			ROS_INFO("Executing, i'm counting down: %i", i);
			ros::Duration(1.0).sleep();
		}

    if(success)
    {
      result_.result = feedback_.result;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "CountdownServer");
  CountdownAction Countdown("CountdownServer");
  ros::spin();

  return 0;
}
