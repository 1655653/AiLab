#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <homework5/CountdownAction.h>
using namespace std;
int main (int argc, char **argv)
{
  ros::init(argc, argv, "Stop_Countdown");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<homework5::CountdownAction> ac("CountdownServer", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ac.cancelAllGoals();
  ROS_INFO("STOPPED COUNTING");

  //exit
  return 0;
}
