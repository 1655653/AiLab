#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <projectAI/goalistAction.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#define pi 3.141592653589793238462643383279502884
#define N 3


class Coordinates
{
public:
	ros::Subscriber point_sub;
	ros::NodeHandle nh;
	std::vector<geometry_msgs::Point> goals_list_aux;
	
	Coordinates() {
		point_sub = nh.subscribe("/clicked_point", N, &Coordinates::getPoint, this);

	}
	///-------------points acquisition------------///
	void getPoint(const geometry_msgs::PointStamped::ConstPtr& coordinates) {
		double x = coordinates->point.x;
		double y = coordinates->point.y;
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		goals_list_aux.push_back(p);
		
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "client");
	ros::NodeHandle n;
	
	///-----------Trasform robot wrt map-------///
	tf::TransformListener listener;
	tf::StampedTransform robot_position;
	try {
		listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), robot_position);
	}
	catch(tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	double current_angle = tf::getYaw(robot_position.getRotation());
	///--------------------------------------------------
	
	actionlib::SimpleActionClient<projectAI::goalistAction> ac("server", true);

	std::cout << "Waiting for action server to start\n";
	ac.waitForServer();
								

	
	projectAI::goalistGoal goal;
	goal.goals_number = 0;
	std::vector<geometry_msgs::Point> goals_list_aux(goal.goals_number);
	goal.goals_list = goals_list_aux;
	
	///-------------First Rotation--------///
	goal.final_theta = current_angle + (pi / 2);
	ac.sendGoal(goal);
	bool finished_before_timeout = ac.waitForResult(ros::Duration(100.0));
	
	
	ac.waitForServer();
	goal.final_theta +=  (pi / 2);
	ac.sendGoal(goal);
	finished_before_timeout &= ac.waitForResult(ros::Duration(100.0));
	
	
	ac.waitForServer();
	goal.final_theta +=  (pi / 2);
	ac.sendGoal(goal);
	finished_before_timeout &= ac.waitForResult(ros::Duration(100.0));
	
	ac.waitForServer();
	goal.final_theta +=  (pi / 2);
	ac.sendGoal(goal);
	finished_before_timeout &= ac.waitForResult(ros::Duration(100.0));
	///---------------------------------------------
	

	Coordinates points;
	
	while(points.goals_list_aux.size() < N) {
		ros::spinOnce();
	}
	
	goal.goals_number = N;
	goal.goals_list = points.goals_list_aux;
	
	ac.waitForServer();
	
	std::cout << "Action server started, sending goal\n";
	ac.sendGoal(goal);
	
	finished_before_timeout &= ac.waitForResult(ros::Duration(100.0));

	if(finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		std::cout << "Action finished: " << state.toString().c_str() << "\n";
	}
	else
		std::cout << "Action did not finish before the time out\n";
	
	return 0;
}
